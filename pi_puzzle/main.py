"""
拼图装置主程序 (缝合版 v2)

状态机流程:
  INIT -> CALIBRATE -> DETECT -> SOLVE -> EXECUTE -> DONE

DETECT/SOLVE 已切换到 123 流水线 (2026-08-01 缝合):
  - DETECT: 固定 ChArUco 单应 (quad_from_homography) + detect_pieces_arbitrary
    多帧投票 + flat_th 扫描 + 清晰轮廓模式 (扑克牌尖角)
  - SOLVE: AssemblySolver (T型接缝/位姿松弛/近似解兜底) + 宽松模式兜底;
    task1 按自备目标矩形 10×6cm 校验反推结果
  - 放置: compute_placement 放入下半区中心 + build_tasks + 贪心排序
"""

import importlib
import sys
import time

import numpy as np

from arm_controller import ArmController
from camera_backend import create_camera_backend, release_camera_backend
from config import (
    BUZZER_PIN,
    LED_PIN,
    SERVO_BAUDRATE,
    SERVO_PORT,
    TASK1_RECT_TOL_MM,
    TASK1_TARGET_RECT_MM,
)
from motion_planner import MotionPlanner
from rect_assembly import AssemblySolver
from solve_paths import (
    build_tasks,
    compute_placement,
    draw_failure_diagnostic,
    optimize_order,
)
from vision import PuzzleVision

# 检测阈值扫描: 噪声场景单一 flat_th 可能不自洽, 多组检测+求解取评分最优。
# (solve_paths.py 同款; 亮光模式下 flat_th 无效, 检测签名相同自动跳过)
FLAT_TH_SWEEP = (0.82, 0.78, 0.86, 0.74, 0.90)
DETECT_VOTE_FRAMES = 3  # 多帧投票帧数 (含 frame_cb 预览)


class PuzzleRobot:
    def __init__(
        self,
        arm=None,
        abort_event=None,
        progress_cb=None,
        log_cb=None,
        frame_cb=None,
        allow_interactive=True,
        camera_type=None,
        camera_kwargs=None,
    ):
        self.state = "INIT"
        self.backend = create_camera_backend(camera_type, **(camera_kwargs or {}))
        self.vision = PuzzleVision(self.backend)
        self.arm = arm if arm is not None else ArmController(SERVO_PORT, SERVO_BAUDRATE)
        self._own_arm = arm is None
        self.planner = MotionPlanner(self.arm, abort_event=abort_event)
        self.abort_event = abort_event
        self.progress_cb = progress_cb
        self.log_cb = log_cb
        self.frame_cb = frame_cb
        self.allow_interactive = allow_interactive
        self.start_time = 0
        self.tasks = []
        self.result_message = ""
        self.detected_count = 0
        self.placed_count = 0
        self.current_piece = 0
        # 缝合诊断信息 (GUI 日志/结果页可用)
        self.solve_meta = {}

    def _log(self, message, level="INFO"):
        print(message)
        if self.log_cb is not None:
            self.log_cb(level, message)

    def _progress(self, stage, fraction, detail="-"):
        if self.progress_cb is not None:
            self.progress_cb(
                stage,
                max(0.0, min(1.0, fraction)),
                detail,
                self.detected_count,
                self.placed_count,
                self.current_piece,
            )

    def _aborted(self):
        return self.abort_event is not None and self.abort_event.is_set()

    def _fail(self, message):
        self.state = "ERROR"
        self.result_message = message
        self._log(f"[Main] {message}", "ERROR")
        self._progress("ERROR", 0.0, message)
        return False

    # ==================== 状态机 ====================

    def run(self, task_mode="task1"):
        """
        主流程
        task_mode: 'task1' (自备碎片, 目标矩形 10×6) 或 'task2' (现场碎片)
        """
        self._log("=" * 50)
        self._log("  拼图装置启动")
        self._log(f"  模式: {task_mode}")
        self._log("=" * 50)

        self.start_time = time.time()
        self.detected_count = 0
        self.placed_count = 0
        self.current_piece = 0
        self.solve_meta = {}

        if self._aborted():
            return self._fail("任务启动前已收到中止信号")

        # INIT: 初始化
        if not self._init():
            return self._fail("机械臂初始化失败")

        # CALIBRATE: 视觉标定
        if not self._calibrate():
            return self._fail("视觉标定不可用，请先在GUI完成系统标定")

        # DETECT: 碎片检测 (123 流水线)
        detection = self._detect(task_mode)
        if detection is None:
            return self._fail("未检测到可用碎片")

        # SOLVE: 拼图求解 (AssemblySolver)
        self.tasks = self._solve(task_mode, detection)
        if not self.tasks:
            return self._fail(self.result_message or "拼图求解失败")

        # EXECUTE: 执行拾取放置
        if not self._execute():
            return False

        # DONE: 完成
        elapsed = time.time() - self.start_time
        if elapsed > 120:
            return self._fail(f"超过120秒限制（已用{elapsed:.1f}秒）")
        self._signal_done()

        self.result_message = f"拼图完成，用时{elapsed:.1f}秒"
        self._log(f"\n{'=' * 50}")
        self._log(f"  拼图完成! 用时: {elapsed:.1f}秒")
        self._log(f"{'=' * 50}")
        self._progress("DONE", 1.0, self.result_message)
        return True

    # ==================== 各状态实现 ====================

    def _init(self):
        """初始化硬件"""
        self.state = "INIT"
        self._log("\n[1/6] 初始化...")
        self._progress("INIT", 0.05, "机械臂回位")
        if self._aborted():
            self.result_message = "任务被中止"
            return False

        # 机械臂回零
        self.arm.go_home()
        self.arm.wait_for_stop()

        # 确保电磁铁断电释放 (5舵机+电磁铁方案: 无夹爪)
        self.arm.magnet_off()
        time.sleep(0.5)

        self._log("  初始化完成")
        self._progress("INIT", 0.10, "初始化完成")
        return not self._aborted()

    def _calibrate(self):
        """视觉标定"""
        self.state = "CALIBRATE"
        self._log("\n[2/6] 视觉标定...")
        self._progress("CALIBRATING", 0.15, "加载工作区标定")
        if self._aborted():
            self.result_message = "任务被中止"
            return False

        # 尝试加载已有标定
        if self.vision.load_calibration():
            self._log("  使用已保存的标定")
            self._progress("CALIBRATING", 0.20, "标定加载完成")
            return True

        if not self.allow_interactive:
            self.result_message = "未找到有效视觉标定，比赛模式禁止交互标定"
            self._log(f"  {self.result_message}", "ERROR")
            return False

        # 命令行模式保留交互式标定
        self._log("  请进行手动标定...")
        if not self.vision.start_interactive_calibration():
            self.result_message = "标定失败"
            self._log("  标定失败!", "ERROR")
            return False

        self._log("  标定完成")
        self._progress("CALIBRATING", 0.20, "标定完成")
        return True

    # ---- 缝合: DETECT (123 流水线) ----

    def _detect(self, task_mode):
        """
        碎片检测 (123 流水线):
        固定单应四角 + detect_pieces_arbitrary 多帧投票,
        再对投票帧做 flat_th 扫描与清晰轮廓模式, 生成多组检测候选。

        返回 (frame, quad, candidates) 或 None:
            candidates: [(pieces, label), ...], label 用于日志
        """
        self.state = "DETECT"
        self._log("\n[3/6] 碎片检测...")
        self._progress("DETECTING", 0.25, "采集图像")

        # 固定 A4 四角 (由 ChArUco 标定单应反推, 不再每帧检测纸边缘)
        if self.vision.homography is None:
            self.result_message = "无标定单应，无法检测"
            self._log(f"  {self.result_message}", "ERROR")
            return None
        quad = self.vision.quad_from_homography()
        if quad is None:
            self.result_message = "标定单应异常，无法反推A4面板"
            self._log(f"  {self.result_message}", "ERROR")
            return None

        # 1) 多帧投票: 碎片数众数为准, 取众数帧中的中间帧
        results = []
        for attempt in range(DETECT_VOTE_FRAMES):
            if self._aborted():
                self.result_message = "任务被中止"
                return None
            frame = self.vision.capture_frame()
            if frame is None:
                continue
            if self.frame_cb is not None:
                self.frame_cb(frame)
            frame = self.vision.preprocess(frame)
            # 暗光场景: 非局部均值去噪 (FPN噪声是检测精度杀手)
            if frame.mean() < 100:
                frame = cv2_fastNlMeans(frame)
            pieces = self.vision.detect_pieces_arbitrary(frame, quad)
            results.append((frame, pieces))
            if len(pieces) > self.detected_count:
                self.detected_count = len(pieces)
            self._progress(
                "DETECTING",
                0.25 + (attempt + 1) * 0.03,
                f"检测帧 {attempt + 1}/{DETECT_VOTE_FRAMES}，当前{self.detected_count}片",
            )
            time.sleep(0.1)

        if not results:
            self.result_message = "相机取帧失败"
            return None
        counts = [len(r[1]) for r in results]
        best_count = max(set(counts), key=counts.count)
        voted = [r for r in results if len(r[1]) == best_count]
        frame, voted_pieces = voted[len(voted) // 2]
        if len(set(counts)) > 1:
            self._log(f"  各帧碎片数 {counts}, 采用 {best_count}")
        if not voted_pieces:
            self.result_message = "未检测到碎片 (上半区无目标)"
            self._log(f"  {self.result_message}", "ERROR")
            return None
        self.detected_count = len(voted_pieces)
        self._log(f"  检测到 {self.detected_count} 片碎片 (投票帧):")
        for p in voted_pieces:
            self._log(
                f"    #{p['id']}: {len(p['vertices'])}顶点 "
                f"面积={p['area_mm2']:.0f}mm² "
                f"中心=({p['centroid'][0]:.0f},{p['centroid'][1]:.0f})"
            )

        # 2) 候选集: flat_th 扫描 + 清晰轮廓模式 (尖角)
        candidates = [(voted_pieces, f"投票帧 flat_th=0.82")]
        prev_sig = [p['vertices'].tobytes() for p in voted_pieces]
        for fth in FLAT_TH_SWEEP:
            pieces_f = self.vision.detect_pieces_arbitrary(frame, quad, flat_th=fth)
            sig = [p['vertices'].tobytes() for p in pieces_f]
            if sig == prev_sig:
                continue
            prev_sig = sig
            candidates.append((pieces_f, f"flat_th={fth}"))
        # 清晰轮廓模式: 黑底扑克牌碎片的尖角/短边可能被默认平滑/精化拟合坏,
        # 保留原始轮廓再解一次
        frame_sharp = self.vision.capture_frame()
        if frame_sharp is not None:
            pieces_sharp = self.vision.detect_pieces_arbitrary(
                frame_sharp, quad, flat_th=0.82, smooth_mask=False, refine_edges=False
            )
            if len(pieces_sharp) >= 2:
                candidates.append((pieces_sharp, "清晰轮廓模式"))

        self._progress("DETECTING", 0.40, f"生成 {len(candidates)} 组检测候选")
        return frame, quad, candidates

    # ---- 缝合: SOLVE (AssemblySolver) ----

    def _solve(self, task_mode, detection):
        """
        拼图求解 (123 流水线):
        对每组检测候选运行 AssemblySolver (T型接缝/位姿松弛/近似解兜底),
        取评分最优; 全部无解时进入宽松模式兜底。
        task1 (自备碎片): 反推矩形必须≈目标 10×6cm, 否则拒绝该候选。
        输出 puzzle_solver 兼容任务格式 (pick_pos/pick_angle/place_pos/place_angle)。
        """
        self.state = "SOLVE"
        self._log("\n[4/6] 拼图求解...")
        self._progress("SOLVING", 0.45, "求解拼图布局")
        if self._aborted():
            self.result_message = "任务被中止"
            return []

        frame, quad, candidates = detection
        best = None  # (score, solution, pieces, label)
        for pieces, label in candidates:
            if self._aborted():
                self.result_message = "任务被中止"
                return []
            if len(pieces) < 1 or len(pieces) > 4:
                continue
            sol = AssemblySolver(verbose=False).solve(
                [p['vertices'] for p in pieces]
            )
            if sol is None:
                self._log(f"  [{label}] 无解")
                continue
            # task1: 目标矩形固定为自备 10×6cm
            if task_mode == "task1" and not self._matches_task1_rect(sol):
                w, h = sol['rect_size']
                self._log(
                    f"  [{label}] 反推 {w:.1f}x{h:.1f}mm 不符合自备目标矩形"
                    f" {TASK1_TARGET_RECT_MM[0]:.0f}x{TASK1_TARGET_RECT_MM[1]:.0f}mm"
                    f"(容差{TASK1_RECT_TOL_MM:.0f}mm)，跳过"
                )
                continue
            score = sol.get('score', 999)
            if best is None or score < best[0]:
                best = (score, sol, pieces, label)
                self._log(
                    f"  [{label}] 可解 {sol['rect_size'][0]:.1f}x"
                    f"{sol['rect_size'][1]:.1f}mm 评分{score:.2f}"
                    f"{' (近似)' if sol.get('approximate') else ''}"
                )

        # 宽松模式兜底 (暗光检测误差大: 边容差8mm, 面积余量30%)
        if best is None:
            self._log("  常规参数无解, 进入宽松模式 (边容差8mm/面积余量30%)")
            import rect_assembly as _ra

            saved = (_ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG)
            _ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG = (
                0.12, 12.0, 15.0
            )
            try:
                for pieces, label in candidates:
                    if self._aborted():
                        self.result_message = "任务被中止"
                        return []
                    if len(pieces) < 1 or len(pieces) > 4:
                        continue
                    sv = AssemblySolver(edge_tol=8.0, verbose=False)
                    sv.fit_margin = 0.30
                    sol = sv.solve([p['vertices'] for p in pieces])
                    if sol is None:
                        continue
                    if task_mode == "task1" and not self._matches_task1_rect(sol):
                        continue
                    score = sol.get('score', 999)
                    if best is None or score < best[0]:
                        best = (score, sol, pieces, label)
                        self._log(
                            f"  [宽松/{label}] 可解 {sol['rect_size'][0]:.1f}x"
                            f"{sol['rect_size'][1]:.1f}mm 评分{score:.2f}"
                            f"{' (近似)' if sol.get('approximate') else ''}"
                        )
            finally:
                (_ra.RECT_AREA_TOL, _ra.RECT_SIDE_TOL_MM, _ra.RECT_ANGLE_TOL_DEG) = saved

        if best is None:
            self.result_message = "拼图求解失败 (检测/几何无法闭合为矩形)"
            self._log(f"  {self.result_message}", "ERROR")
            # 失败诊断图: 便于判断粘连/漏检/标定问题
            try:
                import os
                from solve_paths import OUT_DIR

                os.makedirs(OUT_DIR, exist_ok=True)
                diag = draw_failure_diagnostic(
                    frame,
                    quad,
                    candidates[0][0] if candidates else [],
                    self.result_message,
                    np.linalg.inv(self.vision.homography),
                )
                diag_path = os.path.join(OUT_DIR, "diagnostic.jpg")
                import cv2

                cv2.imwrite(diag_path, diag)
                self._log(f"  诊断图已保存: {diag_path}")
            except Exception as exc:  # 诊断图失败不影响主流程
                self._log(f"  诊断图保存失败: {exc}", "WARN")
            self._progress("SOLVING", 0.55, "求解失败")
            return []

        score, solution, pieces, label = best
        w, h = solution['rect_size']
        approximate = bool(solution.get('approximate', False))
        self.solve_meta = {
            "rect_mm": [float(w), float(h)],
            "score": float(score),
            "approximate": approximate,
            "label": label,
            "pieces": len(pieces),
        }
        self._log(
            f"  目标矩形 {w:.1f} x {h:.1f} mm"
            f"{' [近似解]' if approximate else ''} (来自 {label})"
        )

        # 放置到下半区 + 生成任务
        T_place = compute_placement(solution)
        raw_tasks = build_tasks(pieces, solution, T_place)
        raw_tasks = optimize_order(raw_tasks)

        # 转换为 puzzle_solver 兼容格式 (pick_pos/pick_angle/place_pos/place_angle)
        tasks = []
        for t in raw_tasks:
            tasks.append(
                {
                    "pick_pos": (float(t["pick"]["x"]), float(t["pick"]["y"])),
                    "pick_angle": float(t["_pick_angle"]),
                    "place_pos": (float(t["place"]["x"]), float(t["place"]["y"])),
                    "place_angle": float(t["rotation_deg"]),
                    "id": t["id"],
                    "area_mm2": t["area_mm2"],
                    "approximate": approximate,
                    "rect_mm": [float(w), float(h)],
                }
            )

        self._log(f"  生成 {len(tasks)} 个搬运任务 (已按路径优化排序):")
        for i, t in enumerate(tasks):
            self._log(
                f"    任务{i}: ({t['pick_pos'][0]:.0f},{t['pick_pos'][1]:.0f}) "
                f"{t['pick_angle']:.0f}° -> "
                f"({t['place_pos'][0]:.0f},{t['place_pos'][1]:.0f}) "
                f"{t['place_angle']:.0f}°"
            )
        self._progress("SOLVING", 0.55, f"生成{len(tasks)}个搬运任务")
        return tasks

    @staticmethod
    def _matches_task1_rect(solution):
        """自备碎片目标矩形 10×6cm 校验 (容差 TASK1_RECT_TOL_MM)。"""
        w, h = solution["rect_size"]
        tw, th = TASK1_TARGET_RECT_MM
        return abs(w - tw) <= TASK1_RECT_TOL_MM and abs(h - th) <= TASK1_RECT_TOL_MM

    def _execute(self):
        """执行拾取-放置"""
        self.state = "EXECUTE"
        self._log("\n[5/6] 执行拼图...")
        failed = 0

        for i, task in enumerate(self.tasks):
            if self._aborted():
                return self._fail("任务被中止")
            self.current_piece = i + 1
            self._log(f"\n--- 任务 {i + 1}/{len(self.tasks)} ---")
            elapsed = time.time() - self.start_time
            self._log(f"  已用时: {elapsed:.1f}秒")
            self._progress(
                "EXECUTING",
                0.60 + 0.35 * i / max(1, len(self.tasks)),
                f"搬运碎片 {i + 1}/{len(self.tasks)}",
            )

            # 检查时间是否超限
            if elapsed > 110:  # 2分钟=120秒, 留10秒余量
                return self._fail("时间不足，无法完成剩余搬运任务")

            success = self.planner.pick_and_place(
                task["pick_pos"],
                task["pick_angle"],
                task["place_pos"],
                task["place_angle"],
            )

            if success:
                self.placed_count += 1
            else:
                failed += 1
                self._log(f"  任务{i + 1}失败, 继续下一个", "WARN")
            self._progress(
                "EXECUTING",
                0.60 + 0.35 * (i + 1) / max(1, len(self.tasks)),
                f"已放置 {self.placed_count}/{len(self.tasks)}",
            )

        if failed:
            return self._fail(f"{failed}个碎片搬运失败")
        return True

    def _signal_done(self):
        """完成指示: 声光信号 (gpiozero, Pi 5 兼容)"""
        self.state = "DONE"
        self._log("\n[6/6] 拼图完成信号...")
        self._progress("DONE", 0.98, "声光提示")

        try:
            gpiozero = importlib.import_module("gpiozero")
            led = gpiozero.LED(LED_PIN)
            buzzer = gpiozero.Buzzer(BUZZER_PIN)

            # LED闪烁+蜂鸣器响3次
            for _ in range(3):
                led.on()
                buzzer.on()
                time.sleep(0.2)
                led.off()
                buzzer.off()
                time.sleep(0.2)

            led.close()
            buzzer.close()
        except Exception as exc:
            # 非树莓派环境 / gpiozero 不可用
            self._log(f"  [Sim] LED闪烁 + 蜂鸣器响3次 ({exc})", "WARN")

    # ==================== 清理 ====================

    def cleanup(self):
        """释放资源"""
        self.vision.release()
        release_camera_backend()
        if self._own_arm:
            self.arm.close()
        self._log("\n[Main] 资源已释放")


def cv2_fastNlMeans(frame):
    """暗光去噪 (FPN噪声, 0.4s可接受)"""
    import cv2

    return cv2.fastNlMeansDenoisingColored(frame, None, 15, 15, 7, 21)


# ==================== 入口 ====================

if __name__ == "__main__":
    # 命令行参数: python main.py [task1|task2]
    mode = sys.argv[1] if len(sys.argv) > 1 else "task1"

    robot = PuzzleRobot()

    try:
        success = robot.run(task_mode=mode)
        if not success:
            print(f"\n[Main] 拼图未完成: {robot.result_message}")
    except KeyboardInterrupt:
        print("\n[Main] 用户中断")
    except Exception as e:
        print(f"\n[Main] 异常: {e}")
        import traceback

        traceback.print_exc()
    finally:
        robot.cleanup()
