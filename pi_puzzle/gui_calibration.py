"""树莓派本地系统标定控制器：UVC工作区单应 + 本地舵机触点外参。"""

from __future__ import annotations

import contextlib
import json
import time
from collections.abc import Callable
from pathlib import Path

import cv2
import numpy as np

from calibrate_camera import (
    board_obj_to_world,
    detect,
    draw_a4_panel,
    draw_detection,
    obj_points,
)
from config import MAIXCAM_UVC_INDEX, MAIXCAM_UVC_SIZE, PROD_IMAGE_SIZE, SERVO_IDS
from kinematics import forward_kinematics, servo_to_deg

LogFn = Callable[[str, str], None]


class LocalCalibration:
    """供pygame GUI逐帧调用；不创建窗口、不使用网络。"""

    def __init__(self, arm, log: LogFn):
        self.arm = arm
        self.log = log
        self.camera: cv2.VideoCapture | None = None
        self.frame: np.ndarray | None = None
        self.homography: np.ndarray | None = None  # homography_size像素 -> 世界mm
        self.homography_size: tuple[int, int] | None = None
        self.target_px: tuple[float, float] | None = None
        self.pairs: list[tuple[float, float, float, float]] = []
        self.shots: list[dict] = []
        self.brightness: float | None = None
        self.sharpness: float | None = None
        self.corner_count = 0
        self.message = "打开本地相机后，将标定板平放在固定A4工作区"
        self._load_existing()

    def _load_existing(self):
        path = Path("homography.npy")
        if path.exists():
            try:
                self.homography = np.load(path)
                self.homography_size = (
                    int(PROD_IMAGE_SIZE[0]),
                    int(PROD_IMAGE_SIZE[1]),
                )
                self.message = "已加载现有工作区标定；可预览核对或重新标定"
            except (OSError, ValueError):
                self.homography = None

    @property
    def camera_open(self):
        return self.camera is not None and self.camera.isOpened()

    def open_camera(self):
        if self.camera_open:
            return True, "本地UVC相机已打开"
        width, height = MAIXCAM_UVC_SIZE
        cap = cv2.VideoCapture(MAIXCAM_UVC_INDEX, cv2.CAP_V4L2)
        fourcc_fn = getattr(cv2, "VideoWriter_fourcc", cv2.VideoWriter.fourcc)
        fourcc = fourcc_fn(*"MJPG")
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        if not cap.isOpened():
            cap.release()
            return False, f"无法打开 /dev/video{MAIXCAM_UVC_INDEX}"
        self.camera = cap
        for _ in range(5):
            cap.grab()
        self.log("INFO", f"本地UVC已打开: {width}x{height}，无网络传输")
        return True, "本地相机已打开"

    def close(self):
        if self.camera is not None:
            self.camera.release()
            self.camera = None

    def _current_homography(self):
        """将已存单应按实际UVC取帧尺寸缩放，兼容驱动降级分辨率。"""
        if self.homography is None or self.frame is None:
            return None
        frame_h, frame_w = self.frame.shape[:2]
        source_w, source_h = self.homography_size or (frame_w, frame_h)
        scale = np.diag([source_w / frame_w, source_h / frame_h, 1.0])
        return self.homography @ scale

    def update(self):
        """GUI每帧调用；返回带角点/A4/触点叠加的BGR画面。"""
        camera = self.camera
        if camera is None or not camera.isOpened():
            return self.frame
        ok, frame = camera.read()
        if not ok or frame is None:
            return self.frame
        self.frame = frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            self.brightness = float(gray.mean())
            self.sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        except (TypeError, ValueError, cv2.error):
            self.brightness = None
            self.sharpness = None
        vis = frame.copy()
        corners, ids = detect(frame)
        self.corner_count = 0 if ids is None else len(ids)
        draw_detection(vis, corners, ids)
        current_h = self._current_homography()
        if current_h is not None:
            with contextlib.suppress(np.linalg.LinAlgError, ValueError):
                draw_a4_panel(vis, np.linalg.inv(current_h), label=False)
        if self.target_px is not None:
            try:
                p = tuple(int(v) for v in self.target_px)
            except (TypeError, ValueError, OverflowError):
                p = None
            if p is not None:
                cv2.drawMarker(vis, p, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 36, 3)
        if current_h is not None:
            inv_h = np.linalg.inv(current_h)
            for i, pair in enumerate(self.pairs):
                world = np.array([[[pair[2], pair[3]]]], dtype=np.float32)
                px = cv2.perspectiveTransform(world, inv_h)[0, 0]
                p = tuple(px.astype(int))
                cv2.circle(vis, p, 10, (255, 0, 255), 2)
                cv2.putText(
                    vis,
                    f"P{i + 1}",
                    (p[0] + 10, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2,
                )
        return vis

    def calibrate_workspace(self):
        if self.frame is None:
            return False, "尚无相机画面"
        corners, ids = detect(self.frame)
        if corners is None or ids is None or len(ids) < 50:
            return False, f"角点不足（当前{0 if ids is None else len(ids)}，至少50）"
        px = corners.reshape(-1, 2).astype(np.float64)
        world = board_obj_to_world(obj_points(ids)[:, :2])
        h_uvc, _ = cv2.findHomography(px, world, method=0)
        if h_uvc is None:
            return False, "单应矩阵求解失败"
        projected = cv2.perspectiveTransform(
            px.reshape(-1, 1, 2).astype(np.float32), h_uvc.astype(np.float32)
        ).reshape(-1, 2)
        residual = np.linalg.norm(projected - world, axis=1)
        try:
            rms = float(np.sqrt(np.mean(residual**2)))
            max_err = float(np.max(residual))
        except (TypeError, ValueError) as exc:
            return False, f"残差计算失败: {exc}"
        if rms > 1.0 or max_err > 3.0:
            return (
                False,
                f"残差过大 RMS={rms:.2f}mm max={max_err:.2f}mm，请检查板是否平整",
            )

        frame_h, frame_w = self.frame.shape[:2]
        prod_w, prod_h = PROD_IMAGE_SIZE
        h_prod = h_uvc @ np.diag([frame_w / prod_w, frame_h / prod_h, 1.0])
        # UVC通常就是生产1280x720；1440版本只用于兼容旧工具。
        h_1440 = h_uvc @ np.diag([frame_w / 2560.0, frame_h / 1440.0, 1.0])
        np.save("homography.npy", h_prod)
        np.save("homography_1440.npy", h_1440)
        self.homography = h_uvc
        self.homography_size = (frame_w, frame_h)
        Path("out").mkdir(exist_ok=True)
        summary = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mode": "gui-local-single-plane",
            "camera": f"/dev/video{MAIXCAM_UVC_INDEX}",
            "image_size": [frame_w, frame_h],
            "corners": len(ids),
            "homography_residual_mm": {"rms": rms, "max": max_err},
            "world_frame": "origin=A4左下角, x右0-210, y上0-297",
        }
        Path("out/calib_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.message = f"工作区标定完成：RMS {rms:.3f}mm，最大 {max_err:.3f}mm"
        self.log("INFO", self.message)
        return True, self.message

    def set_target_from_view(self, view_pos, view_rect):
        current_h = self._current_homography()
        if self.frame is None or current_h is None:
            return False, "请先完成工作区标定"
        vx, vy = view_pos
        if not view_rect.collidepoint(vx, vy):
            return False, "点击位置不在相机预览内"
        frame_h, frame_w = self.frame.shape[:2]
        px = (vx - view_rect.x) * frame_w / view_rect.w
        py = (vy - view_rect.y) * frame_h / view_rect.h
        try:
            self.target_px = (float(px), float(py))
        except (TypeError, ValueError) as exc:
            return False, f"目标坐标无效: {exc}"
        world = cv2.perspectiveTransform(
            np.array([[[px, py]]], dtype=np.float32), current_h
        )[0, 0]
        self.message = f"目标：世界 ({world[0]:.1f}, {world[1]:.1f})mm；碰点后记录"
        return True, self.message

    def release_torque(self):
        if self.arm is None or self.arm.simulation:
            return False, "未连接本地机械臂硬件"
        self.arm.disable_torque()
        return True, "力矩已释放，请全程扶住机械臂"

    def record_touch(self):
        current_h = self._current_homography()
        if self.target_px is None or current_h is None:
            return False, "请先在预览中点击目标点"
        if self.arm is None or self.arm.simulation:
            return False, "未连接本地机械臂硬件"
        raw = self.arm.get_current_position()
        if any(sid not in raw for sid in SERVO_IDS):
            return False, "舵机位置读取不完整"
        try:
            deg = [float(servo_to_deg(raw[sid], sid)) for sid in SERVO_IDS]
            xyz = forward_kinematics(deg[:4])
        except (TypeError, ValueError, KeyError) as exc:
            return False, f"机械臂坐标计算失败: {exc}"
        px, py = self.target_px
        world = cv2.perspectiveTransform(
            np.array([[[px, py]]], dtype=np.float32), current_h
        )[0, 0]
        try:
            pair = (float(xyz[0]), float(xyz[1]), float(world[0]), float(world[1]))
        except (TypeError, ValueError, IndexError) as exc:
            return False, f"触点坐标无效: {exc}"
        self.pairs.append(pair)
        self.shots.append(
            {
                "world": world.astype(float).tolist(),
                "arm": xyz.astype(float).tolist(),
                "raw": raw,
                "deg": deg,
            }
        )
        self.target_px = None
        self.message = f"已记录 P{len(self.pairs)}；建议3–4个分散点"
        return True, self.message

    def undo_touch(self):
        if not self.pairs:
            return False, "没有可撤销的触点"
        self.pairs.pop()
        self.shots.pop()
        return True, f"已撤销，剩余 {len(self.pairs)} 点"

    def solve_base(self):
        if len(self.pairs) < 3:
            return False, "至少记录3个分散触点"
        arm = np.array([[p[0], p[1]] for p in self.pairs])
        world = np.array([[p[2], p[3]] for p in self.pairs])
        ac, wc = arm.mean(axis=0), world.mean(axis=0)
        a0, w0 = arm - ac, world - wc
        try:
            arm_spread = float(np.sum(a0**2))
            if arm_spread < 100.0:
                return False, "触点过于集中，请选择分散位置"
            z = np.sum((w0[:, 0] + 1j * w0[:, 1]) * np.conj(a0[:, 0] + 1j * a0[:, 1]))
            theta = float(np.angle(z))
            rot = np.array(
                [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]
            )
            translation = wc - rot @ ac
            prediction = (rot @ arm.T).T + translation
            residual = np.linalg.norm(prediction - world, axis=1)
            scale = float(np.sqrt(np.sum(w0**2) / arm_spread))
            max_err = float(np.max(residual))
            base_x = float(translation[0])
            base_y = float(translation[1])
            rot_deg = float(np.degrees(theta))
        except (TypeError, ValueError, IndexError, FloatingPointError) as exc:
            return False, f"基座解算失败: {exc}"
        if max_err > 8.0:
            return False, f"触点残差过大 max={max_err:.1f}mm，请撤销错误点重试"
        result = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "base_x": base_x,
            "base_y": base_y,
            "rot_deg": rot_deg,
            "residuals_mm": residual.astype(float).tolist(),
            "fk_scale": scale,
            "shots": self.shots,
        }
        Path("arm_base.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        Path("out").mkdir(exist_ok=True)
        Path("out/arm_base_world.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.message = (
            f"基座完成 X={base_x:.1f} Y={base_y:.1f} "
            f"R={rot_deg:+.2f}° max={max_err:.2f}mm"
        )
        self.log("INFO", self.message)
        return True, self.message
