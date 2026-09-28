#!/usr/bin/env python3
"""完整拼图实物操作（压缩包视觉算法驱动）。

流程：
1. 压缩包流水线：多帧投票检测（detect_with_voting）+ flat_th 扫描 +
   AssemblySolver 求解 + 清晰轮廓模式
2. 输出与 workspace run_assembly_4pieces 兼容的 JSON 计划
3. 调用 run_assembly_4pieces.py 执行四片拼装

用法（Pi 上 /home/epuzzle/puzzle）：
    sudo /opt/epuzzle/venv/bin/python run_visionpack_assembly.py [--frames 3] [--dry-run]
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

# 只用压缩包模块（puzzle 目录），不混入 workspace（同名 vision 会冲突）
sys.path.insert(0, "/home/epuzzle/puzzle")
from rect_assembly import AssemblySolver  # noqa: E402
from solve_paths import (  # noqa: E402
    build_tasks,
    compute_placement,
    detect_with_voting,
    fix_paper_orientation,
    optimize_order,
)
from vision import PuzzleVision  # noqa: E402

OUT_DIR = "/home/epuzzle/visionpack_out"
FLAT_TH_SWEEP = (0.82, 0.78, 0.86, 0.74, 0.90)
MAX_PIECES = 4


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", default=None, help="离线图片路径（detect_with_voting 兼容）")
    parser.add_argument("--frames", type=int, default=3, help="多帧投票帧数")
    parser.add_argument("--homography", default="/home/epuzzle/puzzle/homography.npy")
    parser.add_argument("--dry-run", action="store_true", help="只检测求解不执行")
    parser.add_argument("--expected-pieces", type=int, default=4)
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    vision = PuzzleVision()

    # 标定单应固定 A4 面板
    fixed_quad = None
    if vision.load_calibration(args.homography):
        fixed_quad = vision.quad_from_homography()
        print(f"[VisionPack] 使用标定单应 {args.homography}")
    else:
        print("[VisionPack] [警告] 标定加载失败，回退纸边缘检测")
        sys.exit(2)

    # 1. 多帧投票检测（压缩包流水线）
    print(f"[VisionPack] 多帧投票检测（{args.frames} 帧）...")
    frame, quad, pieces = detect_with_voting(vision, args, fixed_quad)
    if frame is None or quad is None:
        print("[VisionPack] A4纸检测失败")
        vision.release()
        return 1
    if not pieces:
        print("[VisionPack] 未检测到碎片")
        vision.release()
        return 1
    pieces0 = pieces
    print(f"[VisionPack] 检测到 {len(pieces0)} 片")
    for p in pieces0:
        c = p["centroid"]
        print(
            f"  #{p['id']}: {len(p['vertices'])}顶点 {p['area_mm2']:.0f}mm² "
            f"中心=({c[0]:.0f},{c[1]:.0f}) 握点={tuple(round(v,1) for v in p['grip_point'])}"
        )

    if len(pieces0) > MAX_PIECES:
        print(f"[VisionPack] 警告: {len(pieces0)} 片超过上限 {MAX_PIECES}")

    # 2. 拼图求解：flat_th 扫描取最优（压缩包流水线）
    solution = None
    pieces_sel = None
    best_score = float("inf")
    n_target = args.expected_pieces or len(pieces0)
    prev_sig = None
    for fth in FLAT_TH_SWEEP:
        pieces_f = vision.detect_pieces_arbitrary(frame, quad, flat_th=fth)
        sig = [p["vertices"].tobytes() for p in pieces_f]
        if sig == prev_sig:
            continue
        prev_sig = sig
        if len(pieces_f) != n_target:
            continue
        if fixed_quad is None:
            pieces_f = fix_paper_orientation(vision, quad, pieces_f)
        sol = AssemblySolver(verbose=False).solve([p["vertices"] for p in pieces_f])
        if sol is not None and sol.get("score", 999) < best_score:
            best_score = sol.get("score", 999)
            solution, pieces_sel = sol, pieces_f
            print(
                f"[VisionPack] flat_th={fth}: 可解 "
                f"{sol['rect_size'][0]:.1f}x{sol['rect_size'][1]:.1f}mm "
                f"评分{best_score:.2f}"
            )
        else:
            print(f"[VisionPack] flat_th={fth}: 无解")

    # 3. 清晰轮廓模式兜底（压缩包流水线）
    if solution is None or solution.get("approximate"):
        print("[VisionPack] 尝试清晰轮廓模式（保留尖角）...")
        frame_sharp = vision.capture_frame()
        if frame_sharp is not None:
            frame_sharp = vision.preprocess(frame_sharp)
            pieces_sharp = vision.detect_pieces_arbitrary(
                frame_sharp, quad, flat_th=0.82, smooth_mask=False, refine_edges=False
            )
            if len(pieces_sharp) == n_target:
                sol_sharp = AssemblySolver(verbose=False).solve(
                    [p["vertices"] for p in pieces_sharp]
                )
                if sol_sharp is not None and (
                    solution is None or sol_sharp.get("score", 999) < best_score
                ):
                    solution, pieces_sel = sol_sharp, pieces_sharp
                    print(
                        f"[VisionPack] 清晰轮廓模式可解 "
                        f"{sol_sharp['rect_size'][0]:.1f}x{sol_sharp['rect_size'][1]:.1f}mm"
                    )

    if solution is None:
        print("[VisionPack] 求解失败：无可行拼法")
        vision.release()
        return 1

    # 4. 生成放置计划（压缩包 compute_placement + build_tasks）
    rect_size = solution["rect_size"]
    print(f"[VisionPack] 求解成功: {rect_size[0]:.1f}x{rect_size[1]:.1f}mm "
          f"评分{solution.get('score', 0):.2f} approx={solution.get('approximate', False)}")

    # 目标区：下半区中心 (105, 74.25)
    target_center = (105.0, 74.25)
    rect_corners = solution["rect_corners"]
    import numpy as np
    assembly_center = np.asarray(rect_corners).mean(axis=0)
    offset = np.array(target_center) - assembly_center
    T_place = ((np.eye(2), offset))  # (R, t) 平移

    tasks = build_tasks(pieces_sel, solution, T_place)
    ordered = optimize_order(tasks, start=(105.0, 74.25))

    # 5. 转成 run_assembly_4pieces 兼容 JSON
    placements = []
    for task in ordered:
        pid = f"piece_{task['id']:02d}"
        pick = task["pick"]
        place = task["place"]
        # 源 yaw：视觉检测角度（握点方向由 detect 给出）
        piece = next(p for p in pieces_sel if p["id"] == task["id"])
        placements.append(
            {
                "piece_id": pid,
                "source": {
                    "frame": "table_mm",
                    "center_mm": [pick["x"], pick["y"]],
                    "yaw_deg": float(piece["angle_deg"]),
                },
                "target": {
                    "frame": "table_mm",
                    "center_mm": [place["x"], place["y"]],
                    "yaw_deg": float(piece["angle_deg"] + task["rotation_deg"]),
                },
            }
        )

    plan = {
        "solve": {
            "status": "SOLVED",
            "error_code": None,
            "inferred_target_mm": [float(rect_size[0]), float(rect_size[1])],
            "placements": placements,
        },
        "vision_pack": True,
    }
    out_path = Path(OUT_DIR) / "plan_visionpack.json"
    out_path.write_text(json.dumps(plan, ensure_ascii=False, indent=2))
    print(f"\n[VisionPack] 计划已生成: {out_path}")
    print(f"  矩形 {rect_size[0]:.1f}x{rect_size[1]:.1f}mm → 下半区中心 {target_center}")
    for pl in placements:
        s = pl["source"]["center_mm"]
        t = pl["target"]["center_mm"]
        dy = pl["target"]["yaw_deg"] - pl["source"]["yaw_deg"]
        print(f"  {pl['piece_id']}: ({s[0]:.1f},{s[1]:.1f}) → ({t[0]:.1f},{t[1]:.1f}) Δyaw={dy:+.1f}°")

    vision.release()

    if args.dry_run:
        print("\n--dry-run：未执行运动")
        return 0

    # 6. 调用现有运动执行器（sudo 会清环境，PYTHONPATH 必须内嵌命令）
    print("\n[VisionPack] 调用 run_assembly_4pieces.py 执行...")
    plugin = "/opt/epuzzle/workspace/src/robot_plugin/lerobot_robot_so101_5d"
    ws_src = "/opt/epuzzle/workspace/src"
    cmd = [
        "sudo",
        f"PYTHONPATH={plugin}:{ws_src}",
        "/opt/epuzzle/venv/bin/python",
        "/opt/epuzzle/workspace/tools/run_assembly_4pieces.py",
        "--plan", str(out_path),
    ]
    subprocess.run(cmd, cwd="/opt/epuzzle")
    return 0


if __name__ == "__main__":
    main()
