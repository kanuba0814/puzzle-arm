"""123 识别+求解算法到 workspace 契约的适配器。

将 Pi ~/puzzle (123项目) 的 vision.py + rect_assembly.py 核心算法
适配到 workspace/src 的 observe_pieces + solve_puzzle 接口，保持数据契约。

自动标定加载：
- 优先使用 workspace/calibration/ 下的正式标定文件（如有）；
- 回退到 pi_puzzle/homography.npy（Pi 现场标定）；
- 都没有则返回标定缺失错误。
"""

import hashlib
import time
from pathlib import Path

import cv2
import numpy as np

from vision._legacy_123_vision import PuzzleVision
from vision.generic_piece_observer import (
    MeasuredContour,
    PieceObservation,
    RectifiedFrame,
    VisionDiagnostics,
    VisionProfile,
    VisionResult,
)
from puzzle._legacy_123_assembly import AssemblySolver
from puzzle.solver import (
    PlacementPlan,
    Pose2,
    PuzzleOptions,
    PuzzleSolveResult,
    PuzzleSolveStatus,
    TargetRegion,
    WorkspaceState,
)


def _find_homography_file() -> Path | None:
    """查找可用的单应矩阵标定文件（workspace 优先，回退 legacy）。
    
    Pi 上路径：
    - /opt/epuzzle/workspace/calibration/*homography*.npy
    - /home/epuzzle/puzzle/homography.npy（legacy 123项目）
    
    宿主上路径：
    - workspace/calibration/*homography*.npy
    - pi_puzzle/homography.npy
    """
    workspace_root = Path(__file__).parent.parent.parent
    
    # 优先：workspace/calibration/ 下任何 homography 文件
    calib_dir = workspace_root / "calibration"
    if calib_dir.exists():
        matches = sorted(calib_dir.glob("*homography*.npy"))
        if matches:
            return matches[0]
    
    # 回退1：../pi_puzzle/homography.npy（宿主机镜像目录）
    pi_puzzle_path = workspace_root.parent / "pi_puzzle" / "homography.npy"
    if pi_puzzle_path.exists():
        return pi_puzzle_path
    
    # 回退2：/home/epuzzle/puzzle/homography.npy（Pi 上 legacy 123项目）
    legacy_pi = Path("/home/epuzzle/puzzle/homography.npy")
    if legacy_pi.exists():
        return legacy_pi
    
    return None


def observe_pieces_123(
    frame: RectifiedFrame, profile: VisionProfile
) -> VisionResult:
    """123 视觉检测适配器：detect_pieces_arbitrary → workspace VisionResult。

    输入：RectifiedFrame（已校正图 + px_per_mm + table_mm_size）
    输出：workspace 标准 VisionResult（含 PieceObservation 列表）
    """
    started_ns = time.monotonic_ns()

    # 加载单应矩阵
    homography_path = _find_homography_file()
    if homography_path is None:
        return VisionResult(
            observations=(),
            status="REJECTED",
            error_code="CALIBRATION_MISSING",
            profile=profile,
            profile_version=profile.profile_id,
            px_per_mm=frame.px_per_mm,
            source_image_sha256="",
            timestamp_ns=started_ns,
            processing_time_ms=0.0,
        )

    # 初始化 123 vision（无相机后端，帧由外部提供）
    vision = PuzzleVision(backend=None)
    vision.load_calibration(str(homography_path))

    # RectifiedFrame 已校正，构造伪 quad 覆盖整图，设置 mm 映射
    h_px, w_px = frame.image.shape[:2]
    table_w_mm, table_h_mm = frame.table_mm_size
    quad_px = np.array(
        [[0, h_px - 1], [w_px - 1, h_px - 1], [w_px - 1, 0], [0, 0]], dtype=np.float32
    )

    # 临时单应矩阵：px → table_mm（Y 轴翻转，原点左下）
    scale = frame.px_per_mm
    vision.homography = np.array(
        [[1 / scale, 0, 0], [0, -1 / scale, table_h_mm], [0, 0, 1]], dtype=np.float64
    )

    # 调用 123 检测
    bgr_frame = (
        frame.image
        if frame.image.ndim == 3
        else cv2.cvtColor(frame.image, cv2.COLOR_GRAY2BGR)
    )
    pieces_123 = vision.detect_pieces_arbitrary(
        frame=bgr_frame, quad=quad_px, flat_th=0.82
    )

    # 转换为 workspace PieceObservation
    observations = []
    image_hash = hashlib.sha256(frame.image.tobytes()).hexdigest()

    for p in pieces_123:
        vertices_mm = p["vertices"]
        contour_px = p.get("contour_px")

        if contour_px is not None:
            contour_points = tuple(
                (float(pt[0]), float(pt[1])) for pt in contour_px.reshape(-1, 2)
            )
        else:
            contour_points = tuple(
                (float(v[0] * scale), float((table_h_mm - v[1]) * scale))
                for v in vertices_mm
            )

        measured = MeasuredContour(frame="rectified_px", points=contour_points)
        polygon = tuple((float(v[0]), float(v[1])) for v in vertices_mm)
        grip_point = p.get("grip_point")
        pick_point = (
            (float(grip_point[0]), float(grip_point[1])) if grip_point else None
        )
        center = tuple(float(c) for c in p["centroid"])
        yaw = float(p.get("angle_deg", 0.0))
        equiv_yaws = (yaw,)

        diag = VisionDiagnostics(
            segmentation_stability=1.0,
            polygon_fit_residual_mm=0.5,
            pick_clearance_ok=None,
            rounded_corner_candidates=0,
            geometry_source="legacy_123_arbitrary",
            touches_frame_border=False,
        )

        obs = PieceObservation(
            piece_id=f"piece_{p['id']:02d}",
            timestamp_ns=started_ns,
            source_image_id=image_hash[:16],
            measured_contour=measured,
            polygon_table_mm=polygon,
            pick_point_table_mm=pick_point,
            pick_clearance_mm=None,
            center_table_mm=center,
            yaw_deg=yaw,
            equivalent_yaws_deg=equiv_yaws,
            confidence=0.85,
            diagnostics=diag,
        )
        observations.append(obs)

    elapsed_ms = (time.monotonic_ns() - started_ns) / 1_000_000.0
    status = "OK" if observations else "REJECTED"
    error_code = None if observations else "NO_PIECES_DETECTED"

    return VisionResult(
        observations=tuple(observations),
        status=status,
        error_code=error_code,
        profile=profile,
        profile_version=profile.profile_id,
        px_per_mm=frame.px_per_mm,
        source_image_sha256=image_hash,
        timestamp_ns=started_ns,
        processing_time_ms=elapsed_ms,
    )


def solve_puzzle_123(
    workspace: WorkspaceState,
    target_region: TargetRegion,
    options: PuzzleOptions,
) -> PuzzleSolveResult:
    """123 拼装求解适配器：AssemblySolver → workspace PuzzleSolveResult。"""
    started = time.monotonic_ns()
    pieces = tuple(sorted(workspace.pieces, key=lambda p: p.piece_id))

    if not 1 <= len(pieces) <= 4:
        return PuzzleSolveResult(
            status=PuzzleSolveStatus.NO_SOLUTION,
            error_code="PIECE_COUNT_OUT_OF_RANGE",
            inferred_target=None,
            placements=(),
            candidates_after_dedup=0,
            best_score=None,
            second_best_score=None,
            score_gap=None,
            score_breakdown={},
            texture_executed=False,
            solver_version="legacy_123_assembly",
            profile_version="",
            calibration_refs={},
            timing_ms={},
            expansions=0,
            pruned=0,
        )

    # 提取顶点列表
    pieces_verts = [np.array(p.polygon_table_mm, dtype=np.float64) for p in pieces]

    # 调用 123 求解器
    # fit_margin 补偿检测轮廓侵蚀；AssemblySolver 通过 getattr 读取，
    # 不是构造参数，所以实例化后再设置。
    solver = AssemblySolver(edge_tol=options.abs_tolerance_mm, verbose=False)
    solver.fit_margin = 0.15
    solution = solver.solve(pieces_verts)

    elapsed_ms = (time.monotonic_ns() - started) / 1_000_000.0

    if solution is None:
        return PuzzleSolveResult(
            status=PuzzleSolveStatus.NO_SOLUTION,
            error_code="ASSEMBLY_FAILED",
            inferred_target=None,
            placements=(),
            candidates_after_dedup=0,
            best_score=None,
            second_best_score=None,
            score_gap=None,
            score_breakdown={},
            texture_executed=False,
            solver_version="legacy_123_assembly",
            profile_version="",
            calibration_refs={},
            timing_ms={"total_ms": elapsed_ms},
            expansions=0,
            pruned=0,
        )

    # 提取拼接结果
    # 123 求解在“碎片 0 为锚点”的原始散落坐标系内完成：
    # placed_verts 已是拼好的多边形，rect_corners 是对应外接矩形。
    # 落位只需一个全局平移：矩形中心 → 目标区中心。
    placed_verts = solution["placed_verts"]
    rect_size = solution["rect_size"]
    rect_corners = solution.get("rect_corners")

    target_center = np.array(
        [
            (target_region.min_x_mm + target_region.max_x_mm) / 2.0,
            (target_region.min_y_mm + target_region.max_y_mm) / 2.0,
        ]
    )
    if rect_corners is not None:
        assembly_center = np.asarray(rect_corners, dtype=np.float64).mean(axis=0)
    else:
        assembly_center = np.vstack(placed_verts).mean(axis=0)
    offset = target_center - assembly_center

    # 构造 PlacementPlan（per-piece）
    # 参考点语义：机械臂磁头吸住的是视觉给出的握点 grip_point（最大内切圆
    # 圆心），不是多边形质心。旋转时碎片绕握点转，因此 PICK/PLACE 的坐标
    # 必须用握点的变换位置。若用质心（早期版本），大旋转片会因握点-质心
    # 偏心被放大成 10mm+ 落位偏差（piece_01 实测偏 12mm，偏心 29.7mm）。
    placements = []
    for index, piece in enumerate(pieces):
        rotation, _ = solution["transforms"][index]
        placed_centroid = np.asarray(placed_verts[index]).mean(axis=0)
        grip = getattr(piece, "grip_point_table_mm", None)
        if grip is not None:
            # 握点在碎片局部坐标 = 检测值 - 质心（检测时碎片未旋转）
            local_grip = np.asarray(grip) - np.asarray(piece.center_table_mm)
            # 拼接后握点 = 拼接质心 + R @ 局部握点
            target_grip = placed_centroid + rotation @ local_grip + offset
        else:
            target_grip = placed_centroid + offset
        target_center_mm = target_grip
        delta_yaw_deg = float(
            np.degrees(np.arctan2(rotation[1, 0], rotation[0, 0]))
        )
        # 关键修复：yaw 必须用有方向的顶点几何朝向（与求解器同一参考系），
        # 不能用 PCA 主轴角（惯性主轴，180° 模糊且与碎片几何边无关）。
        # 磁头吸住碎片后碎片对齐磁头 yaw，PICK yaw 必须等于碎片真实朝向，
        # 否则旋转量错误 → 放置角度偏差大（实机已验证）。
        # 朝向 = 质心→最远顶点方向（对噪声鲁棒、方向唯一）。
        verts_arr = np.asarray(piece.polygon_table_mm, dtype=np.float64)
        geom_centroid = verts_arr.mean(axis=0)
        farthest = max(
            verts_arr, key=lambda v: float(np.hypot(v[0] - geom_centroid[0], v[1] - geom_centroid[1]))
        )
        source_yaw = float(
            np.degrees(np.arctan2(
                farthest[1] - geom_centroid[1], farthest[0] - geom_centroid[0]
            ))
        )
        final_yaw = (source_yaw + delta_yaw_deg + 180.0) % 360.0 - 180.0

        source_pose = Pose2(
            frame="table_mm",
            center_mm=piece.grip_point_table_mm or piece.center_table_mm,
            yaw_deg=source_yaw,
        )
        target_pose = Pose2(
            frame="table_mm",
            center_mm=(float(target_center_mm[0]), float(target_center_mm[1])),
            yaw_deg=final_yaw,
        )

        plan = PlacementPlan(
            piece_id=piece.piece_id,
            order=index,
            source=source_pose,
            target=target_pose,
            geometry_score=float(solution.get("score", 0.0)),
            texture_score=None,
            texture_status="disabled",
            plan_confidence=0.6 if solution.get("approximate") else 0.9,
            source_timestamp_ns=piece.timestamp_ns,
            equivalent_yaws_deg=piece.equivalent_yaws_deg,
            solver_version="legacy_123_assembly",
        )
        placements.append(plan)

    score = solution.get("score", 0.0)
    status = (
        PuzzleSolveStatus.AMBIGUOUS
        if solution.get("approximate", False)
        else PuzzleSolveStatus.SOLVED
    )

    return PuzzleSolveResult(
        status=status,
        error_code=None,
        inferred_target=(float(rect_size[0]), float(rect_size[1])),
        placements=tuple(placements),
        candidates_after_dedup=1,
        best_score=score,
        second_best_score=None,
        score_gap=None,
        score_breakdown={"assembly": score},
        texture_executed=False,
        solver_version="legacy_123_assembly",
        profile_version="",
        calibration_refs={},
        timing_ms={"total_ms": elapsed_ms},
        expansions=0,
        pruned=0,
    )
