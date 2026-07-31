"""
拼图求解诊断工具: 定位"碎片可拼但求解失败"的环节

用法:
    python3 tools/diag_assembly.py [image] [--tol 8]

诊断内容:
    1. 检测碎片多边形 (多档flat_th)
    2. 求解器拒绝原因统计 (重叠/超面积/矩形校验/死路)
    3. 每片生成多个简化变体, 组合求解, 报告最优解
"""
import os
import sys
import argparse
import itertools
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import rect_assembly as ra  # noqa: E402
from vision import PuzzleVision  # noqa: E402


def load_and_detect(img_path, flat_th=0.82):
    vis = PuzzleVision.__new__(PuzzleVision)
    vis.homography = None
    frame = cv2.imread(img_path)
    frame = vis.preprocess(frame)
    if frame.mean() < 100:
        frame = cv2.fastNlMeansDenoisingColored(frame, None, 15, 15, 7, 21)
    quad = vis.auto_detect_paper(frame)
    if quad is None:
        print("纸检测失败")
        sys.exit(1)
    vis.set_paper_homography(quad)
    pieces = vis.detect_pieces_arbitrary(frame, quad, flat_th=flat_th)
    return vis, frame, quad, pieces


def poly_variants(vis, contour_px, max_variants=4):
    """从一个轮廓生成多个简化变体 (不同顶点数)"""
    cnt_mm = vis.pixel_to_world(
        contour_px.reshape(-1, 2)).astype(np.float32).reshape(-1, 1, 2)
    peri = cv2.arcLength(cnt_mm, True)
    seen = {}
    for rel in (0.006, 0.008, 0.012, 0.016, 0.02, 0.026, 0.034, 0.045):
        ap = cv2.approxPolyDP(cnt_mm, rel * peri, True)
        v = PuzzleVision._cleanup_polygon(ap.reshape(-1, 2))
        if len(v) < 3:
            continue
        ref = PuzzleVision._refine_edges(cnt_mm.reshape(-1, 2), v)
        if ref is not None:
            v = ref
        # CCW
        if cv2.contourArea(v.astype(np.float32), oriented=True) < 0:
            v = v[::-1].copy()
        sig = (len(v), tuple(sorted(round(
            np.linalg.norm(v[(i + 1) % len(v)] - v[i]), 0)
            for i in range(len(v)))))
        if sig not in seen:
            seen[sig] = v
        if len(seen) >= max_variants:
            break
    return list(seen.values())


def edges_str(v):
    return [f'{np.linalg.norm(v[(i + 1) % len(v)] - v[i]):.1f}'
            for i in range(len(v))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('image', nargs='?', default='/tmp/now.jpg')
    ap.add_argument('--tol', type=float, default=8.0)
    ap.add_argument('--max-combo', type=int, default=256)
    args = ap.parse_args()

    vis, frame, quad, pieces = load_and_detect(args.image)
    print(f"检测到 {len(pieces)} 片:")
    for p in pieces:
        print(f"  #{p['id']}: {len(p['vertices'])}顶点 "
              f"边长={edges_str(p['vertices'])} 面积={p['area_mm2']:.0f}")

    # 1. 直接求解 (宽容差) + 拒绝统计
    ra.RECT_AREA_TOL = 0.12
    ra.RECT_SIDE_TOL_MM = 12.0
    ra.RECT_ANGLE_TOL_DEG = 15.0
    solver = ra.AssemblySolver(edge_tol=args.tol, verbose=False)
    sol = solver.solve([p['vertices'] for p in pieces])
    print(f"\n[诊断] 直接求解(tol={args.tol}): "
          f"{'成功' if sol else '失败'} 状态={solver._states} "
          f"拒绝统计={solver.stats}")

    # 2. 变体组合求解
    variants = [poly_variants(vis, p['contour_px']) for p in pieces]
    for i, vs in enumerate(variants):
        print(f"  片{i} 变体{len(vs)}个: "
              + ' | '.join(f"{len(v)}点{edges_str(v)}" for v in vs))

    best = []
    n_combo = 0
    for combo in itertools.product(*variants):
        n_combo += 1
        if n_combo > args.max_combo:
            break
        sv = ra.AssemblySolver(edge_tol=args.tol, verbose=False)
        s = sv.solve(list(combo))
        if s is not None:
            best.append((s.get('score', 999), s, combo, sv.stats))
    best.sort(key=lambda b: b[0])
    print(f"\n[诊断] 组合求解: {n_combo}组, 可解 {len(best)} 组")
    for score, s, combo, stats in best[:5]:
        W, H = s['rect_size']
        desc = ' | '.join(f"{len(v)}点{edges_str(v)}" for v in combo)
        print(f"  评分{score:.2f} 矩形{W:.1f}x{H:.1f} 状态={stats}")
        print(f"    {desc}")

    if best:
        score, s, combo, _ = best[0]
        img = np.ones((900, 900, 3), np.uint8) * 245
        all_v = np.vstack(s['placed_verts'])
        lo = all_v.min(axis=0) - 5
        sc = 900 / (all_v.max(axis=0) - lo + 5).max()
        for i, pv in enumerate(s['placed_verts']):
            pts = ((pv - lo) * sc).astype(np.int32)
            color = [(0, 0, 255), (0, 180, 0), (255, 0, 0),
                     (200, 0, 200)][i % 4]
            cv2.polylines(img, [pts], True, color, 2)
        rc = ((s['rect_corners'] - lo) * sc).astype(np.int32)
        cv2.polylines(img, [rc], True, (0, 0, 0), 1)
        cv2.imwrite('diag_best.jpg', img)
        print("最优解已保存: diag_best.jpg")


if __name__ == '__main__':
    main()
