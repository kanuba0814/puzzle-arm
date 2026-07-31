"""
合成场景生成器: 验证 solve_paths 全链路 (不依赖实拍光照)

生成一张虚拟相机图像:
  深色桌面 + 白色A4纸(透视四边形) + 竞赛风格碎片(矩形切割而成)
  叠加高斯噪声, 保存为 jpg, 再用 solve_paths.py --image 跑全流程

用法:
    python3 tools/make_synthetic_scene.py            # 默认种子
    python3 tools/make_synthetic_scene.py --seed 3 --noise 6 --dark
"""
import os
import sys
import argparse
import numpy as np
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rect_assembly import _gen_guillotine_pieces  # noqa: E402


def scatter_pieces(pieces, rng, x_range, y_range, min_gap_mm=5.0):
    """把碎片随机旋转平移到指定区域。
    光栅占用图保证互不重叠且间距>=min_gap (竞赛: 碎片互不重叠)"""
    scale = 2  # px/mm 占用图分辨率
    W = int((x_range[1] - x_range[0]) * scale) + 1
    H = int((y_range[1] - y_range[0]) * scale) + 1
    occ = np.zeros((H, W), np.uint8)
    dil = np.ones((int(min_gap_mm * scale) * 2 + 1,) * 2, np.uint8)
    out = []
    for poly in pieces:
        for _ in range(2000):
            ang = rng.uniform(0, 2 * np.pi)
            c, s = np.cos(ang), np.sin(ang)
            R = np.array([[c, -s], [s, c]])
            p = poly @ R.T
            lo = p.min(axis=0)
            hi = p.max(axis=0)
            if hi[0] - lo[0] > x_range[1] - x_range[0] or \
               hi[1] - lo[1] > y_range[1] - y_range[0]:
                continue
            tx = rng.uniform(x_range[0] - lo[0], x_range[1] - hi[0])
            ty = rng.uniform(y_range[0] - lo[1], y_range[1] - hi[1])
            q = p + np.array([tx, ty])
            m = np.zeros((H, W), np.uint8)
            pts = ((q - [x_range[0], y_range[0]]) * scale).astype(np.int32)
            cv2.fillPoly(m, [pts], 255)
            if cv2.bitwise_and(m, occ).any():
                continue
            occ |= cv2.dilate(m, dil)
            out.append(q)
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', default='synth_scene.jpg')
    ap.add_argument('--noise', type=float, default=4.0, help='高斯噪声std')
    ap.add_argument('--dark', action='store_true', help='模拟暗光+偏色')
    ap.add_argument('--n-pieces', type=int, default=4)
    args = ap.parse_args()

    rng = np.random.RandomState(args.seed)
    W_IMG, H_IMG = 1280, 720

    # 世界系: A4 210x297mm, 碎片放上半区 (y>148.5)
    pieces, true_size = _gen_guillotine_pieces(
        W=100.0, H=65.0, n_pieces=args.n_pieces, seed=args.seed)
    scattered = scatter_pieces(pieces, rng,
                               x_range=(20, 190), y_range=(165, 280))
    if len(scattered) != len(pieces):
        print(f"[Synth] 错误: 只放下 {len(scattered)}/{len(pieces)} 片, "
              "请换种子")
        sys.exit(1)

    # 虚拟单应: 世界mm -> 像素 (横放纸, 与实拍构图类似)
    quad_world = np.array([[0, 0], [0, 297], [210, 297], [210, 0]],
                          dtype=np.float32)
    quad_px = np.array([[350, 118], [953, 136], [948, 554], [367, 541]],
                       dtype=np.float32)
    Hw2p = cv2.getPerspectiveTransform(quad_world, quad_px)

    # 场景绘制
    img = np.full((H_IMG, W_IMG, 3), (45, 35, 30), np.uint8)  # 深色木桌
    cv2.fillPoly(img, [quad_px.astype(np.int32)], (215, 215, 210))
    if args.dark:
        # 光照梯度 + 偏蓝紫
        grad = np.linspace(0.55, 0.95, W_IMG, dtype=np.float32)[None, :, None]
        img = (img.astype(np.float32) * grad).clip(0, 255).astype(np.uint8)
        img[:, :, 0] = (img[:, :, 0].astype(np.float32) * 1.15
                        ).clip(0, 255).astype(np.uint8)

    # 碎片: 深绿色卡纸
    for poly in scattered:
        pts = cv2.perspectiveTransform(
            poly.astype(np.float32).reshape(-1, 1, 2), Hw2p)
        pts_i = pts.reshape(-1, 2).astype(np.int32)
        color = (30, int(rng.randint(60, 90)), 35)
        cv2.fillPoly(img, [pts_i], color)
        cv2.polylines(img, [pts_i], True, (20, 45, 25), 1)

    # 噪声 + 轻微模糊模拟ISP
    noise = rng.normal(0, args.noise, img.shape).astype(np.float32)
    img = (img.astype(np.float32) + noise).clip(0, 255).astype(np.uint8)
    img = cv2.GaussianBlur(img, (3, 3), 0)

    cv2.imwrite(args.out, img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(f"[Synth] 场景已保存: {args.out}")
    print(f"[Synth] 真值: 矩形 100.0x65.0mm, {args.n_pieces}片")
    for i, p in enumerate(scattered):
        e = [float(np.linalg.norm(p[(k + 1) % len(p)] - p[k]))
             for k in range(len(p))]
        print(f"  片{i}: {len(p)}顶点 边长={[f'{x:.1f}' for x in e]}")


if __name__ == '__main__':
    main()
