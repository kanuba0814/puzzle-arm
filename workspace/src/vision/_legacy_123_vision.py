"""123 项目视觉检测核心（pi_puzzle/vision.py 快照 2026-08-01 05:25，含黑底修复）。

移植到 workspace 时改动：
- 移除 camera_backend 依赖（外部传入帧）
- config.* 改为从 _legacy_123_config 导入
- SHOW_IMAGE 强制 False（库内无交互）
"""
import cv2
import numpy as np

from ._legacy_123_config import (
    A4_HEIGHT,
    A4_WIDTH,
    DIVIDER_LINE_Y,
    MIN_EDGE_LEN_MM,
    PIECE_COLOR_HSV,
    SHOW_IMAGE,
)


class PuzzleVision:
    def __init__(self, backend=None):
        """初始化视觉模块。

        workspace 用法：backend 保持 None，帧由调用方显式传入；
        homography 由 set_paper_homography() 或 load_calibration() 设置。
        """
        self.backend = backend
        self.homography = None  # 透视变换矩阵，必须显式加载
        self.calibration_points = []  # 标定点缓存

        # 保留旧属性, 兼容外部直接访问 self.cap 的调试代码
        self.cap = getattr(backend, "cap", None)

    def release(self):
        if self.backend is not None:
            self.backend.release()
        cv2.destroyAllWindows()

    # ==================== 摄像头标定 ====================

    def capture_frame(self):
        """采集一帧图像"""
        if self.backend is None:
            print("[Vision] 相机后端未初始化")
            return None

        frame = self.backend.read_rgb()
        if frame is None:
            print("[Vision] 摄像头读取失败")
            return None
        return frame

    def capture_depth(self):
        """采集一帧深度图像 (Astra S 可用, USB 摄像头返回 None)"""
        if self.backend is None:
            return None
        return self.backend.read_depth()

    def start_interactive_calibration(self):
        """
        交互式标定: 点击A4纸4个角点
        顺序: 左下 -> 右下 -> 右上 -> 左上
        计算像素到世界坐标(mm)的透视变换矩阵
        """
        print("[标定] 请按顺序点击A4纸的4个角:")
        print("  1. 左下角")
        print("  2. 右下角")
        print("  3. 右上角")
        print("  4. 左上角")

        frame = self.capture_frame()
        if frame is None:
            return False

        points = []

        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
                points.append((x, y))
                cv2.circle(frame, (x, y), 8, (0, 0, 255), -1)
                cv2.circle(frame, (x, y), 8, (255, 255, 255), 2)
                cv2.putText(
                    frame,
                    str(len(points)),
                    (x + 10, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("Calibration", frame)

        cv2.imshow("Calibration", frame)
        cv2.setMouseCallback("Calibration", mouse_callback)

        while len(points) < 4:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyWindow("Calibration")
                return False

        cv2.destroyWindow("Calibration")

        # A4纸世界坐标 (mm): 左下(0,0) 右下(210,0) 右上(210,297) 左上(0,297)
        world_pts = np.array(
            [
                [0, 0],
                [A4_WIDTH, 0],
                [A4_WIDTH, A4_HEIGHT],
                [0, A4_HEIGHT],
            ],
            dtype=np.float32,
        )

        pixel_pts = np.array(points, dtype=np.float32)

        self.homography, _ = cv2.findHomography(pixel_pts, world_pts)
        print(f"[标定] 完成, 透视矩阵:\n{self.homography}")

        # 保存标定结果
        np.save("homography.npy", self.homography)
        return True

    def load_calibration(self, path="homography.npy"):
        """加载已保存的标定矩阵"""
        try:
            self.homography = np.load(path)
            print(f"[标定] 已加载: {path}")
            return True
        except:
            print(f"[标定] 加载失败: {path}")
            return False

    def quad_from_homography(self):
        """由单应矩阵反推A4四角像素坐标, 顺序 [TL,TR,BR,BL] (与auto_detect_paper一致)"""
        if self.homography is None:
            return None
        invH = np.linalg.inv(self.homography)
        pts = np.array(
            [[0, A4_HEIGHT], [A4_WIDTH, A4_HEIGHT], [A4_WIDTH, 0], [0, 0]],
            dtype=np.float32,
        ).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(pts, invH).reshape(4, 2)

    def pixel_to_world(self, pixel_pts):
        """
        像素坐标 -> 世界坐标(mm)
        pixel_pts: (N, 2) 或 (2,) 的像素坐标
        返回: (N, 2) 的世界坐标
        """
        if self.homography is None:
            print("[Vision] 未标定!")
            return None

        pts = np.array(pixel_pts, dtype=np.float32).reshape(-1, 1, 2)
        world = cv2.perspectiveTransform(pts, self.homography)
        return world.reshape(-1, 2)

    # ==================== 图像预处理 (暗光鲁棒) ====================

    @staticmethod
    def preprocess(frame, max_width=1280):
        """
        预处理: 降采样到<=1280宽(2x2平均降噪) + 中值滤波。
        检测与标定全部在预处理后的图像上进行 (坐标系一致)。
        """
        h, w = frame.shape[:2]
        if w > max_width:
            nw = max_width
            nh = int(h * max_width / w)
            frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
        return cv2.medianBlur(frame, 5)

    @staticmethod
    def white_balance(frame, roi_mask, target_mean=150):
        """
        灰世界白平衡: 以纸面(应为白色)为基准校正色偏,
        并把整体亮度归一到 target_mean。暗光偏色(此相机偏蓝紫)必备。
        """
        f = frame.astype(np.float32)
        means = cv2.mean(f, mask=roi_mask)[:3]  # B,G,R
        g = means[1] + 1e-6
        f[:, :, 0] *= g / (means[0] + 1e-6)
        f[:, :, 2] *= g / (means[2] + 1e-6)
        mean_now = float(f.mean()) + 1e-6
        if mean_now < target_mean * 0.8:  # 太暗才提亮
            f *= target_mean / mean_now
        return np.clip(f, 0, 255).astype(np.uint8)

    # ==================== A4纸自动标定 (当前阶段) ====================

    def auto_detect_paper(self, frame=None):
        """
        自动检测A4纸四边形 (白纸 vs 桌面, 低饱和+高亮)
        返回: (4,2) ndarray 像素坐标, 顺序 [TL,TR,BR,BL]; 失败返回None
        """
        if frame is None:
            frame = self.capture_frame()
        if frame is None:
            return None
        frame = self.preprocess(frame)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        V, S = hsv[:, :, 2], hsv[:, :, 1]

        # 双阈值策略: Otsu优先(双峰分割最稳), 失败回退分位数规则
        # 暗光下分位数规则会把纸面阴影区切掉导致角点缺失
        t_otsu, mask_otsu = cv2.threshold(
            V, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        v_min = max(110, int(np.percentile(V, 65)))
        mask_pct = cv2.inRange(hsv, np.array([0, 0, v_min]), np.array([180, 90, 255]))
        candidates = [mask_otsu, mask_pct]

        for mask in candidates:
            # 大开运算断开纸与周围亮物的粘连; 不做闭运算——
            # RETR_EXTERNAL只取外轮廓, 纸内碎片空洞不影响, 闭运算反而会桥接杂物
            m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((25, 25), np.uint8))
            quad = self._quad_from_mask(m, frame.shape)
            if quad is not None:
                return quad

        print("[Vision] 未检测到A4纸")
        return None

    @staticmethod
    def _quad_from_mask(mask, frame_shape):
        """从二值掩码提取A4四边形 (最大轮廓->凸包->四角), 失败返回None"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < frame_shape[0] * frame_shape[1] * 0.1:
            return None

        # 凸包近似出4个角, 失败则退化为最小外接矩形
        hull = cv2.convexHull(cnt)
        quad = None
        peri = cv2.arcLength(hull, True)
        for eps in (0.01, 0.02, 0.03, 0.05):
            approx = cv2.approxPolyDP(hull, eps * peri, True)
            if len(approx) == 4:
                quad = approx.reshape(4, 2).astype(np.float32)
                break
        if quad is None:
            rect = cv2.minAreaRect(cnt)
            quad = cv2.boxPoints(rect).astype(np.float32)

        # 长宽比 sanity check (A4 = 1.414), 太离谱的候选直接拒绝
        d01 = np.linalg.norm(quad[1] - quad[0])
        d12 = np.linalg.norm(quad[2] - quad[1])
        aspect = max(d01, d12) / (min(d01, d12) + 1e-9)
        if abs(aspect - 1.414) > 0.25:
            return None

        # 排序: TL, TR, BR, BL (和最小/最大, 差最小/最大)
        s = quad.sum(axis=1)
        d = quad[:, 0] - quad[:, 1]
        ordered = np.array(
            [
                quad[np.argmin(s)],  # TL
                quad[np.argmax(d)],  # TR
                quad[np.argmax(s)],  # BR
                quad[np.argmin(d)],  # BL
            ],
            dtype=np.float32,
        )
        return ordered

    def set_paper_homography(self, quad, flip_y=False):
        """
        由A4四角建立 像素->世界(mm) 单应矩阵。
        世界系: x∈[0,210]短边, y∈[0,297]长边, 原点在纸角。
        自动判断纸在图像中的横竖: 图像里较长的一侧对应297mm。
        flip_y: 世界y轴取反 (用于让碎片区落在 y>148.5 的上半区)
        """
        TL, TR, BR, BL = quad
        top_len = np.linalg.norm(TR - TL)
        left_len = np.linalg.norm(BL - TL)

        if top_len <= left_len:
            # 图像中竖放: TL->(0,297) TR->(210,297) BR->(210,0) BL->(0,0)
            dst = np.array(
                [[0, A4_HEIGHT], [A4_WIDTH, A4_HEIGHT], [A4_WIDTH, 0], [0, 0]],
                dtype=np.float32,
            )
        else:
            # 图像中横放: 图像水平方向=A4长边
            # TL->(0,0) TR->(0,297) BR->(210,297) BL->(210,0)
            dst = np.array(
                [[0, 0], [0, A4_HEIGHT], [A4_WIDTH, A4_HEIGHT], [A4_WIDTH, 0]],
                dtype=np.float32,
            )
        if flip_y:
            dst[:, 1] = A4_HEIGHT - dst[:, 1]

        self.homography = cv2.getPerspectiveTransform(quad.astype(np.float32), dst)
        np.save("homography.npy", self.homography)
        return self.homography

    def paper_mask(self, frame_shape, quad, erode_px=5):
        """A4纸区域的二值掩码 (内缩erode_px, 排除纸边缘干扰)"""
        mask = np.zeros(frame_shape[:2], np.uint8)
        cv2.fillPoly(mask, [quad.astype(np.int32)], 255)
        if erode_px > 0:
            mask = cv2.erode(
                mask, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8)
            )
        return mask

    # ==================== 任意形状碎片检测 (当前阶段) ====================

    def detect_pieces_arbitrary(
        self,
        frame=None,
        quad=None,
        flat_th=0.82,
        smooth_mask=True,
        refine_edges=True,
    ):
        """
        检测纸上的有色碎片, 输出mm坐标的多边形 (CCW)。
        需要先设置 self.homography (set_paper_homography)。

        颜色判据自适应: 以纸面中位数饱和度为基准 +30 (最低90)。
        白纸在此相机下饱和度可达60-70, 固定阈值会把纸面阴影误判为碎片。
        白色碎片与纸同色, 本方法不适用 (后续用边缘法单独处理)。
        flat_th: 暗光平场阈值, 调用方可扫描多值取可解的一组
        smooth_mask: 是否对掩码做高斯平滑；黑底UVC扑克牌碎片可关闭以保留尖角
        refine_edges: 是否用直线拟合精化边缘；检测轮廓较干净时可关闭

        返回: list of dict:
            id, vertices(mm,CCW), edge_lengths(mm), centroid(mm),
            grip_point(mm, 最大内切圆圆心), area_mm2, angle_deg(PCA主轴),
            contour_px, approx_px (可视化用)
        """
        if frame is None:
            frame = self.capture_frame()
        if frame is None or self.homography is None:
            return []
        frame = self.preprocess(frame)
        if quad is None:
            quad = self.auto_detect_paper(frame)
            if quad is None:
                return []

        roi = self.paper_mask(frame.shape, quad, erode_px=8)

        # 暗光白平衡校正 (以纸面为白基准), 用于饱和度线索
        wb = self.white_balance(frame, roi)
        hsv_wb = cv2.cvtColor(wb, cv2.COLOR_BGR2HSV)
        S = hsv_wb[:, :, 1]
        # 亮度线索用原图V: WB亮度归一会把碎片提亮到阈值附近, 破坏V对比度
        # (实测暗光: 原图碎片V~65 纸面V~131, 差60+; WB后差仅~15)
        V = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
        # 纸面基准 (碎片占比小, 中位数即纸色)
        paper_s = np.median(S[roi > 0])
        paper_v = np.median(V[roi > 0])

        black_background = paper_v < 80
        if black_background:
            # 固定黑色工作区 + 任意非黑色拼图片：
            # 1) 亮度线索覆盖白/灰/浅色；2) 色度线索覆盖较暗的红黄绿蓝等。
            # 两者取并集，不能要求“高饱和且高亮”，否则白色和深色片会漏检。
            pv = float(paper_v)
            ps = float(paper_s)
            bright = V > max(pv + 28.0, 45.0)
            chromatic = (S > max(ps + 30.0, 55.0)) & (V > max(pv + 15.0, 32.0))
            pm = (bright | chromatic).astype(np.uint8) * 255
        elif paper_v < 140:
            # 暗光白纸模式: 平场校正 (V/局部背景) 消除光照不均与软阴影,
            # 碎片 ratio~0.55-0.72, 纸面/阴影 ratio~1.0-1.2, 分离干净
            Vf = V.astype(np.float32)
            bg = cv2.GaussianBlur(Vf, (0, 0), 60)
            flat = Vf / (bg + 1e-3)
            # 阈值0.82≈碎片(0.55-0.72)与纸面(1.0)的中点, 抗边界内缩
            pm = (flat < flat_th).astype(np.uint8) * 255
        else:
            # 正常光: 线索1(主)显著暗于纸面; 线索2(辅)饱和度极高且略暗
            # S线索必须严格 — 纸面阴影S只比纸面高10-20, 宽松会吞入阴影
            pm = (
                (V < paper_v - 25) | ((S > paper_s + 40) & (S > 60) & (V < paper_v - 8))
            ).astype(np.uint8) * 255
        pm = cv2.bitwise_and(pm, roi)
        pm = cv2.morphologyEx(pm, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # 闭运算填孔：暗光白纸噪声洞多用大核；黑底上的亮色碎片间距
        # 可能只有数个像素，9×9 会把独立碎片桥接（实拍 4 片合成 1 块），
        # 因此黑底固定 3×3，保持离散连通域。
        close_k = 3 if black_background else (9 if paper_v < 140 else 5)
        pm = cv2.morphologyEx(
            pm, cv2.MORPH_CLOSE, np.ones((close_k, close_k), np.uint8)
        )
        # 平滑掩码边缘 (暗光FPN噪声使轮廓阶梯化, 后续多边形近似会失真)
        # 对黑底UVC实拍可选择关闭，避免扑克牌碎片的尖角和短边被圆滑掉。
        if smooth_mask:
            pm = cv2.GaussianBlur(pm, (9, 9), 0)
            _, pm = cv2.threshold(pm, 127, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(pm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        pieces = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 200:  # ~50mm²以下视为噪点
                continue
            # 统计复核: 整块区域的均值必须与纸面有显著差异 (抗噪点/阴影)
            blob = np.zeros(pm.shape, np.uint8)
            cv2.drawContours(blob, [cnt], 0, 255, -1)
            mean_s = cv2.mean(S, mask=blob)[0]
            mean_v = cv2.mean(V, mask=blob)[0]
            if black_background:
                # 黑底上接受显著更亮（含白/灰）或显著更有色度（含深色）的区域。
                # 阈值必须排除低对比度反光：实测反光 meanV 仅比纸面高 +33
                # （纸面 V=46，反光 V=79），真碎片高 +96（V=142）。
                # 之前 +20 太宽松，反光区域被误检为碎片（6边 620mm² 伪轮廓）。
                differs = mean_v > float(paper_v) + 60.0 or (
                    mean_s > float(paper_s) + 28.0 and mean_v > float(paper_v) + 40.0
                )
            else:
                differs = mean_v < paper_v - 18 or mean_s > paper_s + 35
            if not differs:
                continue
            # 轮廓 -> mm坐标
            cnt_mm = self.pixel_to_world(cnt.reshape(-1, 2))
            if cnt_mm is None:
                continue
            cnt_mm = cnt_mm.astype(np.float32).reshape(-1, 1, 2)
            # mm坐标系下多边形近似: 渐进加大eps直到<=6顶点
            # (题目保证边数<=5; 暗光噪声大时固定eps会产生大量阶梯伪边)
            peri = cv2.arcLength(cnt_mm, True)
            approx = None
            for rel in (0.008, 0.012, 0.016, 0.02, 0.026, 0.034):
                approx = cv2.approxPolyDP(cnt_mm, rel * peri, True)
                if len(approx) <= 6:
                    break
            verts = approx.reshape(-1, 2)
            if len(verts) < 3:
                continue
            verts = self._cleanup_polygon(verts)
            if len(verts) < 3:
                continue
            # 边缘直线精化: 对每条边收集附近轮廓点拟合直线,
            # 相邻直线求交得顶点 — 比approxPolyDP顶点抗噪得多
            cnt_pts = cnt_mm.reshape(-1, 2)
            if refine_edges:
                refined = self._refine_edges(cnt_pts, verts)
                if refined is not None:
                    verts = refined
            # 保证CCW
            if cv2.contourArea(verts.astype(np.float32), oriented=True) < 0:
                verts = verts[::-1].copy()

            area_mm2 = abs(cv2.contourArea(verts.astype(np.float32)))
            # 黑底扑克牌/白色任务碎片由一整张卡裁成4片，真实单片面积通常
            # 远大于纸边伪轮廓；200mm²以下按边框撕裂/噪点处理。
            # 实测：扑克牌不规则切分最小片可低至 263mm²，门槛放宽到 200。
            min_area_mm2 = 200.0 if black_background else 100.0
            if area_mm2 < min_area_mm2:
                continue
            # 上限：单片不可能超过目标矩形面积（题面最大 12×9cm = 10800mm²）。
            # 实测：相邻标定板/垃箱等亮色大物体会形成 18000mm²+ 巨轮廓，
            # 不过滤会占掉 max_pieces 名额并污染求解。
            if area_mm2 > 5000.0:
                continue
            edges = [
                float(np.linalg.norm(verts[(i + 1) % len(verts)] - verts[i]))
                for i in range(len(verts))
            ]
            # 黑底综合色检测可能把白色面板边框截成细长轮廓；单片不可能有
            # 跨越大半张A4的超长边，进入求解器前按物理上限拒绝。
            if max(edges, default=0.0) > 150.0:
                continue
            centroid = verts.mean(axis=0)
            grip = self._grip_point(verts)
            angle = self._pca_angle(verts)

            pieces.append(
                {
                    "id": len(pieces),
                    "vertices": verts,
                    "edge_lengths": edges,
                    "centroid": (float(centroid[0]), float(centroid[1])),
                    "grip_point": grip,
                    "area_mm2": float(area_mm2),
                    "angle_deg": angle,
                    "contour_px": cnt,
                }
            )
        return pieces

    @staticmethod
    def _cleanup_polygon(verts, collinear_tol_mm=0.8, min_edge_mm=MIN_EDGE_LEN_MM):
        """
        多边形清理: 1) 去除近共线顶点(锯齿噪声) 2) 短边坍缩为中点。
        题目最小边长20mm, <8mm的边必是检测噪声。
        """
        pts = [v.copy() for v in verts]
        for _ in range(3 * len(pts) + 6):
            n = len(pts)
            if n < 3:
                break
            # 先找最短边, 过短则坍缩
            lens = [np.linalg.norm(pts[(i + 1) % n] - pts[i]) for i in range(n)]
            i_min = int(np.argmin(lens))
            if lens[i_min] < min_edge_mm and n > 3:
                mid = (pts[i_min] + pts[(i_min + 1) % n]) / 2
                pts = [pts[k] for k in range(n) if k != i_min and k != (i_min + 1) % n]
                # 中点插回原位置 (在被删两点的位置)
                insert_at = (
                    min(i_min, (i_min + 1) % n) if not (i_min == n - 1) else len(pts)
                )
                if i_min == n - 1:  # 跨越首尾: 中点放到末尾, 逻辑等价
                    pts.append(mid)
                else:
                    pts.insert(insert_at, mid)
                continue
            # 共线顶点移除
            removed = False
            for i in range(n):
                prev, cur, nxt = pts[(i - 1) % n], pts[i], pts[(i + 1) % n]
                v1 = cur - prev
                v2 = nxt - cur
                l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if l1 < 1e-6:
                    pts.pop(i)
                    removed = True
                    break
                # 点到直线距离 (2D叉积 = a.x*b.y - a.y*b.x)
                dist = abs(v1[0] * (cur - nxt)[1] - v1[1] * (cur - nxt)[0]) / (
                    l1 + 1e-9
                )
                sin_a = abs(v1[0] * v2[1] - v1[1] * v2[0]) / (l1 * l2 + 1e-9)
                if dist < collinear_tol_mm and sin_a < 0.25:
                    pts.pop(i)
                    removed = True
                    break
            if not removed:
                break
        return np.array(pts, dtype=np.float64)

    @staticmethod
    def _refine_edges(cnt_pts, verts, band_mm=2.5):
        """
        边缘直线精化: 对多边形每条边, 收集距离该边<band的轮廓点
        拟合直线(cv2.fitLine), 相邻直线求交作为精化顶点。
        approxPolyDP顶点落在噪声轮廓上, 直线拟合取平均可显著降噪。
        失败(点太少/近平行)返回None。
        """
        n = len(verts)
        if n < 3 or len(cnt_pts) < 3 * n:
            return None
        lines = []
        for i in range(n):
            a, b = verts[i], verts[(i + 1) % n]
            ab = b - a
            L = np.linalg.norm(ab)
            if L < 1e-6:
                return None
            d = ab / L
            # 轮廓点到边的垂距 与 沿边投影
            rel = cnt_pts - a
            proj = rel @ d
            perp = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0])
            # 投影范围适当外伸, 覆盖顶点附近轮廓点
            sel = (perp < band_mm) & (proj > -L * 0.15) & (proj < L * 1.15)
            pts = cnt_pts[sel]
            if len(pts) < 5:
                return None
            line = cv2.fitLine(
                pts.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01
            ).reshape(4)
            vx, vy, x0, y0 = line
            lines.append((np.array([x0, y0]), np.array([vx, vy])))
        # 相邻直线求交: 顶点j = 边j-1与边j的交点; 近平行保留原值
        out = [None] * n
        for i in range(n):
            j = (i + 1) % n
            p1, d1 = lines[i]
            p2, d2 = lines[j]
            cross = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(cross) < 0.1:  # 近平行(夹角<~6°), 该顶点不精化
                out[j] = verts[j].copy()
                continue
            t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / cross
            out[j] = p1 + t * d1
        out = np.array(out, dtype=np.float64)
        #  sanity: 精化后顶点不能偏离原顶点太远 (>6mm视为异常)
        if np.linalg.norm(out - verts, axis=1).max() > 6.0:
            return None
        return out

    @staticmethod
    def _grip_point(verts, px_per_mm=4):
        """最大内切圆圆心 (电磁铁/夹爪的最佳抓取点, 保证在碎片内)"""
        lo = verts.min(axis=0) - 2
        s = px_per_mm
        size = ((verts.max(axis=0) + 2 - lo) * s).astype(int) + 1
        m = np.zeros((size[1], size[0]), np.uint8)
        cv2.fillPoly(m, [((verts - lo) * s).astype(np.int32)], 255)
        dist = cv2.distanceTransform(m, cv2.DIST_L2, 5)
        _, _, _, max_loc = cv2.minMaxLoc(dist)
        gx = lo[0] + max_loc[0] / s
        gy = lo[1] + max_loc[1] / s
        return (float(gx), float(gy))

    @staticmethod
    def _pca_angle(verts):
        """碎片主轴方向角 (度, [-90,90)), 仅供参考"""
        pts = verts.astype(np.float64)
        mean = pts.mean(axis=0)
        cov = np.cov((pts - mean).T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        v = eigvecs[:, np.argmax(eigvals)]
        return float(np.degrees(np.arctan2(v[1], v[0])))

    # ==================== 碎片检测 (第1项: 自备碎片) ====================

    def detect_colored_pieces(self, frame=None):
        """
        颜色检测 - 适用于第1项(纯色碎片)
        返回: list of dict, 每个碎片包含:
            - center_world: (x, y) 世界坐标
            - angle: 旋转角度 (度)
            - contour: 轮廓
            - bbox: 包围盒 (x,y,w,h) 像素
        """
        if frame is None:
            frame = self.capture_frame()
        if frame is None:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 颜色阈值检测 (红色 hue 跨0度: 低段[0-10] + 高段[160-179] 双区间合并)
        lo, up = PIECE_COLOR_HSV["lower"], PIECE_COLOR_HSV["upper"]
        mask = cv2.inRange(hsv, lo, up)
        if lo[0] <= 10:  # 红色系: 补高 hue 段
            mask |= cv2.inRange(
                hsv, np.array([160, lo[1], lo[2]]), np.array([179, up[1], up[2]])
            )

        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 高斯模糊 + Canny边缘
        blurred = cv2.GaussianBlur(mask, (5, 5), 0)

        # 找轮廓
        contours, _ = cv2.findContours(
            blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pieces = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:  # 过滤小噪点
                continue

            # 最小外接矩形
            rect = cv2.minAreaRect(cnt)  # ((cx,cy), (w,h), angle)
            box = cv2.boxPoints(rect)
            box = np.round(box).astype(np.int32)

            # 中心点 (像素)
            center_px = rect[0]

            # 转世界坐标
            center_world = self.pixel_to_world(center_px)
            if center_world is None:
                continue
            center_world = center_world[0]

            # 旋转角度
            angle = rect[2]
            # 标准化角度: 让长边为基准
            if rect[1][0] < rect[1][1]:
                angle += 90

            pieces.append(
                {
                    "center_world": (float(center_world[0]), float(center_world[1])),
                    "center_pixel": (int(center_px[0]), int(center_px[1])),
                    "angle": angle,
                    "size": rect[1],  # (w, h) 像素
                    "contour": cnt,
                    "box": box,
                }
            )

            if SHOW_IMAGE:
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                cv2.circle(
                    frame, (int(center_px[0]), int(center_px[1])), 5, (0, 0, 255), -1
                )
                text = f"({center_world[0]:.0f},{center_world[1]:.0f}) {angle:.0f}°"
                cv2.putText(
                    frame,
                    text,
                    (int(center_px[0]) + 10, int(center_px[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1,
                )

        if SHOW_IMAGE:
            cv2.imshow("Detection", frame)
            cv2.imshow("Mask", mask)

        return pieces

    # ==================== 碎片检测 (第2项: 现场碎片) ====================

    def detect_pieces_by_edge(self, frame=None):
        """
        边缘检测 - 适用于第2项(白色碎片/扑克牌碎片)
        基于轮廓检测 + 形状分析
        返回: list of dict, 包含位置、角度、顶点、边长
        """
        if frame is None:
            frame = self.capture_frame()
        if frame is None:
            return []

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 自适应阈值二值化 (适应不同光照)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )

        # 形态学处理
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # 边缘检测
        edges = cv2.Canny(binary, 50, 150)

        # 找轮廓
        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pieces = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:
                continue

            # 多边形近似
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            # 最小外接矩形
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.round(box).astype(np.int32)

            center_px = rect[0]
            center_world = self.pixel_to_world(center_px)
            if center_world is None:
                continue
            center_world = center_world[0]

            angle = rect[2]
            if rect[1][0] < rect[1][1]:
                angle += 90

            # 提取顶点 (世界坐标)
            vertices_world = []
            for pt in approx:
                vw = self.pixel_to_world(pt[0])
                if vw is not None:
                    vertices_world.append((float(vw[0][0]), float(vw[0][1])))

            # 计算边长 (世界坐标, mm)
            edge_lengths = []
            for i in range(len(vertices_world)):
                p1 = np.array(vertices_world[i])
                p2 = np.array(vertices_world[(i + 1) % len(vertices_world)])
                edge_lengths.append(float(np.linalg.norm(p2 - p1)))

            pieces.append(
                {
                    "center_world": (float(center_world[0]), float(center_world[1])),
                    "center_pixel": (int(center_px[0]), int(center_px[1])),
                    "angle": angle,
                    "vertices": vertices_world,  # 顶点(世界坐标)
                    "edge_lengths": edge_lengths,  # 边长(mm)
                    "num_vertices": len(vertices_world),  # 顶点数
                    "contour": cnt,
                    "box": box,
                }
            )

            if SHOW_IMAGE:
                cv2.drawContours(frame, [approx], 0, (0, 255, 0), 2)
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 1)
                text = f"V={len(vertices_world)} ({center_world[0]:.0f},{center_world[1]:.0f})"
                cv2.putText(
                    frame,
                    text,
                    (int(center_px[0]) + 10, int(center_px[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    1,
                )

        if SHOW_IMAGE:
            cv2.imshow("Edge Detection", frame)

        return pieces

    # ==================== 扑克牌图案匹配 ====================

    def match_poker_pattern(self, frame, template_pieces):
        """
        扑克牌牌面匹配 (第2项第(2)小题)
        使用特征匹配 (ORB/SIFT) 确定碎片图案朝向

        参数:
            frame: 当前帧
            template_pieces: 模板图片列表 (已拼接好的目标图案)

        返回: 每个碎片的正确旋转角度
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ORB特征检测器
        orb = cv2.ORB_create(nfeatures=500)

        # 模板特征
        results = []
        for template in template_pieces:
            tpl_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
            kp1, des1 = orb.detectAndCompute(tpl_gray, None)
            kp2, des2 = orb.detectAndCompute(gray, None)

            if des1 is None or des2 is None:
                continue

            # BF匹配
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            matches = bf.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)

            results.append(
                {
                    "matches": matches[:20],
                    "template_kp": kp1,
                    "frame_kp": kp2,
                }
            )

        return results

    # ==================== 分界线检测 ====================

    def detect_divider_line(self, frame=None):
        """
        检测A4纸上下区域的分界线
        返回: 分界线在世界坐标系中的Y坐标
        """
        if frame is None:
            frame = self.capture_frame()
        if frame is None:
            return DIVIDER_LINE_Y

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        binary = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)[1]

        # 霍夫直线检测
        lines = cv2.HoughLinesP(
            binary, 1, np.pi / 180, threshold=100, minLineLength=200, maxLineGap=10
        )

        if lines is not None:
            # 找最长的水平线
            best_y = None
            max_len = 0
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if abs(y1 - y2) < 10:  # 近似水平
                    length = abs(x2 - x1)
                    if length > max_len:
                        max_len = length
                        mid_y = (y1 + y2) / 2
                        world = self.pixel_to_world(
                            np.array([[CAMERA_WIDTH / 2, mid_y]])
                        )
                        if world is not None:
                            best_y = world[0][1]

            if best_y is not None:
                return best_y

        return DIVIDER_LINE_Y


if __name__ == "__main__":
    # 视觉模块测试
    vision = PuzzleVision()

    # 尝试加载已有标定
    if not vision.load_calibration():
        print("请先进行标定...")
        if not vision.start_interactive_calibration():
            print("标定失败, 退出")
            vision.release()
            exit()

    print("\n=== 碎片检测测试 ===")
    print("按 'q' 退出, 'c' 切换检测模式, 'd' 显示深度图")

    mode = "color"  # 'color' or 'edge'
    show_depth = False
    while True:
        frame = vision.capture_frame()
        if frame is None:
            break

        if mode == "color":
            pieces = vision.detect_colored_pieces(frame)
        else:
            pieces = vision.detect_pieces_by_edge(frame)

        print(f"\r检测到 {len(pieces)} 个碎片", end="")

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("c"):
            mode = "edge" if mode == "color" else "color"
            print(f"\n切换到 {mode} 模式")
        elif key == ord("d"):
            show_depth = not show_depth
            print(f"\n深度图显示: {show_depth}")

        if show_depth:
            depth = vision.capture_depth()
            if depth is not None:
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth, alpha=255 / 1000), cv2.COLORMAP_JET
                )
                cv2.imshow("Depth", depth_vis)

    vision.release()
