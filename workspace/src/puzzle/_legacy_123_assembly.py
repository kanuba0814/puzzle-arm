"""
任意形状碎片 -> 未知矩形 拼图求解器 (几何核心)

问题: 给定 N (<=4) 个任意多边形碎片 (顶点, mm坐标, 不允许翻转),
求一组刚体变换 (旋转+平移), 使所有碎片无缝拼成一个矩形。
矩形尺寸完全未知, 由碎片边匹配关系反推。

算法:
1. 边匹配: 等长边为公共边候选; 支持 T 型接缝 (短边对齐长边一端)
2. DFS 咬合搜索: 固定碎片0在原位 (商掉整体刚体自由度),
   逐个把未放置碎片沿匹配边咬合到已放置集合上
3. 重叠检查: 光栅化 (RASTER_PX_PER_MM), 超过容差即剪枝
4. 矩形验证: 边界边减法提取外轮廓 (处理 T 型接缝),
   合并共线段 -> 须为 4 个 ~90 度角 + 对边等长 + 面积一致

用法:
    from rect_assembly import AssemblySolver
    solver = AssemblySolver()
    sol = solver.solve(pieces)   # pieces: [np.ndarray(N,2), ...] mm
    # sol = {'transforms': [(R,t), ...], 'rect_corners': (4,2), ...}

自测: python rect_assembly.py  (矩形随机 guillotine 切成4片, 打乱后求解)
"""
import time
import numpy as np
import cv2
from vision._legacy_123_config import (
    EDGE_MATCH_TOL_MM,
    EDGE_MATCH_TOL_RELAX,
    OVERLAP_TOL_RATIO,
    RASTER_PX_PER_MM,
    RECT_AREA_TOL,
    RECT_ANGLE_TOL_DEG,
    RECT_DIM_RANGE,
    RECT_SIDE_TOL_MM,
    SOLVER_MAX_STATES,
)

MIN_COMPLEMENT_MM = 8.0  # T型接缝剩余段最小长度 (题目最小边20mm, 留余量)
DFS_TIME_LIMIT_S = 20.0  # 搜索时间上限


# ==================== 2D 刚体变换 ====================

def make_transform(angle_rad, t):
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    R = np.array([[c, -s], [s, c]])
    return R, np.asarray(t, dtype=float)


def apply_transform(RT, pts):
    R, t = RT
    return pts @ R.T + t


def compose(T2, T1):
    """T = T2 ∘ T1 (先T1后T2)"""
    R2, t2 = T2
    R1, t1 = T1
    return R2 @ R1, R2 @ t1 + t2


def transform_angle(RT):
    R, _ = RT
    return np.degrees(np.arctan2(R[1, 0], R[0, 0]))


IDENTITY = make_transform(0.0, [0.0, 0.0])


# ==================== 多边形工具 ====================

def signed_area(verts):
    x, y = verts[:, 0], verts[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ensure_ccw(verts):
    """保证逆时针 (signed area > 0)"""
    if signed_area(verts) < 0:
        return verts[::-1].copy()
    return verts


def edge_list(verts):
    """返回 [(A, B), ...] 每条有向边的端点 (CCW顺序)"""
    n = len(verts)
    return [(verts[i], verts[(i + 1) % n]) for i in range(n)]


def _cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


# ==================== 求解器 ====================

class AssemblySolver:
    def __init__(self, edge_tol=EDGE_MATCH_TOL_MM,
                 max_states=SOLVER_MAX_STATES, verbose=True):
        self.edge_tol = edge_tol
        self.max_states = max_states
        self.verbose = verbose
        self._states = 0
        self._total_area = 0.0
        self._solutions = []
        self._visited = set()
        self._t0 = 0.0
        # 诊断计数: 各阶段拒绝原因
        self.stats = {'overlap': 0, 'fit': 0, 'full_placed': 0,
                      'validate_fail': 0, 'dead_end': 0}
        self._near_misses = []
        self._full_samples = []

    # ---------- 主入口 ----------

    def solve(self, pieces_verts):
        """
        pieces_verts: list of (N,2) ndarray, mm, 任意摆放位置
        返回 solution dict 或 None:
            transforms: 每片 (R,t) 从原始坐标 -> 拼接坐标
            placed_verts: 每片拼接后的顶点
            rect_corners: (4,2) 目标矩形角点 (拼接坐标系)
            rect_size: (W, H) mm
        """
        n = len(pieces_verts)
        if n == 0:
            return None
        pieces = [ensure_ccw(np.asarray(v, dtype=float)) for v in pieces_verts]

        if n == 1:
            result = self._validate_rectangle(pieces, [IDENTITY])
            if result is None:
                return None
            corners, _, _, _ = result
            return self._make_solution(pieces, [IDENTITY], corners)

        # 预计算边匹配候选
        candidates = self._build_candidates(pieces)
        if self.verbose:
            nfull = sum(1 for c in candidates if c['kind'] == 'full')
            print(f"[Assembly] {n}片, 匹配候选 {len(candidates)} "
                  f"(全等边 {nfull}, T型 {len(candidates) - nfull})")
        if not candidates:
            print("[Assembly] 无任何边匹配, 无解")
            return None

        # DFS: placed = {piece_idx: transform}
        # 收集多个解并打分, 取最优 (避免伪解: 松容差下可能拼出"别的矩形")
        self._states = 0
        self._total_area = sum(abs(signed_area(p)) for p in pieces)
        # 每片最大跨度 (光栅画布边距用)
        self._diameters = [
            max(np.linalg.norm(p[a] - p[b])
                for a in range(len(p)) for b in range(a + 1, len(p)))
            for p in pieces]
        self._solutions = []
        self._near_misses = []
        self._full_samples = []
        self._visited = set()
        self._t0 = time.time()
        self._dfs(pieces, {0: IDENTITY}, candidates)
        solution = self._best_solution()
        if solution is None and self.edge_tol < EDGE_MATCH_TOL_RELAX:
            print(f"[Assembly] 严格容差({self.edge_tol}mm)无解, "
                  f"放宽到{EDGE_MATCH_TOL_RELAX}mm重试")
            self.edge_tol = EDGE_MATCH_TOL_RELAX
            candidates = self._build_candidates(pieces)
            self._states = 0
            self._solutions = []
            self._near_misses = []
            self._full_samples = []
            self._visited = set()
            self._t0 = time.time()
            self._dfs(pieces, {0: IDENTITY}, candidates)
            solution = self._best_solution()

        if solution is None:
            # 近似解兜底: 比赛按相邻顶点距离计分, 近似拼接也能得分。
            # 检测噪声大时严格矩形校验会误杀正确拼法。
            # bbox填充率(外接矩形面积/碎片总面积): 越小越接近矩形,
            # 完美矩形=1.0, 检测侵蚀+接缝缝隙通常<1.35
            nm = self._best_near_miss()
            if nm is not None and nm['score'] < 1.35:
                print(f"[Assembly] 无严格解, 近似解兜底: "
                      f"{nm['W']:.1f}x{nm['H']:.1f}mm "
                      f"填充率{nm['score']:.3f}")
                transforms = [nm['placed'][i] for i in range(n)]
                result = self._make_solution(pieces, transforms,
                                             nm['corners'])
                # 评分加1000: 保证任何严格解都优于近似解 (跨调用比较时)
                result['score'] = 1000.0 + nm['score']
                result['fill_ratio'] = nm['score']
                result['approximate'] = True
                return result
            print(f"[Assembly] 搜索 {self._states} 状态后无解 "
                  "(可能存在非端点对齐的接缝或检测噪声过大)")
            return None

        transforms = [solution['placed'][i] for i in range(n)]
        if self.verbose:
            print(f"[Assembly] 求解成功: 矩形 {solution['W']:.1f}x"
                  f"{solution['H']:.1f}mm, 评分 {solution['score']:.2f}, "
                  f"候选解 {len(self._solutions)} 个, 状态 {self._states}")
        result = self._make_solution(pieces, transforms,
                                     solution['corners'])
        result['score'] = solution['score']
        return result

    def _best_solution(self):
        if not self._solutions:
            return None
        return min(self._solutions, key=lambda s: s['score'])

    def _best_near_miss(self):
        if not self._near_misses:
            return None
        return min(self._near_misses, key=lambda s: s['score'])

    # ---------- 边匹配候选 ----------

    def _build_candidates(self, pieces):
        """
        对所有碎片对(i,j)的所有边对, 生成咬合候选:
        - full: 两边等长, p边反向对齐q边 (1种对齐)
        - partial: p边较短, 对齐到q边的某一端 (flush-A / flush-B)

        T型候选过滤: 短边贴长边时, 剩余长度 lj-li 必须能由拼图中
        其他边 (单条 或 两条之和) 解释, 否则是伪候选。
        不过滤会产生 O(边数^2) 的爆炸伪候选 (任意短边都能齐平到任意长边)。
        """
        all_lens = []
        for p in pieces:
            for A, B in edge_list(p):
                all_lens.append(np.linalg.norm(B - A))

        def has_complement(clen):
            for L in all_lens:
                if abs(L - clen) <= self.edge_tol:
                    return True
            m = len(all_lens)
            for a in range(m):
                for b in range(a, m):
                    if abs(all_lens[a] + all_lens[b] - clen) <= self.edge_tol:
                        return True  # 双T接缝: 剩余段由两片覆盖
                    if abs(abs(all_lens[a] - all_lens[b]) - clen) <= self.edge_tol:
                        return True  # 交错T接缝: 剩余段是两长边之差
            return False

        cands = []
        for i in range(len(pieces)):
            for j in range(len(pieces)):
                if i == j:
                    continue
                ei = edge_list(pieces[i])
                ej = edge_list(pieces[j])
                for a, (p0, p1) in enumerate(ei):
                    li = np.linalg.norm(p1 - p0)
                    if li < 1e-6:
                        continue
                    for b, (q0, q1) in enumerate(ej):
                        lj = np.linalg.norm(q1 - q0)
                        if li <= lj + self.edge_tol:
                            if abs(li - lj) <= self.edge_tol:
                                kind = 'full'
                            else:
                                # T型: 剩余段很小(接近全等,无需解释)
                                # 或剩余长度可被其他边解释时才保留候选
                                if lj - li > MIN_COMPLEMENT_MM and \
                                        not has_complement(lj - li):
                                    continue
                                kind = 'partial'
                            cands.append({
                                'short': i, 'long': j,
                                'edge_s': a, 'edge_l': b,
                                'len_s': li, 'len_l': lj,
                                'kind': kind,
                            })
        return cands

    def _snap_transforms(self, cand, pieces, T_long):
        """
        计算把 short 碎片咬合到 long 碎片(已放置, 变换T_long)上的变换。
        long边放置坐标: A->B (CCW, 内部在左)。
        short边要贴到其右侧 => short边反向: s0 靠 B 侧, s1 靠 A 侧。
        返回 list of (R,t) (full: 1个; partial: 2个端点对齐)
        """
        i, j = cand['short'], cand['long']
        s0, s1 = edge_list(pieces[i])[cand['edge_s']]
        q0, q1 = edge_list(pieces[j])[cand['edge_l']]
        A = apply_transform(T_long, q0.reshape(1, 2))[0]
        B = apply_transform(T_long, q1.reshape(1, 2))[0]

        L = np.linalg.norm(B - A)
        if L < 1e-6:
            return []
        u = (B - A) / L  # A->B 单位向量
        ls = np.linalg.norm(s1 - s0)

        if cand['kind'] == 'full':
            targets = [(B, A)]          # s0->B, s1->A
        else:
            targets = [(A + u * ls, A),  # flush A端: s1->A
                       (B, B - u * ls)]  # flush B端: s0->B

        results = []
        for T0, T1_ in targets:
            # R @ (s1 - s0) = T1_ - T0
            ang_src = np.arctan2(s1[1] - s0[1], s1[0] - s0[0])
            d = T1_ - T0
            ang_dst = np.arctan2(d[1], d[0])
            R, _ = make_transform(ang_dst - ang_src, [0, 0])
            t = T0 - R @ s0
            results.append((R, t))
        return results

    def _snap_long_transforms(self, cand, pieces, T_short):
        """
        反向咬合: short碎片已放置(变换T_short), 求long碎片(未放置)的变换。
        partial匹配中短边必须与长边一端齐平, 所以给定短边位置后,
        长边也只有2种离散位姿 (短边齐平于长边A端 或 B端)。
        返回 list of (R,t)
        """
        i, j = cand['short'], cand['long']
        s0, s1 = edge_list(pieces[i])[cand['edge_s']]
        q0, q1 = edge_list(pieces[j])[cand['edge_l']]
        S0 = apply_transform(T_short, s0.reshape(1, 2))[0]
        S1 = apply_transform(T_short, s1.reshape(1, 2))[0]

        ls = np.linalg.norm(S1 - S0)
        if ls < 1e-6:
            return []
        # 短边放置后, 长边所在直线方向 (与短边反向: 内部在异侧)
        u = (S0 - S1) / ls          # 长边 A->B 方向 = s1->s0 方向
        L = cand['len_l']

        # flush A端: 短边覆盖 [A, A+u*ls], 即 A=S1, B=S1+u*L
        # flush B端: 短边覆盖 [B-u*ls, B], 即 B=S0, A=S0-u*L
        targets = [(S1, S1 + u * L), (S0 - u * L, S0)]

        results = []
        for A, B in targets:
            # R @ (q1 - q0) = B - A
            ang_src = np.arctan2(q1[1] - q0[1], q1[0] - q0[0])
            d = B - A
            ang_dst = np.arctan2(d[1], d[0])
            R, _ = make_transform(ang_dst - ang_src, [0, 0])
            t = A - R @ q0
            results.append((R, t))
        return results

    # ---------- DFS ----------

    def _dfs(self, pieces, placed, candidates):
        self._states += 1
        if self._states > self.max_states:
            return
        if time.time() - self._t0 > DFS_TIME_LIMIT_S:
            return
        if len(self._solutions) >= 6:
            return
        # 已有近乎完美的解 -> 提前结束搜索
        if self._solutions and \
                min(s['score'] for s in self._solutions) < 2.0:
            return
        n = len(pieces)
        if len(placed) == n:
            self.stats['full_placed'] += 1
            transforms = [placed[i] for i in range(n)]
            result = self._validate_rectangle(pieces, transforms)
            if result is not None:
                corners, score, W, H = result
                self._solutions.append({
                    'placed': dict(placed), 'corners': corners,
                    'score': score, 'W': W, 'H': H,
                })
            else:
                self.stats['validate_fail'] += 1
                # 近似解候选: 边界走环太脆弱(接缝缝隙会断环),
                # 改用免走环的 bbox 填充率: 正确拼法的最小外接矩形
                # 面积 / 碎片总面积 最小 (伪拼法碎片外伸, bbox更大)。
                # 检测侵蚀对所有完整摆放近似为常数, 不影响排序。
                all_v = np.vstack(
                    [apply_transform(placed[i], pieces[i])
                     for i in range(n)]).astype(np.float32)
                (cx, cy), (bw, bh), _ang = cv2.minAreaRect(all_v)
                sum_area = sum(abs(signed_area(pieces[i]))
                               for i in range(n))
                fill_ratio = (bw * bh) / max(sum_area, 1e-9)
                corners = cv2.boxPoints(
                    ((cx, cy), (bw, bh), _ang)).astype(np.float64)
                self._near_misses.append({
                    'placed': dict(placed), 'corners': corners,
                    'score': fill_ratio, 'W': bw, 'H': bh,
                })
                # 诊断: 保留前3个完整摆放供可视化
                if len(self._full_samples) < 3:
                    self._full_samples.append(
                        [apply_transform(placed[i], pieces[i])
                         for i in range(n)])
            return

        # 置换表: (已放置集合, 各片位姿量化) 相同的状态只展开一次
        # (先放A再放B 与 先放B再放A 是同一状态)
        key = tuple(sorted(
            (pi, round(np.degrees(np.arctan2(T[0][1, 0], T[0][0, 0])), 2),
             round(T[1][0], 1), round(T[1][1], 1))
            for pi, T in placed.items()))
        if key in self._visited:
            return
        self._visited.add(key)

        # 生成所有未放置碎片的move, 按MRV(候选最少)排序。
        # 注意: 每片都是分支点, 不能只扩展"最少候选"的那一片——
        # 某片在当前上下文可能全是伪匹配(正确搭档还没放置),
        # 只扩展它会让搜索不完备, 必须能回溯改放其他片。
        piece_moves = []
        for pi in range(n):
            if pi in placed:
                continue
            moves = []  # (匹配边长, R, t)
            for cand in candidates:
                if cand['short'] == pi and cand['long'] in placed:
                    for T in self._snap_transforms(
                            cand, pieces, placed[cand['long']]):
                        moves.append((cand['len_s'], T[0], T[1]))
                elif cand['long'] == pi and cand['short'] in placed:
                    # 反向: pi持长边, 已放置短边同样把pi约束到离散位姿
                    for T in self._snap_long_transforms(
                            cand, pieces, placed[cand['short']]):
                        moves.append((cand['len_s'], T[0], T[1]))
            # 去重 (角度+平移量化), 按匹配边长降序(长公共边更可能是正确拼接)
            uniq = []
            seen = set()
            moves.sort(key=lambda m: -m[0])
            for mlen, R, t in moves:
                key = (round(np.arctan2(R[1, 0], R[0, 0]), 3),
                       round(t[0], 1), round(t[1], 1))
                if key not in seen:
                    seen.add(key)
                    uniq.append((R, t))
            if uniq:
                piece_moves.append((pi, uniq))
        piece_moves.sort(key=lambda pm: len(pm[1]))

        if not piece_moves:
            self.stats['dead_end'] += 1
            return None  # 无碎片能接上, 死路

        placed_verts = {k: apply_transform(v, pieces[k])
                        for k, v in placed.items()}
        for pi, moves in piece_moves:
            # 该片的画布: 已放置集合光栅化一次, 其所有move复用
            pv_list = list(placed_verts.values())
            acc_er, lo, s = self._make_canvas(pv_list, self._diameters[pi])
            tol_mm2 = max(OVERLAP_TOL_RATIO *
                          abs(signed_area(pieces[pi])), 15.0)
            for T in moves:
                verts_new = apply_transform(T, pieces[pi])
                ov = self._overlap_mm2(acc_er, lo, s, verts_new)
                if ov > tol_mm2:
                    # 适度重叠: 检测角度误差(长边1-3°=末端数mm)会误杀
                    # 正确拼法, 在邻域内松弛位姿再判
                    relax_cap = max(tol_mm2 * 4.0, 120.0)
                    if ov < relax_cap:
                        rr = self._relax_pose(pieces[pi], T, acc_er,
                                              lo, s, tol_mm2)
                        if rr is not None:
                            T, ov = rr[0], rr[1]
                            verts_new = apply_transform(T, pieces[pi])
                        else:
                            self.stats['overlap'] += 1
                            continue
                    else:
                        self.stats['overlap'] += 1
                        continue
                if not self._fits_in_final_rect(
                        list(placed_verts.values()) + [verts_new]):
                    self.stats['fit'] += 1
                    continue  # 已放置集合的最小外接矩形超总面积 -> 不可能成矩形
                placed[pi] = T
                self._dfs(pieces, placed, candidates)
                del placed[pi]
                if self._states > self.max_states or \
                        time.time() - self._t0 > DFS_TIME_LIMIT_S or \
                        len(self._solutions) >= 6 or \
                        (self._solutions and
                         min(s['score'] for s in self._solutions) < 2.0):
                    return
        return

    # ---------- 重叠检查 (光栅化) ----------

    def _make_canvas(self, placed_verts_list, margin_mm):
        """已放置集合的光栅画布 (已腐蚀), 供同一节点多个move复用"""
        s = RASTER_PX_PER_MM
        lo = np.min([v.min(axis=0) for v in placed_verts_list],
                    axis=0) - margin_mm - 2
        hi = np.max([v.max(axis=0) for v in placed_verts_list],
                    axis=0) + margin_mm + 2
        W = int((hi[0] - lo[0]) * s) + 1
        H = int((hi[1] - lo[1]) * s) + 1
        if W * H > 4_000_000:
            s = max(1.0, np.sqrt(4_000_000 / ((hi[0] - lo[0]) *
                                              (hi[1] - lo[1]))))
            W = int((hi[0] - lo[0]) * s) + 1
            H = int((hi[1] - lo[1]) * s) + 1
        acc = np.zeros((H, W), np.uint8)
        for v in placed_verts_list:
            m = np.zeros((H, W), np.uint8)
            pts = ((v - lo) * s).astype(np.int32)
            cv2.fillPoly(m, [pts], 1)
            acc = cv2.add(acc, m)
        kernel = np.ones((3, 3), np.uint8)
        return cv2.erode(acc, kernel), lo, s

    def _overlap_on_canvas(self, acc_er, lo, s, verts_new, tol_mm2):
        """在预制画布上检查新碎片是否与已放置集合重叠"""
        return self._overlap_mm2(acc_er, lo, s, verts_new) > tol_mm2

    def _overlap_mm2(self, acc_er, lo, s, verts_new):
        """返回新碎片与已放置集合的重叠面积 mm² (双方各腐蚀1px后)"""
        H, W = acc_er.shape
        # 快速路径: 新碎片包围盒与画布不交 -> 必无重叠
        pts = ((verts_new - lo) * s).astype(np.int32)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        pad = 3
        if x1 < -pad or y1 < -pad or x0 >= W + pad or y0 >= H + pad:
            return 0.0
        # 只在新碎片包围盒区域内光栅化 (裁切, 避免整画布操作)
        cx0, cy0 = max(x0 - pad, 0), max(y0 - pad, 0)
        cx1, cy1 = min(x1 + pad + 1, W), min(y1 + pad + 1, H)
        sub_acc = acc_er[cy0:cy1, cx0:cx1]
        if sub_acc.size == 0 or not np.any(sub_acc):
            return 0.0
        m_new = np.zeros(sub_acc.shape, np.uint8)
        cv2.fillPoly(m_new, [pts - [cx0, cy0]], 1)
        # 各向内腐蚀1px再求交: 正确拼接时公共边两侧像素天然有1-2px重叠带
        kernel = np.ones((3, 3), np.uint8)
        new_er = cv2.erode(m_new, kernel)
        overlap_px = int(np.count_nonzero((sub_acc > 0) & (new_er > 0)))
        return overlap_px / (s * s)

    def _relax_pose(self, piece_verts, T, acc_er, lo, s, tol_mm2):
        """
        位姿松弛: 咬合位姿因检测角度误差(长边上几度误差=末端数mm重叠)
        被重叠检查误杀时, 在邻域内采样微调 (绕质心旋转±4°, 平移±3mm),
        取重叠最小且 <tol 的位姿。返回 (T_best, overlap_mm2) 或 None。
        """
        R, t = T
        verts0 = apply_transform(T, piece_verts)
        ov0 = self._overlap_mm2(acc_er, lo, s, verts0)
        if ov0 <= tol_mm2:
            return (R, t), ov0
        center = verts0.mean(axis=0)
        best = (ov0, None, None)

        def try_pose(da_deg, dt):
            nonlocal best
            dR, _ = make_transform(np.radians(da_deg), [0, 0])
            R2 = dR @ R
            t2 = dR @ (t - center) + center + np.asarray(dt, dtype=float)
            ov = self._overlap_mm2(acc_er, lo, s,
                                   apply_transform((R2, t2), piece_verts))
            if ov < best[0]:
                best = (ov, R2, t2)

        # 先纯平移, 再旋转+小平移 (从粗到细)
        for dt in ((0, 0), (1.5, 0), (-1.5, 0), (0, 1.5), (0, -1.5),
                   (3, 0), (-3, 0), (0, 3), (0, -3)):
            try_pose(0.0, dt)
            if best[0] <= tol_mm2:
                return (best[1], best[2]), best[0]
        for da in (-4.0, -2.0, 2.0, 4.0):
            for dt in ((0, 0), (1.5, 0), (-1.5, 0), (0, 1.5), (0, -1.5)):
                try_pose(da, dt)
                if best[0] <= tol_mm2:
                    return (best[1], best[2]), best[0]
        if best[1] is None or best[0] > tol_mm2:
            return None
        return (best[1], best[2]), best[0]

    def _fits_in_final_rect(self, verts_list):
        """
        强剪枝: 已放置碎片位姿已固定, 最终必须拼成面积≈总面积的矩形,
        所以它们的最小外接矩形(任意朝向)面积不能超预算。
        预算 = 总面积 × (1 + RECT_AREA_TOL + self.fit_margin)
        fit_margin 补偿检测侵蚀: 碎片轮廓内缩时, 拼合边界框比
        碎片面积和大出 ~周长×内缩量, 暗光噪声场景需 0.15-0.25。
        """
        if len(verts_list) < 2:
            return True
        pts = np.vstack(verts_list).astype(np.float32)
        (_, _), (w, h), _ = cv2.minAreaRect(pts)
        budget = self._total_area * (1.0 + RECT_AREA_TOL +
                                     getattr(self, 'fit_margin', 0.0))
        return w * h <= budget

    # ---------- 矩形验证 ----------

    def _validate_rectangle(self, pieces, transforms):
        """
        边界边减法提取外轮廓 -> 须拼成矩形。
        返回 (4,2) 角点 或 None。处理 T 型接缝:
        一条边被多条短边部分覆盖时, 只把未覆盖部分算作外轮廓。
        """
        tol = self.edge_tol
        placed_edges = []  # (A, B, piece_idx)
        for pi, T in enumerate(transforms):
            for A, B in edge_list(apply_transform(T, pieces[pi])):
                placed_edges.append((A, B, pi))

        # 1. 边界提取: 每条边减去被其他碎片边覆盖的子区间
        boundary_segs = []  # (P0, P1)
        for A, B, pi in placed_edges:
            d = B - A
            L = np.linalg.norm(d)
            if L < 1e-6:
                continue
            u = d / L
            covered = []
            for C, D, pj in placed_edges:
                if pj == pi:
                    continue
                v = D - C
                Lv = np.linalg.norm(v)
                if Lv < 1e-6:
                    continue
                # 共线检查: 方向平行 且 到直线AB距离小
                if abs(_cross2(u, v)) / Lv > 0.06:
                    continue
                if abs(_cross2(u, C - A)) > tol:
                    continue
                t0 = np.dot(C - A, u) / L
                t1 = np.dot(D - A, u) / L
                lo_t, hi_t = min(t0, t1), max(t0, t1)
                lo_t, hi_t = max(lo_t, 0.0), min(hi_t, 1.0)
                if hi_t - lo_t > (1.0 / L):  # 覆盖长度>1mm才算
                    covered.append((lo_t, hi_t))
            # 区间并集 -> 补集
            covered.sort()
            merged = []
            for lo_t, hi_t in covered:
                if merged and lo_t <= merged[-1][1] + 1e-9:
                    merged[-1][1] = max(merged[-1][1], hi_t)
                else:
                    merged.append([lo_t, hi_t])
            cur = 0.0
            min_seg = 2.0 / L  # 忽略<2mm的碎段
            for lo_t, hi_t in merged:
                if lo_t - cur > min_seg:
                    boundary_segs.append((A + u * (cur * L),
                                          A + u * (lo_t * L)))
                cur = max(cur, hi_t)
            if 1.0 - cur > min_seg:
                boundary_segs.append((A + u * (cur * L), B))

        if len(boundary_segs) < 4:
            return None

        # 2. 端点吸附建图, 走外环
        # 吸附半径随匹配容差自适应: 松弛位姿/边长误差会在接缝端点
        # 留下数mm缝隙, 固定2mm会把真闭环打断
        snap = min(max(2.0, self.edge_tol * 0.6), 5.0)
        pts = []
        for P0, P1 in boundary_segs:
            pts.append(P0)
            pts.append(P1)
        pts = np.array(pts)
        # 并查集合并相近端点
        parent = list(range(len(pts)))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                if np.linalg.norm(pts[a] - pts[b]) < snap:
                    ra, rb = find(a), find(b)
                    if ra != rb:
                        parent[ra] = rb

        # 线段 -> 规范端点id
        adj = {}  # node -> [(node, P_other)]
        node_pos = {}
        for P0, P1 in boundary_segs:
            pass  # 端点id用find索引, 位置取簇均值, 下面统一处理
        # 建立簇位置
        clusters = {}
        for idx, p in enumerate(pts):
            clusters.setdefault(find(idx), []).append(p)
        for root, ps in clusters.items():
            node_pos[root] = np.mean(ps, axis=0)

        for si, (P0, P1) in enumerate(boundary_segs):
            n0, n1 = find(2 * si), find(2 * si + 1)
            if n0 == n1:
                continue
            adj.setdefault(n0, []).append(n1)
            adj.setdefault(n1, []).append(n0)

        # 每个节点度数须为2 (单一闭环)
        if not adj or any(len(v) != 2 for v in adj.values()):
            return None

        # 3. 遍历环
        start = next(iter(adj))
        loop = [start]
        prev, cur = None, start
        while True:
            nxts = [x for x in adj[cur] if x != prev]
            if not nxts:
                return None
            nxt = nxts[0]
            if nxt == start:
                break
            if nxt in loop:
                return None
            loop.append(nxt)
            prev, cur = cur, nxt
            if len(loop) > len(adj) + 1:
                return None
        if len(loop) < 4:
            return None

        poly = np.array([node_pos[k] for k in loop])

        # 4. 合并共线连续段 -> 角点
        corners = self._merge_collinear(poly)
        if corners is None or len(corners) != 4:
            return None

        # 5. 矩形性度量 (无论是否达标都算出评分, 供近似解择优)
        c = corners
        angs = []
        sides = []
        for k in range(4):
            v1 = c[k - 1] - c[k]
            v2 = c[(k + 1) % 4] - c[k]
            cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) *
                                      np.linalg.norm(v2) + 1e-9)
            angs.append(np.degrees(np.arccos(np.clip(cos_a, -1, 1))))
            sides.append(np.linalg.norm(v2))
        W = (sides[0] + sides[2]) / 2
        H = (sides[1] + sides[3]) / 2
        total_area = sum(abs(signed_area(p)) for p in pieces)
        area_err = abs(W * H - total_area) / total_area

        # 评分: 越小越接近完美矩形 (用于多解择优/近似解兜底)
        score = (sum(abs(a - 90) for a in angs) +
                 abs(sides[0] - sides[2]) + abs(sides[1] - sides[3]) +
                 area_err * 100.0)

        # 严格矩形检查
        if any(abs(a - 90) > RECT_ANGLE_TOL_DEG for a in angs):
            return None
        if abs(sides[0] - sides[2]) > RECT_SIDE_TOL_MM or \
           abs(sides[1] - sides[3]) > RECT_SIDE_TOL_MM:
            return None
        if not (RECT_DIM_RANGE[0] <= W <= RECT_DIM_RANGE[1] and
                RECT_DIM_RANGE[0] <= H <= RECT_DIM_RANGE[1]):
            return None
        if area_err > RECT_AREA_TOL:
            return None
        return corners, score, W, H

    def _rect_score(self, pieces, transforms):
        """
        近似解评分: 复用 _validate_rectangle 的边界提取/走环逻辑,
        但不卡矩形阈值 — 只要能走出4角闭环就返回 (corners, score, W, H)。
        结构性失败(无闭环/非4角)返回None。
        """
        global RECT_ANGLE_TOL_DEG, RECT_SIDE_TOL_MM, RECT_AREA_TOL, \
            RECT_DIM_RANGE
        saved = (RECT_ANGLE_TOL_DEG, RECT_SIDE_TOL_MM, RECT_AREA_TOL,
                 RECT_DIM_RANGE)
        RECT_ANGLE_TOL_DEG = 1e9
        RECT_SIDE_TOL_MM = 1e9
        RECT_AREA_TOL = 1e9
        RECT_DIM_RANGE = (0.0, 1e9)
        try:
            return self._validate_rectangle(pieces, transforms)
        finally:
            (RECT_ANGLE_TOL_DEG, RECT_SIDE_TOL_MM, RECT_AREA_TOL,
             RECT_DIM_RANGE) = saved

    def _merge_collinear(self, poly, ang_tol_deg=8.0):
        """合并近共线的连续顶点, 返回角点列表"""
        pts = list(poly)
        for _ in range(len(pts) + 2):
            n = len(pts)
            if n < 3:
                return None
            merged = False
            for i in range(n):
                p_prev = pts[(i - 1) % n]
                p_cur = pts[i]
                p_next = pts[(i + 1) % n]
                v1 = p_cur - p_prev
                v2 = p_next - p_cur
                l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
                if l1 < 1e-6 or l2 < 1e-6:
                    pts.pop(i)
                    merged = True
                    break
                sin_a = abs(_cross2(v1, v2)) / (l1 * l2)
                if sin_a < np.sin(np.radians(ang_tol_deg)):
                    pts.pop(i)
                    merged = True
                    break
            if not merged:
                break
        return np.array(pts)

    # ---------- 结果封装 ----------

    def _make_solution(self, pieces, transforms, corners):
        placed = [apply_transform(T, p) for p, T in zip(pieces, transforms)]
        sides = [np.linalg.norm(corners[(k + 1) % 4] - corners[k])
                 for k in range(4)]
        W = (sides[0] + sides[2]) / 2
        H = (sides[1] + sides[3]) / 2
        return {
            'transforms': transforms,
            'placed_verts': placed,
            'rect_corners': corners,
            'rect_size': (W, H),
        }


# ==================== 自测 ====================

def _gen_guillotine_pieces(W=100.0, H=60.0, n_pieces=4, seed=0):
    """把 W x H 矩形随机 guillotine 切割成 n_pieces 片 (含T型接缝)"""
    rng = np.random.RandomState(seed)
    rects = [(np.array([0.0, 0.0]), np.array([W, H]))]
    while len(rects) < n_pieces:
        # 选最大的切
        idx = max(range(len(rects)),
                  key=lambda i: (rects[i][1] - rects[i][0]).prod())
        lo, hi = rects.pop(idx)
        size = hi - lo
        if size[0] >= size[1]:
            x = lo[0] + rng.uniform(0.3, 0.7) * size[0]
            rects.append((lo, np.array([x, hi[1]])))
            rects.append((np.array([x, lo[1]]), hi))
        else:
            y = lo[1] + rng.uniform(0.3, 0.7) * size[1]
            rects.append((lo, np.array([hi[0], y])))
            rects.append((np.array([lo[0], y]), hi))
    pieces = []
    for lo, hi in rects:
        pieces.append(np.array([
            [lo[0], lo[1]], [hi[0], lo[1]],
            [hi[0], hi[1]], [lo[0], hi[1]]]))
    return pieces, (W, H)


def _scatter(pieces, seed=1):
    """随机旋转+平移打乱"""
    rng = np.random.RandomState(seed)
    out = []
    for p in pieces:
        ang = rng.uniform(0, 2 * np.pi)
        R, _ = make_transform(ang, [0, 0])
        t = rng.uniform(-200, 200, 2)
        out.append(ensure_ccw(p @ R.T + t))
    return out


