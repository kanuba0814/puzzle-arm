"""
拼图算法模块

功能:
1. 第1项: 已知碎片形状 -> 预计算目标位置
2. 第2项: 未知碎片 -> 边长匹配 -> 拼接规划
3. 放置顺序优化 (避免碰撞)
4. 生成拾取-放置任务序列
"""
import numpy as np
from config import *


class PuzzleSolver:
    def __init__(self):
        self.target_positions = []
        self.placement_order = []

    # ==================== 第1项: 自备碎片 ====================

    def solve_task1(self, detected_pieces):
        """
        第1项拼图求解
        碎片形状已知 (图2: 4片, 拼成8cm×8cm矩形)
        每片尺寸: 2cm×8cm, 带切口

        参数:
            detected_pieces: vision模块检测到的碎片列表

        返回:
            tasks: [(pick_pos, pick_angle, place_pos, place_angle), ...]
        """
        # 预定义目标位置: 4片拼成矩形
        # 目标矩形: 8cm × 8cm (4片2cm×8cm竖排)
        # 布局: 4片竖向并排
        target_layout = self._generate_task1_targets()

        print(f"[Solver] 第1项: {len(detected_pieces)}个碎片, "
              f"{len(target_layout)}个目标位置")

        # 匹配: 每个检测到的碎片分配一个目标位置
        # 碎片颜色一致, 可以任意分配, 优化路径
        tasks = self._assign_pieces_optimal(
            detected_pieces, target_layout)

        return tasks

    def _generate_task1_targets(self):
        """
        生成第1项的目标放置位置
        4片2cm×8cm碎片, 拼成8cm×8cm矩形

        布局 (俯视, 世界坐标mm):
            ┌──┬──┬──┬──┐
            │ 1│ 2│ 3│ 4│  每片宽20mm, 高80mm
            └──┴──┴──┴──┘
            总宽80mm, 总高80mm

        目标中心 = (TARGET_CENTER_X - 40 + 10 + i*20, TARGET_CENTER_Y)
        """
        targets = []
        piece_w = PIECE_WIDTH   # 20mm
        piece_h = PIECE_LENGTH  # 80mm
        total_w = piece_w * 4   # 80mm

        start_x = TARGET_CENTER_X - total_w / 2 + piece_w / 2

        for i in range(4):
            targets.append({
                'center': (start_x + i * piece_w, TARGET_CENTER_Y),
                'angle': 0,  # 竖直放置
                'index': i,
            })
        return targets

    # ==================== 第2项: 现场碎片 ====================

    def solve_task2(self, detected_pieces):
        """
        第2项拼图求解
        碎片形状未知, 需要根据边长匹配拼接

        算法:
        1. 提取每个碎片的边长列表
        2. 匹配等长边 (相邻碎片的公共边)
        3. 用约束满足搜索拼接方案
        4. 计算每个碎片的绝对位置和角度

        参数:
            detected_pieces: vision模块检测到的碎片(含vertices和edge_lengths)

        返回:
            tasks: [(pick_pos, pick_angle, place_pos, place_angle), ...]
        """
        if len(detected_pieces) > MAX_PIECES:
            print(f"[Solver] 警告: 碎片数{len(detected_pieces)} > {MAX_PIECES}")

        print(f"[Solver] 第2项: {len(detected_pieces)}个碎片")

        # Step 1: 规范化边长数据
        pieces_data = []
        for i, piece in enumerate(detected_pieces):
            edges = piece.get('edge_lengths', [])
            vertices = piece.get('vertices', [])
            if len(edges) < 3 or len(vertices) < 3:
                print(f"[Solver] 碎片{i}: 顶点不足, 跳过")
                continue

            pieces_data.append({
                'id': i,
                'edges': sorted(edges),  # 排序便于匹配
                'edges_raw': edges,       # 原始顺序
                'vertices': vertices,
                'num_vertices': len(vertices),
                'center': piece['center_world'],
                'angle': piece['angle'],
            })
            print(f"  碎片{i}: {len(edges)}条边, "
                  f"边长={[f'{e:.1f}' for e in edges]}mm")

        if len(pieces_data) < 2:
            print("[Solver] 有效碎片不足")
            return []

        # Step 2: 边长匹配 - 找公共边
        matches = self._find_edge_matches(pieces_data)
        print(f"[Solver] 找到 {len(matches)} 对匹配边")

        # Step 3: 搜索拼接方案
        solution = self._search_puzzle_layout(pieces_data, matches)
        if solution is None:
            print("[Solver] 未找到可行拼接方案")
            return []

        # Step 4: 生成任务
        tasks = self._generate_placement_tasks(
            detected_pieces, solution)
        return tasks

    def _find_edge_matches(self, pieces_data, tolerance=2.0):
        """
        找到等长的边对 (公共边候选)
        tolerance: 边长匹配容差(mm)
        """
        matches = []
        for i, p1 in enumerate(pieces_data):
            for j, p2 in enumerate(pieces_data):
                if j <= i:
                    continue
                for ei, e1 in enumerate(p1['edges_raw']):
                    for ej, e2 in enumerate(p2['edges_raw']):
                        if abs(e1 - e2) <= tolerance:
                            matches.append({
                                'piece_a': p1['id'],
                                'piece_b': p2['id'],
                                'edge_a': ei,
                                'edge_b': ej,
                                'length': (e1 + e2) / 2,
                            })
        return matches

    def _search_puzzle_layout(self, pieces_data, matches):
        """
        搜索拼接布局: 使用回溯搜索
        约束:
        - 每片至少1条边在目标矩形外边上
        - 相邻碎片共享等长边
        - 碎片不重叠

        返回: {piece_id: {'pos': (x,y), 'angle': deg, 'flipped': bool}}
        """
        # 简化策略: 第一个碎片放置在目标矩形左下角
        # 依次尝试匹配的碎片

        n = len(pieces_data)
        placed = {}  # {piece_id: layout}
        used_matches = set()

        def try_place(piece_id, pos, angle, depth):
            if depth == n:
                return True

            placed[piece_id] = {'pos': pos, 'angle': angle, 'flipped': False}

            # 找与当前碎片匹配的未放置碎片
            for m in matches:
                other = None
                if m['piece_a'] == piece_id and m['piece_b'] not in placed:
                    other = m['piece_b']
                    other_edge = m['edge_b']
                    my_edge = m['edge_a']
                elif m['piece_b'] == piece_id and m['piece_a'] not in placed:
                    other = m['piece_a']
                    other_edge = m['edge_a']
                    my_edge = m['edge_b']

                if other is None:
                    continue

                # 计算另一碎片的位置 (基于共享边)
                # 简化: 放在当前碎片的右侧或上方
                new_pos = (pos[0] + 30, pos[1])  # 简化估算
                new_angle = -angle  # 镜像角度

                if try_place(other, new_pos, new_angle, depth + 1):
                    return True

            # 回溯
            if len(placed) == depth + 1:
                placed.pop(piece_id)
            return False

        # 从第一个碎片开始
        start_pos = (TARGET_CENTER_X - 30, TARGET_CENTER_Y - 20)
        if try_place(pieces_data[0]['id'], start_pos, 0, 0):
            return placed
        return None

    # ==================== 任务分配与路径优化 ====================

    def _assign_pieces_optimal(self, pieces, targets):
        """
        将检测到的碎片分配到目标位置
        使用贪心算法: 每次选择最近的碎片-目标对

        返回: [(piece, target), ...] 按执行顺序
        """
        available_pieces = list(range(len(pieces)))
        available_targets = list(range(len(targets)))
        assignments = []

        while available_pieces and available_targets:
            best_dist = float('inf')
            best_pair = None

            for pi in available_pieces:
                for ti in available_targets:
                    px, py = pieces[pi]['center_world']
                    tx, ty = targets[ti]['center']
                    dist = np.sqrt((px - tx)**2 + (py - ty)**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_pair = (pi, ti)

            if best_pair is None:
                break

            pi, ti = best_pair
            assignments.append((pieces[pi], targets[ti]))
            available_pieces.remove(pi)
            available_targets.remove(ti)

        # 生成任务序列
        tasks = []
        for piece, target in assignments:
            tasks.append({
                'pick_pos': piece['center_world'],
                'pick_angle': piece['angle'],
                'place_pos': target['center'],
                'place_angle': target['angle'],
            })

        return tasks

    def _generate_placement_tasks(self, detected_pieces, solution):
        """为第2项生成放置任务"""
        tasks = []
        for piece_id, layout in solution.items():
            piece = detected_pieces[piece_id]
            tasks.append({
                'pick_pos': piece['center_world'],
                'pick_angle': piece['angle'],
                'place_pos': layout['pos'],
                'place_angle': layout['angle'],
            })
        return tasks

    # ==================== 碰撞检查 ====================

    def check_collision(self, pos, placed_positions, min_dist=30):
        """检查位置是否与已放置碎片碰撞"""
        for p in placed_positions:
            dist = np.sqrt((pos[0] - p[0])**2 + (pos[1] - p[1])**2)
            if dist < min_dist:
                return True
        return False

    # ==================== 放置顺序优化 ====================

    def optimize_order(self, tasks, arm_home=(0, -100)):
        """
        优化放置顺序: 最小化总移动距离
        贪心: 每次从当前位置选最近的碎片
        """
        if not tasks:
            return []

        remaining = list(range(len(tasks)))
        ordered = []
        current = arm_home  # 从初始位置开始

        while remaining:
            best_dist = float('inf')
            best_idx = None
            for idx in remaining:
                px, py = tasks[idx]['pick_pos']
                dist = np.sqrt((px - current[0])**2 + (py - current[1])**2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            ordered.append(tasks[best_idx])
            current = tasks[best_idx]['place_pos']
            remaining.remove(best_idx)

        return ordered


if __name__ == '__main__':
    solver = PuzzleSolver()

    # 测试第1项
    print("=== 第1项: 自备碎片测试 ===")
    mock_pieces = [
        {'center_world': (50, 30), 'angle': 45},
        {'center_world': (150, 20), 'angle': 90},
        {'center_world': (80, 100), 'angle': 0},
        {'center_world': (180, 80), 'angle': 30},
    ]
    tasks = solver.solve_task1(mock_pieces)
    tasks = solver.optimize_order(tasks)
    print(f"\n生成 {len(tasks)} 个任务:")
    for i, t in enumerate(tasks):
        print(f"  任务{i}: 拾取({t['pick_pos'][0]:.0f},{t['pick_pos'][1]:.0f}) "
              f"{t['pick_angle']:.0f}° -> "
              f"放置({t['place_pos'][0]:.0f},{t['place_pos'][1]:.0f}) "
              f"{t['place_angle']:.0f}°")

    # 测试第2项
    print("\n=== 第2项: 现场碎片测试 ===")
    mock_pieces2 = [
        {
            'center_world': (50, 30), 'angle': 0,
            'vertices': [(0, 0), (30, 0), (30, 20), (0, 20)],
            'edge_lengths': [30, 20, 30, 20],
        },
        {
            'center_world': (150, 30), 'angle': 0,
            'vertices': [(0, 0), (30, 0), (30, 20), (0, 20)],
            'edge_lengths': [30, 20, 30, 20],
        },
    ]
    tasks2 = solver.solve_task2(mock_pieces2)
    print(f"\n生成 {len(tasks2)} 个任务")
