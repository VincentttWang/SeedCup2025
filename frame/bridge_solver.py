import math

# 配置参数
MAX_BEAM_LENGTH = 8.0
OPTIMAL_TRUSS_HEIGHT = 5.0  # 理想的桁架高度
WALKABLE_ANGLE_THRESHOLD = 35.0  # 可通行路面的最大倾角
GRID_STEP = 2.0  # 寻路网格精度

class BridgeSolver:
    def __init__(self, map_data: dict):
        """接收地图数据字典并准备求解。"""
        self.map_data = map_data
        self.nodes = {n['id']: n for n in self.map_data['nodes']}
        self.pinned_nodes = [n for n in self.map_data['nodes'] if n.get('pinned', False)]
        
        # 关键特征点
        self.start_platform_nodes = []
        self.start_anchors = []
        self.start_main_anchor = None
        self.end_platform_nodes = []
        self.end_anchor = None
        self.end_attach_node = None
        self.end_right_anchor = None
        self.walkable_road_ends = set()
        self.walkable_road_nodes = []
        self.support_nodes = []
        self.attractor_nodes = []
        
        self._analyze_map()

    def _analyze_map(self):
        """分析地图特征：识别起点、终点、可用路面、支撑点"""
        print("正在分析地图特征...")
        
        # 1. 识别出发平台
        self.start_platform_nodes = self._find_start_platform_nodes()
        if self.start_platform_nodes:
            anchors_by_y = {}
            for node in self.start_platform_nodes:
                y = node['y']
                if y not in anchors_by_y or node['x'] > anchors_by_y[y]['x']:
                    anchors_by_y[y] = node

            self.start_anchors = [anchors_by_y[y] for y in sorted(anchors_by_y.keys())]
            self.start_main_anchor = max(self.start_anchors, key=lambda n: (n['y'], n['x']))

            for anchor in self.start_anchors:
                print(f"  -> 出发平台层 y={anchor['y']} 末端: Node {anchor['id']} ({anchor['x']}, {anchor['y']})")
            anchor_desc = ", ".join(
                [
                    f"Node {anchor['id']} ({anchor['x']}, {anchor['y']})"
                    for anchor in self.start_anchors
                ]
            )
            print(f"  -> 出发主锚点: {anchor_desc}")
        else:
            print("  -> 未找到出发平台，后续结果可能为空。")

        # 2. 识别结束平台
        self.end_platform_nodes = self._find_end_platform_nodes()
        if self.end_platform_nodes:
            self.end_anchor = self.end_platform_nodes[0]
            self.end_attach_node = self.end_platform_nodes[0]
            self.end_right_anchor = self.end_platform_nodes[-1]
            anchor_desc = ", ".join(
                [
                    f"Node {node['id']} ({node['x']}, {node['y']})"
                    for node in self.end_platform_nodes
                ]
            )
            print(f"  -> 结束平台节点: {anchor_desc}")
            print(
                f"  -> 结束平台锚点(左端): Node {self.end_anchor['id']} ({self.end_anchor['x']}, {self.end_anchor['y']})"
            )
        else:
            print("  -> 未找到结束平台。")

        # 3. 识别已有路面 (Existing Roads)
        self.walkable_road_ends.clear()
        road_bridges = self.map_data.get('bridges', {}).get('road', [])
        for road in road_bridges:
            p0 = self.nodes[road['p0']]
            p1 = self.nodes[road['p1']]
            dx = abs(p1['x'] - p0['x'])
            dy = abs(p1['y'] - p0['y'])
            angle = math.degrees(math.atan2(dy, dx))
            
            if angle < WALKABLE_ANGLE_THRESHOLD:
                # 可通行路面
                self.walkable_road_ends.add(road['p0'])
                self.walkable_road_ends.add(road['p1'])
                print(f"  -> 发现可利用路面: Node {road['p0']} - {road['p1']} (Angle: {angle:.1f}°)")
            else:
                # 陡峭路面，仅作支撑
                pass

        self.walkable_road_nodes = [self.nodes[nid] for nid in self.walkable_road_ends]

        # 4. 归类支撑点
        exclude_ids = {n['id'] for n in self.walkable_road_nodes}
        exclude_ids.update(n['id'] for n in self.start_platform_nodes)
        exclude_ids.update(n['id'] for n in self.end_platform_nodes)
        self.support_nodes = [n for n in self.pinned_nodes if n['id'] not in exclude_ids]
        print(f"  -> 识别到 {len(self.support_nodes)} 个支撑点用于桁架连接。")
        support_desc = [f"Node {n['id']} ({n['x']}, {n['y']})" for n in self.support_nodes]
        print(f"    支撑点列表: {support_desc}")

        # 5. 汇总吸引目标
        attractor_index = {}
        for node in self.start_anchors:
            attractor_index[node['id']] = node
        for node in self.end_platform_nodes:
            attractor_index[node['id']] = node
        for node in self.walkable_road_nodes:
            attractor_index[node['id']] = node
        self.attractor_nodes = list(attractor_index.values())

        print("地图分析完成。")

    def _find_start_platform_nodes(self):
        vehicles = []
        for entries in self.map_data.get('vehicles', {}).values():
            vehicles.extend(entries)
        if not vehicles:
            return []

        min_vehicle_x = min(v['x'] for v in vehicles)
        max_vehicle_x = max(v['x'] for v in vehicles)

        candidate_layers = set()
        for vehicle in vehicles:
            for offset in (1, 2):
                candidate_layers.add(vehicle['y'] + offset)

        collected_ids = set()
        bridges = []
        for material in ('road', 'steel'):
            bridges.extend(self.map_data.get('bridges', {}).get(material, []))

        for target_y in sorted(candidate_layers):
            print(f"  -> 检查候选起点层 Y={target_y}...")
            seeds = [
                n for n in self.pinned_nodes
                if n['y'] == target_y and min_vehicle_x - 5 <= n['x'] <= max_vehicle_x + 5
            ]
            if not seeds:
                continue

            layer_ids = {node['id'] for node in seeds}
            expanded = True
            while expanded:
                expanded = False
                for bridge in bridges:
                    if bridge['p0'] in layer_ids and bridge['p1'] not in layer_ids:
                        node = self.nodes[bridge['p1']]
                        if node['y'] == target_y:
                            layer_ids.add(bridge['p1'])
                            expanded = True
                    elif bridge['p1'] in layer_ids and bridge['p0'] not in layer_ids:
                        node = self.nodes[bridge['p0']]
                        if node['y'] == target_y:
                            layer_ids.add(bridge['p0'])
                            expanded = True

            collected_ids.update(layer_ids)
        # print(f"    收集到 {len(collected_ids)} 个起点层节点，坐标分别是：{[self.nodes[nid] for nid in collected_ids]}。")

        if not collected_ids:
            # 回退到旧策略避免空结果
            candidates = [n for n in self.pinned_nodes if n['x'] < -10]
            if not candidates:
                return []
            layers = {}
            for node in candidates:
                layers.setdefault(node['y'], []).append(node)
            # 选择包含节点最多且 x 最大的那层
            best_layer = max(layers.values(), key=lambda nodes: (len(nodes), max(n['x'] for n in nodes)))
            collected_ids = {node['id'] for node in best_layer}

        unique_nodes = [self.nodes[nid] for nid in collected_ids]

        return sorted(unique_nodes, key=lambda n: (n['y'], n['x']))

    def _find_end_platform_nodes(self):
        if not self.pinned_nodes:
            return []

        rightmost = max(self.pinned_nodes, key=lambda n: n['x'])
        same_row = [n for n in self.pinned_nodes if n['y'] == rightmost['y']]
        return sorted(same_row, key=lambda n: n['x'])

    def _within_bounds(self, x: float, y: float) -> bool:
        bounds = self.map_data.get('grid_bounds', {})
        return (
            bounds.get('min_x', float('-inf')) <= x <= bounds.get('max_x', float('inf'))
            and bounds.get('min_y', float('-inf')) <= y <= bounds.get('max_y', float('inf'))
        )

    def _distance(self, p1, p2) -> float:
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _emit_segment(self, start, end, material, add_edge):
        """将起点到终点的连线切分为合法长度的杆件。"""
        start_f = (float(start[0]), float(start[1]))
        end_f = (float(end[0]), float(end[1]))

        if start_f == end_f:
            return

        segments = 1
        max_checks = 16

        while segments <= max_checks:
            points = []
            for i in range(segments + 1):
                t = i / segments
                x = start_f[0] + (end_f[0] - start_f[0]) * t
                y = start_f[1] + (end_f[1] - start_f[1]) * t
                points.append([int(round(x)), int(round(y))])

            filtered = [points[0]]
            for pt in points[1:]:
                if pt != filtered[-1]:
                    filtered.append(pt)

            if len(filtered) < 2:
                return

            lengths = [self._distance(filtered[i], filtered[i + 1]) for i in range(len(filtered) - 1)]
            if lengths and max(lengths) > MAX_BEAM_LENGTH + 1e-6:
                segments += 1
                continue

            for i in range(len(filtered) - 1):
                add_edge(filtered[i], filtered[i + 1], material)
            return

        # 兜底：使用等距步长（小于 1 单位）逐段逼近
        step = MAX_BEAM_LENGTH / 4.0
        total_len = self._distance(start_f, end_f)
        steps = max(2, int(math.ceil(total_len / step)))
        prev = [int(round(start_f[0])), int(round(start_f[1]))]
        for i in range(1, steps):
            t = i / steps
            x = start_f[0] + (end_f[0] - start_f[0]) * t
            y = start_f[1] + (end_f[1] - start_f[1]) * t
            current = [int(round(x)), int(round(y))]
            if current != prev:
                add_edge(prev, current, material)
                prev = current
        final_point = [int(round(end_f[0])), int(round(end_f[1]))]
        if final_point != prev:
            add_edge(prev, final_point, material)

    def _select_next_point(self, current, target):
        cx, cy = current
        best_candidate = None
        best_score = float('inf')

        radii = [MAX_BEAM_LENGTH * r for r in (0.6, 0.8, 1.0)]
        for radius in radii:
            for angle in range(-60, 61, 15):
                rad = math.radians(angle)
                nx = cx + math.cos(rad) * radius
                ny = cy + math.sin(rad) * radius

                if nx <= cx:
                    continue
                if not self._within_bounds(nx, ny):
                    continue

                potential, _, _ = self._get_potential_cost(nx, ny)
                if math.isinf(potential):
                    continue

                dist = self._distance((nx, ny), target)
                score = potential + dist * 0.3

                if score < best_score:
                    best_score = score
                    best_candidate = (nx, ny)

        return best_candidate

    def _build_path(self, start_anchor):
        """从指定起点锚点到终点附着节点构建桥面路径。"""
        if not start_anchor or not self.end_attach_node:
            return []

        start_pt = (start_anchor['x'], start_anchor['y'])
        target_pt = (self.end_attach_node['x'], self.end_attach_node['y'])

        path = [start_pt]
        current = start_pt
        max_steps = 64

        for _ in range(max_steps):
            if self._distance(current, target_pt) <= MAX_BEAM_LENGTH:
                path.append(target_pt)
                break

            next_point = self._select_next_point(current, target_pt)
            if not next_point:
                break

            if self._distance(current, next_point) < 1e-3:
                break

            path.append(next_point)
            current = next_point
        
        if path[-1] != target_pt and self._distance(path[-1], target_pt) <= MAX_BEAM_LENGTH:
            path.append(target_pt)

        return path if path[-1] == target_pt else []

    def _get_potential_cost(self, x, y):
        """
        计算势能场代价。
        返回: (cost_penalty, best_connect_node_id, connection_type)
        connection_type: 'direct' (直接连接), 'truss' (桁架支撑), None
        """
        min_cost = float('inf')
        best_node = None
        conn_type = None

        # 1. 检查是否接近吸引目标 (起终平台、现有路面)
        for node in self.attractor_nodes:
            dist = math.sqrt((x - node['x'])**2 + (y - node['y'])**2)
            if dist < MAX_BEAM_LENGTH:
                # 距离越近，代价越小。如果非常近，代价接近0
                # 代价函数设计：dist * weight
                cost = dist * 1.0 
                if cost < min_cost:
                    min_cost = cost
                    best_node = node
                    conn_type = 'direct'

        # 2. 检查是否接近普通支撑点 (桁架引力，R=OPTIMAL_TRUSS_HEIGHT)
        # 我们希望桥面在支撑点上方 H_opt 处
        for node in self.support_nodes:
            dx = abs(x - node['x'])
            dy = y - node['y'] # 正值表示桥面在上方
            
            if dx < MAX_BEAM_LENGTH:
                # 垂直距离偏离 H_opt 的程度
                target_y_diff = abs(dy - OPTIMAL_TRUSS_HEIGHT)
                
                # 如果在下方 (dy < 0)，通常很难做支撑，除非是悬挂
                # 这里简化：只考虑上方支撑 (桥墩模式)
                if dy > 0:
                    # 综合距离：水平距离 + 垂直偏离度
                    # 我们希望 dx 适中，dy 接近 H_opt
                    effective_dist = math.sqrt(dx**2 + target_y_diff**2)
                    
                    if effective_dist < MAX_BEAM_LENGTH:
                        cost = effective_dist * 2.0 # 权重稍低，不如直接连接重要
                        if cost < min_cost:
                            min_cost = cost
                            best_node = node
                            conn_type = 'truss'

        if math.isinf(min_cost):
            min_cost = MAX_BEAM_LENGTH * 3.0

        return min_cost, best_node, conn_type

    def solve(self) -> dict:
        """生成桥梁杆件结构并返回 sticks 字典。"""
        all_sticks = []
        seen_edges = set()

        def add_edge(p0, p1, material):
            key = (tuple(p0), tuple(p1), material)
            rev_key = (tuple(p1), tuple(p0), material)
            if key in seen_edges or rev_key in seen_edges:
                return
            all_sticks.append({'p0': p0, 'p1': p1, 'material': material})
            seen_edges.add(key)
            seen_edges.add(rev_key)

        for anchor in self.start_anchors:
            path = self._build_path(anchor)
            if not path:
                print(f"  -> 起点锚点 Node {anchor['id']} 未找到有效路径。")
                continue

            if self.end_anchor:
                end_pos = (self.end_anchor['x'], self.end_anchor['y'])
                if path[-1] != end_pos and self._distance(path[-1], end_pos) <= MAX_BEAM_LENGTH:
                    path.append(end_pos)

            for i in range(len(path) - 1):
                self._emit_segment(path[i], path[i + 1], 'road', add_edge)

                _, best_node, conn_type = self._get_potential_cost(path[i + 1][0], path[i + 1][1])
                if not best_node:
                    continue

                target_p = (float(best_node['x']), float(best_node['y']))

                if conn_type == 'direct':
                    self._emit_segment(path[i + 1], target_p, 'road', add_edge)
                elif conn_type == 'truss':
                    self._emit_segment(path[i + 1], target_p, 'wood', add_edge)
                    dist_prev = self._distance(path[i], target_p)
                    if dist_prev < MAX_BEAM_LENGTH:
                        self._emit_segment(path[i], target_p, 'wood', add_edge)

        return {'sticks': all_sticks}
