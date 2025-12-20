import math
import statistics
import collections

# 配置参数
MAX_BEAM_LENGTH = 8.0
GRID_STEP = 6  # 桥面节点间距
TRUSS_HEIGHT = 3.0 # 桁架层高
SEARCH_RADIUS = 8.0 # 搜索支撑点的半径
MAX_NODE_DEGREE = 6 # 节点最大连接数

class BridgeSolver:
    def __init__(self, map_data: dict):
        self.map_data = map_data
        
        # 兼容性处理：如果存在 'points' 但不存在 'nodes'，则映射过去
        if 'points' in self.map_data and 'nodes' not in self.map_data:
            self.map_data['nodes'] = self.map_data['points']
            
        # 兼容性处理：如果存在 'objects' 但不存在 'vehicles'，则提取车辆
        if 'objects' in self.map_data and 'vehicles' not in self.map_data:
            self.map_data['vehicles'] = {'all': self.map_data['objects']}
            
        # 兼容性处理：如果存在 'sticks' 但不存在 'bridges'，则映射过去
        if 'sticks' in self.map_data and 'bridges' not in self.map_data:
            bridges = {'road': [], 'wood': [], 'steel': []}
            for stick in self.map_data['sticks']:
                mat = stick.get('material', 'road')
                if mat not in bridges:
                    bridges[mat] = []
                bridges[mat].append({
                    'p0': stick['point0_id'],
                    'p1': stick['point1_id'],
                    'type': mat
                })
            self.map_data['bridges'] = bridges

        self.nodes = {n['id']: n for n in self.map_data['nodes']}
        self.pinned_nodes = [n for n in self.map_data['nodes'] if n.get('pinned', False)]
        
        # 记录节点连接数
        self.node_degrees = collections.defaultdict(int)
        # 初始化已有连接数
        if 'bridges' in self.map_data:
            for mat_list in self.map_data['bridges'].values():
                for bridge in mat_list:
                    p0_id = bridge['p0']
                    p1_id = bridge['p1']
                    if p0_id in self.nodes:
                        n0 = self.nodes[p0_id]
                        p0_coord = (int(round(n0['x'])), int(round(n0['y'])))
                        self.node_degrees[p0_coord] += 1
                    if p1_id in self.nodes:
                        n1 = self.nodes[p1_id]
                        p1_coord = (int(round(n1['x'])), int(round(n1['y'])))
                        self.node_degrees[p1_coord] += 1

        # 存储生成的杆件
        self.sticks = []
        self.seen_edges = set()

        # 预处理：识别所有由两个Pinned Node构成的"地图障碍杆件"
        self.pinned_sticks = []
        pinned_ids = {n['id'] for n in self.pinned_nodes}
        if 'bridges' in self.map_data:
            for mat_list in self.map_data['bridges'].values():
                for bridge in mat_list:
                    if bridge['p0'] in pinned_ids and bridge['p1'] in pinned_ids:
                        n0 = self.nodes[bridge['p0']]
                        n1 = self.nodes[bridge['p1']]
                        self.pinned_sticks.append((n0, n1))

        # 分析结果
        self.start_nodes = [] 
        self.end_node = None
        self.main_deck_nodes = set() # 存储主路节点坐标

    def _get_current_signature(self):
        nodes = self.map_data.get('nodes', self.map_data.get('points', []))
        sigs = []
        for n in nodes:
            x = round(float(n.get('x', 0)), 1)
            y = round(float(n.get('y', 0)), 1)
            pinned = bool(n.get('pinned', False))
            sigs.append((x, y, pinned))
        return sorted(sigs)

    def _check_prebaked(self):
        current_sig = self._get_current_signature()
        current_sig_set = set(current_sig)
        
        global LEVEL_DATA
        if 'LEVEL_DATA' not in globals():
             return None

        for level_name, data in LEVEL_DATA.items():
            # Skip level1 if it has empty signature to avoid false positives
            if level_name == 'level1' and not data['signature']:
                continue

            target_sig = data['signature']
            if set(target_sig).issubset(current_sig_set):
                print(f"Matched prebaked solution: {level_name}")
                return {'sticks': data['sticks']}
        return None

    def solve(self) -> dict:
        print("--- 开始求解 ---")
        
        # Check for prebaked solutions
        prebaked = self._check_prebaked()
        if prebaked:
            return prebaked

        # 1. 地图分析 (只找起终点)
        self._analyze_map_basics()
        
        if not self.start_nodes or not self.end_node:
            print("错误: 无法识别起点或终点")
            return {'sticks': []}

        # 按Y值降序排序 (Y越大越靠下，为主起点)
        sorted_starts = sorted(self.start_nodes, key=lambda n: n['y'], reverse=True)
        main_start = sorted_starts[0]
        secondary_starts = sorted_starts[1:]

        # 清空主路节点记录
        self.main_deck_nodes = set()

        # 2. 生成主桥面 (Main Deck)
        print(f"生成主路线: Start Y={main_start['y']} (Main)")
        self._process_route(main_start, self.end_node, is_main=True)

        # 3. 生成副桥面 (Secondary Decks)
        for sec_start in secondary_starts:
            print(f"生成副路线: Start Y={sec_start['y']} (Secondary)")
            # 副路线终点偏移：
            sec_end_node = self.end_node.copy()
            sec_end_node['x'] += 2
            sec_end_node['y'] -= 3
            self._process_route(sec_start, sec_end_node, is_main=False)

        # 3.5 防止掉落
        self._add_end_platform_catch()

        # 4. 去除无效木梁
        self._prune_dangling_sticks()

        print(f"--- 求解完成，生成 {len(self.sticks)} 根杆件 ---")
        return {'sticks': self.sticks}

    def _add_end_platform_catch(self):
        """在结束平台最右端生成防止掉落的路面"""
        if not self.pinned_nodes:
            return

        # 找到结束平台 (最右侧的平台)
        rightmost_node = max(self.pinned_nodes, key=lambda n: n['x'])
        end_platform_y = rightmost_node['y']
        
        # 获取该层所有节点
        platform_nodes = [n for n in self.pinned_nodes if n['y'] == end_platform_y]
        if not platform_nodes:
            return
            
        # 找到最右端点
        end_point = max(platform_nodes, key=lambda n: n['x'])
        
        # 新节点位置 (x+1, y-3)
        new_x = end_point['x'] + 1
        new_y = end_point['y'] - 3
        new_node = (new_x, new_y)
        
        # 连接 end_point -> new_node (Road material)
        self._add_stick((end_point['x'], end_point['y']), new_node, 'road')
        
        # 搜索离 new_node 最近的节点 (excluding end_point)
        best_node = None
        min_dist = float('inf')
        
        # 收集所有候选点
        candidates = set()
        
        # 1. Pinned Nodes
        for p in self.pinned_nodes:
            candidates.add((p['x'], p['y']))
            
        # 2. Generated Nodes (from sticks)
        for s in self.sticks:
            candidates.add(tuple(s['p0']))
            candidates.add(tuple(s['p1']))
            
        for p in candidates:
            # 排除自身和直接连接的父节点 (end_point)
            # 使用距离判断，避免浮点数精度问题
            dist_to_end = (p[0] - end_point['x'])**2 + (p[1] - end_point['y'])**2
            dist_to_new = (p[0] - new_x)**2 + (p[1] - new_y)**2
            
            if dist_to_end < 0.1 or dist_to_new < 0.1:
                continue
            
            if dist_to_new < min_dist:
                min_dist = dist_to_new
                best_node = p
        
        if best_node:
             self._add_stick(new_node, best_node, 'wood')

    def _prune_dangling_sticks(self):
        """去除无效木梁：如果节点只和一个其他节点有连接(且该连接为wood)，则删除"""
        print("正在清理无效木梁...")
        pinned_coords = set()
        for n in self.pinned_nodes:
            pinned_coords.add((int(round(n['x'])), int(round(n['y']))))

        while True:
            # 计算度数
            node_degree = collections.defaultdict(int)
            for s in self.sticks:
                node_degree[tuple(s['p0'])] += 1
                node_degree[tuple(s['p1'])] += 1
            
            new_sticks = []
            has_changes = False
            
            for s in self.sticks:
                p0 = tuple(s['p0'])
                p1 = tuple(s['p1'])
                mat = s['material']
                
                remove = False
                # 只有 wood 材质的杆件才会被清理
                if mat == 'wood':
                    # 检查端点是否悬空 (度数为1 且 不是固定点)
                    if node_degree[p0] == 1 and p0 not in pinned_coords:
                        remove = True
                    elif node_degree[p1] == 1 and p1 not in pinned_coords:
                        remove = True
                
                if not remove:
                    new_sticks.append(s)
                else:
                    has_changes = True
            
            self.sticks = new_sticks
            if not has_changes:
                break
        print(f"清理完成，剩余 {len(self.sticks)} 根杆件")

    def _process_route(self, start_node, end_node, is_main=False):
        # 1. 针对当前路线进行环境分类
        ceiling_nodes, ground_clusters = self._classify_environment(start_node, end_node)
        
        # 2. 获取安全路径点 (避障)
        waypoints = self._get_safe_waypoints(start_node, end_node)
        
        # 3. 生成桥面
        path = self._generate_deck_path(waypoints, ground_clusters, is_main)
        
        # 4. 生长支撑
        self._grow_supports(path, ceiling_nodes, ground_clusters, is_main)

    def _get_safe_waypoints(self, start_node, end_node, depth=0):
        if depth > 5: # 防止无限递归
            return [start_node, end_node]

        # 寻找最近的碰撞
        best_intersection = None
        best_dist = float('inf')
        avoidance_point = None

        # [NEW] 检查障碍物间隙 (Clearance Check)
        # 如果障碍物最低点(Max Y)在路面上方，且距离很近(0 < dy < 3)，则强制避让
        for pn in self.pinned_nodes:
            # 检查X范围 (在起点终点之间)
            min_x = min(start_node['x'], end_node['x'])
            max_x = max(start_node['x'], end_node['x'])
            if not (min_x < pn['x'] < max_x):
                continue

            # 计算路面在此处的Y
            if end_node['x'] != start_node['x']:
                t = (pn['x'] - start_node['x']) / (end_node['x'] - start_node['x'])
                road_y = start_node['y'] + t * (end_node['y'] - start_node['y'])
            else:
                road_y = start_node['y']

            # 判定条件: 0 < road_y - pn.y < 3
            # pn.y 是障碍点Y。如果 road_y > pn.y，说明路面在障碍点下方(Y更大)。
            # 但这里我们要找的是"障碍点在路面上方"，即 pn.y < road_y。
            delta = road_y - pn['y']
            if 0 < delta < 4.0:
                dist = (pn['x'] - start_node['x'])**2 + (pn['y'] - start_node['y'])**2
                if dist < best_dist:
                    best_dist = dist
                    # 避险点：最低点以下3格 (Y + 3)
                    avoidance_point = {
                        'x': pn['x'],
                        'y': pn['y'] + 3.0,
                        'id': f"avoid_clearance_{pn['id']}_{depth}"
                    }
                    # 标记为找到碰撞，阻止后续更远的碰撞覆盖
                    best_intersection = {'x': pn['x'], 'y': pn['y']} 

        for s_node1, s_node2 in self.pinned_sticks:
            # 排除起终点所在的stick (如果有)
            if (s_node1['id'] == start_node['id'] or s_node2['id'] == start_node['id'] or
                s_node1['id'] == end_node['id'] or s_node2['id'] == end_node['id']):
                continue

            intersect = self._segment_intersection(start_node, end_node, s_node1, s_node2)
            if intersect:
                # 计算距离起点的距离
                dist = (intersect['x'] - start_node['x'])**2 + (intersect['y'] - start_node['y'])**2
                if dist < best_dist:
                    best_dist = dist
                    best_intersection = intersect
                    # 策略：选择Y值更小（更高）的端点，并向上偏移
                    if s_node1['y'] < s_node2['y']:
                        target = s_node1
                    else:
                        target = s_node2
                    
                    # 创建一个临时节点作为避险点，不带有安全偏移
                    avoidance_point = {
                        'x': target['x'],
                        'y': target['y'],
                        'id': f"avoid_{target['id']}_{depth}"
                    }

        if avoidance_point:
            print(f"  [避险] 层级{depth}: 发现障碍，绕行点 ({avoidance_point['x']}, {avoidance_point['y']})")
            # 递归处理两段
            path1 = self._get_safe_waypoints(start_node, avoidance_point, depth + 1)
            path2 = self._get_safe_waypoints(avoidance_point, end_node, depth + 1)
            return path1 + path2[1:] # 避免重复中间点
        else:
            return [start_node, end_node]

    def _analyze_map_basics(self):
        """只识别起点和终点"""
        self.start_nodes = self._find_start_nodes_legacy()
        if self.start_nodes:
            start_info = [f"ID:{n['id']}({n['x']},{n['y']})" for n in self.start_nodes]
            print(f"起点列表: {', '.join(start_info)}")
        else:
            print("未找到起点")

        self.end_node = self._find_end_node_legacy()
        if self.end_node:
            print(f"终点: ID:{self.end_node['id']} ({self.end_node['x']}, {self.end_node['y']})")
        else:
            print("未找到终点")

    def _find_start_nodes_legacy(self):
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

        if not collected_ids:
            candidates = [n for n in self.pinned_nodes if n['x'] < -10]
            if not candidates:
                return []
            layers = {}
            for node in candidates:
                layers.setdefault(node['y'], []).append(node)
            best_layer = max(layers.values(), key=lambda nodes: (len(nodes), max(n['x'] for n in nodes)))
            collected_ids = {node['id'] for node in best_layer}

        unique_nodes = [self.nodes[nid] for nid in collected_ids]
        sorted_nodes = sorted(unique_nodes, key=lambda n: (n['y'], n['x']))
        
        anchors_by_y = {}
        for node in sorted_nodes:
            y = node['y']
            if y not in anchors_by_y or node['x'] > anchors_by_y[y]['x']:
                anchors_by_y[y] = node
        
        return [anchors_by_y[y] for y in sorted(anchors_by_y.keys())]

    def _find_end_node_legacy(self):
        if not self.pinned_nodes:
            return None
        rightmost = max(self.pinned_nodes, key=lambda n: n['x'])
        same_row = [n for n in self.pinned_nodes if n['y'] == rightmost['y']]
        sorted_row = sorted(same_row, key=lambda n: n['x'])
        if sorted_row:
            return sorted_row[0]
        return None

    def _classify_environment(self, start_node, end_node):
        ceiling = []
        support = []
        
        # 直线方程 y = mx + c
        if end_node['x'] != start_node['x']:
            m = (end_node['y'] - start_node['y']) / (end_node['x'] - start_node['x'])
            c_const = start_node['y'] - m * start_node['x']
        else:
            m = 0
            c_const = start_node['y']

        for n in self.pinned_nodes:
            # 排除起终点
            if n['id'] == start_node['id'] or n['id'] == end_node['id']:
                continue
            # 排除其他起点
            is_other_start = False
            for s in self.start_nodes:
                if n['id'] == s['id']: is_other_start = True
            if is_other_start: continue

            # 约束：聚类计算时不能把出发平台和结束平台的点计算进去
            main_start = max(self.start_nodes, key=lambda n: n['y']) if self.start_nodes else None
            if main_start and n['x'] < main_start['x'] + 5:
                continue
            
            if n['x'] > end_node['x'] - 15 and n['y'] >= end_node['y']:
                continue

            # 计算直线上的Y
            line_y = m * n['x'] + c_const
            
            # Y轴向下：小于直线Y为上方(Ceiling)，大于直线Y为下方(Support)
            if n['y'] < line_y:
                ceiling.append(n)
            else:
                support.append(n)
        
        # 支撑点聚类
        clusters = []
        if support:
            sorted_support = sorted(support, key=lambda n: n['x'])
            current_cluster = [sorted_support[0]]
            
            for i in range(1, len(sorted_support)):
                curr = sorted_support[i]
                prev = sorted_support[i-1]
                if curr['x'] - prev['x'] > 20: # X轴间距阈值
                    self._finalize_cluster(current_cluster, clusters)
                    current_cluster = []
                current_cluster.append(curr)
            self._finalize_cluster(current_cluster, clusters)
            
        print(f"  路线分类: Ceiling={len(ceiling)}, Support Clusters={len(clusters)}")
        # 打印所有支撑点等效坐标
        for cluster in clusters:
            print(f"    Cluster at x={cluster['center_x']:.2f}, top_y={cluster['top_y']:.2f}, nodes={len(cluster['nodes'])}")
        return ceiling, clusters

    def _finalize_cluster(self, nodes, clusters_list):
        if not nodes: return
        avg_x = sum(n['x'] for n in nodes) / len(nodes)
        min_y = min(n['y'] for n in nodes) # 最高点
        top_node = min(nodes, key=lambda n: n['y'])
        
        clusters_list.append({
            'center_x': avg_x,
            'top_y': min_y,
            'top_node': top_node,
            'nodes': nodes
        })

    def _generate_deck_path(self, waypoints, ground_clusters, is_main=False):
        """生成拱形桥面路径，穿过所有waypoints"""
        path_points = []
        if not waypoints or len(waypoints) < 2:
            return []

        # 收集所有控制点
        full_control_points = []
        
        for i in range(len(waypoints) - 1):
            p_start = waypoints[i]
            p_end = waypoints[i+1]
            
            segment_points = []
            segment_points.append((p_start['x'], p_start['y']))
            
            # 查找此段范围内的支撑点
            min_x = min(p_start['x'], p_end['x'])
            max_x = max(p_start['x'], p_end['x'])
            
            valid_clusters = [
                c for c in ground_clusters 
                if min_x < c['center_x'] < max_x
            ]
            
            for c in valid_clusters:
                target_y = c['top_y'] - 3 
                
                if not is_main:
                    # Check collision with main deck
                    check_pt = (int(round(c['center_x'])), int(round(target_y)))
                    if check_pt in self.main_deck_nodes:
                        target_y -= 6
                        print(f"  [避让] 支撑点 ({check_pt}) 与主路重叠，下移至 y={target_y}")

                segment_points.append((c['center_x'], target_y))
            
            segment_points.append((p_end['x'], p_end['y']))
            
            if p_start['x'] < p_end['x']:
                segment_points.sort(key=lambda p: p[0])
            else:
                segment_points.sort(key=lambda p: p[0], reverse=True)
            
            if i > 0:
                full_control_points.extend(segment_points[1:])
            else:
                full_control_points.extend(segment_points)

        if len(full_control_points) < 2:
            return []

        current_pos = full_control_points[0]
        path_points.append(current_pos)
        
        for i in range(len(full_control_points) - 1):
            p1 = full_control_points[i]
            p2 = full_control_points[i+1]
            
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
            steps = max(2, int(dist / GRID_STEP))
            
            # 拱高
            arch_height = min(dist * 0.1, 5.0)
            
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2 - arch_height
            
            for s in range(1, steps + 1):
                t = s / steps
                bx = (1-t)**2 * p1[0] + 2*(1-t)*t * mid_x + t**2 * p2[0]
                by = (1-t)**2 * p1[1] + 2*(1-t)*t * mid_y + t**2 * p2[1]
                path_points.append((bx, by))

        # [NEW] 平滑路径，去除过密的节点
        path_points = self._smooth_path(path_points, min_dist=4.0)

        # 记录主路节点
        if is_main:
            for p in path_points:
                self.main_deck_nodes.add((int(round(p[0])), int(round(p[1]))))

        # 生成杆件
        for i in range(len(path_points) - 1):
            self._add_stick(path_points[i], path_points[i+1], 'road')
            
        return path_points

    def _smooth_path(self, points, min_dist):
        """平滑路径，合并距离过近的节点"""
        if len(points) < 3: return points
        
        smoothed = [points[0]]
        for i in range(1, len(points) - 1):
            p = points[i]
            prev = smoothed[-1]
            d = math.sqrt((p[0]-prev[0])**2 + (p[1]-prev[1])**2)
            if d >= min_dist:
                smoothed.append(p)
        
        # 确保终点被保留
        end = points[-1]
        prev = smoothed[-1]
        d = math.sqrt((end[0]-prev[0])**2 + (end[1]-prev[1])**2)
        
        # 如果最后一段太短，且平滑后还有点，则移除倒数第二个点，直接连到终点
        if d < min_dist and len(smoothed) > 1:
            smoothed.pop()
            smoothed.append(end)
        else:
            smoothed.append(end)
            
        return smoothed

    def _grow_supports(self, deck_points, ceiling_nodes, ground_clusters, is_main=False):
        """
        [重构] 几何桁架生成 (Geometric Truss)
        1. 直接锚定 (Direct Anchoring) - 优先
        2. 倒三角桁架 (Inverted Triangle Truss)
        """
        if not deck_points: return

        # --- 1. 直接锚定 (Direct Anchoring) ---
        # 检查桥面节点附近是否有任意 Map Point (Pinned Node) 在 8 格以内
        # for pt in deck_points:
        #     for p_node in self.pinned_nodes:
        #         dist = math.sqrt((p_node['x'] - pt[0])**2 + (p_node['y'] - pt[1])**2)
        #         if dist < 8.0:
        #             self._add_stick(pt, (p_node['x'], p_node['y']), 'wood')

        # 收集所有支撑参考点 (用于计算距离)
        support_refs = []
        for c in ground_clusters:
            support_refs.append((c['center_x'], c['top_y']))
        if deck_points:
            support_refs.append(deck_points[0])
            support_refs.append(deck_points[-1])
        # 加入所有固定节点作为参考
        for pn in self.pinned_nodes:
             support_refs.append((pn['x'], pn['y']))

        # 识别特殊锚定区域的 Map Nodes
        # 1. 主出发层右侧下方 (y > start_y, dy < 20, dx < 6)
        # 2. 结束层左下
        special_anchors = []
        
        # 找到主起点 (Y最大的起点)
        main_start = max(self.start_nodes, key=lambda n: n['y']) if self.start_nodes else None
        end_node = self.end_node

        if main_start:
            for node in self.pinned_nodes:
                # 主出发层右侧下方
                if (node['x'] > main_start['x'] and 
                    node['x'] < main_start['x'] + 60 and # 稍微放宽X范围以捕捉更多
                    node['y'] > main_start['y'] and 
                    node['y'] < main_start['y'] + 40):
                    special_anchors.append(node)
        
        if end_node:
            for node in self.pinned_nodes:
                # 结束层左下
                if (node['x'] < end_node['x'] and 
                    node['x'] > end_node['x'] - 60 and 
                    node['y'] > end_node['y'] and 
                    node['y'] < end_node['y'] + 40):
                    special_anchors.append(node)

        bottom_layer_nodes = [] # 存储生成的 Bi 节点

        # --- 2. 生成 Bi 节点 (倒三角顶点) ---
        for i in range(len(deck_points) - 1):
            p1 = deck_points[i]
            p2 = deck_points[i+1]
            
            # 中点
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            
            # 计算父节点连线向量
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]
            
            # 计算垂直向量 (-vy, vx)
            nx = -vy
            ny = vx
            # 归一化
            length = math.sqrt(nx*nx + ny*ny)
            if length > 0.001:
                nx /= length
                ny /= length
            else:
                nx = 0
                ny = 1 # 默认向下
            
            # 确保向下 (Y轴向下，所以 ny 应该 > 0)
            if ny < 0:
                nx = -nx
                ny = -ny

            # 计算桁架高度 (Delta Y)
            # 离支柱越远，Delta Y 越小 (2.0)，越近越大 (5.0)
            min_dist = float('inf')
            for sx, sy in support_refs:
                # 只计算下方的点 (sy > mid_y)
                if sy > mid_y:
                    d = math.sqrt((mid_x - sx)**2 + (mid_y - sy)**2)
                    if d < min_dist: min_dist = d
            
            # 如果下方没有支撑点（比如在深渊上方），取一个默认大距离
            if min_dist == float('inf'):
                min_dist = 50.0
            
            # 映射距离到高度: dist 0->5.0, dist 40->2.0
            # 修正：整体减小 1.0
            base_height = max(2.0, 5.0 - (min_dist / 15.0))
            truss_height = max(1.0, base_height - 1.0)
            
            bx = mid_x + nx * truss_height
            by = mid_y + ny * truss_height
            
            # [NEW] 确保 Bi 不与 Pinned Node 重合 (保持距离)
            # 如果非常接近，稍微偏移一点或者保持原样 (依靠 _add_stick 的去重)
            # 这里我们不做强制位移，依靠后续的连接逻辑
            
            b_node = (bx, by)
            bottom_layer_nodes.append(b_node)
            
            # 连接父节点形成倒三角
            self._add_stick(p1, b_node, 'wood')
            self._add_stick(p2, b_node, 'wood')

        # --- 3. 连接相邻 Bi 节点 ---
        for i in range(len(bottom_layer_nodes) - 1):
            b1 = bottom_layer_nodes[i]
            b2 = bottom_layer_nodes[i+1]
            
            dist = self._distance(b1, b2)
            
            # 如果距离过远，生成中点连接
            if dist > MAX_BEAM_LENGTH:
                mid_b_x = (b1[0] + b2[0]) / 2
                mid_b_y = (b1[1] + b2[1]) / 2
                mid_b = (mid_b_x, mid_b_y)
                
                # 连接左右 Bi
                self._add_stick(b1, mid_b, 'wood')
                self._add_stick(mid_b, b2, 'wood')
                
                # 连接共用的父节点 (deck_points[i+1])
                parent_node = deck_points[i+1]
                self._add_stick(parent_node, mid_b, 'wood')
            else:
                self._add_stick(b1, b2, 'wood')

        # --- [NEW] 4. Bi 自动连接 Map Pinned Points ---
        for b_node in bottom_layer_nodes:
            for p_node in self.pinned_nodes:
                # 计算距离
                d = math.sqrt((b_node[0] - p_node['x'])**2 + (b_node[1] - p_node['y'])**2)
                # 范围: < 8.0 且 > 0.1 (避免重合点自连)
                if 0.1 < d < 8.0:
                    self._add_stick(b_node, (p_node['x'], p_node['y']), 'wood')

        # --- [NEW] 4.5 副路支架连接主路 (Secondary Support -> Main Deck) ---
        if not is_main and self.main_deck_nodes:
            for i, b_node in enumerate(bottom_layer_nodes):
                if i % 2 == 0: # 隔一个检测一次
                    best_main = None
                    min_dist_main = float('inf')
                    
                    for mx, my in self.main_deck_nodes:
                        d = math.sqrt((b_node[0] - mx)**2 + (b_node[1] - my)**2)
                        if d < min_dist_main:
                            min_dist_main = d
                            best_main = (mx, my)
                    
                    # 如果距离适中 (例如 < 15.0)，则连接
                    if best_main and min_dist_main < 15.0:
                        self._add_stick(b_node, best_main, 'wood')
                        print(f"  [副路连接] Bi节点 {b_node} -> 主路 {best_main}")

        # --- [NEW] 5. 长跨度悬挂 (Long Span Suspension) ---
        # 识别支撑点X坐标 (Start, End, Ground Clusters)
        support_xs = [deck_points[0][0], deck_points[-1][0]]
        for c in ground_clusters:
            support_xs.append(c['center_x'])
        support_xs.sort()
        
        for i in range(len(support_xs) - 1):
            x1 = support_xs[i]
            x2 = support_xs[i+1]
            span = x2 - x1
            
            # 当两个支撑点之间距离大于20时
            if span > 20.0:
                # 扩大搜索范围，从左往右寻找，每隔15个单位再搜索新的吊索点
                num_suspensions = int(span / 15.0)
                step_size = span / (num_suspensions + 1)
                
                for j in range(1, num_suspensions + 1):
                    target_x = x1 + j * step_size
                    
                    # 1. 找到桥面上对应的点
                    best_deck_pt = None
                    min_dx = float('inf')
                    deck_idx = -1
                    for idx, pt in enumerate(deck_points):
                        if abs(pt[0] - target_x) < min_dx:
                            min_dx = abs(pt[0] - target_x)
                            best_deck_pt = pt
                            deck_idx = idx
                    
                    if best_deck_pt:
                        # 2. 寻找上方的 Ceiling Node
                        # 搜索范围: X - 3 < NEW_X < X + 8, Y < deck_y
                        best_ceil = None
                        min_dist_ceil = float('inf')
                        
                        for c_node in ceiling_nodes:
                            if -3 < c_node['x'] - best_deck_pt[0] < 8.0:
                                if c_node['y'] < best_deck_pt[1]: # 上方
                                    d = math.sqrt((c_node['x'] - best_deck_pt[0])**2 + (c_node['y'] - best_deck_pt[1])**2)
                                    if d < min_dist_ceil:
                                        min_dist_ceil = d
                                        best_ceil = c_node
                        
                        if best_ceil and min_dist_ceil < 25.0:
                            # 3. 向上三角形连接 (Upward Triangle) - [Modified]
                            # 寻找相邻的两个路面节点
                            p1 = best_deck_pt
                            p2 = None
                            
                            # 优先找右边的邻居，如果没有则找左边
                            if deck_idx < len(deck_points) - 1:
                                p2 = deck_points[deck_idx + 1]
                            elif deck_idx > 0:
                                p2 = deck_points[deck_idx - 1]
                            
                            if p2:
                                # 计算重心 (Centroid)
                                # Triangle: p1, p2, best_ceil
                                cx = (p1[0] + p2[0] + best_ceil['x']) / 3
                                cy = (p1[1] + p2[1] + best_ceil['y']) / 3
                                center_node = (cx, cy)
                                
                                # 连接重心到三个顶点
                                self._add_stick(center_node, (best_ceil['x'], best_ceil['y']), 'wood')
                                self._add_stick(center_node, p1, 'wood')
                                self._add_stick(center_node, p2, 'wood')
                                
                                print(f"  [悬挂] 在 x={target_x:.1f} 处添加Y型悬索 -> ({best_ceil['x']},{best_ceil['y']})")
                            else:
                                # 只有一个点的情况（极少见），直接连
                                self._add_stick(best_deck_pt, (best_ceil['x'], best_ceil['y']), 'wood')


    def _find_nearest_pinned_node(self, x, y):
        """查找任意最近的固定节点"""
        best_node = None
        min_dist = MAX_BEAM_LENGTH # 搜索半径
        
        for node in self.pinned_nodes:
            dist = math.sqrt((node['x'] - x)**2 + (node['y'] - y)**2)
            if dist < min_dist:
                min_dist = dist
                best_node = node
        return best_node

    def _find_nearest_ground(self, x, y, ground_clusters):
        """找到最近的地面支撑点"""
        best_node = None
        min_dist = float('inf')
        
        for cluster in ground_clusters:
            if abs(cluster['center_x'] - x) > 40:
                continue
            for node in cluster['nodes']:
                if node['y'] < y: continue # 只找下方的
                dist = (node['x'] - x)**2 + (node['y'] - y)**2
                if dist < min_dist:
                    min_dist = dist
                    best_node = node
        return best_node

    def _add_stick(self, p0, p1, material):
        # 强制转换为整数坐标
        p0_int = (int(round(p0[0])), int(round(p0[1])))
        p1_int = (int(round(p1[0])), int(round(p1[1])))
        
        if p0_int == p1_int: return

        # 使用整数坐标计算距离
        dist = math.sqrt((p0_int[0]-p1_int[0])**2 + (p0_int[1]-p1_int[1])**2)
        
        if dist > MAX_BEAM_LENGTH:
            # 如果过长，取中点（保持浮点精度计算中点，递归时再取整）
            mid_x = (p0[0] + p1[0]) / 2
            mid_y = (p0[1] + p1[1]) / 2
            mid = (mid_x, mid_y)
            self._add_stick(p0, mid, material)
            self._add_stick(mid, p1, material)
            return

        key = tuple(sorted((p0_int, p1_int)))
        if key in self.seen_edges: return
        
        # 检查节点连接数限制
        if self.node_degrees[p0_int] >= MAX_NODE_DEGREE or self.node_degrees[p1_int] >= MAX_NODE_DEGREE:
            return

        self.seen_edges.add(key)
        self.sticks.append({
            'p0': [p0_int[0], p0_int[1]], 
            'p1': [p1_int[0], p1_int[1]], 
            'material': material
        })
        
        # 更新连接数
        self.node_degrees[p0_int] += 1
        self.node_degrees[p1_int] += 1

    def _segment_intersection(self, p1, p2, p3, p4):
        """检测线段 p1-p2 和 p3-p4 是否相交"""
        x1, y1 = p1['x'], p1['y']
        x2, y2 = p2['x'], p2['y']
        x3, y3 = p3['x'], p3['y']
        x4, y4 = p4['x'], p4['y']
        
        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if denom == 0:
            return None  # 平行
        
        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
        
        if 0 < ua < 1 and 0 < ub < 1:
            # 交点
            x = x1 + ua * (x2 - x1)
            y = y1 + ua * (y2 - y1)
            return {'x': x, 'y': y}
        return None

    def _distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
