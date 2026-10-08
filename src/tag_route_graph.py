import heapq
import json
import math

def angle_error(a, b):
    #计算从角度b转到a的最短有符号角差
    return math.atan2(math.sin(a - b), math.cos(a - b))

def _number(value, name):
    #把配置项校验并转换为有限浮点数
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("%s must be a finite number" % name)
    return float(value)

def _pose(value, name):
    if not isinstance(value, dict):
        raise ValueError("%s must be an object" % name)
    return tuple(_number(value[key], "%s.%s" % (name, key)) for key in ("x", "y", "yaw"))

class TagRouteGraph:
    #保存 Tag 节点、有向边，并提供 Dijkstra 路径规划
    def __init__(self, route_file):
        #从 JSON 文件创建有向路网并检查节点、边的有效性
        with open(route_file, "r") as stream:
            data = json.load(stream)
        self.frame_id = data["frame_id"]
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a nonempty string")
        self.nodes = {}
        self.edges = {}
        self.outgoing = {} # 记录每个tag的出边

        for item in data["nodes"]:
            tag_id = item["id"]
            if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0 or tag_id in self.nodes:
                raise ValueError("Tag IDs must be unique nonnegative integers")
            targetable = item.get("targetable", True)
            if not isinstance(targetable, bool):
                raise ValueError("node.targetable must be a boolean")
            self.nodes[tag_id] = {
                "tag_pose": _pose(item["tag_pose"], "tag_pose"),
                "targetable": targetable,
            }
            self.outgoing[tag_id] = []
        
        if not self.nodes:
            raise ValueError("route graph has no nodes")

        for item in data["edges"]:
            source, destination = item["source"], item["destination"]
            key = (source, destination)
            if source not in self.nodes or destination not in self.nodes or source == destination or key in self.edges:
                raise ValueError("invalid or duplicate directed edge %s" % (key,))
            cost = _number(item["cost"], "edge.cost")
            speed_limit = _number(item["speed_limit"], "edge.speed_limit")
            motion = item.get("motion", "forward")
            if cost <= 0 or speed_limit <= 0 or motion not in ("forward", "reverse"):
                raise ValueError("invalid cost, speed_limit or motion for edge %s" % (key,))
            waypoints = [_pose(p, "waypoint") for p in item.get("waypoints", [])]
            self.edges[key] = {
                "source": source, "destination": destination, "cost": cost,
                "speed_limit": speed_limit, "motion": motion, "waypoints": waypoints,
            }
            self.outgoing[source].append(key)

    # 沿有向边用 Dijkstra 搜索累计代价最小的路径
    # 返回(总代价, 有向边列表)，边用(source, destination)表示
    def shortest_path(self, start, goal):
        if start not in self.nodes or goal not in self.nodes:
            raise ValueError("start or goal Tag is absent from graph")
        queue = [(0.0, start)] # (当前累计代价, 节点ID)
        distance = {start: 0.0} # 记录从起点到各节点，目前找到的最小代价
        previous = {} # 记录到达某个节点时使用的是哪一条边，用于最后反向还原完整路线
        while queue:
            current_cost, current = heapq.heappop(queue)
            if current_cost != distance[current]:
                continue
            if current == goal:
                route = []
                while current != start:
                    edge_key = previous[current]
                    route.append(edge_key)
                    current = edge_key[0]
                route.reverse()
                return current_cost, route
            for key in self.outgoing[current]:
                edge = self.edges[key]
                next_cost = current_cost + edge["cost"]
                if next_cost < distance.get(edge["destination"], math.inf):
                    distance[edge["destination"]] = next_cost
                    previous[edge["destination"]] = key
                    heapq.heappush(queue, (next_cost, edge["destination"]))
        return math.inf, None

    @staticmethod
    # 计算二维点到有限线段的最近距离和投影比例
    #返回(距离, t)，其中t限制在 [0, 1];0 为起点,1 为终点
    def _segment_projection(point, start, end):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length2 = dx * dx + dy * dy
        if length2 == 0:
            raise ValueError("directed edge has zero geometric length")
        t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2))
        projection = (start[0] + t * dx, start[1] + t * dy)
        return math.hypot(point[0] - projection[0], point[1] - projection[1]), t

    # 返回 Tag 原点处、朝向 Tag +Y 的机器人目标位姿
    def tag_target_pose(self, tag_id):
        tag_x, tag_y, tag_yaw = self.nodes[tag_id]["tag_pose"]
        target_yaw = math.atan2(
            math.sin(tag_yaw + math.pi / 2.0),
            math.cos(tag_yaw + math.pi / 2.0))
        return tag_x, tag_y, target_yaw

    # 用整条规划路线上的节点拟合直线，并让方向从路线起点指向终点
    def route_line_yaw(self, route):
        if not route:
            raise ValueError("cannot fit a line from an empty route")
        node_ids = [route[0][0]] + [key[1] for key in route]
        points = [self.nodes[tag_id]["tag_pose"][:2] for tag_id in node_ids]
        mean_x = sum(point[0] for point in points) / len(points)
        mean_y = sum(point[1] for point in points) / len(points)
        xx = sum((point[0] - mean_x) ** 2 for point in points)
        yy = sum((point[1] - mean_y) ** 2 for point in points)
        xy = sum((point[0] - mean_x) * (point[1] - mean_y)
                 for point in points)
        if xx + yy <= 1e-12:
            raise ValueError("route nodes cannot define a line")
        yaw = 0.5 * math.atan2(2.0 * xy, xx - yy)
        start, goal = points[0], points[-1]
        if (math.cos(yaw) * (goal[0] - start[0]) + math.sin(yaw) * (goal[1] - start[1])) < 0.0:
            yaw += math.pi
        return math.atan2(math.sin(yaw), math.cos(yaw))

        #按行驶顺序取出一条边的起点、中间航点和终点
        #key: (source, destination)有向边标识
        #返回(x, y, yaw)位姿组成的列表
    def edge_points(self, key):
        edge = self.edges[key]
        return ([self.tag_target_pose(key[0])] + edge["waypoints"] +
                [self.tag_target_pose(key[1])])

        #将二维点投影到有向边的折线路径上 ？？？
        #point: 待投影点
        #key:(source, destination)有向边标识。
        #返回(横向距离, 已走比例, 最近线段下标)，已走比例按折线长度计算
    def project_to_edge(self, point, key):
        points = self.edge_points(key)
        lengths = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])]
        total_length = sum(lengths)
        if total_length == 0:
            raise ValueError("directed edge %s has zero geometric length" % (key,))
        best = (math.inf, 0.0, 0)
        traveled = 0.0
        for index, length in enumerate(lengths):
            if length > 0:
                distance, fraction = self._segment_projection(point, points[index], points[index + 1])
                candidate = (distance, (traveled + fraction * length) / total_length, index)
                if candidate[0] < best[0]:
                    best = candidate
            traveled += length
        return best

        # 判断点是否位于有向边起点后方的延长线附近。
        # point: 待判断的二维点或包含偏航角的二维位姿。
        # key: (source, destination)有向边标识。
        # Returns:
        #  点位于首个非零长度线段的起点后方时，返回(到延长线的横向距离, 到边起点的直线距离)；否则返回None
    def project_to_start_extension(self, point, key):
        points = self.edge_points(key)
        start = points[0]
        for end in points[1:]:
            dx, dy = end[0] - start[0], end[1] - start[1]
            length2 = dx * dx + dy * dy
            if length2 == 0:
                continue
            fraction = ((point[0] - start[0]) * dx +
                        (point[1] - start[1]) * dy) / length2
            if fraction >= 0.0:
                return None
            projection = (start[0] + fraction * dx,
                          start[1] + fraction * dy)
            lateral_distance = math.hypot(
                point[0] - projection[0], point[1] - projection[1])
            source_distance = math.hypot(
                point[0] - start[0], point[1] - start[1])
            return lateral_distance, source_distance
        raise ValueError("directed edge %s has zero geometric length" % (key,))

        # 从机器人在首条边的位置，取后续需要跟踪的航点；若机器人位于
        # source 后方的延长线上，则把 source 也作为第一个航点。
        # Returns:
        #     投影所在折线段之后的位姿列表，包含该边终点
    def remaining_waypoints(self, point, key):
        # 位于首边起点后方时必须先到 source，不能把 source 当成已驶过。
        if self.project_to_start_extension(point, key) is not None:
            return self.edge_points(key)
        _, _, segment_index = self.project_to_edge(point, key)
        return self.edge_points(key)[segment_index + 1:]

        # 把机器人位姿接入路网，再规划到目标 Tag 的有向边序列。优先
        # 匹配附近节点或有限边；在所有有限边之外时，也允许从首边 source
        # 后方的延长线接入，并先驶向 source。
        # Args:
        #     pose: 机器人在路网坐标系中的(x, y, yaw)
        #     goal: 目标 Tag ID
        #     node_snap_distance: 匹配节点的最大距离，单位 m
        #     edge_snap_distance: 匹配边的最大横向距离，单位 m
        # Returns:
        #     (source, destination)有向边列表；离路网过远或目标
        #     不可达时返回None，已经位于目标节点时返回空列表
    def plan_from_pose(self, pose, goal, node_snap_distance, edge_snap_distance):
        if goal not in self.nodes:
            raise ValueError("target Tag is absent from graph")
        if not self.nodes[goal]["targetable"]:
            raise ValueError("target Tag is localization-only")
        node_id = min(
            self.nodes,
            key=lambda i: math.hypot(
                pose[0] - self.nodes[i]["tag_pose"][0],
                pose[1] - self.nodes[i]["tag_pose"][1]))
        tag_pose = self.nodes[node_id]["tag_pose"]
        node_distance = math.hypot(
            pose[0] - tag_pose[0], pose[1] - tag_pose[1])
        if node_distance <= node_snap_distance:
            return self.shortest_path(node_id, goal)[1]

        candidates = []
        interior_candidates = []
        start_extension_candidates = []
        for key, edge in self.edges.items():
            lateral_distance, fraction, _ = self.project_to_edge(pose, key)
            if lateral_distance <= edge_snap_distance:
                _, rest = self.shortest_path(key[1], goal)
                if rest is not None:
                    remaining = (1.0 - fraction) * edge["cost"]
                    candidate = (lateral_distance, remaining, key, rest)
                    candidates.append(candidate)
                    if 0.0 < fraction < 1.0:
                        interior_candidates.append(candidate)

            extension = self.project_to_start_extension(pose, key)
            if extension is not None and extension[0] <= edge_snap_distance:
                route_cost, route = self.shortest_path(key[0], goal)
                # 只有从 source 出发后的首边仍是当前边时，才能沿这条
                # 延长线安全接入；目标就是 source 时则由空路径直接到达。
                if route is not None and (not route or route[0] == key):
                    start_extension_candidates.append(
                        (extension[0], extension[1], route_cost, route))

        # 真正位于某条有限边中途时，优先保持中途接入行为，避免被后续
        # 边的同一直线延长线错误吸附。
        if interior_candidates:
            candidates = interior_candidates
        elif start_extension_candidates:
            nearest_lateral = min(c[0] for c in start_extension_candidates)
            start_extension_candidates = [
                c for c in start_extension_candidates
                if c[0] <= nearest_lateral + 0.02
            ]
            # 同一直线上的多个 source 都可能命中；选择离机器人最近的
            # source，保证 Tag 0 后方从 Tag 0 接入，而不是跳到后续 Tag。
            return min(start_extension_candidates,
                       key=lambda c: (c[1], c[2]))[3]
        if not candidates:
            return None

        nearest_distance = min(candidate[0] for candidate in candidates)
        # 只在几何位置几乎相同的候选边之间比较剩余路径，避免跨路匹配。
        candidates = [c for c in candidates if c[0] <= nearest_distance + 0.02]
        _, _, first_key, rest = min(candidates, key=lambda c: c[1] + sum(self.edges[k]["cost"] for k in c[3]))
        return [first_key] + rest
