import heapq  # 堆队列算法，用于实现优先队列
import time    # 时间模块，用于计算算法运行时间
from typing import List, Tuple, Dict, Optional, Set  # 类型提示
from map_visual import VisualMap  # 自定义地图可视化模块


class Node:
    """A*算法节点类，表示搜索过程中的每个节点"""

    def __init__(self, pos: Tuple[int, int], parent: Optional['Node'] = None):
        """初始化节点
        参数:
            pos: 节点坐标(x,y)
            parent: 父节点，用于回溯路径
        """
        self.pos = pos        # 节点位置坐标
        self.parent = parent  # 父节点指针
        self.g = 0           # 从起点到当前节点的实际代价
        self.h = 0           # 启发式估计代价(到终点的估计距离)
        self.f = 0           # 总代价 f = g + h

    def __lt__(self, other):
        """重载小于运算符，用于堆排序"""
        return self.f < other.f

    def __eq__(self, other):
        """重载等于运算符，用于节点比较"""
        return self.pos == other.pos


class AStar:
    """A*路径规划器(带过程记录)，用于寻路并记录搜索过程"""

    def __init__(self, map: VisualMap, allow_diagonal: bool = False):
        """初始化A*规划器
        参数:
            map: 地图对象
            allow_diagonal: 是否允许对角线移动
        """
        self.map = map                # 地图对象
        self.allow_diagonal = allow_diagonal  # 是否允许对角线移动

    @staticmethod
    def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """欧几里得距离启发式函数
        参数:
            a: 起点坐标
            b: 终点坐标
        返回:
            两点间的欧几里得距离
        """
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Dict[str, any]:
        """
        执行A*路径规划并返回包含过程数据的字典
        返回:
            包含以下键的字典:
                - 'path': 最终路径
                - 'explored': 探索过的节点列表(按探索顺序)
                - 'open_set_history': 开放集历史记录(列表的集合)
                - 'start': 起点
                - 'goal': 终点
                - 'obstacles': 障碍物列表
                - 'width': 地图宽度
                - 'height': 地图高度
                - 'runtime': 运行时间(秒)
        """
        start_time = time.time()  # 记录开始时间

        # 初始化结果字典
        result = {
            'path': None,  # 最终路径
            'explored': [],  # 已探索节点列表
            'open_set_history': [],  # 开放集历史记录
            'start': start,  # 起点坐标
            'goal': goal,  # 终点坐标
            'obstacles': [(x, y) for y in range(self.map.height)
                          for x in range(self.map.width) if not self.map.is_free(x, y)],  # 障碍物列表
            'width': self.map.width,  # 地图宽度
            'height': self.map.height,  # 地图高度
            'runtime': 0  # 运行时间
        }

        # 检查起点和终点是否有效(是否可通行)
        if not self.map.is_free(*start) or not self.map.is_free(*goal):
            result['runtime'] = time.time() - start_time
            return result

        # 初始化开放列表(优先队列)和关闭列表(已探索集合)
        open_list = []
        closed_list = set()

        # 创建起点节点和目标节点
        start_node = Node(start)
        goal_node = Node(goal)
        heapq.heappush(open_list, start_node)  # 将起点加入开放列表

        # 用于快速查找节点(维护开放列表中的节点)
        open_dict = {start_node.pos: start_node}

        # 记录初始开放集状态
        result['open_set_history'].append(set(open_dict.keys()))

        while open_list:  # 主循环，直到开放列表为空
            current_node = heapq.heappop(open_list)  # 取出f值最小的节点
            del open_dict[current_node.pos]  # 从开放字典中移除
            closed_list.add(current_node.pos)  # 将当前节点加入关闭列表

            # 记录已探索节点(用于可视化)
            result['explored'].append(current_node.pos)

            # 检查是否到达目标
            if current_node.pos == goal_node.pos:
                path = []
                current = current_node
                # 回溯构建路径
                while current is not None:
                    path.append(current.pos)
                    current = current.parent
                result['path'] = path[::-1]  # 反转路径(从起点到终点)
                result['runtime'] = time.time() - start_time
                return result

            # 探索邻居节点
            for neighbor_pos in self.map.get_neighbors(*current_node.pos, self.allow_diagonal):
                if neighbor_pos in closed_list:  # 跳过已探索节点
                    continue

                # 计算移动代价(对角线移动代价更高)
                move_cost = 1.0  # 默认直线移动代价
                if self.allow_diagonal and abs(neighbor_pos[0] - current_node.pos[0]) == 1 \
                        and abs(neighbor_pos[1] - current_node.pos[1]) == 1:
                    move_cost = 1.414  # √2，对角线移动代价

                neighbor_g = current_node.g + move_cost  # 计算新g值

                # 检查是否已在开放列表中
                if neighbor_pos in open_dict:
                    neighbor_node = open_dict[neighbor_pos]
                    if neighbor_g < neighbor_node.g:  # 找到更优路径
                        # 更新节点信息
                        neighbor_node.g = neighbor_g
                        neighbor_node.h = self.heuristic(neighbor_pos, goal)
                        neighbor_node.f = neighbor_node.g + neighbor_node.h
                        neighbor_node.parent = current_node
                        heapq.heapify(open_list)  # 更新堆结构
                else:
                    # 创建新节点
                    neighbor_node = Node(neighbor_pos, current_node)
                    neighbor_node.g = neighbor_g
                    neighbor_node.h = self.heuristic(neighbor_pos, goal)
                    neighbor_node.f = neighbor_node.g + neighbor_node.h
                    heapq.heappush(open_list, neighbor_node)  # 加入开放列表
                    open_dict[neighbor_pos] = neighbor_node  # 加入开放字典

            # 记录当前开放集状态(用于可视化)
            result['open_set_history'].append(set(open_dict.keys()))

        # 开放列表为空但未找到路径
        result['runtime'] = time.time() - start_time
        return result

    def plan_with_visualization(self, start: Tuple[int, int], goal: Tuple[int, int],
                                save_path: str = None, fps: int = 10) -> Dict[str, any]:
        """
        执行规划并直接生成可视化
        参数:
            start: 起点坐标
            goal: 终点坐标
            save_path: 视频保存路径(None则不保存)
            fps: 动画帧率
        返回:
            包含规划结果和动画对象的字典
        """
        from path_animation import PathAnimation  # 动态导入可视化模块

        # 执行规划
        result = self.plan(start, goal)

        # 创建动画
        animator = PathAnimation(result, result['path'])
        anim = animator.animate(save_path=save_path, fps=fps)

        return {
            'result': result,  # 规划结果
            'animation': anim  # 动画对象
        }