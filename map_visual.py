import numpy as np  # 数值计算库
import matplotlib.pyplot as plt  # 绘图库
import matplotlib.patches as patches  # 图形绘制模块
from matplotlib.animation import FuncAnimation  # 动画功能
from typing import List, Tuple, Optional, Dict, Any  # 类型提示


class VisualMap:
    """带有高级可视化功能的网格地图类，用于创建和显示路径规划地图"""

    def __init__(self, width: int, height: int, cell_size: float = 1.0):
        """
        初始化地图
        参数:
            width: 地图宽度(格子数)
            height: 地图高度(格子数)
            cell_size: 每个格子的大小(用于可视化)
        """
        self.width = width  # 地图宽度
        self.height = height  # 地图高度
        self.cell_size = cell_size  # 单元格大小
        # 初始化网格(0表示空闲，1表示障碍)
        self.grid = np.zeros((height, width), dtype=np.uint8)

        # 创建图形和坐标轴
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        # 设置坐标轴范围
        self.ax.set_xlim(0, width * cell_size)
        self.ax.set_ylim(0, height * cell_size)
        self.ax.set_aspect('equal')  # 保持纵横比一致
        # 设置网格线
        self.ax.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5)
        # 设置刻度
        self.ax.set_xticks(np.arange(0, width * cell_size + cell_size, cell_size))
        self.ax.set_yticks(np.arange(0, height * cell_size + cell_size, cell_size))
        self.ax.tick_params(labelbottom=False, labelleft=False)  # 隐藏刻度标签

        # 初始化可视化元素
        self.obstacles = []  # 障碍物图形列表
        self.path_lines = []  # 路径线列表
        self.start_marker = None  # 起点标记
        self.goal_marker = None  # 终点标记
        self._draw_grid()  # 绘制基础网格

    def _draw_grid(self):
        """绘制基础网格线"""
        # 绘制垂直线
        for x in range(self.width + 1):
            self.ax.axvline(x * self.cell_size, color='gray', linestyle='-', linewidth=0.5)
        # 绘制水平线
        for y in range(self.height + 1):
            self.ax.axhline(y * self.cell_size, color='gray', linestyle='-', linewidth=0.5)

    def set_obstacle(self, x: int, y: int):
        """
        设置单个障碍物并更新可视化
        参数:
            x: x坐标
            y: y坐标
        """
        if 0 <= x < self.width and 0 <= y < self.height:  # 检查坐标有效性
            self.grid[y, x] = 1  # 标记为障碍
            self._update_obstacles()  # 更新障碍物显示

    def clear_obstacle(self, x: int, y: int):
        """
        清除单个障碍物并更新可视化
        参数:
            x: x坐标
            y: y坐标
        """
        if 0 <= x < self.width and 0 <= y < self.height:  # 检查坐标有效性
            self.grid[y, x] = 0  # 清除障碍标记
            self._update_obstacles()  # 更新障碍物显示

    def set_obstacles(self, obstacles: List[Tuple[int, int]]):
        """
        批量设置障碍物
        参数:
            obstacles: 障碍物坐标列表
        """
        for x, y in obstacles:
            if 0 <= x < self.width and 0 <= y < self.height:  # 检查每个坐标有效性
                self.grid[y, x] = 1  # 标记为障碍
        self._update_obstacles()  # 更新障碍物显示

    def _update_obstacles(self):
        """更新障碍物可视化"""
        # 清除现有障碍物图形
        for patch in self.obstacles:
            patch.remove()
        self.obstacles.clear()

        # 绘制新障碍物
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y, x] == 1:  # 如果是障碍物
                    # 创建矩形补丁
                    rect = patches.Rectangle(
                        (x * self.cell_size, y * self.cell_size),  # 左下角坐标
                        self.cell_size, self.cell_size,  # 宽度和高度
                        linewidth=1, edgecolor='black',  # 边框样式
                        facecolor='red', alpha=0.7  # 填充样式
                    )
                    self.ax.add_patch(rect)  # 添加到坐标轴
                    self.obstacles.append(rect)  # 添加到障碍物列表
        plt.draw()  # 重绘图形

    def set_start(self, x: int, y: int) -> bool:
        """
        设置起点位置
        参数:
            x: x坐标
            y: y坐标
        返回:
            是否设置成功
        """
        if not self.is_free(x, y):  # 检查是否可通行
            return False

        # 清除现有起点标记
        if self.start_marker:
            self.start_marker.remove()

        # 创建圆形起点标记
        self.start_marker = patches.Circle(
            ((x + 0.5) * self.cell_size, (y + 0.5) * self.cell_size),  # 中心坐标
            radius=self.cell_size * 0.4,  # 半径
            facecolor='green', edgecolor='black'  # 颜色
        )
        self.ax.add_patch(self.start_marker)  # 添加到坐标轴
        plt.draw()  # 重绘图形
        return True

    def set_goal(self, x: int, y: int) -> bool:
        """
        设置终点位置
        参数:
            x: x坐标
            y: y坐标
        返回:
            是否设置成功
        """
        if not self.is_free(x, y):  # 检查是否可通行
            return False

        # 清除现有终点标记
        if self.goal_marker:
            self.goal_marker.remove()

        # 创建四边形终点标记
        self.goal_marker = patches.RegularPolygon(
            ((x + 0.5) * self.cell_size, (y + 0.5) * self.cell_size),  # 中心坐标
            numVertices=4, radius=self.cell_size * 0.4,  # 4边形，半径
            facecolor='blue', edgecolor='black'  # 颜色
        )
        self.ax.add_patch(self.goal_marker)  # 添加到坐标轴
        plt.draw()  # 重绘图形
        return True

    def draw_path(self, path: List[Tuple[int, int]], color: str = 'lime', linewidth: int = 2):
        """
        绘制路径
        参数:
            path: 路径坐标列表
            color: 路径颜色
            linewidth: 线宽
        """
        self.clear_path()  # 清除现有路径

        if not path:  # 空路径检查
            return

        # 转换坐标为格子中心点
        x_coords = [(x + 0.5) * self.cell_size for x, y in path]
        y_coords = [(y + 0.5) * self.cell_size for x, y in path]

        # 绘制路径线
        line, = self.ax.plot(x_coords, y_coords,
                             color=color, linewidth=linewidth,
                             marker='o', markersize=self.cell_size * 0.3,
                             markerfacecolor=color)
        self.path_lines.append(line)  # 添加到路径线列表
        plt.draw()  # 重绘图形

    def clear_path(self):
        """清除所有路径线"""
        for line in self.path_lines:
            line.remove()  # 移除每条路径线
        self.path_lines.clear()  # 清空列表
        plt.draw()  # 重绘图形

    def is_free(self, x: int, y: int) -> bool:
        """
        检查位置是否可通行
        参数:
            x: x坐标
            y: y坐标
        返回:
            是否可通行
        """
        if 0 <= x < self.width and 0 <= y < self.height:  # 检查边界
            return self.grid[y, x] == 0  # 检查是否为0(可通行)
        return False  # 超出边界视为不可通行

    def get_neighbors(self, x: int, y: int, allow_diagonal: bool = False) -> List[Tuple[int, int]]:
        """
        获取相邻可通行格子
        参数:
            x: 当前x坐标
            y: 当前y坐标
            allow_diagonal: 是否允许对角线移动
        返回:
            可通行邻居坐标列表
        """
        neighbors = []
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4方向移动(上、右、下、左)
        if allow_diagonal:
            directions += [(1, 1), (1, -1), (-1, 1), (-1, -1)]  # 添加4个对角线方向

        # 检查每个方向
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_free(nx, ny):  # 检查是否可通行
                neighbors.append((nx, ny))
        return neighbors

    def show(self):
        """显示地图"""
        plt.title("路径规划地图")  # 设置标题
        plt.show()  # 显示图形

    def save(self, filename: str):
        """
        保存地图图像
        参数:
            filename: 保存文件名
        """
        plt.savefig(filename)  # 保存图像
        plt.close()  # 关闭图形y