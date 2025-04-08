import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle, Circle, Polygon
import numpy as np
from typing import List, Tuple, Dict, Any


class PathAnimation:
    """路径规划动画可视化类，用于展示A*算法等路径规划过程"""

    def __init__(self, map_data: Dict[str, Any], path: List[Tuple[int, int]] = None):
        """
        初始化方法，设置地图数据和路径

        参数:
            map_data: 包含地图信息的字典，需包含以下键:
                - 'width': 地图宽度
                - 'height': 地图高度
                - 'obstacles': 障碍物坐标列表
                - 'start': 起点坐标
                - 'goal': 终点坐标
                - 'explored': 已探索区域坐标列表
                - 'open_set_history'或'open_set': 开放集历史记录
            path: 最终路径的坐标列表，可选
        """
        # 处理可能的键名不一致问题，确保使用'open_set_history'作为键
        if 'open_set' in map_data and 'open_set_history' not in map_data:
            map_data['open_set_history'] = map_data['open_set']

        self.map_data = map_data  # 存储地图数据
        self.path = path  # 存储路径数据
        self.fig, self.ax = plt.subplots(figsize=(10, 10))  # 创建10x10大小的图形和坐标轴
        self.cell_size = 1.0  # 每个网格单元格的大小
        self.setup_plot()  # 调用绘图设置方法

    def setup_plot(self):
        """设置基础绘图元素，包括地图、障碍物、起点终点等"""
        # 设置坐标轴范围和属性
        self.ax.set_xlim(0, self.map_data['width'] * self.cell_size)  # x轴范围
        self.ax.set_ylim(0, self.map_data['height'] * self.cell_size)  # y轴范围
        self.ax.set_aspect('equal')  # 保持纵横比一致
        self.ax.grid(True, which='both', color='gray', linestyle='-', linewidth=0.5)  # 显示网格
        self.ax.set_xticks(np.arange(0, self.map_data['width'] + 1, 1))  # 设置x轴刻度
        self.ax.set_yticks(np.arange(0, self.map_data['height'] + 1, 1))  # 设置y轴刻度
        self.ax.tick_params(labelbottom=False, labelleft=False)  # 隐藏刻度标签

        # 绘制障碍物
        obstacles = []
        for x, y in self.map_data['obstacles']:
            # 为每个障碍物创建矩形补丁
            obstacles.append(Rectangle((x, y), 1, 1))
        # 将所有障碍物组合成集合
        self.obstacles = PatchCollection(obstacles, facecolor='red', alpha=0.7)
        self.ax.add_collection(self.obstacles)  # 将障碍物集合添加到坐标轴

        # 绘制起点和终点
        start_x, start_y = self.map_data['start']
        goal_x, goal_y = self.map_data['goal']
        # 起点用圆形表示
        self.start_marker = Circle((start_x + 0.5, start_y + 0.5), 0.4,
                                   facecolor='green', edgecolor='black')
        # 终点用三角形表示
        self.goal_marker = Polygon([[goal_x + 0.1, goal_y + 0.1],
                                    [goal_x + 0.9, goal_y + 0.1],
                                    [goal_x + 0.5, goal_y + 0.9]],
                                   facecolor='blue', edgecolor='black')
        self.ax.add_patch(self.start_marker)  # 添加起点到坐标轴
        self.ax.add_patch(self.goal_marker)  # 添加终点到坐标轴

        # 初始化探索区域和开放集的可视化元素
        self.explored_plot = self.ax.scatter([], [], c='lightblue', s=100, alpha=0.5)  # 探索区域散点图
        self.open_set_plot = self.ax.scatter([], [], c='yellow', s=100, alpha=0.7)  # 开放集散点图
        self.path_line, = self.ax.plot([], [], 'lime', linewidth=2, marker='o',  # 路径线
                                       markersize=8, markerfacecolor='lime')

        # 添加标题
        self.title = self.ax.text(self.map_data['width'] / 2, self.map_data['height'] + 0.5,
                                  "A* 路径规划", ha='center', va='center', fontsize=12)

    def update(self, frame: int):
        """更新动画帧，根据当前帧号更新可视化元素"""
        # 更新探索区域
        explored = self.map_data['explored'][:frame]  # 获取到当前帧为止的探索区域
        if explored:
            x, y = zip(*explored)  # 解压坐标
            # 设置探索区域散点图的位置(坐标+0.5使点位于格子中央)
            self.explored_plot.set_offsets(np.column_stack([np.array(x) + 0.5,
                                                            np.array(y) + 0.5]))

        # 更新开放集
        if frame < len(self.map_data['open_set_history']):
            open_set = list(self.map_data['open_set_history'][frame])  # 获取当前帧的开放集
            if open_set:
                x, y = zip(*open_set)
                # 设置开放集散点图的位置
                self.open_set_plot.set_offsets(np.column_stack([np.array(x) + 0.5,
                                                                np.array(y) + 0.5]))

        # 更新路径(最后几帧显示)
        if self.path and len(self.path) > 0:  # 确保路径存在且不为空
            # 计算要显示的路径长度(在探索完成后开始显示)
            path_to_show = min(max(0, frame - (len(self.map_data['explored']) - 10)),
                               len(self.path))
            if path_to_show > 0:  # 只有当有路径要显示时才处理
                x, y = zip(*self.path[:path_to_show])
                # 设置路径线的数据
                self.path_line.set_data(np.array(x) + 0.5, np.array(y) + 0.5)

        # 更新标题，显示当前帧数
        self.title.set_text(f"A* 路径规划 (帧: {frame}/{len(self.map_data['explored'])})")

        # 返回需要更新的图形元素
        return self.explored_plot, self.open_set_plot, self.path_line, self.title

    def animate(self, save_path: str = None, fps: int = 10):
        """创建并运行动画

        参数:
            save_path: 动画保存路径，可选
            fps: 帧率，默认为10
        """
        try:
            # 计算总帧数(探索帧数+额外20帧用于显示最终路径)
            frames = len(self.map_data['explored']) + 20

            # 创建动画对象
            anim = animation.FuncAnimation(
                self.fig, self.update, frames=frames,
                interval=1000 / fps, blit=True, repeat=False
            )

            if save_path:
                try:
                    # 尝试使用FFmpeg保存为视频
                    writer = animation.FFMpegWriter(
                        fps=fps,
                        metadata={'title': 'A* Path Planning', 'artist': 'Matplotlib'},
                        extra_args=['-vcodec', 'libx264']  # 使用x264编码
                    )
                    anim.save(save_path, writer=writer)
                    print(f"动画已成功保存至 {save_path}")
                except Exception as e:
                    print(f"视频保存失败，将保存为GIF: {str(e)}")
                    # 回退到GIF格式
                    save_path = save_path.replace('.mp4', '.gif')
                    anim.save(save_path, writer='pillow', fps=fps)  # 使用pillow保存GIF
                    print(f"动画已保存为GIF格式: {save_path}")
            else:
                plt.show()  # 如果没有保存路径，直接显示动画

        except Exception as e:
            print(f"生成动画时出错: {str(e)}")
            if "ffmpeg" in str(e).lower():
                # 提供FFmpeg安装建议
                print(">> 解决方案：请安装FFmpeg并添加到系统PATH")
                print(">> Windows用户可下载：https://www.gyan.dev/ffmpeg/builds/")
            raise  # 重新抛出异常