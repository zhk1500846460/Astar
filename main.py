import random
from map_visual import VisualMap
from astar import AStar
from path_animation import PathAnimation
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题


def main():
    """主函数：演示完整的A*路径规划流程"""
    print("=== A*路径规划演示 ===")

    # 1. 创建地图
    map_width = 20
    map_height = 20
    grid_map = VisualMap(map_width, map_height, cell_size=0.8)
    print(f"已创建 {map_width}x{map_height} 的地图")

    # 2. 设置障碍物 (随机生成或手动设置)
    obstacle_density = 0.1  # 障碍物密度(20%)
    obstacles = []
    for y in range(map_height):
        for x in range(map_width):
            if random.random() < obstacle_density and (x, y) not in [(0, 0), (map_width - 1, map_height - 1)]:
                obstacles.append((x, y))
    grid_map.set_obstacles(obstacles)
    print(f"已设置 {len(obstacles)} 个障碍物")

    # 3. 设置起点和终点
    start_pos = (0, 0)
    goal_pos = (map_width - 1, map_height - 1)
    grid_map.set_start(*start_pos)
    grid_map.set_goal(*goal_pos)
    print(f"起点: {start_pos}, 终点: {goal_pos}")

    # 4. 创建A*规划器
    allow_diagonal = True  # 是否允许对角线移动
    planner = AStar(grid_map, allow_diagonal=allow_diagonal)
    print("A*规划器已创建", "(允许对角线移动)" if allow_diagonal else "(仅允许四方向移动)")

    # 5. 执行路径规划
    print("\n开始路径规划...")
    result = planner.plan(start_pos, goal_pos)

    # 在main函数中修改这部分：
    if result['path']:
        print(f"\n规划成功! 路径长度: {len(result['path'])}步")
        print(f"探索节点数: {len(result['explored'])}")
        print(f"计算时间: {result['runtime']:.4f}秒")

        # 显示静态路径
        grid_map.draw_path(result['path'], color='purple')
        grid_map.show()

        # 生成动画
        generate_animation = input("\n是否要生成搜索过程动画? (y/n): ").lower() == 'y'
        if generate_animation:
            # 确保结果字典包含正确的键
            if 'open_set' in result and 'open_set_history' not in result:
                result['open_set_history'] = result['open_set']

            try:
                animator = PathAnimation(result, result['path'])
                save_option = input("是否保存为视频文件? (y/n): ").lower() == 'y'
                if save_option:
                    filename = input("输入视频文件名(如 astar.mp4): ") or "astar.mp4"
                    if not filename.endswith('.mp4'):
                        filename += '.mp4'
                    fps = int(input("输入帧率(推荐10-20): ") or 15)
                    print("正在生成动画，请稍候...")
                    animator.animate(save_path=filename, fps=fps)
                    print(f"动画已保存至 {filename}")
                else:
                    print("正在显示动画...")
                    animator.animate()
            except Exception as e:
                print(f"生成动画时出错: {str(e)}")
                print("请检查FFmpeg是否已安装并配置正确")


if __name__ == "__main__":
    main()