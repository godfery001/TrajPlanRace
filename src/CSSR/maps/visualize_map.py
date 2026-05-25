import json
import numpy as np
import matplotlib.pyplot as plt

def visualize_map(json_path):
    # 读取JSON文件
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    # 获取地图尺寸
    nx = data['boxes_num_x']
    ny = data['boxes_num_y']
    
    # 初始化栅格矩阵 (y在前, x在后, 符合图像坐标系)
    grid = np.zeros((ny, nx))
    
    # 填充栅格数据
    # local_map_element 是一个嵌套列表，外层可能是y或x，内层是对应的元素
    elements = data['local_map_element']
    
    for row in elements:
        for cell in row:
            x_id = cell['x_id']
            y_id = cell['y_id']
            # driveable_index: 1 通常表示可行驶，0 表示障碍物
            # 我们将其可视化为背景和目标
            grid[y_id, x_id] = cell['driveable_index']

    # 绘图
    plt.figure(figsize=(8, 8))
    # origin='lower' 确保 y_id 从下往上增加
    # cmap='Greys' 0为黑色(障碍物)，1为白色(可行驶)
    plt.imshow(grid, origin='lower', cmap='gray', interpolation='nearest')
    
    plt.colorbar(label='Driveable Index (1: Yes, 0: No)')
    plt.title(f"Occupancy Grid Map ({nx}x{ny})")
    plt.xlabel("X Grid ID")
    plt.ylabel("Y Grid ID")
    plt.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
    
    # 保存并显示
    output_png = "/home/siegf/TrajPlanRace/src/CSSR/maps/local_map_visualization.png"
    plt.savefig(output_png)
    print(f"Visualization saved to {output_png}")
    plt.show()

if __name__ == "__main__":
    map_file = "/home/siegf/TrajPlanRace/src/CSSR/maps/local_map.json"
    visualize_map(map_file)
