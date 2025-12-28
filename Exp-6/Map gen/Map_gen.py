import matplotlib.pyplot as plt
import re

def read_shapes_from_file(filename):
    """从文件中读取矩形和圆数据，返回矩形列表和圆列表"""
    rectangles = []
    circles = []
    current_rect = []
    
    # 用于跟踪是否正在读取圆数据
    reading_circle = False
    circle_data = []
    
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()  # 去除首尾空白
            if not line:
                continue  # 跳过空行
            
            if line == 'Rect':
                # 遇到新的矩形标记，如果当前有未完成的矩形，先保存
                if current_rect:
                    rectangles.append(current_rect)
                    current_rect = []
                # 确保退出圆数据读取状态
                reading_circle = False
                circle_data = []
            elif line == 'Circle':
                # 遇到圆标记，开始读取圆数据
                reading_circle = True
                circle_data = []
                # 确保退出矩形数据读取状态
                if current_rect:
                    rectangles.append(current_rect)
                    current_rect = []
            elif reading_circle:
                # 正在读取圆数据，收集每个参数
                circle_data.append(line)
                # 当收集到6个参数时（Circle, id, x, y, direction, radius）
                if len(circle_data) == 5:  # 已经有Circle标记，现在收集了5个参数
                    try:
                        circle_id = int(circle_data[0])
                        x = float(circle_data[1])
                        y = float(circle_data[2])
                        # 方向未启用，跳过circle_data[3]
                        radius = float(circle_data[4])
                        circles.append({
                            'id': circle_id,
                            'x': x,
                            'y': y,
                            'radius': radius
                        })
                    except (ValueError, IndexError):
                        print(f"警告：无法解析圆数据：{circle_data}")
                    # 重置圆数据状态
                    reading_circle = False
                    circle_data = []
            else:
                # 解析坐标行（格式如：(x,y)）用于矩形
                match = re.match(r'\((-?\d+\.?\d*),(-?\d+\.?\d*)\)', line)
                if match:
                    x = float(match.group(1))
                    y = float(match.group(2))
                    current_rect.append((x, y))
    
    # 添加最后一个矩形（如果文件结束时存在未保存的）
    if current_rect:
        rectangles.append(current_rect)
    
    return rectangles, circles

def plot_shapes(rectangles, circles):
    """绘制所有矩形和圆"""
    plt.figure(figsize=(12, 10))
    
    # 定义区分颜色
    colors = ['#FF5733', '#33FF57', '#3357FF', '#F3FF33', '#FF33F3', '#33FFF3', '#8844FF', '#FF8800']
    
    # 收集所有坐标用于计算坐标轴范围
    all_x = []
    all_y = []
    
    # 绘制每个矩形
    for i, rect in enumerate(rectangles):
        if len(rect) != 4:
            print(f"警告：第{i+1}个矩形顶点数量不正确（应为4个），已跳过")
            continue
        
        # 提取x、y坐标并闭合矩形（最后一点连接回起点）
        x_coords = [p[0] for p in rect] + [rect[0][0]]
        y_coords = [p[1] for p in rect] + [rect[0][1]]
        
        # 绘制矩形
        plt.plot(x_coords, y_coords, color=colors[i % len(colors)], linewidth=2, label=f'Rect {i+1}')
        
        # 收集坐标
        all_x.extend(p[0] for p in rect)
        all_y.extend(p[1] for p in rect)
    
    # 绘制每个圆
    for i, circle in enumerate(circles):
        # 为圆选择颜色，从矩形颜色之后开始
        color_index = (len(rectangles) + i) % len(colors)
        
        # 创建圆形
        circle_patch = plt.Circle((circle['x'], circle['y']), circle['radius'], 
                                 color=colors[color_index], alpha=0.7, 
                                 label=f'Circle {circle["id"]}')
        plt.gca().add_patch(circle_patch)
        
        # 收集坐标（考虑圆心和半径以确保完整显示）
        all_x.extend([circle['x'] - circle['radius'], circle['x'] + circle['radius']])
        all_y.extend([circle['y'] - circle['radius'], circle['y'] + circle['radius']])
    
    # 计算坐标轴范围（添加10%余量，确保所有图形完整显示）
    if all_x and all_y:
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        x_pad = (x_max - x_min) * 0.1 if x_max != x_min else 10
        y_pad = (y_max - y_min) * 0.1 if y_max != y_min else 10
        plt.xlim(x_min - x_pad, x_max + x_pad)
        plt.ylim(y_min - y_pad, y_max + y_pad)
    
    # 设置图形属性
    plt.xlabel('X', fontsize=12)
    plt.ylabel('Y', fontsize=12)
    plt.title('Map', fontsize=14)
    plt.axis('equal')  # 等比例显示，保证图形形状正确
    plt.grid(linestyle='--', alpha=0.6)  # 网格线辅助观察
    plt.legend(loc='best', fontsize=10)  # 自动选择最佳图例位置
    plt.tight_layout()  # 自动调整布局
    
    # 显示图形
    plt.show()

if __name__ == "__main__":
    # 读取文件中的矩形和圆信息
    rectangles, circles = read_shapes_from_file('data.txt')
    
    # 打印读取结果信息
    print(f"成功读取到 {len(rectangles)} 个矩形")
    print(f"成功读取到 {len(circles)} 个圆")
    
    # 绘制图形
    if rectangles or circles:
        print("开始绘制图形...")
        plot_shapes(rectangles, circles)
    else:
        print("未从文件中读取到任何图形信息")