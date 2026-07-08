import json
import os
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import matplotlib as mpl

# 设置默认字体避免中文警告
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 使用通用无衬线字体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
mpl.rcParams['pdf.fonttype'] = 42  # 避免PDF保存时的字体问题

def create_output_directory(json_file_path):
    """在JSON文件所在目录创建同名的输出目录"""
    base_dir = os.path.dirname(json_file_path)
    base_name = os.path.basename(json_file_path)
    file_name_without_ext = os.path.splitext(base_name)[0]
    
    output_dir = os.path.join(base_dir, file_name_without_ext)
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"创建输出目录: {output_dir}")
    return output_dir

def load_position_data(file_path):
    """从JSON文件加载位置数据"""
    data_points = []
    
    with open(file_path, 'r') as f:
        for line_num, line in enumerate(f):
            try:
                data = json.loads(line)
                data_points.append(data)
            except json.JSONDecodeError:
                print(f"警告: 跳过第 {line_num} 行 - JSON格式错误")
                continue
    
    if not data_points:
        print("错误: 未找到有效数据点!")
        return None
    
    # 提取时间戳
    timestamps = []
    for data in data_points:
        sec = data['timestamp']['sec']
        nanosec = data['timestamp']['nanosec'] / 1e9
        timestamp = sec + nanosec
        timestamps.append(timestamp)
    
    # 确保timestamps是NumPy数组
    timestamps = np.array(timestamps, dtype=np.float64)
    
    # 创建相对时间（从0开始）
    base_time = np.min(timestamps)
    relative_times = timestamps - base_time
    
    # 提取位置数据
    x_positions = np.array([data['position']['x'] for data in data_points], dtype=np.float64)
    y_positions = np.array([data['position']['y'] for data in data_points], dtype=np.float64)
    z_positions = np.array([data['position']['z'] for data in data_points], dtype=np.float64)
    
    # 计算统计信息
    start_datetime = datetime.fromtimestamp(base_time).strftime("%Y-%m-%d %H:%M:%S")
    duration = np.max(relative_times) - np.min(relative_times)
    
    return {
        'file_name': os.path.basename(file_path),
        'times': relative_times,
        'x': x_positions,
        'y': y_positions,
        'z': z_positions,
        'data_points': data_points,
        'metadata': {
            'start_time': start_datetime,
            'duration': duration,
            'point_count': len(data_points),
        }
    }

def plot_time_series(position_data, output_dir, image_prefix):
    """绘制位置分量的时间序列图"""
    if len(position_data['times']) == 0:
        print("Warning: No data for time series plot")
        return
    
    plt.figure(figsize=(14, 10))
    
    # 主标题和子图
    plt.suptitle(f"Position Components Time Series - {position_data['metadata']['start_time']}", fontsize=16)
    
    # X位置分量
    plt.subplot(3, 1, 1)
    plt.plot(position_data['times'], position_data['x'], 'r-', linewidth=1.8)
    plt.plot(position_data['times'], position_data['x'], 'ro', markersize=3, alpha=0.3)
    plt.grid(True, alpha=0.4)
    plt.ylabel('X (m)', fontsize=12)
    plt.title('X Position Component', fontsize=13)
    
    # Y位置分量
    plt.subplot(3, 1, 2)
    plt.plot(position_data['times'], position_data['y'], 'g-', linewidth=1.8)
    plt.plot(position_data['times'], position_data['y'], 'go', markersize=3, alpha=0.3)
    plt.grid(True, alpha=0.4)
    plt.ylabel('Y (m)', fontsize=12)
    plt.title('Y Position Component', fontsize=13)
    
    # Z位置分量
    plt.subplot(3, 1, 3)
    plt.plot(position_data['times'], position_data['z'], 'b-', linewidth=1.8)
    plt.plot(position_data['times'], position_data['z'], 'bo', markersize=3, alpha=0.3)
    plt.grid(True, alpha=0.4)
    plt.ylabel('Z (m)', fontsize=12)
    plt.xlabel('Time (s)', fontsize=12)
    plt.title('Z Position Component', fontsize=13)
    
    # 保存并显示
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    output_path = os.path.join(output_dir, f"{image_prefix}_time_series.png")
    plt.savefig(output_path, dpi=200, bbox_inches='tight')
    print(f"Saved time series to: {output_path}")
    plt.close()

def get_latest_file(directory, extension=".json"):
    """获取指定目录下最新的指定扩展名文件"""
    files = [os.path.join(directory, f) for f in os.listdir(directory) if f.endswith(extension)]
    if not files:
        print(f"Error: No {extension} files found in {directory}")
        return None
    latest_file = max(files, key=os.path.getmtime)
    return latest_file

def main():
     # 配置输入文件目录
    json_dir = "/home/elf/ros2_local_position_pose_logs"
    
    # 获取最新的 JSON 文件
    json_file = get_latest_file(json_dir, extension=".json")
    if not json_file:
        return
    
    print(f"Latest file selected: {json_file}")
    
    # 在JSON文件所在目录创建同名输出目录
    output_dir = create_output_directory(json_file)
    
    # 获取文件名前缀
    file_name_without_ext = os.path.splitext(os.path.basename(json_file))[0]
    
    # 加载数据
    print(f"Loading data: {json_file}")
    position_data = load_position_data(json_file)
    
    if not position_data:
        return
    
    # 生成时间序列图
    print("\nGenerating time series plot:")
    plot_time_series(position_data, output_dir, file_name_without_ext)
    
    print("\nProcessing complete! All plots saved to:")
    print(f"  {output_dir}")

if __name__ == "__main__":
    main()
