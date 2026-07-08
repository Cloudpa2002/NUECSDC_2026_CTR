import os
import cv2  # 导入 OpenCV 库

GPIO_BASE = "/sys/class/gpio"

def control_gpio(pin, value=None):
    """
    控制指定GPIO引脚
    :param pin: GPIO引脚编号 (如103)
    :param value: 设置的值 (0/1/None)，None表示仅初始化不设置值
    """
    # 确保引脚已导出
    gpio_dir = os.path.join(GPIO_BASE, f"gpio{pin}")
    export_path = os.path.join(GPIO_BASE, "export")
    
    if not os.path.exists(gpio_dir):
        try:
            with open(export_path, 'w') as f:
                f.write(str(pin))
        except IOError as e:
            raise RuntimeError(f"无法导出GPIO {pin}: {e}")

    # 设置方向为输出
    direction_path = os.path.join(gpio_dir, "direction")
    try:
        with open(direction_path, 'w') as f:
            f.write("out")
    except IOError as e:
        raise RuntimeError(f"无法设置GPIO{pin}方向: {e}")

    # 设置值 (如果指定了值)
    if value is not None:
        value_path = os.path.join(gpio_dir, "value")
        try:
            with open(value_path, 'w') as f:
                f.write("1" if value else "0")
        except IOError as e:
            raise RuntimeError(f"无法设置GPIO{pin}值: {e}")


def take_photo(save_path):
    """
    使用 OpenCV 拍照并保存到指定位置
    :param save_path: 照片保存的完整路径 (如 "/home/user/photos/photo.jpg")
    """
    # 打开默认摄像头 (设备索引为 0)
    cap = cv2.VideoCapture(21)

    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头，请检查设备连接。")

    # 设置摄像头分辨率为 640 x 480
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 读取一帧图像
    ret, frame = cap.read()

    if not ret:
        cap.release()
        raise RuntimeError("无法从摄像头读取图像。")

    # 保存图像到指定路径
    cv2.imwrite(save_path, frame)
    print(f"照片已保存到: {save_path}")

    # 释放摄像头资源
    cap.release()


# 示例使用
if __name__ == "__main__":
    # 初始化GPIO103并设置高电平
     #control_gpio(103, value=1)
    
    # # 设置GPIO103为低电平
     control_gpio(103, value=0)
    
    # # 仅初始化引脚不改变值
    # control_gpio(103)
