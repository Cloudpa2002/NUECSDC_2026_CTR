# consumer.py
"""
ssh elf@192.168.43.226  'source /opt/ros/humble/setup.bash && python3 -u /home/elf/1main_controller/environmental_monitoring.py'  |  python3 -u consumer.py
"""

import sys

for line in sys.stdin:
    line = line.strip()

    if not line:
        continue

    try:
        parts = line.split(',')
        if len(parts) != 5:
            continue

        temperature = float(parts[0])
        humidity = float(parts[1])
        pm25 = float(parts[2])
        x = float(parts[3])
        y = float(parts[4])

        print(
            f"温度: {temperature}°C | "
            f"湿度: {humidity}%RH | "
            f"PM2.5: {pm25}μg/m³ | "
            f"坐标: x={x:.2f}, y={y:.2f}",
            flush=True
        )
    except ValueError:
        pass
