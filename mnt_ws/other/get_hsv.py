import cv2
import numpy as np

# 鼠标回调函数
def get_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        frame = param['frame']
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv_value = hsv_frame[y, x]
        print(f"点击坐标: ({x}, {y}) -> HSV值: {hsv_value}")

# 打开摄像头
cap = cv2.VideoCapture(21)  # 0 表示默认摄像头

cv2.namedWindow("Camera")
# 设置摄像头分辨率
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度
frame_container = {'frame': None}
cv2.setMouseCallback("Camera", get_hsv_value, frame_container)

print("按 ESC 键退出程序")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_container['frame'] = frame.copy()
    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC 键退出
        break

cap.release()
cv2.destroyAllWindows()
