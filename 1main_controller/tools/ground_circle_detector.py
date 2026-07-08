"""
圆形检测器
功能：
    检测指定摄像头画面中的圆，并输出圆心相对于摄像头的三维坐标
用法：
    调用 ground_circle_detector 或 ground_circle_detector 函数
"""

import cv2
import math
import time


def ground_circle_detector(CAMERA_INDEX=21, D_REAL=0.80, fx=732.22, fy=730.01,
                           cx_camera=623.39, cy_camera=445.52):
    """
    打开摄像头，识别画面中的圆形，计算圆心相对于相机坐标系的三维坐标。

    参数:
        CAMERA_INDEX: 相机设备编号，默认 21
        D_REAL:      圆的真实直径（米），默认 0.41
        fx:          相机x方向焦距，默认 732.22
        fy:          相机y方向焦距，默认 730.01
        cx_camera:   相机主点横向像素坐标，默认 623.39
        cy_camera:   相机主点纵向像素坐标，默认 445.52

    返回:
        True
    """
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)

    if not cap.isOpened():
        cap.release()
        return True

    start_time = time.time()
    period = 1.0 / 10.0  # 10Hz

    while True:
        loop_start = time.time()

        # 运行至少 n 秒后自动退出
        if loop_start - start_time >= 4.0:
            break

        ret, frame = cap.read()
        if not ret:
            continue

        if len(frame.shape) == 2:
            gray = frame
            out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            out = frame.copy()

        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 180)

        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 2000:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri < 100:
                continue

            if len(cnt) < 5:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = w / float(h)
            if ratio < 0.75 or ratio > 1.25:
                continue

            circularity = 4 * math.pi * area / (peri * peri)
            if circularity < 0.75:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (u, v), (axis1, axis2), angle = ellipse

            major_axis = max(axis1, axis2)
            minor_axis = min(axis1, axis2)

            # 用长短轴平均值估算圆的像素直径
            D_pixel = (major_axis + minor_axis) / 2.0

            if D_pixel <= 0:
                continue

            # 深度估算，单位：米
            Z = fx * D_REAL / D_pixel

            # 圆心相对于相机坐标系的坐标，单位：米
            X = (u - cx_camera) * Z / fx
            Y = (v - cy_camera) * Z / fy

            cv2.ellipse(out, ellipse, (0, 255, 0), 2)
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(out, (int(u), int(v)), 5, (0, 0, 255), -1)

            cv2.putText(
                out,
                f"X={X:.3f}m Y={Y:.3f}m Z={Z:.3f}m",
                (int(u) + 10, int(v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            cv2.putText(
                out,
                f"D_pixel={D_pixel:.1f}px",
                (int(u) + 10, int(v) + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        cv2.imshow("result", out)

        cv2.waitKey(1)

        # 维持 10Hz 帧率
        elapsed = time.time() - loop_start
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    cap.release()
    cv2.destroyAllWindows()
    return True


def ground_circle_detector1(CAMERA_INDEX=21, D_REAL=0.80, fx=732.22, fy=730.01,
                           cx_camera=623.39, cy_camera=445.52):
    """
    打开摄像头，识别画面中的圆形，计算圆心相对于相机坐标系的三维坐标。

    参数:
        CAMERA_INDEX: 相机设备编号，默认 21
        D_REAL:      圆的真实直径（米），默认 0.41
        fx:          相机x方向焦距，默认 732.22
        fy:          相机y方向焦距，默认 730.01
        cx_camera:   相机主点横向像素坐标，默认 623.39
        cy_camera:   相机主点纵向像素坐标，默认 445.52

    返回:
        True
    """
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)

    if not cap.isOpened():
        cap.release()
        return True

    detect_count = 0
    TARGET_COUNT = 10  # 计算出10次圆心坐标后退出
    period = 1.0 / 10.0  # 10Hz

    while True:
        loop_start = time.time()

        # 计算出指定次数圆心坐标后自动退出
        if detect_count >= TARGET_COUNT:
            break

        ret, frame = cap.read()
        if not ret:
            continue

        if len(frame.shape) == 2:
            gray = frame
            out = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            out = frame.copy()

        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 180)

        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 2000:
                continue

            peri = cv2.arcLength(cnt, True)
            if peri < 100:
                continue

            if len(cnt) < 5:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = w / float(h)
            if ratio < 0.75 or ratio > 1.25:
                continue

            circularity = 4 * math.pi * area / (peri * peri)
            if circularity < 0.75:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (u, v), (axis1, axis2), angle = ellipse

            major_axis = max(axis1, axis2)
            minor_axis = min(axis1, axis2)

            # 用长短轴平均值估算圆的像素直径
            D_pixel = (major_axis + minor_axis) / 2.0

            if D_pixel <= 0:
                continue

            # 深度估算，单位：米
            Z = fx * D_REAL / D_pixel

            # 圆心相对于相机坐标系的坐标，单位：米
            X = (u - cx_camera) * Z / fx
            Y = (v - cy_camera) * Z / fy

            # 成功计算出一次圆心坐标，计数+1
            detect_count += 1

            cv2.ellipse(out, ellipse, (0, 255, 0), 2)
            cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(out, (int(u), int(v)), 5, (0, 0, 255), -1)

            cv2.putText(
                out,
                f"X={X:.3f}m Y={Y:.3f}m Z={Z:.3f}m",
                (int(u) + 10, int(v) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2
            )

            cv2.putText(
                out,
                f"D_pixel={D_pixel:.1f}px",
                (int(u) + 10, int(v) + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2
            )

        cv2.imshow("result", out)

        cv2.waitKey(1)

        # 维持 10Hz 帧率
        elapsed = time.time() - loop_start
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    cap.release()
    cv2.destroyAllWindows()
    return True
