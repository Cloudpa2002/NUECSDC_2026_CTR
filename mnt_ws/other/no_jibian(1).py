import cv2
import numpy as np
import os

# ========== 配置参数 ==========
template_path = './images/trcir.png'          # 模板图片路径
output_size = (640, 480)             # 图像统一尺寸
camera_index = 21                   # 摄像头编号

def is_close_center(c1, c2, threshold=10):
    """判断两个圆心距离是否接近"""
    return np.hypot(c1[0] - c2[0], c1[1] - c2[1]) < threshold

def extract_circles_from_image(image):
    """从图像中提取圆环，返回圆心和半径"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = np.ones((3, 3), np.uint8)
    morphed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, hierarchy = cv2.findContours(morphed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    rings = []
    if hierarchy is None:
        return rings
    for i, cnt in enumerate(contours):
        if hierarchy[0][i][3] == -1 or cv2.contourArea(cnt) < 50:
            continue
        parent_idx = hierarchy[0][i][3]
        outer_cnt = contours[parent_idx]
        area_outer = cv2.contourArea(outer_cnt)
        area_inner = cv2.contourArea(cnt)
        if area_inner < 0.1 * area_outer or area_outer < 50:
            continue
        (x, y), r = cv2.minEnclosingCircle(outer_cnt)
        area_circle = np.pi * r * r
        circularity = area_outer / (area_circle + 1e-6)
        if r < 10 or circularity < 0.5:
            continue
        rings.append((int(x), int(y), int(r)))
    return rings

def match_circles(template_circles, test_circles, threshold_center=10, threshold_radius=10):
    """判断检测到的圆环组是否与模板结构匹配"""
    if len(test_circles) < 3 or len(template_circles) < 3:
        return False
    template_sorted = sorted(template_circles, key=lambda c: c[2])
    template_rs = sorted([c[2] for c in template_sorted[:3]])
    for i in range(len(test_circles)):
        cx, cy, _ = test_circles[i]
        group = [test_circles[i]]
        for j in range(len(test_circles)):
            if i == j:
                continue
            if is_close_center((cx, cy), (test_circles[j][0], test_circles[j][1]), threshold_center):
                group.append(test_circles[j])
        if len(group) >= 3:
            group = sorted(group, key=lambda c: c[2])[:3]
            test_rs = sorted([c[2] for c in group])
            ratios_template = [template_rs[1] / template_rs[0], template_rs[2] / template_rs[0]]
            ratios_test = [test_rs[1] / test_rs[0], test_rs[2] / test_rs[0]]
            if all(abs(r1 - r2) < 0.2 for r1, r2 in zip(ratios_template, ratios_test)):
                if all(abs(r1 - r2) < threshold_radius for r1, r2 in zip(template_rs, test_rs)):
                    return True
    return False

def draw_center_x_symbol(img, center, color=(0,0,255), size=14, thickness=2):
    """在指定位置画红色X"""
    x, y = center
    cv2.line(img, (x-size, y-size), (x+size, y+size), color, thickness)
    cv2.line(img, (x-size, y+size), (x+size, y-size), color, thickness)

def process_frame(frame, template_circles):
    # 旋转180度
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    # resize到统一尺寸
    # frame = cv2.resize(frame, output_size)
    h, w = frame.shape[:2]
    x0, y0 = int(w * 0.1), int(h * 0)
    x1, y1 = int(w * 0.9), int(h * 0.7)
    roi_frame = frame[y0:y1, x0:x1].copy()
    circles = extract_circles_from_image(roi_frame)
    found = False
    group = []
    for i in range(len(circles)):
        cx, cy, _ = circles[i]
        temp_group = [circles[i]]
        for j in range(len(circles)):
            if i == j:
                continue
            if is_close_center((cx, cy), (circles[j][0], circles[j][1])):
                temp_group.append(circles[j])
        if len(temp_group) >= 3:
            temp_group = sorted(temp_group, key=lambda c: c[2])[:3]
            group = temp_group
            found = True
            break
    match_template = False
    mm_per_px = None
    dx_mm = dy_mm = None
    diameters = None
    output = frame.copy()
    cv2.rectangle(output, (x0, y0), (x1, y1), (255, 0, 0), 2)
    cx_img, cy_img = w // 2, h // 2
    draw_center_x_symbol(output, (cx_img, cy_img), color=(0,0,255), size=14, thickness=2)
    if found:
        match_template = match_circles(template_circles, group)
        actual_diams = [110, 250, 400]
        pixel_diams = [2*r for (_, _, r) in sorted(group, key=lambda c: c[2])]
        scale_list = [real/pixel for real, pixel in zip(actual_diams, pixel_diams)]
        mm_per_px = sum(scale_list) / len(scale_list)
        diameters = pixel_diams
        xc, yc = x0 + group[0][0], y0 + group[0][1]
        dx = xc - cx_img
        dy = yc - cy_img
        dx_mm = dx * mm_per_px
        dy_mm = dy * mm_per_px
        # 横/竖红线
        cv2.line(output, (cx_img, cy_img), (xc, cy_img), (0, 0, 255), 2)
        cv2.line(output, (xc, cy_img), (xc, yc), (0, 0, 255), 2)
        # 圆环和圆心
        for x, y, r in group:
            cv2.circle(output, (x0 + x, y0 + y), r, (0, 255, 0), 2)
        cv2.circle(output, (xc, yc), 3, (0, 0, 255), -1)
        # 右下角显示dx_mm和dy_mm
        margin = 10
        txt1 = f"dx: {dx_mm:.1f} mm"
        txt2 = f"dy: {dy_mm:.1f} mm"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.8
        thickness = 2
        ((tw1, th1), _) = cv2.getTextSize(txt1, font, scale, thickness)
        ((tw2, th2), _) = cv2.getTextSize(txt2, font, scale, thickness)
        bx = w - max(tw1, tw2) - margin
        by1 = h - margin - th2 - 5
        by2 = h - margin
        cv2.putText(output, txt1, (bx, by1), font, scale, (0,0,255), thickness)
        cv2.putText(output, txt2, (bx, by2), font, scale, (0,0,255), thickness)
        if match_template:
            cv2.putText(output, "Triple rings match template!", (20, 40),
                        font, 1, (0, 200, 255), 2)
        else:
            cv2.putText(output, "Triple concentric rings detected", (20, 40),
                        font, 1, (0, 255, 0), 2)
    else:
        cv2.putText(output, "No triple rings detected", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    return output

def main():
    # 读取模板图片
    template_img_raw = cv2.imread(template_path)
    if template_img_raw is None:
        raise FileNotFoundError(f"Template image not found. Ensure {template_path} exists.")
    # template_img = cv2.resize(template_img_raw, output_size)
    template_circles = extract_circles_from_image(template_img_raw)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("Camera open failed!")
        return
    # # 设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 设置宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置高度

    print("按q退出。")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera frame not read!")
            break
        output = process_frame(frame, template_circles)
        cv2.imshow("Triple Ring Detection", output)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()