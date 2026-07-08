#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <vector>

class DroneTargetPositioning {
private:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::map<std::string, std::vector<double>> objectSizes;

public:
    DroneTargetPositioning(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) 
        : cameraMatrix(cameraMatrix), distCoeffs(distCoeffs) {
        // 初始化物体尺寸
        objectSizes["A"] = {0.224, 0.306};
        objectSizes["B"] = {0.191, 0.31};
        objectSizes["C"] = {0.07, 0.15};
    }

    std::vector<double> pnpMethod(const std::vector<double>& yoloDetection, const std::string& objectKey = "A") {
        // 检查物体类型是否存在
        if (objectSizes.find(objectKey) == objectSizes.end()) {
            std::cout << "Warning: 未找到 '" << objectKey << "' 的尺寸，使用默认尺寸 A" << std::endl;
            return pnpMethod(yoloDetection, "A");
        }

        double realWidth = objectSizes[objectKey][0];
        double realHeight = objectSizes[objectKey][1];

        // 解析检测框坐标
        double x_min = yoloDetection[0];
        double y_min = yoloDetection[1];
        double x_max = yoloDetection[2];
        double y_max = yoloDetection[3];

        // 提取4个角点的2D坐标（顺时针顺序：左上、右上、右下、左下）
        std::vector<cv::Point2f> imagePoints;
        imagePoints.push_back(cv::Point2f(x_min, y_min));  // 左上
        imagePoints.push_back(cv::Point2f(x_max, y_min));  // 右上
        imagePoints.push_back(cv::Point2f(x_max, y_max));  // 右下
        imagePoints.push_back(cv::Point2f(x_min, y_max));  // 左下

        // 定义物体的3D角点（以中心为原点）
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(-realWidth/2, -realHeight/2, 0));  // 左上
        objectPoints.push_back(cv::Point3f(realWidth/2, -realHeight/2, 0));   // 右上
        objectPoints.push_back(cv::Point3f(realWidth/2, realHeight/2, 0));    // 右下
        objectPoints.push_back(cv::Point3f(-realWidth/2, realHeight/2, 0));   // 左下

        // PnP解算
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

        if (success) {
            return {tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)};
        } else {
            return {};
        }
    }
};

int main() {
    // 摄像头参数
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
        478.3976, 0, 333.4439,
        0, 478.2616, 216.3604,
        0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 
        -0.0710900, 0.22020602229390, 0.0010901565, -0.0012054, -0.1979014);

    // 初始化定位器
    DroneTargetPositioning positioning(cameraMatrix, distCoeffs);

    // 模拟YOLO检测结果 [x_min, y_min, x_max, y_max]
    std::vector<double> yoloResult = {0.6, 0.4, 0.2, 0.15};

    // 计算物体位置
    auto position = positioning.pnpMethod(yoloResult, "B");

    if (!position.empty()) {
        std::cout << "物体相对无人机的位置偏移:" << std::endl;
        std::cout << "X: " << position[0] << " 米" << std::endl;
        std::cout << "Y: " << position[1] << " 米" << std::endl;
        std::cout << "Z: " << position[2] << " 米" << std::endl;
    } else {
        std::cout << "定位失败" << std::endl;
    }

    return 0;
}