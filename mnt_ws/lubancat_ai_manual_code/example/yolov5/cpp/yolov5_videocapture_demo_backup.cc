// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "yolov5.h"
#include "easy_timer.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <map>
#include <vector>

static const unsigned char colors[19][3] = {
    {54, 67, 244},
    {99, 30, 233},
    {176, 39, 156},
    {183, 58, 103},
    {181, 81, 63},
    {243, 150, 33},
    {244, 169, 3},
    {212, 188, 0},
    {136, 150, 0},
    {80, 175, 76},
    {74, 195, 139},
    {57, 220, 205},
    {59, 235, 255},
    {7, 193, 255},
    {0, 152, 255},
    {34, 87, 255},
    {72, 85, 121},
    {158, 158, 158},
    {139, 125, 96}
};


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


/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov5s.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov5s.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_name = argv[2];

    // 添加摄像头参数
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 
        478.3976, 0, 333.4439,
        0, 478.2616, 216.3604,
        0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << 
        -0.0710900, 0.22020602229390, 0.0010901565, -0.0012054, -0.1979014);

    // 初始化定位器
    DroneTargetPositioning positioning(cameraMatrix, distCoeffs);



    int ret;
    TIMER timer;
    cv::Mat image, frame;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;
    object_detect_result_list od_results;

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));

    cv::VideoCapture cap;
    // if (isdigit(device_name[0])) {
    //     // 打开摄像头
    //     int camera_id = atoi(argv[2]);
    //     cap.open(camera_id);
    //     if (!cap.isOpened()) {
    //         printf("Error: Could not open camera.\n");
    //         return -1;
    //     }
    //     // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //     // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);//宽度
    //     // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);//高度
    // } else {
    //     // 打开视频文件等
    //     cap.open(argv[2]);
    //     if (!cap.isOpened()) {  
    //         printf("Error: Could not open video file.\n");
    //         return -1;
    //     }
    // }

    // 修改摄像头/视频打开逻辑：
    bool is_camera_id = true;
    for (int i = 0; i < strlen(device_name); i++) {
        if (!isdigit(device_name[i])) {
            is_camera_id = false;
            break;
        }
    }

    if (is_camera_id) {
        int camera_id = atoi(device_name);     

        std::string pipeline = "v4l2src device=/dev/video11 ! video/x-raw, width=640, height=640, format=NV12, framerate=30/1 ! videoconvert ! appsink";
        cap.open(pipeline, cv::CAP_GSTREAMER);

        
        // cap.open(camera_id);
        
        if (!cap.isOpened()) {
            printf("Error: Could not open camera %d.\n", camera_id);
            return -1;
        }

        // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);//宽度
        // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 640);//高度
    } else {
        // 视频文件模式
        cap.open(device_name);
        if (!cap.isOpened()) {  
            printf("Error: Could not open video file: %s\n", device_name);
            return -1;
        }
    }


    // 初始化
    init_post_process();
#ifndef ENABLE_ZERO_COPY
    ret = init_yolov5_model(model_path, &rknn_app_ctx);
#else
    ret = init_yolov5_zero_copy_model(model_path, &rknn_app_ctx);
#endif
    if (ret != 0)
    {
        printf("init yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    
    // 推理，画框，显示
	while(true) {
        gettimeofday(&start_time, NULL);

		// cap >> frame;
        if (!cap.read(frame)) {  
            printf("cap read frame fail!\n");
            break;  
        }  

        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        // image.convertTo(image, CV_8UC3);
        src_image.width  = image.cols;
        src_image.height = image.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = (unsigned char*)image.data;

        timer.tik();
#ifndef ENABLE_ZERO_COPY
        ret = inference_yolov5_model(&rknn_app_ctx, &src_image, &od_results);
#else
        ret = inference_yolov5_zero_copy_model(&rknn_app_ctx, &src_image, &od_results);
#endif
        if (ret != 0)
        {
            printf("inference yolov5_model fail! ret=%d\n", ret);
            goto out;
        }
        timer.tok();
        timer.print_time("inference_yolov5_model");

        char text[256];
        int color_index = 0;
        for (int i = 0; i < od_results.count; i++)
        {
            const unsigned char* color = colors[color_index % 19];
            cv::Scalar cc(color[0], color[1], color[2]);
            color_index++;

            object_detect_result *det_result = &(od_results.results[i]);
            printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                det_result->prop);
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
            
            std::vector<double> yoloDetection = {
                det_result->box.left,
                det_result->box.top,
                det_result->box.right,
                det_result->box.bottom
            };
            // 调用PnP方法（假设检测到的物体类型为"B"）
            auto position = positioning.pnpMethod(yoloDetection, "B");
            if (!position.empty()) {
                // 在图像上显示位置信息
                char pos_text[256];
                printf("Detected %s position: X=%.2fm Y=%.2fm Z=%.2fm\n", 
                      coco_cls_to_name(det_result->cls_id),
                      position[0], position[1], position[2]);
            }
            
            cv::rectangle(frame, cv::Rect(cv::Point(det_result->box.left, det_result->box.top), 
                            cv::Point(det_result->box.right, det_result->box.bottom)), cc, 2);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int x = det_result->box.left;
            int y = det_result->box.top - label_size.height - baseLine;
            if (y < 0)
                y = 0;
            if (x + label_size.width > frame.cols)
                x = frame.cols - label_size.width;

            cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),cc,-1);
            cv::putText(frame, text, cv::Point(x, y + label_size.height),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));
        }
		cv::imshow("YOLOv5 Videocapture Demo", frame);

		char c = cv::waitKey(1);
		if (c == 27) { // ESC
			break;
		}

    }

out:
    deinit_post_process();

#ifndef ENABLE_ZERO_COPY
    ret = release_yolov5_model(&rknn_app_ctx);
#else
    ret = release_yolov5_zero_copy_model(&rknn_app_ctx);
#endif
    if (ret != 0)
    {
        printf("release yolov5_model fail! ret=%d\n", ret);
    }

    return 0;
}
