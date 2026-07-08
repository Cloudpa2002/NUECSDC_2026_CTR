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
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


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

/*-------------------------------------------
                  Main Function
-------------------------------------------*/


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("yolov5_videocapture_demo_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>("yolov5_results", 10);
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov5s.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov5s.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_name = argv[2];

    int ret;
    TIMER timer;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    cv::Mat frame1, frame2; // 摄像头帧
    cv::Mat image1, image2;
    // image_buffer_t src_image;
    image_buffer_t src_image1, src_image2;
    object_detect_result_list od_results1, od_results2;

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    // memset(&src_image, 0, sizeof(image_buffer_t));
    memset(&src_image1, 0, sizeof(image_buffer_t));
    memset(&src_image2, 0, sizeof(image_buffer_t));

    cv::VideoCapture cap1, cap2;
    bool has_cap1 = false;  // cap1 (ISP/MIPI) 是否可用

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

        // cap1: ISP/MIPI 摄像头（可选，失败不致命）
        std::string pipeline = "v4l2src device=/dev/video11 ! video/x-raw, width=640, height=480, format=NV12, framerate=30/1 ! videoconvert ! appsink";
        cap1.open(pipeline, cv::CAP_GSTREAMER);
        has_cap1 = cap1.isOpened();
        if (!has_cap1) {
            printf("[WARN] cap1 (/dev/video11 ISP) not available, continuing with cap2 only.\n");
        }

        // cap2: USB 摄像头（必须）
        cap2.open(camera_id);
        if (!cap2.isOpened()) {
            printf("Error: Could not open camera %d.\n", camera_id);
            return -1;
        }

        cap2.set(cv::CAP_PROP_FRAME_WIDTH, 640);//宽度
        cap2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);//高度
    } else {
        // 视频文件模式
        cap1.open(device_name);
        has_cap1 = cap1.isOpened();
        if (!has_cap1) {  
            printf("Error: Could not open video file: %s\n", device_name);
            return -1;
        }
    }


    int frame_interval = 2; // 每隔 5 帧进行一次检测
    int frame_count = 0;    // 帧计数器

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
	while (true)
    {
        gettimeofday(&start_time, NULL);

        // 读取第一个摄像头的帧（如果可用）
        if (has_cap1) {
            if (!cap1.read(frame1))
            {
                printf("cap1 read frame fail, disabling cap1!\n");
                has_cap1 = false;
            }
        }

        // 读取第二个摄像头的帧
        if (!cap2.read(frame2))
        {
            printf("cap2 read frame fail!\n");
            break;
        }

        // 每隔 frame_interval 帧进行一次检测
        if (frame_count % frame_interval == 0)
        {
            // 处理第一个摄像头的帧（如果可用）
            if (has_cap1) {
                cv::cvtColor(frame1, image1, cv::COLOR_BGR2RGB);
                src_image1.width = image1.cols;
                src_image1.height = image1.rows;
                src_image1.format = IMAGE_FORMAT_RGB888;
                src_image1.virt_addr = (unsigned char *)image1.data;

    #ifndef ENABLE_ZERO_COPY
                ret = inference_yolov5_model(&rknn_app_ctx, &src_image1, &od_results1);
    #else
                ret = inference_yolov5_zero_copy_model(&rknn_app_ctx, &src_image1, &od_results1);
    #endif
                if (ret == 0)
                {
                    for (int i = 0; i < od_results1.count; i++)
                    {
                        const unsigned char *color = colors[i % 19];
                        cv::Scalar cc(color[0], color[1], color[2]);

                        object_detect_result *det_result = &(od_results1.results[i]);
                        cv::rectangle(frame1, cv::Rect(cv::Point(det_result->box.left, det_result->box.top),
                                                    cv::Point(det_result->box.right, det_result->box.bottom)),
                                    cc, 2);

                        char text[OBJ_NAME_MAX_SIZE];
                        snprintf(text, sizeof(text), "%s: %.2f", coco_cls_to_name(det_result->cls_id), det_result->prop);

                        int baseline = 0;
                        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
                        int x = det_result->box.left + (det_result->box.right - det_result->box.left) / 2 - label_size.width / 2;
                        int y = det_result->box.top + (det_result->box.bottom - det_result->box.top) / 2 + label_size.height / 2;
                        cv::putText(frame1, text, cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 1);
                    }
                }else{
                    printf("inference yolov5_model fail! ret=%d\n", ret);
                    goto out;
                }
            }

            // 处理第二个摄像头的帧
            cv::cvtColor(frame2, image2, cv::COLOR_BGR2RGB);
            src_image2.width = image2.cols;
            src_image2.height = image2.rows;
            src_image2.format = IMAGE_FORMAT_RGB888;
            src_image2.virt_addr = (unsigned char *)image2.data;

    #ifndef ENABLE_ZERO_COPY
            ret = inference_yolov5_model(&rknn_app_ctx, &src_image2, &od_results2);
    #else
            ret = inference_yolov5_zero_copy_model(&rknn_app_ctx, &src_image2, &od_results2);
    #endif
            if (ret == 0)
            {
                for (int i = 0; i < od_results2.count; i++)
                {
                    const unsigned char *color = colors[i % 19];
                    cv::Scalar cc(color[0], color[1], color[2]);

                    object_detect_result *det_result = &(od_results2.results[i]);
                    cv::rectangle(frame2, cv::Rect(cv::Point(det_result->box.left, det_result->box.top),
                                                cv::Point(det_result->box.right, det_result->box.bottom)),
                                cc, 2);

                    char text[OBJ_NAME_MAX_SIZE];
                    snprintf(text, sizeof(text), "%s: %.2f", coco_cls_to_name(det_result->cls_id), det_result->prop);
                    if (i == 0 && det_result->prop > 0.95f){
                    	// 发布识别结果到 ROS 2 网络
                        auto message = std_msgs::msg::String();
                        message.data = std::string(coco_cls_to_name(det_result->cls_id));
                        publisher->publish(message);
                    	
                    }
                    // cv::putText(frame2, text, cv::Point(det_result->box.left, det_result->box.top - 5),
                    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scala1r(255, 255, 255), 1);

                    // 计算文本大小 td
                    int baseline = 0;
                    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
                    // 确定文本显示位置（框的中间）
                    int x = det_result->box.left + (det_result->box.right - det_result->box.left) / 2 - label_size.width / 2;
                    int y = det_result->box.top + (det_result->box.bottom - det_result->box.top) / 2 + label_size.height / 2;
                    cv::putText(frame2, text, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 1); // 红色文字
                }
            }else{
                printf("inference yolov5_model fail! ret=%d\n", ret);
                goto out;
            }
        }

        // 显示摄像头结果
        if (has_cap1) {
            cv::imshow("Camera 1", frame1);
        }
        cv::imshow("Camera 2", frame2);

        char c = cv::waitKey(1);
        if (c == 27)
        { // ESC
            break;
        }

        frame_count++; // 增加帧计数器
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

