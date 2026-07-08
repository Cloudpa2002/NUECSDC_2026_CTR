/*
node_01.cpp 
VERSION: 0.1
此版本改动：
1.面向对象变成
2.定时器每500ms通过串口ttyS9发送"Hello,serial"
3.改动了/dev/ttyS9的权限 chmod 666 /dev/ttyS9
*/
#include "rclcpp/rclcpp.hpp"
#include "SerialStream.h"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using namespace LibSerial;

class TopicPublisher01 : public rclcpp::Node
{
public:
SerialStream serial_port;
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

    

		try{
			serial_port.Open("/dev/ttyS9");
			serial_port.SetBaudRate(BaudRate::BAUD_115200);
            RCLCPP_INFO(this->get_logger(),"串口打开成功！");
            system("ls /dev/ttyS* /dev/ttyUSB* 2>/dev/null");  // 列出可用设备
			
		}catch (const LibSerial::OpenFailed& e) {
			RCLCPP_FATAL(this->get_logger(), "串口打开失败: %s", e.what());
			RCLCPP_INFO(this->get_logger(), "可用串口设备列表:");
			system("ls /dev/ttyS* /dev/ttyUSB* 2>/dev/null");  // 列出可用设备
			exit(1);
		}

		timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
		std::bind(&TopicPublisher01::timer_callback, this));
	}

	
private:
    // 声明节点
	void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        serial_port << "Hello, serial!";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
	//node->serial_run();

	//const LibSerial::OpenFailed& e


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}