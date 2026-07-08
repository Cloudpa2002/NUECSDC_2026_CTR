/*
pwm_ctl.cpp 
VERSION: 0.3
此版本改动：
1.通过订阅器接收占空比
2.通过回调函数设置占空比

*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <iostream>
#include <fstream>

class Node03 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Node03(std::string name) : Node(name), duty_cycle_(0)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        // 配置 PWM export
        std::ofstream pwm_export(pwm_config.export_path);
        if (pwm_export.is_open()) {
            pwm_export << "0";
            pwm_export.close();
            RCLCPP_INFO(this->get_logger(), "PWM export configured successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", pwm_config.export_path.c_str());
            exit(1);
        }

        // 设置周期
        std::ofstream pwm_period(pwm_config.period_path);
        if (pwm_period.is_open()) {
            pwm_period << pwm_config.period;
            pwm_period.close();
            RCLCPP_INFO(this->get_logger(), "PWM period set to %d ns.", pwm_config.period);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", pwm_config.period_path.c_str());
            exit(1);
        }

        // 创建订阅器
        pwm_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "/pwm/duty_cycle", 10,
            std::bind(&Node03::pwm_callback, this, std::placeholders::_1));
    }

private:
    // void timer_callback()
    // {
    //     // 输出当前占空比
    //     RCLCPP_INFO(this->get_logger(), "Current Duty Cycle: %d%%", duty_cycle_);
    // }
    // 订阅器回调函数
    void pwm_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int new_duty_cycle = msg->data;

        // 检查占空比范围是否有效
        if (new_duty_cycle < 0 || new_duty_cycle > 100) {
            RCLCPP_ERROR(this->get_logger(), "Invalid duty cycle: %d. Must be between 0 and 100.", new_duty_cycle);
            return;
        }

        duty_cycle_ = new_duty_cycle;

        // 设置占空比
        std::ofstream pwm_duty_cycle(pwm_config.duty_cycle_path);
        if (pwm_duty_cycle.is_open()) {
            pwm_duty_cycle << (pwm_config.period * duty_cycle_ / 100);
            pwm_duty_cycle.close();
            RCLCPP_INFO(this->get_logger(), "PWM duty cycle set to %d%%.", duty_cycle_);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", pwm_config.duty_cycle_path.c_str());
        }

        // 启用 PWM
        std::ofstream pwm_enable(pwm_config.enable_path);
        if (pwm_enable.is_open()) {
            pwm_enable << "1";
            pwm_enable.close();
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open %s", pwm_config.enable_path.c_str());
        }
    }

    // 声明订阅器
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pwm_subscriber_;
    int duty_cycle_;

    struct PWMConfig {
        std::string export_path = "/sys/class/pwm/pwmchip0/export";
        std::string period_path = "/sys/class/pwm/pwmchip0/pwm0/period";
        std::string duty_cycle_path = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle";
        std::string enable_path = "/sys/class/pwm/pwmchip0/pwm0/enable";
        int period = 20000000; // 默认周期 20ms (20000000ns)
    } pwm_config;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /* 产生一个 node_03 的节点 */
    auto node = std::make_shared<Node03>("node_03");
    /* 运行节点，并检测退出信号 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}