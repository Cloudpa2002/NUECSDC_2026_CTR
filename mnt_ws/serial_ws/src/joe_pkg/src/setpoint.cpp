#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath> // 包含 M_PI

class SetpointRawPublisher : public rclcpp::Node
{
public:
    SetpointRawPublisher() : Node("setpoint_raw_publisher"), init_yaw_(0.0), initialized_(false)
    {
        // 创建订阅器，订阅 /mavros/local_position/pose
        local_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::QoS(10).best_effort(),
            std::bind(&SetpointRawPublisher::local_position_callback, this, std::placeholders::_1));

        // 创建发布器，发布到 /mavros/setpoint_position/local
        setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);

        // 创建定时器，每 100ms 发布一次期望值
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SetpointRawPublisher::publish_setpoint, this));

        RCLCPP_INFO(this->get_logger(), "Setpoint Raw Publisher Node has been started.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double init_yaw_;       // 初始偏航角
    bool initialized_;      // 是否已初始化

    void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!initialized_)
        {
            // 提取四元数
            tf2::Quaternion quat(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);

            // 转换为欧拉角
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // 保存初始偏航角
            init_yaw_ = yaw;
            initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "Initialized with yaw=%.2f (radians)", init_yaw_);
        }
    }

    void publish_setpoint()
    {
        if (!initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial yaw data...");
            return;
        }

        // 创建期望位置消息
        auto target = geometry_msgs::msg::PoseStamped();
        target.header.stamp = this->now();
        target.header.frame_id = "map";

        // 设置期望位置
        target.pose.position.x = 0.0;
        target.pose.position.y = 0.0;
        target.pose.position.z = 0.3;

        // 设置期望姿态（仅使用初始偏航角）
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, init_yaw_); // roll=0, pitch=0, yaw=init_yaw_
        target.pose.orientation.x = quat.x();
        target.pose.orientation.y = quat.y();
        target.pose.orientation.z = quat.z();
        target.pose.orientation.w = quat.w();

        // 发布期望位置和姿态
        setpoint_publisher_->publish(target);

        RCLCPP_INFO(this->get_logger(), "Publishing setpoint: [x=%.2f, y=%.2f, z=%.2f, yaw=%.2f (radians)]",
                    target.pose.position.x, target.pose.position.y, target.pose.position.z, init_yaw_* 180.0 / M_PI);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetpointRawPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
