#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class OdomToPose : public rclcpp::Node
{
public:
    OdomToPose() : Node("odom_to_pose")
    {
        std::string name = this->get_name();
        RCLCPP_INFO(this->get_logger(), "%s 节点已经启动.", name.c_str());
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&OdomToPose::odomCallback, this, _1));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = msg->header;

    // 原始位置
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 位置绕 Z 轴旋转 -90°（顺时针）
    pose_msg.pose.position.x = -y;
    pose_msg.pose.position.y = x;
    pose_msg.pose.position.z = z;

    // 原始四元数
    tf2::Quaternion q_orig(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 绕 Z 轴旋转 90° 的四元数
    tf2::Quaternion q_rot;
    q_rot.setRPY(0, 0, M_PI_2);  // 90 度

    // 应用旋转
    tf2::Quaternion q_new = q_rot * q_orig;
    q_new.normalize();  // 归一化以避免数值误差

    pose_msg.pose.orientation.x = q_new.x();
    pose_msg.pose.orientation.y = q_new.y();
    pose_msg.pose.orientation.z = q_new.z();
    pose_msg.pose.orientation.w = q_new.w();

    pose_pub_->publish(pose_msg);
}

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToPose>());
    rclcpp::shutdown();
    return 0;
}
