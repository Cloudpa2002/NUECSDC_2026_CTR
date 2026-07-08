#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "rclcpp/qos.hpp"
#include <cmath> // 包含 M_PI
#include <array>
class quad2euler : public rclcpp::Node
{
public:
    quad2euler() : Node("setpoint_raw_publisher")
    {
        // 创建订阅器，订阅 /mavros/local_position/pose
        local_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",
            rclcpp::QoS(10).best_effort(), // 设置 QoS 为 BEST_EFFORT
            std::bind(&quad2euler::local_position_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Setpoint Raw Publisher Node has been started.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_subscriber_;

    void local_position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
        std::array<double, 3> euler_ =  quaternionToEuler(msg->pose.orientation.w,msg->pose.orientation.x,
            msg->pose.orientation.y,msg->pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), " Euler  (degrees): roll=%.2f, pitch=%.2f, yaw=%.2f",
        euler_[0], euler_[1], euler_[2]);
        // 将弧度转换为角度
        // roll = roll * 180.0 / M_PI;
        // pitch = pitch * 180.0 / M_PI;
        // yaw = yaw * 180.0 / M_PI;

        // 打印四元数和欧拉角（角度）
        RCLCPP_INFO(this->get_logger(), "Received quaternion: [x=%.2f, y=%.2f, z=%.2f, w=%.2f]",
                    msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "Converted to Euler angles (degrees): roll=%.2f, pitch=%.2f, yaw=%.2f",
                    roll, pitch, yaw);
    }



    /**
     * @brief 将四元数转换为欧拉角（Z-Y-X顺序，即yaw-pitch-roll）
     * @param w 四元数的实部
     * @param x 四元数的i分量
     * @param y 四元数的j分量
     * @param z 四元数的k分量
     * @return 包含三个欧拉角的数组，顺序为[roll, pitch, yaw]（弧度）
     */
    std::array<double, 3> quaternionToEuler(double w, double x, double y, double z) {
        std::array<double, 3> euler;
        
        // 滚转 (x轴旋转)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        euler[0] = std::atan2(sinr_cosp, cosr_cosp);
        
        // 俯仰 (y轴旋转)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1) {
            // 使用90度，如果超出范围（处理万向锁情况）
            euler[1] = std::copysign(M_PI / 2, sinp);
        } else {
            euler[1] = std::asin(sinp);
        }
        
        // 偏航 (z轴旋转)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        euler[2] = std::atan2(siny_cosp, cosy_cosp);
        
        return euler;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<quad2euler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}