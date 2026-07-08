#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

/*
    创建一个类节点，名字叫做Interface_Mavros,继承自Node.
*/
class Interface_Mavros : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Interface_Mavros(std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        // 创建客户端，用于调用 /mavros/cmd/arming 服务
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");

        // 创建定时器，启动后立即 Arm，10 秒后 Disarm
        arm_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&Interface_Mavros::arm_uav, this));
    }

private:
    // 客户端指针
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::TimerBase::SharedPtr arm_timer_;
    bool is_armed_ = false; // 标志位，记录当前是否已 Arm
    int elapsed_time_ = 0;  // 记录经过的时间

    // Arm 和 Disarm UAV
    void arm_uav()
    {
        if (!is_armed_ && elapsed_time_ == 0) {
            // Arm 飞机
            send_arm_command(true);
            is_armed_ = true;
        } else if (is_armed_ && elapsed_time_ >= 10) {
            // 10 秒后 Disarm 飞机
            send_arm_command(false);
            arm_timer_->cancel(); // 停止定时器
        }

        elapsed_time_++;
    }

    // 发送 Arm/Disarm 命令
    void send_arm_command(bool arm_state)
    {
        // 创建服务请求
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = arm_state;

        // 异步调用服务
        auto future = arming_client_->async_send_request(request);

        // 等待服务响应
        try {
            auto response = future.get();
            if (response->success) {
                if (arm_state) {
                    RCLCPP_INFO(this->get_logger(), "UAV armed successfully.");
                } else {
                    RCLCPP_INFO(this->get_logger(), "UAV disarmed successfully.");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to change UAV arm state.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /* 创建 interface_mavros 节点 */
    auto node = std::make_shared<Interface_Mavros>("interface_mavros");
    /* 运行节点，并检测退出信号 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}