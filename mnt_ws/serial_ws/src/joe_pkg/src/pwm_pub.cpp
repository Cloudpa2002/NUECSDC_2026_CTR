#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

/*
    创建一个类节点，名字叫做PWM_PUB,继承自Node.
*/
class PWM_PUB : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    PWM_PUB(std::string name) : Node(name), duty_cycle_(0)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());

        // 声明参数
        this->declare_parameter<int>("duty_cycle", 0);

        // 创建发布器
        pwm_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/pwm/duty_cycle", 10);

        // 创建定时器，每 100ms 调用一次 timer_callback
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PWM_PUB::timer_callback, this));
    }

private:
    // 定时器回调函数
    void timer_callback()
    {
        // 从参数服务器获取占空比
        this->get_parameter("duty_cycle", duty_cycle_);

        // 创建消息
        auto message = std_msgs::msg::Int32();
        message.data = duty_cycle_;

        // 发布消息
        pwm_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published PWM duty cycle: %d%%", duty_cycle_);
    }

    // 发布器
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pwm_publisher_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // 当前占空比
    int duty_cycle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /* 产生一个 pwm_pub 的节点 */
    auto node = std::make_shared<PWM_PUB>("pwm_pub");
    /* 运行节点，并检测退出信号 */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}