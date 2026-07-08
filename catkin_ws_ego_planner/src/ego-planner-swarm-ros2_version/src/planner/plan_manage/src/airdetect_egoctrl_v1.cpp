#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cstdlib>
#include "ego_planner/srv/set_position.hpp"

#define VELOCITY2D_CONTROL 0b101111000111
#define POSITION2D_CONTROL 0b101111111000
#define POSITION_VELOCITY2D_2D_CONTROL 0b101111000000





enum class CtrlState {
    HOVER,           // 未接收到目标
    EGO_FLIGHT,      // 根据 EGO Planner 点飞
    SERVICE_FLIGHT   // 根据 SetPosition 服务飞行
  };

class Ctrl : public rclcpp::Node
{
public:
    Ctrl() : Node("cxr_egoctrl_v1_node")
    {
        using std::placeholders::_1;
        using std::placeholders::_2;

        auto best_effort = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        
        // Subscribers
        position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", best_effort, std::bind(&Ctrl::position_cb, this, _1));
        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", best_effort, std::bind(&Ctrl::target_cb, this, _1));
        twist_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "drone_0_planning/pos_cmd", best_effort, std::bind(&Ctrl::twist_cb, this, _1));
        init_pos_pub_ = this->create_publisher<geometry_msgs::msg::Point>("init_position", 10);


        // Publishers
        local_pos_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", best_effort);
        pubMarker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/track_drone_point", best_effort);

        // Services
        set_position_service_ = this->create_service<ego_planner::srv::SetPosition>(
            "set_position",
            std::bind(&Ctrl::handleSetPosition, this, _1, _2));

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&Ctrl::control, this));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        get_now_pos_ = false;
        receive_ = false;
        use_service_target_ = false;
    }

    void position_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        position_msg_ = *msg;
        static std::vector<std::pair<double,double>> initial_pose;
        if(initial_pose.size() < 10)
            initial_pose.push_back(std::make_pair(msg->pose.position.x, msg->pose.position.y));
        else if(initial_pose.size() == 10){
            double sum_x = 0.0, sum_y = 0.0;
            for (const auto& p : initial_pose) {
                sum_x += p.first;
                sum_y += p.second;
            }
            initPos_x = sum_x / initial_pose.size();
            initPos_y = sum_y / initial_pose.size();
            init_pos_ready_ = true;  // 标记初始化完成
            initial_pose.push_back(std::make_pair(0.0, 0.0));
        }
        else{
            RCLCPP_INFO(get_logger(), "initPos_x:%.2f,initPos_y:%.2f",initPos_x,initPos_y );
            geometry_msgs::msg::Point init_point;
            init_point.x = initPos_x;
            init_point.y = initPos_y;
            init_point.z = 0.0;
            init_pos_pub_->publish(init_point);
        }
        
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.orientation, quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
        // 1. 先计算雷达坐标系下的变量
        const double lidar_x = msg->pose.position.y;      // radar_x = enu_y
        const double lidar_y = -msg->pose.position.x;     // radar_y = -enu_x
        const double lidar_z = msg->pose.position.z;      // radar_z = enu_z
    
        // 2. 发布 TF（雷达坐标系）
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp = this->get_clock()->now();
        ts.header.frame_id = "world";
        ts.child_frame_id = "base_s";
        
        ts.transform.translation.x = lidar_x;
        ts.transform.translation.y = lidar_y;
        ts.transform.translation.z = lidar_z;
        
        // 姿态转换：绕Z轴旋转 -90°（-1.57弧度）
        tf2::Quaternion rotation_z;
        rotation_z.setRotation(tf2::Vector3(0, 0, 1), -M_PI_2);
        // 绕Z轴旋转 -90°
        tf2::Quaternion quat_rotated = rotation_z *quat;
        quat_rotated.normalize();
        ts.transform.rotation = tf2::toMsg(quat_rotated);
        
        tf_broadcaster_->sendTransform(ts);
    
        // 3. 更新类成员变量（雷达坐标系）
        if (!get_now_pos_)
        {
            now_x_ = lidar_x;
            now_y_ = lidar_y;
            now_yaw_ = yaw;
            get_now_pos_ = true;
        }
    
        position_x_ = lidar_x;
        position_y_ = lidar_y;
        position_z_ = lidar_z;
        current_yaw_ = yaw;
    

        RCLCPP_INFO(get_logger(), "neuNowP_x:%.2f,neuNowP_y:%.2f",msg->pose.position.x, msg->pose.position.y);

        if (isFirst && use_service_target_)
        {
            err_position_x = position_x_;
            err_position_y = position_y_;
            err_position_z = position_z_;
            isFirst = false;
        }
        

        have_odom_ = true;
    }

    void target_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        receive_ = true;
        use_service_target_ = false; // 切换到ego planner目标
        (void)msg;
        // pid_x_.reset();
        // pid_y_.reset();
        // pid_z_.reset();
    }

    void twist_cb(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
    {
        if (!use_service_target_) {
            ego_ = *msg;
            ego_pos_x_ = ego_.position.x;
            ego_pos_y_ = ego_.position.y;
            ego_pos_z_ = ego_.position.z;
            ego_vel_x_ = ego_.velocity.x;
            ego_vel_y_ = ego_.velocity.y;
            ego_vel_z_ = ego_.velocity.z;
            ego_yaw_ = ego_.yaw+M_PI_2;
            ego_yaw_rate_ = ego_.yaw_dot;
            RCLCPP_INFO(this->get_logger(), "egoP:%.2f,egoV:%.2f,egoY%.2f",ego_pos_x_,ego_vel_x_,ego_.yaw);
        }
    }

    void handleSetPosition(
        const std::shared_ptr<ego_planner::srv::SetPosition::Request> request,
        std::shared_ptr<ego_planner::srv::SetPosition::Response> response)
    {
        service_target_x_ = request->x;
        service_target_y_ = request->y;
        service_target_z_ = request->z;
 
        // pid_x_.reset();
        // pid_y_.reset();
        // pid_y_.reset();
        
        
        use_service_target_ = true;
        receive_ = true;
        
        
        response->success = true;
        response->message = "Target position set successfully";
        
        RCLCPP_INFO(this->get_logger(), "New target position set: (%.2f, %.2f, %.2f)",
                   service_target_x_, service_target_y_, service_target_z_);
    }



    void control() {
        
        if (!have_odom_ || !init_pos_ready_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for odom...");
          return;
        }
    
        mavros_msgs::msg::PositionTarget goal;
        goal.header.stamp = now();
        goal.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
    
        switch(state_) {
          case CtrlState::HOVER:
            std::cout << "HOVER" << std::endl;
            goal.type_mask = POSITION2D_CONTROL;
            goal.position.x = initPos_x;//东北天下xy数据
            goal.position.y = initPos_y;
            goal.position.z = 0.6;                                                                             //大模型0.3
            goal.yaw = now_yaw_;
            
            if (receive_) {
              state_ = CtrlState::EGO_FLIGHT;
            }
            break;
    
          case CtrlState::EGO_FLIGHT:{
            std::cout << "EGO_FLIGHT" << std::endl;
            goal.type_mask = POSITION_VELOCITY2D_2D_CONTROL;
            if(!isserver){
            goal.position.x = -ego_pos_y_;
            goal.position.y = ego_pos_x_;
            goal.position.z = ego_pos_z_;
            goal.velocity.x = -ego_vel_y_;
            goal.velocity.y = ego_vel_x_;
            goal.velocity.z = ego_vel_z_;
            goal.yaw = now_yaw_; // 飞行过程中机头始终跟随飞行方向
            }
            else{
            goal.position.x = -ego_pos_y_;
            goal.position.y = ego_pos_x_;
            goal.position.z = ego_pos_z_;
            goal.velocity.x = -ego_vel_y_;
            goal.velocity.y = ego_vel_x_;
            goal.velocity.z = ego_vel_z_;
            goal.yaw = -M_PI_2;
            }
            if (use_service_target_) state_ = CtrlState::SERVICE_FLIGHT;
            }
            break;
          case CtrlState::SERVICE_FLIGHT:
            {
              std::cout << "SERVICE_FLIGHT" << std::endl;
              goal.type_mask = POSITION2D_CONTROL;
            // if(!isFirstserver){
            //   goal.position.x = -service_target_y_;
            //   goal.position.y = service_target_x_;
            //   goal.position.z = service_target_z_;
            //   const double target_yaw = -M_PI_2;
            //   const double step_size = 0.4;      // 
            //   // 计算当前 yaw 与目标 yaw 的误差
            //   double yaw_err = atan2(sin(target_yaw - current_yaw_), cos(target_yaw - current_yaw_));
            //   // 如果误差较大，逐步调整
            //   if (std::abs(yaw_err) > step_size) {
        	//   // 向目标方向调整一步
        	//   goal.yaw = current_yaw_ + (yaw_err > 0 ? step_size : -step_size);
            //   goal.yaw = atan2(sin(goal.yaw), cos(goal.yaw));
    	    //   } 
            //   else {
        	//   // 误差足够小，直接设为目标值
        	//   goal.yaw = target_yaw;
    	    //   }
            // }
            
           // else{
                goal.position.x = -service_target_y_;
                goal.position.y = service_target_x_;
                goal.position.z = service_target_z_;
                goal.yaw = -M_PI_2;
                isserver = true;
            //}
             
            
              if (!use_service_target_) {
                state_ = CtrlState::EGO_FLIGHT;
                //isFirstserver = true;
              }
              
            }
            break;
        }
        
        local_pos_pub_->publish(goal);
        
    }
      

private:
    // ROS2 members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubMarker_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr twist_sub_;
    rclcpp::Service<ego_planner::srv::SetPosition>::SharedPtr set_position_service_;
    // rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr init_pos_pub_;


    // Data members
    geometry_msgs::msg::PoseStamped position_msg_;
    quadrotor_msgs::msg::PositionCommand ego_;


    // State variables
    float position_x_, position_y_, position_z_, err_position_x, err_position_y,err_position_z;
    float now_x_, now_y_, now_yaw_, current_yaw_;
    double roll, pitch, yaw;
    float ego_pos_x_, ego_pos_y_, ego_pos_z_;
    float ego_vel_x_, ego_vel_y_, ego_vel_z_;
    float ego_yaw_, ego_yaw_rate_;
    double service_target_x_, service_target_y_, service_target_z_;
    double initPos_x, initPos_y;
    bool isserver = false;
    //bool isFirstserver = false;
    // uint8_t landed_state_ = 0;

    // // Controllers
    // PIDController pid_x_, pid_y_, pid_z_;

    // Flags
    bool get_now_pos_, receive_, have_odom_;
    bool isFirst = true;
    bool use_service_target_;
    //bool allow_yaw_ = true;
    std::atomic<bool> init_pos_ready_{false};
    // 状态机
    CtrlState state_ = CtrlState::HOVER;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ctrl>());
    rclcpp::shutdown();
    return 0;
}

