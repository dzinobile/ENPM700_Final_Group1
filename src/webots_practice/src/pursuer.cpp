#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "webots_ros2_msgs/msg/float_stamped.hpp"

class PursuitController : public rclcpp::Node
{
public:
    PursuitController()
    : Node("pursuit_controller")
    {
        // Parameters
        this->declare_parameter<int>("robot_id", 1);
        this->declare_parameter<int>("target_robot_id", 0);
        this->declare_parameter<double>("pursuit_distance", 0.5);
        this->declare_parameter<double>("max_linear_speed", 0.6);
        this->declare_parameter<double>("max_angular_speed", 2.0);
        this->declare_parameter<double>("kp_angular", 2.0);
        this->declare_parameter<double>("alignment_threshold", 0.3);

        robot_id_ = this->get_parameter("robot_id").as_int();
        target_id_ = this->get_parameter("target_robot_id").as_int();
        pursuit_distance_ = this->get_parameter("pursuit_distance").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        alignment_threshold_ = this->get_parameter("alignment_threshold").as_double();

        // GPS subscriptions
        my_gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/robot" + std::to_string(robot_id_) + "/robot" + std::to_string(robot_id_) + "/gps",
            10,
            std::bind(&PursuitController::my_gps_callback, this, std::placeholders::_1)
        );

        target_gps_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/robot" + std::to_string(target_id_) + "/robot" + std::to_string(target_id_) + "/gps",
            10,
            std::bind(&PursuitController::target_gps_callback, this, std::placeholders::_1)
        );

        // Compass subscription
        my_compass_sub_ = this->create_subscription<webots_ros2_msgs::msg::FloatStamped>(
            "/robot" + std::to_string(robot_id_) + "/robot" + std::to_string(robot_id_) + "/compass/bearing",
            10,
            std::bind(&PursuitController::my_compass_callback, this, std::placeholders::_1)
        );

        // Velocity publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot" + std::to_string(robot_id_) + "/diffdrive_controller/cmd_vel_unstamped",
            10
        );

        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PursuitController::control_loop, this)
        );

        RCLCPP_INFO(this->get_logger(), "Robot %d pursuing Robot %d", robot_id_, target_id_);
    }

private:
    void my_gps_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        my_pos_ = msg;
    }

    void target_gps_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        target_pos_ = msg;
    }

    void my_compass_callback(const webots_ros2_msgs::msg::FloatStamped::SharedPtr msg)
    {
        // CRITICAL FIX: Normalize the compass reading immediately
        // The compass gives cumulative rotation, we need [-pi, pi]
        my_yaw_ = normalize_angle(msg->data);
    }

    double normalize_angle(double angle)
    {
        // Wrap angle to [-pi, pi]
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void control_loop()
    {
        // Wait for sensor data
        if (!my_pos_ || !target_pos_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for GPS data...");
            return;
        }

        // Calculate vector from robot to target
        double dx = target_pos_->point.x - my_pos_->point.x;
        double dy = target_pos_->point.y - my_pos_->point.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Calculate angle to target in world frame
        double angle_to_target = std::atan2(dy, dx);
        
        // Calculate heading error (difference between where we're facing and where target is)
        double angle_error = normalize_angle(angle_to_target - my_yaw_);

        geometry_msgs::msg::Twist cmd;

        // STATE 1: Target reached
        if (distance < pursuit_distance_)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Target reached! Distance: %.3f m", distance);
        }
        // STATE 2: Need to rotate (not facing target)
        else if (std::abs(angle_error) > alignment_threshold_)
        {
            // Stop and rotate in place
            cmd.linear.x = 0.0;
            cmd.angular.z = std::clamp(kp_angular_ * angle_error,
                                       -max_angular_speed_,
                                       max_angular_speed_);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "Rotating: error = %.1f deg", angle_error * 180.0 / M_PI);
        }
        // STATE 3: Aligned with target - drive straight
        else
        {
            // Move forward, allow minor steering corrections
            cmd.linear.x = max_linear_speed_;
            cmd.angular.z = std::clamp(kp_angular_ * angle_error * 0.3,
                                       -0.5, 0.5);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "Driving: distance = %.3f m, heading error = %.1f deg",
                                 distance, angle_error * 180.0 / M_PI);
        }

        cmd_vel_pub_->publish(cmd);
    }

    // Parameters
    int robot_id_;
    int target_id_;
    double pursuit_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
    double kp_angular_;
    double alignment_threshold_;

    // State
    geometry_msgs::msg::PointStamped::SharedPtr my_pos_;
    geometry_msgs::msg::PointStamped::SharedPtr target_pos_;
    double my_yaw_ = 0.0;

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr my_gps_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_gps_sub_;
    rclcpp::Subscription<webots_ros2_msgs::msg::FloatStamped>::SharedPtr my_compass_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<PursuitController>();
    rclcpp::spin(controller);
    rclcpp::shutdown();
    return 0;
}