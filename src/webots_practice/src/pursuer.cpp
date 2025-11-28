#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class PursuitController : public rclcpp::Node
{
public:
    PursuitController()
    : Node("pursuit_controller")
    {
        // Declare parameters with default values
        this->declare_parameter<int>("robot_id", 1);
        this->declare_parameter<int>("target_robot_id", 0);
        this->declare_parameter<double>("pursuit_distance", 0.01);
        this->declare_parameter<double>("max_linear_speed", 0.22);
        this->declare_parameter<double>("max_angular_speed", 2.0);

        robot_id_ = this->get_parameter("robot_id").as_int();
        target_id_ = this->get_parameter("target_robot_id").as_int();
        pursuit_distance_ = this->get_parameter("pursuit_distance").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

        // Subscribers
        my_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot" + std::to_string(robot_id_) + "/diffdrive_controller/odom", 10,
            std::bind(&PursuitController::odom_callback, this, std::placeholders::_1));

        target_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "robot" + std::to_string(target_id_) + "/diffdrive_controller/odom", 10,
            std::bind(&PursuitController::target_odom_callback, this, std::placeholders::_1));

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "robot" + std::to_string(robot_id_) + "/diffdrive_controller/cmd_vel_unstamped", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PursuitController::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Robot %d pursuing Robot %d", robot_id_, target_id_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        my_odom_ = msg;
    }

    void target_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        target_odom_ = msg;
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void control_loop()
    {
        if (!my_odom_ || !target_odom_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Waiting for odometry data...");
            return;
        }

        double my_x = my_odom_->pose.pose.position.x;
        double my_y = my_odom_->pose.pose.position.y;
        double my_yaw = get_yaw_from_quaternion(my_odom_->pose.pose.orientation);

        double target_x = target_odom_->pose.pose.position.x;
        double target_y = target_odom_->pose.pose.position.y;

        double dx = target_x - my_x;
        double dy = target_y - my_y;
        double distance = std::sqrt(dx*dx + dy*dy);
        double angle_to_target = std::atan2(dy, dx);

        double angle_diff = angle_to_target - my_yaw;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));

        geometry_msgs::msg::Twist cmd;

        if (distance < pursuit_distance_)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Reached target! Distance: %.2f m", distance);
        }
        else
        {
            double speed_factor = std::max(0.1, std::cos(angle_diff));
            cmd.linear.x = std::min(max_linear_speed_, distance * 0.5 * speed_factor);
            cmd.angular.z = std::clamp(angle_diff * 2.0, -max_angular_speed_, max_angular_speed_);

            RCLCPP_DEBUG(this->get_logger(),
                         "Distance: %.2f m, Angle: %.1f deg",
                         distance, angle_diff * 180.0 / M_PI);
        }

        cmd_vel_pub_->publish(cmd);
    }

    int robot_id_;
    int target_id_;
    double pursuit_distance_;
    double max_linear_speed_;
    double max_angular_speed_;

    nav_msgs::msg::Odometry::SharedPtr my_odom_;
    nav_msgs::msg::Odometry::SharedPtr target_odom_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr my_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr target_odom_sub_;
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
