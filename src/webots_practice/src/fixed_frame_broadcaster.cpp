#include "fixed_frame_broadcaster.hpp"

using namespace std::chrono_literals;

FixedFrameBroadcaster::FixedFrameBroadcaster() : rclcpp::Node("fixed_frame_broadcaster") {
    num_bots_ = 10;
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
      100ms, std::bind(&FixedFrameBroadcaster::broadcastTimerCallback, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void FixedFrameBroadcaster::broadcastTimerCallback(){
    geometry_msgs::msg::TransformStamped robot0_tf;
    try{
        robot0_tf = tf_buffer_->lookupTransform(
            "world",
            "robot0",
            tf2::TimePointZero
        );
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "could not get transform");
        return;
    }

    std::vector<std::string> others;
    for (int i = 1; i < num_bots_ + 1; i++){
        std::string r_name = "robot" + std::to_string(i);
        others.push_back(r_name);
    }

    double range = 0.1;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double carrot_dist = 0.5;
    int count = 0;

    for (const auto &name : others) {
        geometry_msgs::msg::TransformStamped other_tf;
        try {
            other_tf = tf_buffer_->lookupTransform(
                "world", name, tf2::TimePointZero
            );
        } catch (...) {
            continue;
        }

        double dx = other_tf.transform.translation.x - robot0_tf.transform.translation.x;
        double dy = other_tf.transform.translation.x - robot0_tf.transform.translation.y;
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));

        if (dist < range){
            sum_x += other_tf.transform.translation.x;
            sum_y += other_tf.transform.translation.y;
            count++;
        }

    }

    double carrot_x = robot0_tf.transform.translation.x;
    double carrot_y = robot0_tf.transform.translation.y;

    if (count > 0){
        double avg_x = sum_x / count;
        double avg_y = sum_y / count;

        double rx = robot0_tf.transform.translation.x - avg_x;
        double ry = robot0_tf.transform.translation.x - avg_y;

        double mag = sqrt(pow(rx, 2) + pow(ry, 2));
        if (mag > 0.001) {
            rx /= mag;
            ry /= mag;
        }

        carrot_x = robot0_tf.transform.translation.x + rx * carrot_dist;
        carrot_y = robot0_tf.transform.translation.y + ry * carrot_dist;
    }

    geometry_msgs::msg::TransformStamped r0;
    r0.header.stamp = this->get_clock()->now();
    r0.header.frame_id = "world";
    r0.child_frame_id = "carrot0";
    r0.transform.translation.x = carrot_x;
    r0.transform.translation.y = carrot_y;
    r0.transform.translation.z = 0.0;
    r0.transform.rotation.x = 0.0;
    r0.transform.rotation.y = 0.0;
    r0.transform.rotation.z = 0.0;
    r0.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(r0);

    std::vector<double> x_offset = {
        carrot_dist, 0.0, 0.0, -carrot_dist, 
        carrot_dist/sqrt(2.0), 
        -carrot_dist/sqrt(2.0), 
        carrot_dist/sqrt(2.0), 
        -carrot_dist/sqrt(2.0),
        (carrot_dist/2)*sqrt(3.0),
        -(carrot_dist/2)*sqrt(3.0)

    };

    std::vector<double> y_offset = {
        0.0, carrot_dist, -carrot_dist, 0.0,
        carrot_dist/sqrt(2.0),
        -carrot_dist/sqrt(2.0),
        -carrot_dist/sqrt(2.0),
        carrot_dist/sqrt(2.0),
        carrot_dist/2,
        -carrot_dist/2
    };

    for (int i = 1; i < num_bots_ + 1; i++) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "carrot" + std::to_string(i);
        t.transform.translation.x = robot0_tf.transform.translation.x + x_offset[i-1];
        t.transform.translation.y = robot0_tf.transform.translation.y + y_offset[i-1];
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 0.0;
        tf_broadcaster_->sendTransform(t);

    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FixedFrameBroadcaster>());
    rclcpp::shutdown();
    return 0;
}