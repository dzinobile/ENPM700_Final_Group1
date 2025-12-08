#include "SheepdogNode.hpp"
#include "ExploreState.hpp"
#include "EncircleState.hpp"
#include "HerdState.hpp"


using namespace std::chrono_literals;

SheepdogNode::SheepdogNode() : Node("sheepdog_node"),
                                    sheep_detection_radius_(1.0),
                                    distance_to_sheep_(std::numeric_limits<double>::max()),
                                    sheep_detected_(false){
    this->declare_parameter("sheep_detection_radius", 1.0);
    bool use_sim_time = this->get_parameter_or("use_sim_time", false);

    if (!use_sim_time) {
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }
    sheep_detection_radius_ = this->get_parameter("sheep_detection_radius").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    currentState_ = std::make_unique<ExploreState>();

    timer_ = this->create_wall_timer(
        100ms, std::bind(&SheepdogNode::updateCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "sheepdogNode initialized in EXPLORE state");
}


void SheepdogNode::updateCallback() {
    updateSheepDetection();
    if (currentState_) {
        currentState_->update(*this);

        States* nextState = currentState_->transition(*this);
        if (nextState != nullptr && nextState != currentState_.get()) {
            changeState(nextState);
        }
    }
}

void SheepdogNode::changeState(States* newState) {
    if (newState != nullptr) {
        RCLCPP_INFO(this->get_logger(), "Changing state");
        currentState_.reset(newState);
    }
}

void SheepdogNode::run() {
    rclcpp::spin(shared_from_this());
}

void SheepdogNode::updateSheepDetection() {
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(
                "base_link",
                "sheep",
                tf2::TimePointZero
            );

        double dx = transform_stamped.transform.translation.x;
        double dy = transform_stamped.transform.translation.y;
        double dz = transform_stamped.transform.translation.z;

        distance_to_sheep_ = std::sqrt(dx*dx + dy*dy + dz*dz);
        RCLCPP_INFO(this->get_logger(), "distance to sheep %.2f", distance_to_sheep_);
        bool was_detected = sheep_detected_;
        sheep_detected_ = (distance_to_sheep_ <= sheep_detection_radius_);

        if (sheep_detected_&& !was_detected) {
            RCLCPP_INFO(this->get_logger(), "sheep detected %.2f meters", distance_to_sheep_);
        } else if (!sheep_detected_ && was_detected) {
            RCLCPP_INFO(this->get_logger(), " sheep no longer in detection range" );
        } 
    } catch (tf2::TransformException &ex) {
        sheep_detected_ = false;
        distance_to_sheep_ = std::numeric_limits<double>::max();

        RCLCPP_INFO(this->get_logger(), "could not get transform");
    }
}



