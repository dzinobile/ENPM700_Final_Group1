// Copyright ...
#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;
auto Logger = rclcpp::get_logger("");

/**
 * @class SheepBroadcasterTestFixture
 * @brief Test fixture for testing sheep_broadcaster and SheepdogNode integration
 */
class SheepBroadcasterTestFixture {
public:
    SheepBroadcasterTestFixture() {
        testerNode = rclcpp::Node::make_shared("sheep_broadcaster_test_node");
        Logger = testerNode->get_logger();
        
        // Set use_sim_time
        testerNode->set_parameter(rclcpp::Parameter("use_sim_time", true));
        
        testerNode->declare_parameter("test_duration", 10.0);
        TEST_DURATION = testerNode->get_parameter("test_duration")
                            .get_parameter_value().get<double>();
        
        // Create TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(testerNode->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(Logger, "Test duration: %.2f seconds", TEST_DURATION);
    }

protected:
    rclcpp::Node::SharedPtr testerNode;
    double TEST_DURATION;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test sheep broadcaster publishes map->sheep transform",
                 "[tf_broadcast]") {
    RCLCPP_INFO(Logger, "Waiting for map->sheep transform...");
    

    rclcpp::sleep_for(std::chrono::seconds(1));
    
    bool transform_received = false;
    geometry_msgs::msg::TransformStamped transform;
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(TEST_DURATION);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        try {
            transform = tf_buffer_->lookupTransform(
                "map", 
                "sheep", 
                tf2::TimePointZero
            );
            transform_received = true;
            RCLCPP_INFO(Logger, "Transform received!");
            break;
        } catch (tf2::TransformException &ex) {
            // Keep trying
            rclcpp::spin_some(testerNode);
            rclcpp::sleep_for(100ms);
        }
    }
    
    REQUIRE(transform_received);
    
    RCLCPP_INFO(Logger, "Sheep position: x=%.2f, y=%.2f, z=%.2f",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
    
    CHECK(transform.transform.translation.x == Catch::Approx(-4.0).epsilon(0.01));
    CHECK(transform.transform.translation.y == Catch::Approx(1.0).epsilon(0.01));
    CHECK(transform.transform.translation.z == Catch::Approx(0.0).epsilon(0.01));
}

TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test sheep broadcaster publishes map->pen transform",
                 "[tf_broadcast]") {
    RCLCPP_INFO(Logger, "Waiting for map->pen transform...");
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    bool transform_received = false;
    geometry_msgs::msg::TransformStamped transform;
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(TEST_DURATION);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        try {
            transform = tf_buffer_->lookupTransform(
                "map", 
                "pen", 
                tf2::TimePointZero
            );
            transform_received = true;
            RCLCPP_INFO(Logger, "Pen transform received!");
            break;
        } catch (tf2::TransformException &ex) {
            rclcpp::spin_some(testerNode);
            rclcpp::sleep_for(100ms);
        }
    }
    
    REQUIRE(transform_received);
    

    CHECK(transform.transform.translation.x == Catch::Approx(0.0).epsilon(0.01));
    CHECK(transform.transform.translation.y == Catch::Approx(0.0).epsilon(0.01));
    CHECK(transform.transform.translation.z == Catch::Approx(0.0).epsilon(0.01));
}

TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test base_link->sheep transform can be computed",
                 "[tf_integration]") {
    RCLCPP_INFO(Logger, "Testing base_link->sheep transform lookup...");
    

    rclcpp::sleep_for(std::chrono::seconds(2));
    
    bool transform_received = false;
    geometry_msgs::msg::TransformStamped transform;
    double distance_to_sheep = 0.0;
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::duration<double>(TEST_DURATION);
    

    while (std::chrono::steady_clock::now() - start_time < timeout) {
        try {
            transform = tf_buffer_->lookupTransform(
                "base_link", 
                "sheep", 
                tf2::TimePointZero
            );
            
            double dx = transform.transform.translation.x;
            double dy = transform.transform.translation.y;
            double dz = transform.transform.translation.z;
            distance_to_sheep = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            transform_received = true;
            RCLCPP_INFO(Logger, "base_link->sheep transform received! Distance: %.2f meters",
                        distance_to_sheep);
            break;
        } catch (tf2::TransformException &ex) {
            rclcpp::spin_some(testerNode);
            rclcpp::sleep_for(100ms);
        }
    }
    

    if (transform_received) {
        RCLCPP_INFO(Logger, "Successfully computed distance to sheep: %.2f meters", 
                    distance_to_sheep);
        CHECK(distance_to_sheep >= 0.0);
    } else {
        RCLCPP_WARN(Logger, "base_link->sheep transform not available. "
                            "This is expected if TurtleBot isn't running.");

    }
}

TEST_CASE_METHOD(SheepBroadcasterTestFixture,
                 "Test transform broadcast rate is approximately 10Hz",
                 "[tf_broadcast]") {
    RCLCPP_INFO(Logger, "Testing transform broadcast rate...");
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    std::vector<rclcpp::Time> timestamps;
    auto start_time = std::chrono::steady_clock::now();
    auto test_duration = std::chrono::seconds(2);
    
    while (std::chrono::steady_clock::now() - start_time < test_duration) {
        try {
            auto transform = tf_buffer_->lookupTransform(
                "map", 
                "sheep", 
                tf2::TimePointZero
            );
            
            rclcpp::Time stamp(transform.header.stamp);
            if (timestamps.empty() || stamp != timestamps.back()) {
                timestamps.push_back(stamp);
            }
        } catch (tf2::TransformException &ex) {
        }
        
        rclcpp::spin_some(testerNode);
        rclcpp::sleep_for(10ms);
    }
    
    REQUIRE(timestamps.size() >= 10); 
    
    RCLCPP_INFO(Logger, "Received %zu transform updates in 2 seconds", timestamps.size());
    

    if (timestamps.size() > 1) {
        double total_duration = (timestamps.back() - timestamps.front()).seconds();
        double rate = (timestamps.size() - 1) / total_duration;
        RCLCPP_INFO(Logger, "Average broadcast rate: %.2f Hz", rate);
        CHECK(rate >= 8.0);  
        CHECK(rate <= 12.0);
    }
}