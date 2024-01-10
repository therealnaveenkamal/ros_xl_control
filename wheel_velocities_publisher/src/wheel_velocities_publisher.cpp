#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("wheel_velocities_publisher");
    RCLCPP_INFO(node->get_logger(), "Initialized wheel velocities publisher node");
    auto publisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);

    rclcpp::Rate rate(1);

    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        auto forward = std_msgs::msg::Float32MultiArray();
        forward.data = {1.0, 1.0, 1.0, 1.0};
        publisher->publish(forward);
        RCLCPP_INFO(node->get_logger(), "Move forward");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        auto backward = std_msgs::msg::Float32MultiArray();
        backward.data = {-1.0, -1.0, -1.0, -1.0};
        publisher->publish(backward);
        RCLCPP_INFO(node->get_logger(), "Move backward");
        std::this_thread::sleep_for(std::chrono::seconds(3));


        auto left = std_msgs::msg::Float32MultiArray();
        left.data = {-1.0, 1.0, -1.0, 1.0};
        publisher->publish(left);
        RCLCPP_INFO(node->get_logger(), "Move left");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto right = std_msgs::msg::Float32MultiArray();
        right.data = {1.0, -1.0, 1.0, -1.0};
        publisher->publish(right);
        RCLCPP_INFO(node->get_logger(), "Move right");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto clock = std_msgs::msg::Float32MultiArray();
        clock.data = {1.0, -1.0, -1.0, 1.0};
        publisher->publish(clock);
        RCLCPP_INFO(node->get_logger(), "Turn clockwise");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto anticlock = std_msgs::msg::Float32MultiArray();
        anticlock.data = {-1.0, 1.0, 1.0, -1.0};
        publisher->publish(anticlock);
        RCLCPP_INFO(node->get_logger(), "Turn anti-clockwise");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        auto stop = std_msgs::msg::Float32MultiArray();
        stop.data = {0.0, 0.0, 0.0, 0.0};
        publisher->publish(stop);
        RCLCPP_INFO(node->get_logger(), "Stop");

        rclcpp::shutdown();
        return 0;
    }

    rclcpp::shutdown();
    return 0;
}
