#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <Eigen/Dense>


class KinematicModelNode : public rclcpp::Node {
public:
    KinematicModelNode() : Node("kinematic_model_node") {
        wheel_speed_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                calculateTwist(msg);
            });
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {
            publishTwist();
        });
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double wbz = 0;
    double vbx = 0;
    double vby = 0;

    void calculateTwist(const std_msgs::msg::Float32MultiArray::SharedPtr wheel_speed_msg) {
        double u1 = wheel_speed_msg->data[0];
        double u2 = wheel_speed_msg->data[1];
        double u3 = wheel_speed_msg->data[2];
        double u4 = wheel_speed_msg->data[3];

        Eigen::MatrixXd A(4, 3);
        A << -4.397, 20, -20,
             4.397, 20, 20,
             4.397, 20, -20,
            -4.397, 20, 20;

        Eigen::VectorXd B(4);
        B << u1, u2, u3, u4;

        // Solve the system using the least squares method
        Eigen::VectorXd x = A.colPivHouseholderQr().solve(B);

        // Extract the values of wbz, vbx, and vby
        wbz = x(0);
        vbx = x(1);
        vby = x(2);
    }

    void publishTwist() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = vbx;
        twist_msg.linear.y = vby;
        twist_msg.angular.z = wbz;
        cmd_vel_publisher_->publish(twist_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicModelNode>());
    rclcpp::shutdown();
    return 0;
}
