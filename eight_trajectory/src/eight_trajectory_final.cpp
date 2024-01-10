#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"




class AbsoluteMotion : public rclcpp::Node {
public:
    AbsoluteMotion() : Node("absolute_motion") {
        pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("wheel_speed", 10);
        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&AbsoluteMotion::odomCallback, this, std::placeholders::_1));
        //rclcpp::sleep_for(std::chrono::seconds(3));
        timer1 = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AbsoluteMotion::timerCallback1, this));

        //timer2 = this->create_wall_timer(10ms, std::bind(&EightTrajectory::timerCallback2, this));
        
        
        //

        
        motions = {
            {0.0, 1.0, -1.0}, {0.0, 1.0, 1.0}, {0.0, 1.0, 1.0}, {1.570796, 1.0, -1.0},
            {-3.141592, -1.0, -1.0}, {0.0, -1.0, 1.0}, {0.0, -1.0, 1.0}, {1.570796, -1.0, -1.0}};
        start = std::chrono::high_resolution_clock::now();



        /*
        std::for_each(motions.begin(), motions.end(), [&](const auto& motion) {
            std::tie(dphi_, dx_, dy_) = std::make_tuple(motion[0], motion[1], motion[2]);
            //publish_motion();
            std::this_thread::sleep_for(std::chrono::seconds(9));
            RCLCPP_INFO(get_logger(), "Next WP Execution");

        });


        std_msgs::msg::Float32MultiArray stop_msg;
        stop_msg.data = {0, 0, 0, 0};
        pub_->publish(stop_msg);*/

    }

private:

    void timerCallback1() {
        

        

        double dphi = motions[count][0]/4;
        double dx = motions[count][1]/4;
        double dy = motions[count][2]/4;

        auto twist = velocity2twist(dphi, dx, dy);
        auto wheels = twist2wheels(twist[0], twist[1], twist[2]);

        std_msgs::msg::Float32MultiArray wsm;
        wsm.data.assign(wheels.begin(), wheels.end());
        //RCLCPP_INFO(get_logger(), "u8t = [%f, %f, %f, %f]", wheels[0], wheels[1], wheels[2], wheels[3]);
        pub_->publish(wsm);

        end = std::chrono::high_resolution_clock::now();

        if(std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count() > 9.0){
            count++;
            RCLCPP_INFO(get_logger(), "Execution Complete WP: %d", count);
            start = std::chrono::high_resolution_clock::now();
        }

        if(count == motions.size()){
            std_msgs::msg::Float32MultiArray stop_msg;
            stop_msg.data = {0, 0, 0, 0};
            pub_->publish(stop_msg);
            RCLCPP_INFO(get_logger(), "EXIT!");
            rclcpp::shutdown();
            return;
        }

    }


    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        phi = yaw;
    }

    void publish_motion() {
        for (int i = 0; i < 750; ++i) {
            double dphi = dphi_ / 5;
            double dx = dx_ / 5;
            double dy = dy_ / 5;
            auto twist = velocity2twist(dphi, dx, dy);
            auto u = twist2wheels(twist[0], twist[1], twist[2]);

            std_msgs::msg::Float32MultiArray msg;
            msg.data.assign(u.begin(), u.end());

            RCLCPP_INFO(get_logger(), "u8t = [%f, %f, %f, %f]", u[0], u[1], u[2], u[3]);
            pub_->publish(msg);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
    }

    std::vector<double> velocity2twist(double dphi, double dx, double dy) {
        std::vector<double> twist =  {};
        twist.push_back(dphi);
        twist.push_back(dx * cos(phi) + dy * sin(phi));
        twist.push_back(-dx * sin(phi) + dy * cos(phi));
        return twist;
    }

    std::vector<double> twist2wheels(double wz, double vx, double vy) {
        std::vector<double> wheels = {};

        double r = 0.1/2;
        double wb = 0.17/2;
        double l = 0.26969/2;
        wheels.push_back((-wz*(wb+l)+vx-vy)/r);
        wheels.push_back((wz*(wb+l)+vx+vy)/r);
        wheels.push_back((wz*(wb+l)+vx-vy)/r);
        wheels.push_back((-wz*(wb+l)+vx+vy)/r);

        return wheels;
    }


private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    double phi = 0;
    double dphi_=0;
    double dx_=0;
    double dy_=0;
    std::vector<std::vector<double>> motions;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    int count = 0;
    rclcpp::TimerBase::SharedPtr timer1, timer2;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AbsoluteMotion>());
    rclcpp::shutdown();
    return 0;
}
