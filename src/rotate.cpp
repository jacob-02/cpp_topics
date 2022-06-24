#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;
using namespace rclcpp;

class movePublisher : public rclcpp::Node
{
public:
    movePublisher()
        : Node("move_publisher")
    {
        this->declare_parameter<std::string>("tf_prefix", "r0");

        this->get_parameter("tf_prefix", tf_prefix);

        publish_ = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel", 50);
        move_ = this->create_wall_timer(10ms, std::bind(&movePublisher::moveCb, this));
        subscribe_robot = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 50, std::bind(&movePublisher::robotCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist move;

    double angle_goal = 20.0, angleTolerance = 0.01;
    double errorSum = 0.0, prevDistance = 0.0, errorDiff, angle = 0.0;
    double derivative, integral, proportional, pid, constants;
    double Kp = 0.1, Ki = 0.000005, Kd = 0.1;
    double roll, pitch, yaw;

    void moveCb()
    {
        angle = 1.57 - yaw;
        errorSum += angle;
        errorDiff = angle - prevDistance;

        proportional = Kp * angle;
        integral = Ki * errorSum;
        derivative = Kd * errorDiff;

        constants = Kp + Ki + Kd;
        pid = proportional + integral + derivative;

        if (pid > 0.5)
        {
            pid = 0.5;
        }
        else if (pid < -0.5)
        {
            pid = -0.5;
        }
        RCLCPP_INFO(this->get_logger(), "angle: %f", angle);

        move.angular.z = pid;
        publish_->publish(move);

        prevDistance = angle;

        if (std::fabs(angle) < angleTolerance)
        {
            proportional = 0;
            integral = 0;
            derivative = 0;
            move.angular.z = 0.0;
            publish_->publish(move);
        }
    }

    void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // x = msg->pose.pose.position.x;
        // y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    rclcpp::TimerBase::SharedPtr move_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;

    std::string tf_prefix;
    std::double_t x_pos;
    std::double_t y_pos;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movePublisher>());
    rclcpp::shutdown();
    return 0;
}