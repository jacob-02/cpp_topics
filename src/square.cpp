#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
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
        subscribe_x = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/x_goal", 50, std::bind(&movePublisher::xCb, this, std::placeholders::_1));
        subscribe_y = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/y_goal", 50, std::bind(&movePublisher::yCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist move;

    double x, y;
    double x_goal, y_goal, distanceTolerance = 0.02, angleTolerance = 0.02;
    // double x_new_goal, y_new_goal;
    double errorSum = 0.0, prevDistance = 0.0, errorDiff, distance = 0.0, angle = 0.0;
    double angleSum = 0.0, prevAngle = 0.0, angleDiff;
    double derivative, integral, proportional, pid, constants;
    double derivativeAngle, integralAngle, proportionalAngle, pidAngle, constantsAngle;
    // double Kp = 0.06, Ki = 0.0, Kd = 0;
    // double KpAngle = 1.69, KiAngle = 0, KdAngle = 0;
    double Kp = 0.06, Ki = 0.0, Kd = 0;
    double KpAngle = 1.69, KiAngle = 0, KdAngle = 0;
    double roll, pitch, yaw;

    void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    void xCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        x_goal = msg->data;
    }

    void yCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        y_goal = msg->data;
    }

    void moveCb()
    {
        distance = sqrt(std::pow(x_goal - x, 2) + std::pow(y_goal - y, 2));
        errorSum += distance;
        errorDiff = distance - prevDistance;

        proportional = Kp * distance;
        integral = Ki * errorSum;
        derivative = Kd * errorDiff;

        constants = Kp + Ki + Kd;
        pid = proportional + integral + derivative;

        // RCLCPP_INFO(this->get_logger(), "distance: %f, error sum: %f, error diff: %f", distance, errorSum, errorDiff);

        if (pid > 0.3)
        {
            pid = 0.3;
        }
        else if (pid < -0.3)
        {
            pid = -0.3;
        }

        move.linear.x = pid;

        angle = atan2(y_goal - y, x_goal - x) - yaw;

        // if (std::fabs(angle) > 0.2)
        // {
        //     move.linear.x = 0.0;
        // }

        angleSum += angle;
        errorDiff = angle - prevAngle;

        proportionalAngle = KpAngle * angle;
        integralAngle = KiAngle * angleSum;
        derivativeAngle = KdAngle * errorDiff;

        constantsAngle = KpAngle + KiAngle + KdAngle;
        pidAngle = proportionalAngle + integralAngle + derivativeAngle;

        // RCLCPP_INFO(this->get_logger(), "angle: %f, error sum: %f, error diff: %f", angle, angleSum, errorDiff);

        // if (pidAngle > 0.3)
        // {
        //     pidAngle = 0.3;
        // }
        // else if (pidAngle < -0.3)
        // {
        //     pidAngle = -0.3;
        // }

        move.angular.z = pidAngle;

        prevAngle = angle;

        publish_->publish(move);

        prevDistance = distance;

        if (std::fabs(distance) < distanceTolerance)
        {
            move.linear.x = 0.0;
            move.angular.z = 0.0;

            publish_->publish(move);
        }

        if (std::fabs(angle) < angleTolerance)
        {
            move.angular.z = 0.0;
            // move.linear.x = 0.0;

            publish_->publish(move);
        }

        if (std::fabs(errorSum) > 5000)
        {
            errorSum = 0.0;
        }
    }

    rclcpp::TimerBase::SharedPtr move_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_x;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_y;

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