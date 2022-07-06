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
        publish_error = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel_error", 50);
        error = this->create_wall_timer(10ms, std::bind(&movePublisher::publish_error_callback, this));
        move_ = this->create_wall_timer(10ms, std::bind(&movePublisher::moveCb, this));
        subscribe_robot = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 50, std::bind(&movePublisher::robotCb, this, std::placeholders::_1));
        subscribe_error = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/pose_updated", 50, std::bind(&movePublisher::errorCb, this, std::placeholders::_1));
        subscribe_x = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/x_goal", 50, std::bind(&movePublisher::xCb, this, std::placeholders::_1));
        subscribe_y = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/y_goal", 50, std::bind(&movePublisher::yCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist move;
    geometry_msgs::msg::Twist move_error;

    double x, y;
    double x_error, y_error;
    double x_goal, y_goal, distanceTolerance = 0.02, angleTolerance = 0.02;
    double errorSum = 0.0, prevDistance = 0.0, errorDiff, distance = 0.0, angle = 0.0;
    double errorSum_error = 0.0, prevDistance_error = 0.0, errorDiff_error, distance_error = 0.0, angle_error = 0.0;
    double angleSum = 0.0, prevAngle = 0.0, angleDiff;
    double angleSum_error = 0.0, prevAngle_error = 0.0, angleDiff_error;
    double derivative, integral, proportional, pid, constants;
    double derivative_error, integral_error, proportional_error, pid_error, constants_error;
    double derivativeAngle, integralAngle, proportionalAngle, pidAngle, constantsAngle;
    double derivativeAngle_error, integralAngle_error, proportionalAngle_error, pidAngle_error, constantsAngle_error;
    double Kp = 0.06, Ki = 0.0, Kd = 0;
    double Kp_error = 0.06, Ki_error = 0.0, Kd_error = 0;
    double KpAngle = 0.69, KiAngle = 0, KdAngle = 0;
    double KpAngle_error = 3, KiAngle_error = 0, KdAngle_error = 0;
    double roll, pitch, yaw;
    double roll_error, pitch_error, yaw_error;

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

    void errorCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_error = msg->pose.pose.position.x;
        y_error = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_error, pitch_error, yaw_error);
    }

    void xCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        x_goal = msg->data;
    }

    void yCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        y_goal = msg->data;
    }

    void publish_error_callback()
    {
        distance_error = sqrt(pow(x_goal - x_error, 2) + pow(y_goal - y_error, 2));
        errorDiff_error = distance_error - prevDistance_error;
        errorSum_error += distance_error;

        derivative_error = Kd_error * errorDiff_error;
        integral_error = Ki_error * errorSum_error;
        proportional_error = Kp_error * distance_error;

        pid_error = proportional_error + integral_error + derivative_error;

        if (pid_error > 0.5)
        {
            pid_error = 0.5;
        }
        else if (pid_error < -0.5)
        {
            pid_error = -0.5;
        }

        move_error.linear.x = pid_error;

        angle_error = atan2(y_goal - y_error, x_goal - x_error) - yaw_error;
        angleDiff_error = angle_error - prevAngle_error;
        angleSum_error += angle_error;

        derivativeAngle_error = KdAngle_error * angleDiff_error;
        integralAngle_error = KiAngle_error * angleSum_error;
        proportionalAngle_error = KpAngle_error * angle_error;

        pidAngle_error = proportionalAngle_error + integralAngle_error + derivativeAngle_error;
        move_error.angular.z = pidAngle_error;

        prevDistance_error = distance_error;
        prevAngle_error = angle_error;

        if(std::fabs(distance_error) < distanceTolerance)
        {
            move_error.linear.x = 0;
        }

        if(std::fabs(angle_error) < angleTolerance)
        {
            move_error.angular.z = 0;
        }

        if (std::fabs(distance_error) < distanceTolerance && std::fabs(angle_error) < angleTolerance)
        {
            move_error.linear.x = 0;
            move_error.angular.z = 0;
        }

        publish_error->publish(move_error);
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

        angleSum += angle;
        errorDiff = angle - prevAngle;

        proportionalAngle = KpAngle * angle;
        integralAngle = KiAngle * angleSum;
        derivativeAngle = KdAngle * errorDiff;

        constantsAngle = KpAngle + KiAngle + KdAngle;
        pidAngle = proportionalAngle + integralAngle + derivativeAngle;

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

        if (std::fabs(angle) < angleTolerance && std::fabs(distance) < distanceTolerance)
        {
            move.linear.x = 0.0;
            move.angular.z = 0.0;

            publish_->publish(move);

        }
        if (std::fabs(errorSum) > 5000)
        {
            errorSum = 0.0;
        }
    }

    rclcpp::TimerBase::SharedPtr move_;
    rclcpp::TimerBase::SharedPtr error;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_error;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_x;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_y;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_error;

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