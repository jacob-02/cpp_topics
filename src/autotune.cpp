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
    double x_goal, y_goal, angleTolerance = 0.000029;
    double angleSum = 0.0, prevAngle = 0.0, angleDiff;
    double angle, errorDiff;
    double derivativeAngle, integralAngle, proportionalAngle, pidAngle, constantsAngle;
    double KpAngle = 1.69, KiAngle = 0, KdAngle = 0;
    double roll, pitch, yaw;
    double distance;
    double time;
    int i = 0;

    rclcpp::Time now_1 = this->get_clock()->now();

    void moveCb()
    {
        distance = sqrt(std::pow(x_goal - x, 2) + std::pow(y_goal - y, 2));

        move.linear.x = 0.3;

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

        RCLCPP_INFO(this->get_logger(), "Angle: %f", angle);

        rclcpp::Time now_current = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(), "Time: %f", now_current.seconds() - now_1.seconds());

        if ((now_current.seconds() - now_1.seconds()) > 25)
        {
            RCLCPP_INFO(this->get_logger(), "Very wrong Kp");
            rclcpp::shutdown();
        }

        if (std::fabs(angle) < (angleTolerance))
        {
            move.angular.z = 0.0;
            move.linear.x = 0.0;

            i++;
            if (i > 10)
            {
                rclcpp::Time now_2 = this->get_clock()->now();
                RCLCPP_INFO(this->get_logger(), "Angle: %f, Time: %f", angle, now_2.seconds() - now_1.seconds());

                move.angular.z = 0.0;
                move.linear.x = 0.0;

                rclcpp::shutdown();
            }
        }

        if (distance < 0.01)
        {
            move.linear.x = 0.0;
        }

        publish_->publish(move);
    }

    void xCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        x_goal = msg->data;
    }

    void yCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        y_goal = msg->data;
    }

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

    rclcpp::TimerBase::SharedPtr move_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_x;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_y;

    std::string tf_prefix;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movePublisher>());
    rclcpp::shutdown();
    return 0;
}