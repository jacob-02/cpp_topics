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

class SquarePublisher : public rclcpp::Node
{
public:
    SquarePublisher()
        : Node("square_publisher")
    {
        this->declare_parameter<std::string>("tf_prefix", "r0");
        this->declare_parameter<std::double_t>("x_pos", 1.8);
        this->declare_parameter<std::double_t>("y_pos", -0.7);

        this->get_parameter("tf_prefix", tf_prefix);
        this->get_parameter("x_pos", x_pos);
        this->get_parameter("y_pos", y_pos);

        publish_ = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel", 50);
        square_ = this->create_wall_timer(100ms, std::bind(&SquarePublisher::squareCb, this));
        subscribe_robot = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 50, std::bind(&SquarePublisher::robotCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist square;
    double x_current, y_current, x_goal = 3.0, y_goal = 0.0;
    double roll, pitch, yaw_current;
    int flag = 0;

    void squareCb()
    {
        if (flag != 1)
        {
            double error_x = (sqrt(pow(x_goal - x_current, 2) + pow(y_goal - y_current, 2)));
            square.linear.x = 0.1 * error_x;
            square.angular.z = 0;

            // double error_theta = atan2(y_goal - y_current, x_goal - x_current);
            // square.angular.z = 0.1 * error_theta;
            RCLCPP_INFO(this->get_logger(), "FLAG: 1");
            RCLCPP_INFO(this->get_logger(), "ERROR_X: '%F'", error_x);

            if (error_x < 0.01 && flag == 0)
            {
                square.linear.x = 0;
                flag = 1;
            }

            if (error_x < 0.1 && flag == 2)
            {
                RCLCPP_INFO(this->get_logger(), "FLAG: 3");

                square.linear.x = 0;
                flag = 3;
            }
        }

        if (flag == 3)
        {
            RCLCPP_INFO(this->get_logger(), "FLAG: 3");

            square.linear.x = 0;
            square.angular.z = 0;
        }

        if (flag == 1)
        {
            for (int i = 0; i < 26; i++)
            {
                square.linear.x = 0;
                square.angular.z = 0.1;
                RCLCPP_INFO(this->get_logger(), "FLAG: 2");
                RCLCPP_INFO(this->get_logger(), "counter: '%d'", i);
                // this->get_clock()->sleep_for(10ms);
                rclcpp::sleep_for(std::chrono::nanoseconds(105300000));
                publish_->publish(square);
            }
            flag = 2;
            y_goal = 3.0;
        }

        publish_->publish(square);
    }

    void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_current = msg->pose.pose.position.x;
        y_current = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        roll, pitch, yaw_current;
        m.getRPY(roll, pitch, yaw_current);
    }

    rclcpp::TimerBase::SharedPtr square_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;

    std::string tf_prefix;
    std::double_t x_pos;
    std::double_t y_pos;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}