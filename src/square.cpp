#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

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

        publish_ = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix+"/cmd_vel", 50);
        square_ = this->create_wall_timer(100ms, std::bind(&SquarePublisher::squareCb, this));
        subscribe_robot = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix+"/robot", 50, std::bind(&SquarePublisher::robotCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist square;
    double x1 = 0, y1 = 0, x2 = 3, y2 = 0, x3 = 3, y3 = 3, x4 = 0, y4 = 3, x_pos, y_pos;

    void squareCb()
    {
        
    }

    void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_pos = msg->pose.pose.position.x;
        y_pos = msg->pose.pose.position.y;
    }

    rclcpp::TimerBase::SharedPtr square_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;

    std::string tf_prefix;
    std::double_t x_pos;
    std::double_t y_pos;
    std::double_t a_pos;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}