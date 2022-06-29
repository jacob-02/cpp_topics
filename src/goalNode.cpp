#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>

using namespace std::chrono_literals;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class GoalPublisher : public rclcpp::Node
{
public:
  GoalPublisher()
      : Node("goal_publisher")
  {
    this->declare_parameter<std::string>("tf_prefix", "r0");
    this->get_parameter("tf_prefix", tf_prefix);

    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 50, std::bind(&GoalPublisher::robotCb, this, std::placeholders::_1));

    publisher_x = this->create_publisher<std_msgs::msg::Int16>(tf_prefix + "/x_goal", 1);
    publisher_y = this->create_publisher<std_msgs::msg::Int16>(tf_prefix + "/y_goal", 1);

    goal_x = this->create_wall_timer(50ms, std::bind(&GoalPublisher::goalCB_x, this));
    goal_y = this->create_wall_timer(50ms, std::bind(&GoalPublisher::goalCB_y, this));
  }

private:
  std_msgs::msg::Int16 goal_x_msg;
  std_msgs::msg::Int16 goal_y_msg;

  double x, y, yaw;
  double x_vel, yaw_vel;

  void goalCB_x()
  {
    goal_x_msg.data = -5.0;

    publisher_x->publish(goal_x_msg);
  }

  void goalCB_y()
  {
    goal_y_msg.data = 5.0;

    publisher_y->publish(goal_y_msg);
  }

  void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_vel = msg->twist.twist.linear.x;
    yaw_vel = msg->twist.twist.angular.z;
  }

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_x;
  rclcpp::TimerBase::SharedPtr goal_x;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_y;
  rclcpp::TimerBase::SharedPtr goal_y;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
  std::string tf_prefix;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPublisher>());
  rclcpp::shutdown();
  return 0;
}