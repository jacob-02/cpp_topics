#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
        this->get_parameter("tf_prefix", tf_prefix);

        publish_ = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix+"/cmd_vel", 50);
        square_ = this->create_wall_timer(100ms, std::bind(&SquarePublisher::squareCb, this));
    }

private:
    geometry_msgs::msg::Twist square;
    void squareCb()
    {
        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.linear.x = 0.45;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(2));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.angular.z = 5.5;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.linear.x = 0.45;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(2));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.angular.z = 5.5;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.linear.x = 0.45;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(2));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.angular.z = 5.5;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.linear.x = 0.45;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(2));

        square.linear.x = 0.0;
        square.angular.z = 0.0;
        square.angular.z = 5.5;
        publish_->publish(square);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    rclcpp::TimerBase::SharedPtr square_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    std::string tf_prefix;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}