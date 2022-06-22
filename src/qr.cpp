#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

using namespace std::chrono_literals;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class QRPublisher : public rclcpp::Node
{
public:
  QRPublisher()
      : Node("qr_publisher")
  {
    this->declare_parameter<std::string>("tf_prefix", "r0");
    this->get_parameter("tf_prefix", tf_prefix);

    publisher_ = this->create_publisher<std_msgs::msg::Int16>(tf_prefix + "/qr", 50);
    qr_ = this->create_wall_timer(100ms, std::bind(&QRPublisher::qrCB, this));
  }

private:
  std_msgs::msg::Int16 qr;
  int n = 1;
  void qrCB()
  {
    qr.data = n;
    publisher_->publish(qr);
    n++;
  }
  rclcpp::TimerBase::SharedPtr qr_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  std::string tf_prefix;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRPublisher>());
  rclcpp::shutdown();
  return 0;
}