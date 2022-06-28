#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <random>

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

    publisher_ = this->create_publisher<std_msgs::msg::Int16>(tf_prefix + "/qr", 1);
    subscriber_true = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 1, std::bind(&QRPublisher::trueCb, this, std::placeholders::_1));
    // qr_ = this->create_wall_timer(50ms, std::bind(&QRPublisher::qrCB, this));
  }

private:
  std_msgs::msg::Int16 qr;
  double x, y, roll, pitch, yaw;

  int qrCB()
  {
    for (float i = -10; i < 10; i++)
    {
      for (float j = -10; j < 10; j++)
      {
        if ((std::fabs(x - i) <= 0.01) && (std::fabs(y - j) <= 0.01))
          return 1;
        // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x, y);
      }
    }
    //
    return 0;
  }

  void trueCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // RCLCPP_INFO(this->get_logger(), "x: %f, y: %f", x, y);

    qr.data = qrCB();
    // RCLCPP_INFO(this->get_logger(), "qrCb: %d", qr.data);

    publisher_->publish(qr);
  }

  rclcpp::TimerBase::SharedPtr qr_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_true;
  std::string tf_prefix;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QRPublisher>());
  rclcpp::shutdown();
  return 0;
}

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int16.hpp>
// #include <random>

// using namespace std::chrono_literals;
// using namespace rclcpp;

// /* This example creates a subclass of Node and uses std::bind() to register a
//  * member function as a callback from the timer. */

// class QRPublisher : public rclcpp::Node
// {
// public:
//   QRPublisher()
//       : Node("qr_publisher")
//   {
//     this->declare_parameter<std::string>("tf_prefix", "r0");
//     this->get_parameter("tf_prefix", tf_prefix);

//     publisher_ = this->create_publisher<std_msgs::msg::Int16>(tf_prefix + "/qr", 1);
//     qr_ = this->create_wall_timer(1000ms, std::bind(&QRPublisher::qrCB, this));
//   }

// private:
//   std_msgs::msg::Int16 qr;
//   int n = rand() % 2;
//   void qrCB()
//   {
//     n = rand() % 2;
//     qr.data = n;
//     publisher_->publish(qr);
//     RCLCPP_INFO(this->get_logger(), "QR %d", n);
//   }
//   rclcpp::TimerBase::SharedPtr qr_;
//   rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr publisher_;
//   std::string tf_prefix;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<QRPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }