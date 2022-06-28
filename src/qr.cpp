#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
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
    qr_ = this->create_wall_timer(1000ms, std::bind(&QRPublisher::qrCB, this));
  }

private:
  std_msgs::msg::Int16 qr;
  int n = rand() % 2;
  void qrCB()
  {
    n = rand() % 2;
    qr.data = n;
    publisher_->publish(qr);
    RCLCPP_INFO(this->get_logger(), "QR %d", n);
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





































// #include <chrono>
// #include <functional>
// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/int16.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2/LinearMath/Quaternion.h>
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

//     publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(tf_prefix + "/qr", 1);
//     subscriber_true = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 1, std::bind(&QRPublisher::trueCb, this, std::placeholders::_1));
//     qr_ = this->create_wall_timer(50ms, std::bind(&QRPublisher::qrCB, this));
//   }

// private:
//   nav_msgs::msg::Odometry qr;
//   int n = rand() % 2;
//   double x, y, roll, pitch, yaw;
//   void qrCB()
//   {
//     for(int i = 0; i < 10; i++)
//     {
//       for(int j = 0; j < 10; j++)
//       {
//         if(x - 0.2 < i && i < x + 0.2 && y - 0.2 < j && j < y + 0.2)
//         {
//           qr.pose.pose.position.x = x;
//           qr.pose.pose.position.y = y;
//           qr.pose.pose.position.z = 0;

//           tf2::Quaternion q;
//           q.setRPY(roll, pitch, yaw);
//           qr.pose.pose.orientation.x = q.x();
//           qr.pose.pose.orientation.y = q.y();
//           qr.pose.pose.orientation.z = q.z();
//           qr.pose.pose.orientation.w = q.w();
          
//           publisher_->publish(qr);
//         }
//       }
//     }
//   }

//   void trueCb(const nav_msgs::msg::Odometry::SharedPtr msg)
//   {
//     x = msg->pose.pose.position.x;
//     y = msg->pose.pose.position.y;
//     tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
//     tf2::Matrix3x3 m(q);
//     m.getRPY(roll, pitch, yaw);
//   }

//   rclcpp::TimerBase::SharedPtr qr_;
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
//   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_true;
//   std::string tf_prefix;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<QRPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }