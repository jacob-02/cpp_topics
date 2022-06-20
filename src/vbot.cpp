
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;

class SimulationRobot : public rclcpp::Node
{

public:
  double x = 0.0, y = 0.0, fi = 0.0;
  double periodTime = 0.1;

  SimulationRobot()
      : Node("vbot")
  {
    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 5, std::bind(&SimulationRobot::cmdVelCb, this, _1));
    odomBroadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    baseLinkBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped odomTf;
    rclcpp::Time now = this->get_clock()->now();
    odomTf.header.stamp = now;
    odomTf.header.frame_id = "/map";
    odomTf.child_frame_id = "odom";
    odomTf.transform.translation.x = 5; // initPose["x"];
    odomTf.transform.translation.y = 5; // initPose["y"];
    tf2::Quaternion qt;
    qt.setEuler(0, 0, 1.2); // initPose["a"]);
    odomTf.transform.rotation.w = qt.w();
    odomTf.transform.rotation.x = qt.x();
    odomTf.transform.rotation.y = qt.y();
    odomTf.transform.rotation.z = qt.z();
    odomBroadcaster_->sendTransform(odomTf);
  }

private:
  geometry_msgs::msg::Twist currentTwist;

  void cmdVelCb(const geometry_msgs::msg::Twist &msg)
  {
    //    ROS_ERROR("RECV: %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);

    currentTwist.angular.z = 0;
    currentTwist.linear.x = 0;
    currentTwist.linear.y = 0;
    currentTwist = msg;

    fi = fi + currentTwist.angular.z * periodTime;
    // Keep orient_ between -pi and +pi
    fi -= 2 * M_PI * std::floor((fi + M_PI) / (2 * M_PI));
    x += std::cos(fi) * currentTwist.linear.x * periodTime;
    y += std::sin(fi) * currentTwist.linear.x * periodTime;
    geometry_msgs::msg::TransformStamped baseLinkTf;
    rclcpp::Time now = this->get_clock()->now();
    baseLinkTf.header.stamp = now;
    baseLinkTf.header.frame_id = "odom";
    baseLinkTf.child_frame_id = "base_link";
    baseLinkTf.transform.translation.x = x;
    baseLinkTf.transform.translation.y = y;
    baseLinkTf.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, fi);
    baseLinkTf.transform.rotation.x = q.x();
    baseLinkTf.transform.rotation.y = q.y();
    baseLinkTf.transform.rotation.z = q.z();
    baseLinkTf.transform.rotation.w = q.w();
    baseLinkBroadcaster_->sendTransform(baseLinkTf);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> baseLinkBroadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odomBroadcaster_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationRobot>());
  rclcpp::shutdown();
  return 0;
}
