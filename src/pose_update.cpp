#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <chrono>
#include <functional>
#include <memory>

using std::placeholders::_1;
using namespace std::chrono_literals;

class PoseUpdater : public rclcpp::Node
{
public:
    PoseUpdater()
        : Node("pose_updater")
    {
        subscription_robot = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100, std::bind(&PoseUpdater::robotCb, this, _1));
        subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 100, std::bind(&PoseUpdater::odomCb, this, _1));
        subscription_qr = this->create_subscription<std_msgs::msg::Int16>("qr", 100, std::bind(&PoseUpdater::qrCb, this, _1));
        publisher_pose = this->create_publisher<nav_msgs::msg::Odometry>("/pose_updated", 100);

        pose_updated = this->create_wall_timer(50ms, std::bind(&PoseUpdater::poseUpdatedCb, this));
    }

private:
    nav_msgs::msg::Odometry correctPose;
    nav_msgs::msg::Odometry robotPose;
    std_msgs::msg::Int16 qr;
    nav_msgs::msg::Odometry finalPose;

    void robotCb(const tf2_msgs::msg::TFMessage &msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        correctPose.header.stamp = now;
        correctPose.header.frame_id = "r1/odom";
        correctPose.child_frame_id = "r1/base_link";
    
        correctPose.pose.pose.position.x = msg.transforms[0].transform.translation.x;        
        correctPose.pose.pose.position.y = msg.transforms[0].transform.translation.y;        
        correctPose.pose.pose.position.z = msg.transforms[0].transform.translation.z;

        correctPose.pose.pose.orientation.x = msg.transforms[0].transform.rotation.x;        
        correctPose.pose.pose.orientation.y = msg.transforms[0].transform.rotation.y;        
        correctPose.pose.pose.orientation.z = msg.transforms[0].transform.rotation.z;        
    }

    void odomCb(const nav_msgs::msg::Odometry &msg)
    {
        rclcpp::Time now = this->get_clock()->now();

        robotPose = msg;
        robotPose.header.stamp = now;
        robotPose.header.frame_id = "r1/odom";
        robotPose.header.frame_id = "r1/base_link";

        publisher_pose->publish(robotPose);       
    }

    void qrCb(const std_msgs::msg::Int16 &msg)
    {
        qr = msg;
    }

    void poseUpdatedCb()
    {
        finalPose = robotPose;

        if (qr.data == 1)
        {
            finalPose.pose.pose = correctPose.pose.pose;
        }
        // publisher_pose->publish(robotPose);       
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_robot;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_qr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_pose;
    rclcpp::TimerBase::SharedPtr pose_updated;
};

int
main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseUpdater>());
    rclcpp::shutdown();
    return 0;
}