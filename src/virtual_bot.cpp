#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
public:
    double x = 0.0, y = 0.0, fi = 0.0;
    double periodTime = 0.1;
    FramePublisher()
        : Node("transform_publisher")
    {
        // Initialize the transform broadcaster

        this->declare_parameter<std::string>("tf_prefix", "r2");
        this->declare_parameter<std::double_t>("x_pos", 1.8);
        this->declare_parameter<std::double_t>("y_pos", -0.7);
        this->declare_parameter<std::double_t>("a_pos", 1.56);

        this->get_parameter("tf_prefix", tf_prefix);
        this->get_parameter("x_pos", x_pos);
        this->get_parameter("y_pos", y_pos);
        this->get_parameter("a_pos", a_pos);

        odomFrame = tf_prefix + "/odom";
        baseLinkFrame = tf_prefix + "/base_link";

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 100, std::bind(&FramePublisher::cmdVelPb, this, _1));
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("robot", 100);

        marker_ = this->create_wall_timer(50ms, std::bind(&FramePublisher::markerCb, this));
        tf_ = this->create_wall_timer(50ms, std::bind(&FramePublisher::tfCb, this));

        baseLinkBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odomBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

private:
    geometry_msgs::msg::Twist currentTwist;
    geometry_msgs::msg::TransformStamped baseLinkTf;
    geometry_msgs::msg::TransformStamped odomTf;
    visualization_msgs::msg::Marker robot;

    tf2::Quaternion q;

    void cmdVelPb(const geometry_msgs::msg::Twist &msg)
    {
        currentTwist = msg;

        fi = fi + currentTwist.angular.z * periodTime;

        fi -= 2 * M_PI * floor((fi + M_PI) / (2 * M_PI));
        x += cos(fi) * currentTwist.linear.x * periodTime;
        y += sin(fi) * currentTwist.linear.x * periodTime;
    }

    void tfCb()
    {
        rclcpp::Time now = this->get_clock()->now();

        baseLinkTf.header.frame_id = odomFrame;
        baseLinkTf.header.stamp = now;
        baseLinkTf.child_frame_id = baseLinkFrame;

        baseLinkTf.transform.translation.x = x;
        baseLinkTf.transform.translation.y = y;
        baseLinkTf.transform.translation.z = 0;

        q.setRPY(0, 0, fi);
        baseLinkTf.transform.rotation.w = q.w();
        baseLinkTf.transform.rotation.x = q.x();
        baseLinkTf.transform.rotation.y = q.y();
        baseLinkTf.transform.rotation.z = q.z();

        baseLinkBroadcaster->sendTransform(baseLinkTf);

        odomTf.header.stamp = now;
        odomTf.header.frame_id = "/map";
        odomTf.child_frame_id = odomFrame;
        odomTf.transform.translation.x = x_pos;
        odomTf.transform.translation.y = y_pos;

        tf2::Quaternion qt;
        qt.setEuler(0, 0, a_pos);
        odomTf.transform.rotation.w = qt.w();
        odomTf.transform.rotation.x = qt.x();
        odomTf.transform.rotation.y = qt.y();
        odomTf.transform.rotation.z = qt.z();

        odomBroadcaster->sendTransform(odomTf);
    }

    void markerCb()
    {
        rclcpp::Time now = this->get_clock()->now();

        robot.header.frame_id = baseLinkFrame;
        robot.header.stamp = now;
        robot.ns = tf_prefix;
        robot.id = 0;
        robot.type = visualization_msgs::msg::Marker::ARROW;
        robot.action = visualization_msgs::msg::Marker::ADD;
        robot.scale.x = 1;
        robot.scale.y = 0.1;
        robot.scale.z = 0.1;
        robot.color.a = 1.0;
        robot.color.r = 1.0;
        robot.color.g = 0.0;
        robot.color.b = 0.0;

        publisher_->publish(robot);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr marker_;
    rclcpp::TimerBase::SharedPtr tf_;

    std::string tf_prefix;
    std::string odomFrame;
    std::string baseLinkFrame;
    std::double_t x_pos;
    std::double_t y_pos;
    std::double_t a_pos;

    std::unique_ptr<tf2_ros::TransformBroadcaster> baseLinkBroadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odomBroadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}