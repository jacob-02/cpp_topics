#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_msgs/msg/int16.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <random>

using std::placeholders::_1;
using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher()
        : Node("transform_publisher")
    {
        // Initialize the transform broadcaster

        this->declare_parameter<std::string>("tf_prefix", "r0");
        this->declare_parameter<std::double_t>("x_pos", 1.8);
        this->declare_parameter<std::double_t>("y_pos", -0.7);
        this->declare_parameter<std::double_t>("a_pos", 1.56);

        this->get_parameter("tf_prefix", tf_prefix);
        this->get_parameter("x_pos", x_pos);
        this->get_parameter("y_pos", y_pos);
        this->get_parameter("a_pos", a_pos);

        odomFrame = tf_prefix + "/odom";
        baseLinkFrame = tf_prefix + "/base_link";
        fusedFrame = tf_prefix + "/fused";

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel", 10, std::bind(&FramePublisher::cmdVelPb, this, _1));
        subscription_error = this->create_subscription<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel_error", 10, std::bind(&FramePublisher::errorcmdVelPb, this, _1));

        publisher_true = this->create_publisher<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 10);
        publisher_pose = this->create_publisher<nav_msgs::msg::Odometry>(tf_prefix + "/pose_updated", 10);

        subscription_qr = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/qr", 100, std::bind(&FramePublisher::qrCb, this, _1));
        subscribe_x = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/x_goal", 50, std::bind(&FramePublisher::xCb, this, _1));
        subscribe_y = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/y_goal", 50, std::bind(&FramePublisher::yCb, this, _1));

        pose_updated = this->create_wall_timer(50ms, std::bind(&FramePublisher::poseUpdatedCb, this));
        marker_ = this->create_wall_timer(50ms, std::bind(&FramePublisher::trueCb, this));
        tf_ = this->create_wall_timer(50ms, std::bind(&FramePublisher::tfCb, this));

        baseLinkBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        fusedOdomBroadcaster =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odomBroadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    }

private:
    geometry_msgs::msg::Twist currentTwist;
    geometry_msgs::msg::Twist currentErrorTwist;
    geometry_msgs::msg::TransformStamped baseLinkTf;
    geometry_msgs::msg::TransformStamped odomTf;
    geometry_msgs::msg::TransformStamped fusedTf;
    nav_msgs::msg::Odometry robot;
    nav_msgs::msg::Odometry odometry;
    tf2::Quaternion q;
    std_msgs::msg::Int16 qr;
    nav_msgs::msg::Odometry fusedOdom;

    double x = 0.0, y = 0.0, fi = 0.0;
    double x_error = x, y_error = y, fi_error = 0.1;
    double periodTime = 0.1;
    double error_x = 0.0, error_y = 0.0, error_theta = 0.0;
    double error_x_new = 0.0, error_y_new = 0.0, error_theta_new = 0.0;
    double error_x_bot = 0.0, error_y_bot = 0.0, error_theta_bot = 0.0;
    double error_x_new_bot = 0.0, error_y_new_bot = 0.0, error_theta_new_bot = 0.0;
    double x_goal = 10, y_goal = 0, angleTolerance = 0.05;
    double x_bot = x, y_bot = y, fi_bot = 0.0;
    double x_bot_new, y_bot_new, fi_bot_new;

    void poseUpdatedCb()
    {
        fi_error = fi_error + currentErrorTwist.angular.z * periodTime;

        fi_error -= 2 * M_PI * floor((fi_error + M_PI) / (2 * M_PI));
        x_error += cos(fi_error) * (currentErrorTwist.linear.x * periodTime);
        y_error += sin(fi_error) * (currentErrorTwist.linear.x * periodTime);

        error_x_new = ((double(rand()) / RAND_MAX) - 0.0) / 1000;
        error_y_new = ((double(rand()) / RAND_MAX) - 0.0) / 1000;
        error_theta_new = ((double(rand()) / RAND_MAX) - 0.0) / 1000;

        error_x_new_bot = ((double(rand()) / RAND_MAX) - 0.0) / 1000;
        error_y_new_bot = ((double(rand()) / RAND_MAX) - 0.0) / 1000;
        error_theta_new_bot = ((double(rand()) / RAND_MAX) - 0.0) / 1000;

        x_bot = x_error + error_x_new + error_x_new_bot;
        y_bot = y_error + error_y_new + error_y_new_bot;
        fi_bot = fi_error + error_theta_new + error_theta_new_bot;

        // if ((qr.data == 1))
        // {
        //     error_x_new = 0.0;
        //     error_y_new = 0.0;
        //     error_theta_new = 0.0;
        // }

        rclcpp::Time now = this->get_clock()->now();

        fusedOdom.header.stamp = now;
        fusedOdom.header.frame_id = odomFrame;
        fusedOdom.child_frame_id = fusedFrame;

        fusedOdom.pose.pose.position.x = x_bot;
        fusedOdom.pose.pose.position.y = y_bot;
        fusedOdom.pose.pose.position.z = 0;

        tf2::Quaternion q_new;
        q_new.setEuler(0, 0, fi_error);
        fusedOdom.pose.pose.orientation.x = q_new.x();
        fusedOdom.pose.pose.orientation.y = q_new.y();
        fusedOdom.pose.pose.orientation.z = q_new.z();
        fusedOdom.pose.pose.orientation.w = q_new.w();

        fusedOdom.twist.twist = currentErrorTwist;

        publisher_pose->publish(fusedOdom);
    }

    void trueCb()
    {
        fi = fi + currentTwist.angular.z * periodTime;

        fi -= 2 * M_PI * floor((fi + M_PI) / (2 * M_PI));
        x += cos(fi) * (currentTwist.linear.x * periodTime);
        y += sin(fi) * (currentTwist.linear.x * periodTime);

        rclcpp::Time now = this->get_clock()->now();

        robot.header.frame_id = odomFrame;
        robot.child_frame_id = baseLinkFrame;
        robot.header.stamp = now;

        robot.pose.pose.position.x = x;
        robot.pose.pose.position.y = y;
        robot.pose.pose.position.z = 0;

        tf2::Quaternion q_true;
        q_true.setEuler(0, 0, fi);
        robot.pose.pose.orientation.x = q_true.x();
        robot.pose.pose.orientation.y = q_true.y();
        robot.pose.pose.orientation.z = q_true.z();
        robot.pose.pose.orientation.w = q_true.w();

        robot.twist.twist = currentTwist;

        publisher_true->publish(robot);
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

        fusedTf.header.frame_id = odomFrame;
        fusedTf.header.stamp = now;
        fusedTf.child_frame_id = fusedFrame;

        fusedTf.transform.translation.x = x_bot;
        fusedTf.transform.translation.y = y_bot;
        fusedTf.transform.translation.z = 0;

        q.setRPY(0, 0, fi_bot);
        fusedTf.transform.rotation.w = q.w();
        fusedTf.transform.rotation.x = q.x();
        fusedTf.transform.rotation.y = q.y();
        fusedTf.transform.rotation.z = q.z();

        fusedOdomBroadcaster->sendTransform(fusedTf);

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

    void cmdVelPb(const geometry_msgs::msg::Twist &msg)
    {
        currentTwist = msg;
    }

    void errorcmdVelPb(const geometry_msgs::msg::Twist &msg)
    {
        currentErrorTwist = msg;
    }

    void qrCb(const std_msgs::msg::Int16 &msg)
    {
        qr = msg;
    }

    void xCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        x_goal = msg->data;
    }

    void yCb(const std_msgs::msg::Int16::SharedPtr msg)
    {
        y_goal = msg->data;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_error;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_true;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_qr;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_pose;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_x;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_y;
    rclcpp::TimerBase::SharedPtr pose_updated;
    rclcpp::TimerBase::SharedPtr marker_;
    rclcpp::TimerBase::SharedPtr tf_;
    rclcpp::TimerBase::SharedPtr odom_;

    std::string tf_prefix;
    std::string odomFrame;
    std::string baseLinkFrame;
    std::string fusedFrame;
    std::double_t x_pos;
    std::double_t y_pos;
    std::double_t a_pos;

    std::unique_ptr<tf2_ros::TransformBroadcaster> baseLinkBroadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> fusedOdomBroadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> odomBroadcaster;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}