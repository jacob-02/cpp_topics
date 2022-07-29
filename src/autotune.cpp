#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;
using namespace rclcpp;

struct PID()
{
    double kp;
    double ki;
    double kd;
    double vKp;
    double vKi;
    double vKd;
    double output;
};

class movePublisher : public rclcpp::Node
{
public:
    movePublisher()
        : Node("move_publisher")
    {
        this->declare_parameter<std::string>("tf_prefix", "r0");

        this->get_parameter("tf_prefix", tf_prefix);

        publish_ = this->create_publisher<geometry_msgs::msg::Twist>(tf_prefix + "/cmd_vel", 50);
        move_ = this->create_wall_timer(10ms, std::bind(&movePublisher::moveCb, this));
        subscribe_robot = this->create_subscription<nav_msgs::msg::Odometry>(tf_prefix + "/robot", 50, std::bind(&movePublisher::robotCb, this, std::placeholders::_1));
        subscribe_x = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/x_goal", 50, std::bind(&movePublisher::xCb, this, std::placeholders::_1));
        subscribe_y = this->create_subscription<std_msgs::msg::Int16>(tf_prefix + "/y_goal", 50, std::bind(&movePublisher::yCb, this, std::placeholders::_1));
    }

private:
    geometry_msgs::msg::Twist move;
    geometry_msgs::msg::Twist move_error;

    double x, y;
    double x_goal, y_goal, distanceTolerance = 0.02, angleTolerance = 0.02;
    double errorSum = 0.0, prevDistance = 0.0, errorDiff, distance = 0.0, angle = 0.0;
    double dt = 0.01;
    double angleSum = 0.0, prevAngle = 0.0, angleDiff;
    double derivative, integral, proportional, pid, constants;
    double derivativeAngle, integralAngle, proportionalAngle, pidAngle, constantsAngle;
    double Kp = 0.06, Ki = 0.0, Kd = 0;
    double KpAngle = 0.69, KiAngle = 0, KdAngle = 0;
    double roll, pitch, yaw;
    double w, v, bot_x, bot_y, W, cost;
    int popsize = 30, npar = 3, maxit = 300, j = 0;
    double c = 0.05, c1 = 0.0, c2 = 2, c3 = 2;
    double gcost = 0.0;
    std::vector<PID> par, localpar, parp, globalpar;
    std::vector<double> coast, icost;

    void gen_particles(int popsize, int npar, float mean, float std_dev)
    {
        par.resize(popsize);
        for (int i = 0; i < popsize; i++)
        {
            par[i].kp = mean + std_dev * (((double(rand()) / RAND_MAX) - 0.5));
            par[i].ki = mean + std_dev * (((double(rand()) / RAND_MAX) - 0.5));
            par[i].kd = mean + std_dev * (((double(rand()) / RAND_MAX) - 0.5));

            par[i].vKp = 0;
            par[i].vKi = 0;
            par[i].vKd = 0;
        }
    }

    void robotCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    double diff_cost(double kp, double ki, double kd, double angle_error, double angleDiff_error, double angleSum_error)
    {
        w = kp * angle_error + ki * angleSum_error + kd * angleDiff_error;

        v = v + W * dt;

        bot_x = bot_x + v * cos(yaw) * dt;
        bot_y = bot_y + v * sin(yaw) * dt;

        W = w;

        cost = std::fabs(angle_error);

        return cost;
    }

    void moveCb()
    {
        distance = sqrt(std::pow(x_goal - x, 2) + std::pow(y_goal - y, 2));
        errorSum += distance;
        errorDiff = distance - prevDistance;

        proportional = Kp * distance;
        integral = Ki * errorSum;
        derivative = Kd * errorDiff;

        constants = Kp + Ki + Kd;
        pid = proportional + integral + derivative;

        if (pid > 0.3)
        {
            pid = 0.3;
        }
        else if (pid < -0.3)
        {
            pid = -0.3;
        }

        move.linear.x = pid;

        c1 = (maxit - j) / maxit;

        angle = atan2(y_goal - y, x_goal - x) - yaw;

        angleSum += angle;
        errorDiff = angle - prevAngle;

        for (int i = 1; i < popsize; i++)
        {
            double coaster = diff_cost(par[i].kp, par[i].ki, par[i].kd, angle, angleDiff, angleSum);

            coast[i] = coaster;
        }

        if (j > 1)
        {
            localpar = parp;
        }
        else
        {
            localpar = par;
        }

        for (int i = 1; i <= popsize; i++)
        {
            if (i == 1)
            {
                icost.reserve(popsize);
                for (int k = 1; k < popsize; k++)
                {
                    icost[k] = diff_cost(localpar[k].kp, localpar[k].ki, localpar[k].kd, angle, angleDiff, angleSum);
                }
                std::vector<double>::iterator it = std::min_element(icost.begin(), icost.end());
                int index = std::distance(icost.begin(), it);
                globalpar = localpar[index];
                gcost = icost[index];
            }
            else
            {
                double coaster = diff_cost(localpar[i].kp, localpar[i].ki, localpar[i].kd, angle, angleDiff, angleSum);
                if (coaster < gcost)
                    globalpar = localpar[i];
            }
        }

        v = c * (c1 * )

        proportionalAngle = KpAngle * angle;
        integralAngle = KiAngle * angleSum;
        derivativeAngle = KdAngle * errorDiff;

        constantsAngle = KpAngle + KiAngle + KdAngle;
        pidAngle = proportionalAngle + integralAngle + derivativeAngle;

        move.angular.z = pidAngle;

        prevAngle = angle;

        publish_->publish(move);

        prevDistance = distance;

        if (std::fabs(distance) < distanceTolerance)
        {
            move.linear.x = 0.0;
            move.angular.z = 0.0;

            publish_->publish(move);
        }

        if (std::fabs(angle) < angleTolerance && std::fabs(distance) < distanceTolerance)
        {
            move.linear.x = 0.0;
            move.angular.z = 0.0;

            publish_->publish(move);
        }

        parp = par;
        j++;
    }

    rclcpp::TimerBase::SharedPtr move_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publish_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscribe_robot;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_x;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscribe_y;

    std::string tf_prefix;
    std::double_t x_pos;
    std::double_t y_pos;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<movePublisher>());
    rclcpp::shutdown();
    return 0;
}