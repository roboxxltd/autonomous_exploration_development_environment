#include "../include/robot_base/serial/serial.hpp"
#include <thread>
#include <functional>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
using namespace std::chrono_literals;

class RobotBaseNode
{
private:
  void rosSend();
  void serialWrite();
  void serialRead();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
private:
  std::unique_ptr<RoboSerial> serial_;
  RoboCmd     robo_cmd_;
  RoboInf     robo_inf_;
  ros::Subscriber cmd_vel_sub;
  double last_angle = 0.f;
  double angle      = 0.f;
  double k_angle    = 0.f;
  bool        mode_;

public:
  void spin();
  RobotBaseNode();
  void cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& p);
  ~RobotBaseNode();
};