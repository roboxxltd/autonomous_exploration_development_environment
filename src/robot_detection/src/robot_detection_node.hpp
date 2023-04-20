#include "../include/robot_detection/net/basic_net.h"
#include <atomic>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
class RobotDetectionNode
{
private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
private:
    std::unique_ptr<basic_net::Detector> net_armor_;
    image_transport::Publisher image_pub_;
    geometry_msgs::TwistStamped cmd_vel;
    ros::Publisher pose_pub_;
    ros::Publisher pubSpeed;
    basic_net::armor_detection armor_;
    cv::Mat src_img_;
    std::mutex img_mutex;
    cv::Point3f xyz_;
    tf2_ros::Buffer buffer_;
    float vehicleSpeed = 0.f;
    float vehicleYawRate = 0.f;
    cv::Mat rvec_ = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat tvec_ = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::Mat cameraMatrix_, distCoeffs_;
    std::vector<cv::Point3f> small_object_3d_;
public:
    RobotDetectionNode(std::string _camera_path);
  ~RobotDetectionNode();
};