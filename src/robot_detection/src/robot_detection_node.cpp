#include "robot_detection_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  RobotDetectionNode detection("/home/xx/ws/src/robot_detection/config/mv_camera_config_359.xml");
  return 0;
}

void RobotDetectionNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    src_img_ = cv_bridge::toCvShare(msg, "bgr8")->image;
    net_armor_->process_frame(src_img_, armor_);
     if (net_armor_->screen_armor(armor_, src_img_)) {
        std::vector<cv::Point2d> pu(armor_.rst[0].pts, armor_.rst[0].pts + 4);
        cv::solvePnP(small_object_3d_, pu, cameraMatrix_, distCoeffs_, rvec_, tvec_);
        const double *_xyz  = reinterpret_cast<const double *>(tvec_.data);
        img_mutex.lock();
        xyz_.x = _xyz[2];
        xyz_.y = _xyz[0];
        xyz_.z = _xyz[1];
        float x = static_cast<float>(atan2(_xyz[0], _xyz[2]));
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "body";
        point_laser.header.stamp = ros::Time();
        point_laser.point.x = _xyz[2];
        point_laser.point.y = -_xyz[0];
        point_laser.point.z = _xyz[1];
        geometry_msgs::PointStamped point_base;
        try
        {
            point_base = buffer_.transform(point_laser,"body");
            ROS_INFO("abcd world abcd:(%.2f,%.2f,%.2f)",point_base.point.x,point_base.point.y,point_base.point.z);
            //  5-1.创建 TF 广播器
            static tf2_ros::TransformBroadcaster broadcaster;
            //  5-2.创建 广播的数据(通过 pose 设置)
            geometry_msgs::TransformStamped tfs;
            //  |----头设置
            tfs.header.frame_id = "body";
            tfs.header.stamp = ros::Time::now();

            //  |----坐标系 ID
            tfs.child_frame_id = "armor";

            //  |----坐标系相对信息设置
            tfs.transform.translation.x = point_base.point.x;
            tfs.transform.translation.y = point_base.point.y;
            tfs.transform.translation.z = point_base.point.z; 
            //  |--------- 四元数设置
            tf2::Quaternion qtn;
            qtn.setRPY(0,0,0);
            tfs.transform.rotation.x = qtn.getX();
            tfs.transform.rotation.y = qtn.getY();
            tfs.transform.rotation.z = qtn.getZ();
            tfs.transform.rotation.w = qtn.getW();
            //  5-3.广播器发布数据
            broadcaster.sendTransform(tfs);
            geometry_msgs::PointStamped myPoint;
            // 点的坐标 
            // point_base.point.x = point_base.point.x/2;
            if (point_base.point.x > 1) point_base.point.x -=1;
            myPoint.point.x = point_base.point.x;
            myPoint.point.y = point_base.point.y;
            myPoint.point.z = 0.0;
            // frame id
            myPoint.header.frame_id = "body";
            myPoint.header.stamp = ros::Time::now();
            pose_pub_.publish(myPoint);
        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常:%s",e.what());
        }
        img_mutex.unlock();
      // std::cout << "tvec_ = " << xyz_.x <<", "<< xyz_.y <<", " << xyz_.z << std::endl;

        float dis = sqrt(_xyz[0] * _xyz[0] + _xyz[1] * _xyz[1] + _xyz[2] * _xyz[2]);
        float pathDir = x;
       std::cout << "dis = " << dis << std::endl;
       std::cout << "pathDir = " << pathDir << std::endl;
        float dirDiff = pathDir;
        if (dirDiff > CV_PI) dirDiff -= 2 * CV_PI;
        else if (dirDiff < -CV_PI) dirDiff += 2 * CV_PI;
        if (dirDiff > CV_PI) dirDiff -= 2 * CV_PI;
        else if (dirDiff < -CV_PI) dirDiff += 2 * CV_PI;

        if (fabs(vehicleSpeed) < 2.0 * 2.5 / 100.0) vehicleYawRate = -7.5 * dirDiff;
        else vehicleYawRate = -7.5 * dirDiff;

        if (vehicleYawRate > 90 * CV_PI / 180.0) vehicleYawRate = 90 * CV_PI / 180.0;
        else if (vehicleYawRate < -90 * CV_PI / 180.0) vehicleYawRate = -90 * CV_PI / 180.0;

        if (fabs(dirDiff) < 0.1 && dis > 0.6) {
          if (vehicleSpeed < 0.7) vehicleSpeed += 1.5 / 100.0;
          else if (vehicleSpeed > 0.7) vehicleSpeed -= 1.5 / 100.0;
        } else if (fabs(dirDiff) < 0.1 && dis < 0.3) {
          if (vehicleSpeed < 0.2) vehicleSpeed -= 1.5 / 100.0;
          else if (vehicleSpeed > 0.2) vehicleSpeed += 1.5 / 100.0;
        } else {
          vehicleSpeed  = 0;
        }
        cmd_vel.header.stamp = ros::Time();
        if (fabs(vehicleSpeed) <= 2.5 / 100.0) cmd_vel.twist.linear.x = 0;
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        pubSpeed.publish(cmd_vel);
     } else {
        cmd_vel.header.stamp = ros::Time();
       
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        pubSpeed.publish(cmd_vel);
     }


    cv_bridge::CvImage msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", src_img_);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "camera_link";
    image_pub_.publish(msg.toImageMsg());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  armor_.rst.clear();
  memset(armor_.quantity, 0, sizeof(armor_.quantity));
}

RobotDetectionNode::RobotDetectionNode(std::string _camera_path){
  net_armor_ = std::make_unique<basic_net::Detector>();
  net_armor_->detection_init("/home/xx/ws/src/robot_detection/config/opt4_FP16.xml", "CPU");
  armor_.rst.reserve(128);
  std::cout<<"armor init" << std::endl;
  cameraMatrix_ = (cv::Mat_<double>(3, 3) << 386.3064880371094, 0.0, 326.46343994140625, 0.0, 385.44415283203125, 242.4378204345703, 0.0, 0.0, 1.0);
  distCoeffs_ = (cv::Mat_<double>(5, 1) << -0.056345000863075256, 0.06687835603952408, -0.0010148290311917663, 0.0006438795826397836, -0.02150525338947773);
  small_object_3d_ = {// 单位：m
                    {-0.066, 0.029,  0.},
                    {-0.066, -0.029, 0.},
                    {0.066,  -0.029, 0.},
                    {0.066,  0.029,  0.}};
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_pub_= it.advertise("result_image", 1);
  ros::Subscriber sub = n.subscribe("/camera/color/image_raw", 1, &RobotDetectionNode::imageCallback, this);
  pose_pub_ = n.advertise<geometry_msgs::PointStamped> ("/way_point", 1);
  pubSpeed = n.advertise<geometry_msgs::TwistStamped> ("/cmd_vel", 5);
  cmd_vel.header.frame_id = "vehicle";
  tf2_ros::TransformListener listener(buffer_);
  ros::spin();
}

RobotDetectionNode::~RobotDetectionNode(){}
