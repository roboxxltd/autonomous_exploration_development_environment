#include "robot_base_node.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_publisher");
  auto node = std::make_unique<RobotBaseNode>();
  node->spin();
  return 0;
}
RobotBaseNode::RobotBaseNode(){
  serial_ = std::make_unique<RoboSerial>("/dev/ttyUSB0", 115200);
}

RobotBaseNode::~RobotBaseNode(){}

void RobotBaseNode::spin() {
  std::thread uartWriteThread(std::bind(&RobotBaseNode::serialWrite,this));
  std::thread uartReadThread(std::bind(&RobotBaseNode::serialRead,this));
  std::thread dataSendThread(std::bind(&RobotBaseNode::rosSend,this));
  while (true) {
    if (uartWriteThread.joinable())
      uartWriteThread.detach();
    if (uartReadThread.joinable())
      uartReadThread.detach();
    if (dataSendThread.joinable())
      dataSendThread.detach();
  }
}

void RobotBaseNode::cmdCallback(const geometry_msgs::TwistStamped::ConstPtr& p) {
    geometry_msgs::Twist twist = p->twist;
    robo_cmd_.x_v = twist.linear.x;
    robo_cmd_.y_v = twist.linear.y;
    robo_cmd_.z_r = twist.angular.z;
    std::cout << "robo_cmd_.x_v = " << robo_cmd_.x_v << std::endl;
    std::cout << "robo_cmd_.y_v = " << robo_cmd_.y_v << std::endl;
    std::cout << "robo_cmd_.z_r = " << robo_cmd_.z_r << std::endl;
}

void RobotBaseNode::rosSend() {
  ros::NodeHandle n, nh;
  ros::Subscriber cmd_vel_sub = n.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 1, &RobotBaseNode::cmdCallback, this);
  ros::spin();
}

/**
 * @brief 串口发送
 * 
 */
void RobotBaseNode::serialWrite() {
  while (true) try {
      if (serial_->isOpen()) {
        serial_->WriteInfo(robo_cmd_);
        // fmt::print("[{}] WriteInfo success.\n", idntifier_green);
      } else {
        serial_->open();
      }
      std::this_thread::sleep_for(10ms);
    } catch (const std::exception& e) {
      serial_->close();
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial_ excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial_ exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}

/**
 * @brief 接受串口
 * 
 */
void RobotBaseNode::serialRead() {
  while (true) try {
      if (serial_->isOpen()) {
        serial_->ReceiveInfo(robo_inf_);
      }
      std::this_thread::sleep_for(1ms);
    } catch (const std::exception& e) {
      static int serial_read_excepted_times{0};
      if (serial_read_excepted_times++ > 3) {
        std::this_thread::sleep_for(10000ms);
        fmt::print("[{}] read serial_ excepted to many times, sleep 10s.\n", idntifier_red);
        serial_read_excepted_times = 0;
      }
      fmt::print("[{}] serial_ exception: {}\n", idntifier_red, e.what());
      std::this_thread::sleep_for(1000ms);
    }
}
