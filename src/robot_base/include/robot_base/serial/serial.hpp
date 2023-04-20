#pragma once
#include <fmt/color.h>
#include <fmt/core.h>
#include "serial/serial.h"
#include "utils.hpp"
#include <iostream>

auto idntifier_green = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "serial");
auto idntifier_red   = fmt::format(fg(fmt::color::red)   | fmt::emphasis::bold, "serial");

class RoboSerial : public serial::Serial {
 public:
  RoboSerial(std::string port, unsigned long baud) {
    auto timeout = serial::Timeout::simpleTimeout(serial::Timeout::max());
    this->setPort(port);
    this->setBaudrate(baud);
    this->setTimeout(timeout);
    try {
      this->open();
      fmt::print("[{}] Serial init successed.\n", idntifier_green);
    } catch(const std::exception& e) {
      fmt::print("[{}] Serial init failed, {}.\n", idntifier_red, e.what());
    }
  }

  void WriteInfo(RoboCmd &robo_cmd) {
    RoboCmdUartBuff robo_cmd_uart_temp;
    robo_cmd_uart_temp.x_v = robo_cmd.x_v;
    robo_cmd_uart_temp.y_v = robo_cmd.y_v;
    robo_cmd_uart_temp.z_r = robo_cmd.z_r;
    this->write((uint8_t*)&robo_cmd_uart_temp, sizeof(robo_cmd_uart_temp));
  }

  void ReceiveInfo(RoboInf &robo_inf) {
    RoboInfUartBuff robo_inf_uart_temp;
    uint8_t temp;
    this->read(&temp, 1);
    while (temp != 'S')
      this->read(&temp, 1);
    this->read((uint8_t*)&robo_inf_uart_temp, sizeof(robo_inf_uart_temp));
    // std::cout << "robo_inf_uart_temp" << robo_inf_uart_temp.end<< "\n";
    if (robo_inf_uart_temp.end == 'E') {
      robo_inf.x   = robo_inf_uart_temp.x;
      robo_inf.y   = -robo_inf_uart_temp.y;
      robo_inf.z   = robo_inf_uart_temp.z;
      robo_inf.yaw = robo_inf_uart_temp.yaw;
      robo_inf.pitch = robo_inf_uart_temp.pitch;
      robo_inf.roll = robo_inf_uart_temp.roll;
      robo_inf.x_v = robo_inf_uart_temp.x_v;
      robo_inf.y_v = -robo_inf_uart_temp.y_v;
      robo_inf.z_v = robo_inf_uart_temp.z_v;
      robo_inf.x_r = robo_inf_uart_temp.x_r;
      robo_inf.y_r = robo_inf_uart_temp.y_r;
      robo_inf.z_r = robo_inf_uart_temp.z_r;
      // std::cout << "robo_inf.x = " << robo_inf.x << "\n";
      // std::cout << "robo_inf.y = " << robo_inf.y << "\n";
      // std::cout << "robo_inf.z = " << robo_inf.z << "\n";
      // std::cout << "robo_inf.yaw = " << robo_inf.yaw << "\n";
      // std::cout << "robo_inf.pitch = " << robo_inf.pitch << "\n";
      // std::cout << "robo_inf.roll = " << robo_inf.roll << "\n";
      // std::cout << "robo_inf.x_v = " << robo_inf.x_v << "\n";
      // std::cout << "robo_inf.y_v = " << robo_inf.y_v << "\n";
      // std::cout << "robo_inf.z_v = " << robo_inf.z_v << "\n";
    }
  }

 private:
  static constexpr unsigned char CRC8_Table[] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31, 65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220, 35,  125, 159, 193, 66,
    28,  254, 160, 225, 191, 93,  3,   128, 222, 60,  98,  190, 224, 2,   92, 223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218,
    56,  102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25, 71,  165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122,
    36,  248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,  231, 185, 140, 210, 48,  110, 237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243,
    112, 46,  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19, 77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208, 83,  13,  239, 177, 240,
    174, 76,  18,  145, 207, 45,  115, 202, 148, 118, 40,  171, 245, 23,  73, 8,   86,  180, 234, 105, 55,  213, 139, 87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170,
    72,  22,  233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74, 20,  246, 168, 116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53};

  inline uint8_t checksumCRC(unsigned char* buf, uint16_t len) {
    uint8_t check = 0;
    while (len--) {
      check = CRC8_Table[check ^ (*buf++)];
    }
    return check;
  }
};