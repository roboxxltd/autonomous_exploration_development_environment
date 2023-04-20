#pragma once
#include <atomic>

struct RoboCmd {
  std::atomic<float> x_v;
  std::atomic<float> y_v;
  std::atomic<float> z_r;
};

struct RoboCmdUartBuff{
  uint8_t  start       = (unsigned)'S';
  float    x_v         = 0.f;
  float    y_v         = 0.f;
  float    z_r         = 0.f;
  uint8_t  end         = (unsigned)'E';
} __attribute__((packed));

struct RoboInf {
  std::atomic<float>   x;
  std::atomic<float>   y;
  std::atomic<float>   z;
  std::atomic<float>   yaw;
  std::atomic<float>   pitch;
  std::atomic<float>   roll;
  std::atomic<float>   x_v;
  std::atomic<float>   y_v;
  std::atomic<float>   z_v;
  std::atomic<float>   x_r;
  std::atomic<float>   y_r;
  std::atomic<float>   z_r;
};

struct RoboInfUartBuff {
  float    x           = 0.f;
  float    y           = 0.f;
  float    z           = 0.f;
  float    yaw         = 0.f;
  float    pitch       = 0.f;
  float    roll        = 0.f;
  float    x_v         = 0.f;
  float    y_v         = 0.f;
  float    z_v         = 0.f;
  float    x_r         = 0.f;
  float    y_r         = 0.f;
  float    z_r         = 0.f;
  uint8_t  end;
} __attribute__((packed));