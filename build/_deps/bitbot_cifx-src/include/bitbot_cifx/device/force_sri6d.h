#pragma once

#include "bitbot_cifx/device/cifx_device.hpp"

#include <array>

namespace bitbot
{
  struct ForceSensorRuntimeData
  {
    float f_x = 0;
    float f_y = 0;
    float f_z = 0;
    float m_x = 0;
    float m_y = 0;
    float m_z = 0;
  };

  struct ForceSensorData
  {
    ForceSensorRuntimeData runtime;
  };

  // 南宁宇立，六维力传感器 M37XX
  // 该传感器接收数据前两个字节为空
  class ForceSri6d final: public CifxDevice
  {
  public:
    ForceSri6d(const pugi::xml_node& force_sensor_node);
    ~ForceSri6d() = default;

    void InitSDO(){}

    virtual void Input(void* bus_data) final override;
    
    virtual void* Output() final override;
    
    float GetForceX()
    {
      return force_data_.runtime.f_x;
    }
    float GetForceY()
    {
      return force_data_.runtime.f_y;
    }
    float GetForceZ()
    {
      return force_data_.runtime.f_z;
    }
    float GetTorqueX()
    {
      return force_data_.runtime.m_x;
    }
    float GetTorqueY()
    {
      return force_data_.runtime.m_y;
    }
    float GetTorqueZ()
    {
      return force_data_.runtime.m_z;
    }

  private:
    virtual void UpdateRuntimeData() final override;

    bool init_ = false;
    int init_count_ = 0;
    double bias_fx_ = 0, bias_fy_ = 0, bias_fz_ = 0, bias_mx_ = 0, bias_my_ = 0, bias_mz_ = 0;

    ForceSensorData force_data_;
    std::array<int,3> force_x_axis_ = {1,0,0};
    std::array<int,3> force_y_axis_ = {0,1,0};
    std::array<int,3> force_z_axis_ = {0,0,1};
    std::array<int,3> torque_x_axis_ = {1,0,0};
    std::array<int,3> torque_y_axis_ = {0,1,0};
    std::array<int,3> torque_z_axis_ = {0,0,1};

    std::array<uint8_t, 6> bus_send_data_{0};
    std::array<uint8_t, 26> bus_receive_data_{0};
  };


}
