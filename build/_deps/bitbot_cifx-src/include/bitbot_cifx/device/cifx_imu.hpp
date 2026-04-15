#pragma once

#include "bitbot_cifx/device/cifx_device.hpp"

#include <fstream>

namespace bitbot
{
  struct ImuRuntimeData
  {
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
    float a_x = 0;
    float a_y = 0;
    float a_z = 0;
    float w_x = 0;
    float w_y = 0;
    float w_z = 0;
  };

  struct ImuData
  {
    ImuRuntimeData runtime;

  };

  class CifxImu : public CifxDevice
  {
  public:
    CifxImu(const pugi::xml_node& imu_node)
      : CifxDevice(imu_node)
    {
      basic_type_ = static_cast<uint32_t>(BasicDeviceType::IMU);

      monitor_header_.headers = {"roll", "pitch", "yaw", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"};
      monitor_data_.resize(monitor_header_.headers.size());
    }
    virtual ~CifxImu() = default;
  
    ImuData GetData()
    {
      return imu_data_;
    }

    float GetRoll()
    {
      return imu_data_.runtime.roll;
    }
    float GetPitch()
    {
      return imu_data_.runtime.pitch;
    }
    float GetYaw()
    {
      return imu_data_.runtime.yaw;
    }
    float GetAccX()
    {
      return imu_data_.runtime.a_x;
    }
    float GetAccY()
    {
      return imu_data_.runtime.a_y;
    }
    float GetAccZ()
    {
      return imu_data_.runtime.a_z;
    }
    float GetGyroX()
    {
      return imu_data_.runtime.w_x;
    }
    float GetGyroY()
    {
      return imu_data_.runtime.w_y;
    }
    float GetGyroZ()
    {
      return imu_data_.runtime.w_z;
    }
    
  protected:
    ImuData imu_data_;

  };
  
  
}
