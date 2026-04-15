#pragma once

#include "bitbot_kernel/device/device.hpp"

namespace bitbot
{
  enum class CifxDeviceType : uint32_t
  {
    CIFX_DEVICE = 1000,
    JOINT_ELMO_OLD,
    JOINT_ELMO,
    JOINT_ELMO_PUSHROD,
    IMU_XSENSE_MTI300,
    IMU_OPTIC,
    AHRS_XSENS,
    FORCE_SRI6D,
    ENCODER_3D,
    KUAFU_FOOT
  };

  class CifxDevice : public Device
  {
  public:
    CifxDevice(const pugi::xml_node &device_node)
        : Device(device_node)
    {
    }
    virtual ~CifxDevice() = default;

    virtual void InitSDO() = 0;

    virtual void Input(void *bus_data) = 0;

    virtual void *Output() = 0;

    inline uint32_t InputDataLength()
    {
      return input_data_length_;
    }
    inline uint32_t OutputDataLength()
    {
      return output_data_length_;
    }

  protected:
    uint32_t input_data_length_ = 0;
    uint32_t output_data_length_ = 0;
  };

}