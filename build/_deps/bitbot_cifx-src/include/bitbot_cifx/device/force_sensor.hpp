#ifndef _FORCE_SENSOR_HPP
#define _FORCE_SENSOR_HPP

#include "bitbot_cifx/device/cifx_device.hpp"
// #include "bitbot_kernel/device/device.hpp"
#include "bitbot_kernel/kernel/config_parser.h"

#include <pugixml.hpp>

#include <fstream>

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

  class ForceSensor : public CifxDevice
  {
  public:
    ForceSensor() {}

    ForceSensor(const pugi::xml_node& force_sensor_node)
    {
      monitor_header_.headers = {"fx", "fy", "fz", "mx", "my", "mz"};
      monitor_data_.resize(monitor_header_.headers.size());
    }

    virtual ~ForceSensor() = default;
  
    ForceSensorData GetData()
    {
      return force_data_;
    }

    float GetFX()
    {
      return force_data_.runtime.f_x;
    }
    float GetFY()
    {
      return force_data_.runtime.f_y;
    }
    float GetFZ()
    {
      return force_data_.runtime.f_z;
    }
    float GetMX()
    {
      return force_data_.runtime.m_x;
    }
    float GetMY()
    {
      return force_data_.runtime.m_y;
    }
    float GetMZ()
    {
      return force_data_.runtime.m_z;
    }

    void PrintInfo() {}

  protected:

    ForceSensorData force_data_;

  };

}
#endif // _FORCE_SENSOR_HPP