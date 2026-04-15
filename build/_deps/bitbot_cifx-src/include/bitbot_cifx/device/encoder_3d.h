#pragma once

#include "bitbot_cifx/device/cifx_device.hpp"

namespace bitbot
{
  class Encoder3d final: public CifxDevice
  {
  public:
    Encoder3d(const pugi::xml_node& imu_node);

    void InitSDO(){}

    void Input(void* bus_data) final override;
    
    void* Output() final override;
    
  private:
    virtual void UpdateRuntimeData() final override;

    std::array<uint8_t, 2> bus_send_data_{0};
    std::array<uint8_t, 12> bus_receive_data_{0};

    double pos1 = 0;
    double pos2 = 0;
    double pos3 = 0;
  };  
}
