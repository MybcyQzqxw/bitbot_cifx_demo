#pragma once

#include "bitbot_cifx/device/cifx_imu.hpp"

namespace bitbot
{
  class ImuOptic final: public CifxImu
  {
  public:
    ImuOptic(const pugi::xml_node& imu_node);

    void InitSDO(){}

    void Input(void* bus_data) override;
    
    void* Output() override;
    
  private:
    virtual void UpdateRuntimeData() final override;

    std::array<uint8_t, 1> bus_send_data_{0};
    std::array<uint8_t, 8> bus_receive_data_{0};
  };  
}
