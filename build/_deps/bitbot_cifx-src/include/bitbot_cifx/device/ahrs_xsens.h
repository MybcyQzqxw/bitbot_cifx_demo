#pragma once

#include "bitbot_cifx/device/cifx_imu.hpp"

namespace bitbot
{
  class AhrsXsens final : public CifxImu
  {
  public:
    AhrsXsens(const pugi::xml_node &imu_node);

    void InitSDO() {}

    void Input(void *bus_data) final override;

    void *Output() final override;

  private:
    virtual void UpdateRuntimeData() final override;

    // std::array<uint8_t, 2> bus_send_data_{0};
    std::array<uint8_t, 36> bus_receive_data_{0};
  };
}
