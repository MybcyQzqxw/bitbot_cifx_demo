#include "bitbot_cifx/device/kuafu_foot.h"

namespace bitbot
{
  namespace 
  {
    constexpr struct Mti300ReceiveAddr
    {
      uint16_t roll = 0;
      uint16_t pitch = 4; 
      uint16_t yaw = 8;
      uint16_t a_x = 12;
      uint16_t a_y = 16;
      uint16_t a_z = 20;
      uint16_t w_x = 24;
      uint16_t w_y = 28;
      uint16_t w_z = 32;
    } mti300_receive_addr;
  } // namespace 

  KuafuFoot::KuafuFoot(const pugi::xml_node& device_node)
    : CifxDevice(device_node)
  {
    basic_type_ = static_cast<uint32_t>(BasicDeviceType::USER_DEFINE);
    type_ = static_cast<uint32_t>(CifxDeviceType::KUAFU_FOOT);
    input_data_length_ = 10;
    output_data_length_ = 2;

    monitor_header_.headers = {"io1", "io2"};
    monitor_data_.resize(monitor_header_.headers.size());
  }
  
  void KuafuFoot::Input(void* bus_data)
  {
    // std::memcpy(bus_receive_data_.data(), bus_data, 10);

    // std::memcpy(&imu_data_.runtime.roll, &bus_receive_data_[mti300_receive_addr.roll], sizeof(float));
    // std::memcpy(&imu_data_.runtime.pitch, &bus_receive_data_[mti300_receive_addr.pitch], sizeof(float));
    // std::memcpy(&imu_data_.runtime.yaw, &bus_receive_data_[mti300_receive_addr.yaw], sizeof(float));
    // std::memcpy(&imu_data_.runtime.a_x, &bus_receive_data_[mti300_receive_addr.a_x], sizeof(float));
    // std::memcpy(&imu_data_.runtime.a_y, &bus_receive_data_[mti300_receive_addr.a_y], sizeof(float));
    // std::memcpy(&imu_data_.runtime.a_z, &bus_receive_data_[mti300_receive_addr.a_z], sizeof(float));
    // std::memcpy(&imu_data_.runtime.w_x, &bus_receive_data_[mti300_receive_addr.w_x], sizeof(float));
    // std::memcpy(&imu_data_.runtime.w_y, &bus_receive_data_[mti300_receive_addr.w_y], sizeof(float));
    // std::memcpy(&imu_data_.runtime.w_z, &bus_receive_data_[mti300_receive_addr.w_z], sizeof(float));
  }
  
  void* KuafuFoot::Output()
  {
    return &bus_send_data_;
  }

  void KuafuFoot::UpdateRuntimeData()
  {
    // monitor_data_[0] = imu_data_.runtime.roll;
    // monitor_data_[1] = imu_data_.runtime.pitch;
    // monitor_data_[2] = imu_data_.runtime.yaw;
    // monitor_data_[3] = imu_data_.runtime.a_x;
    // monitor_data_[4] = imu_data_.runtime.a_y;
    // monitor_data_[5] = imu_data_.runtime.a_z;
    // monitor_data_[6] = imu_data_.runtime.w_x;
    // monitor_data_[7] = imu_data_.runtime.w_y;
    // monitor_data_[8] = imu_data_.runtime.w_z;
  }

}