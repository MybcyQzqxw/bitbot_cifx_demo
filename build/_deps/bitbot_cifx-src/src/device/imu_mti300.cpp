#include "bitbot_cifx/device/imu_mti300.h"

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

  ImuMti300::ImuMti300(const pugi::xml_node& imu_node)
    : CifxImu(imu_node)
  {
    basic_type_ = static_cast<uint32_t>(BasicDeviceType::IMU);
    type_ = static_cast<uint32_t>(CifxDeviceType::IMU_XSENSE_MTI300);
    input_data_length_ = 36;
    output_data_length_ = 2;
  }
  
  void ImuMti300::Input(void* bus_data)
  {
    std::memcpy(bus_receive_data_.data(), bus_data, 36);

    std::memcpy(&imu_data_.runtime.roll, &bus_receive_data_[mti300_receive_addr.roll], sizeof(float));
    std::memcpy(&imu_data_.runtime.pitch, &bus_receive_data_[mti300_receive_addr.pitch], sizeof(float));
    std::memcpy(&imu_data_.runtime.yaw, &bus_receive_data_[mti300_receive_addr.yaw], sizeof(float));
    std::memcpy(&imu_data_.runtime.a_x, &bus_receive_data_[mti300_receive_addr.a_x], sizeof(float));
    std::memcpy(&imu_data_.runtime.a_y, &bus_receive_data_[mti300_receive_addr.a_y], sizeof(float));
    std::memcpy(&imu_data_.runtime.a_z, &bus_receive_data_[mti300_receive_addr.a_z], sizeof(float));
    std::memcpy(&imu_data_.runtime.w_x, &bus_receive_data_[mti300_receive_addr.w_x], sizeof(float));
    std::memcpy(&imu_data_.runtime.w_y, &bus_receive_data_[mti300_receive_addr.w_y], sizeof(float));
    std::memcpy(&imu_data_.runtime.w_z, &bus_receive_data_[mti300_receive_addr.w_z], sizeof(float));
  }
  
  void* ImuMti300::Output()
  {
    return &bus_send_data_;
  }

  void ImuMti300::UpdateRuntimeData()
  {
    monitor_data_[0] = imu_data_.runtime.roll;
    monitor_data_[1] = imu_data_.runtime.pitch;
    monitor_data_[2] = imu_data_.runtime.yaw;
    monitor_data_[3] = imu_data_.runtime.a_x;
    monitor_data_[4] = imu_data_.runtime.a_y;
    monitor_data_[5] = imu_data_.runtime.a_z;
    monitor_data_[6] = imu_data_.runtime.w_x;
    monitor_data_[7] = imu_data_.runtime.w_y;
    monitor_data_[8] = imu_data_.runtime.w_z;
  }

}