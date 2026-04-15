#include "bitbot_cifx/device/encoder_3d.h"

namespace bitbot
{
  namespace 
  {
    constexpr struct Encoder3dReceiveAddr
    {
      int32_t pos1 = 0;
      int32_t pos2 = 4;
      int32_t pos3 = 8;
    } encoder3d_receive_addr;
  } // namespace 

  Encoder3d::Encoder3d(const pugi::xml_node& imu_node)
    : CifxDevice(imu_node)
  {
    basic_type_ = static_cast<uint32_t>(BasicDeviceType::POSITION_SENSOR);
    type_ = static_cast<uint32_t>(CifxDeviceType::ENCODER_3D);
    input_data_length_ = 12;
    output_data_length_ = 2;
    
    monitor_header_.headers = {"pos1", "pos2", "pos3"};
    monitor_data_.resize(monitor_header_.headers.size());
  }
  
  void Encoder3d::Input(void* bus_data)
  {
    std::memcpy(bus_receive_data_.data(), bus_data, 12);

    int32_t pos;
    std::memcpy(&pos, &bus_receive_data_[encoder3d_receive_addr.pos1], sizeof(float));
    pos1 = static_cast<double>(pos & 0x03FFFF) / 262144.0 * 2 * M_PI; // 0x03FFFF = 262144

    std::memcpy(&pos, &bus_receive_data_[encoder3d_receive_addr.pos2], sizeof(float));
    pos2 = static_cast<double>(pos & 0x03FFFF) / 262144.0 * 2 * M_PI; // 0x03FFFF = 262144

    std::memcpy(&pos, &bus_receive_data_[encoder3d_receive_addr.pos3], sizeof(float));
    pos3 = static_cast<double>(pos & 0x03FFFF) / 262144.0 * 2 * M_PI; // 0x03FFFF = 262144
  }
  
  void* Encoder3d::Output()
  {
    return &bus_send_data_;
  }

  void Encoder3d::UpdateRuntimeData()
  {
    constexpr double rad2deg = 180.0/M_PI;
    monitor_data_[0] = pos1 * rad2deg;
    monitor_data_[1] = pos2 * rad2deg;
    monitor_data_[2] = pos3 * rad2deg;
  }

}