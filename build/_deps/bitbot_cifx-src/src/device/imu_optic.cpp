#include "bitbot_cifx/device/imu_optic.h"
#include "bitbot_kernel/kernel/config_parser.h"

namespace bitbot
{
  ImuOptic::ImuOptic(const pugi::xml_node& imu_node)
    : CifxImu(imu_node)
  {
    type_ = static_cast<uint32_t>(CifxDeviceType::IMU_OPTIC);
    input_data_length_ = 8;
    output_data_length_ = 1;
  }
  
  void ImuOptic::Input(void* bus_data)
  {
    uint8_t* data = static_cast<uint8_t*>(bus_data);

    // for(int i = 0; i < input_data_length_; i++)
    // {
    //   printf(" %x ", data[i]);
    // }
    // printf("\n");

    long temp_Pitch = 0, temp_Roll = 0, temp_Ax = 0, temp_Ay = 0, temp_Az = 0;
    temp_Roll =((data[0] & 0x7F )<< 7) | ((data[1] >> 1) & 0x7f);
    temp_Pitch = (data[2]  << 3) | ((data[3] >> 5) & 0x07);

    temp_Ax = ((data[3] & 0x0F) << 7) | ((data[4] >> 1) & 0x7F);
    temp_Ay = ((data[5] << 3) | ((data[6] >> 5) & 0x07));
    temp_Az = ((data[6] & 0x0F) << 7) | ((data[7] >> 1) & 0x7F);

    if (data[0] & 0x80)
      imu_data_.runtime.roll = (0 - (temp_Roll & 0x3FFF))*0.01;
    else
      imu_data_.runtime.roll = (temp_Roll & 0x3FFF)*0.01;

    if (data[1] & 0x01)
      imu_data_.runtime.pitch = (0 - (temp_Pitch & 0x7FF))*0.1;
    else
      imu_data_.runtime.pitch = (temp_Pitch & 0x7FF)*0.1;

    if (data[3] & 0x10)
      imu_data_.runtime.a_x = (0 - (temp_Ax & 0x7FF))*0.01;
    else
      imu_data_.runtime.a_x = ((temp_Ax & 0x7FF))*0.01;

    if (data[4] & 0x01)
      imu_data_.runtime.a_y = (temp_Ay & 0x7FF)*0.01;
    else
      imu_data_.runtime.a_y = (0 - (temp_Ay & 0x3FF))*0.01;

    if (data[6] & 0x10)
      imu_data_.runtime.a_z = (temp_Az & 0x7FF)*0.01;
    else
      imu_data_.runtime.a_z = (0 - (temp_Az & 0x7FF))*0.01;
  }
  
  void* ImuOptic::Output()
  {
    return &bus_send_data_;
  }

  void ImuOptic::UpdateRuntimeData()
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