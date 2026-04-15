#include "bitbot_cifx/device/force_sri6d.h"
#include "bitbot_kernel/kernel/config_parser.h"

namespace bitbot
{
  constexpr struct SRI6DReceiveAddr
  {
    uint16_t f_x = 2;
    uint16_t f_y = 6;
    uint16_t f_z = 10;
    uint16_t m_x = 14;
    uint16_t m_y = 18;
    uint16_t m_z = 22;
  } force_receive_addr;
  
  ForceSri6d::ForceSri6d(const pugi::xml_node& force_sensor_node)
    : CifxDevice(force_sensor_node)
  {
    basic_type_ = static_cast<uint32_t>(BasicDeviceType::FORCE_SENSOR);
    monitor_header_.headers = {"fx", "fy", "fz", "mx", "my", "mz"};
    monitor_data_.resize(monitor_header_.headers.size());
    
    auto set_axis = [&force_sensor_node](std::string attribute, std::array<int,3> &axis){
      std::string axis_string;
      axis_string =  force_sensor_node.attribute(attribute.c_str()).as_string();
      if(!axis_string.empty())
      {
        std::stringstream ss(axis_string);
        ss >> axis[0] >> axis[1] >> axis[2];
      }
    };

    set_axis("force_x_axis", force_x_axis_);
    set_axis("force_y_axis", force_y_axis_);
    set_axis("force_z_axis", force_z_axis_);
    set_axis("torque_x_axis", torque_x_axis_);
    set_axis("torque_y_axis", torque_y_axis_);
    set_axis("torque_z_axis", torque_z_axis_);
    //     axis_string =  torque_sensor_node.attribute("force_y_axis").as_string();
    // if(!axis_string.empty())
    // {
    //   std::stringstream ss(axis_string);
    //   ss >> y_axis_[0] >> y_axis_[1] >> y_axis_[2];
    // }

    // axis_string =  force_sensor_node.attribute("force_z_axis").as_string();
    // if(!axis_string.empty())
    // {
    //   std::stringstream ss(axis_string);
    //   ss >> z_axis_[0] >> z_axis_[1] >> z_axis_[2];
    // }

    input_data_length_ = 26;
    output_data_length_ = 6;
  }
  
  void ForceSri6d::Input(void* bus_data)
  {
    constexpr int init_count_max = 200;

    memcpy(bus_receive_data_.data(), bus_data, 26);

    memcpy(&force_data_.runtime.f_x, bus_receive_data_.data()+force_receive_addr.f_x, sizeof(float));
    memcpy(&force_data_.runtime.f_y, bus_receive_data_.data()+force_receive_addr.f_y, sizeof(float));
    memcpy(&force_data_.runtime.f_z, bus_receive_data_.data()+force_receive_addr.f_z, sizeof(float));
    memcpy(&force_data_.runtime.m_x, bus_receive_data_.data()+force_receive_addr.m_x, sizeof(float));
    memcpy(&force_data_.runtime.m_y, bus_receive_data_.data()+force_receive_addr.m_y, sizeof(float));
    memcpy(&force_data_.runtime.m_z, bus_receive_data_.data()+force_receive_addr.m_z, sizeof(float));

    force_data_.runtime.f_x = force_x_axis_[0]*force_data_.runtime.f_x + force_x_axis_[1]*force_data_.runtime.f_y + force_x_axis_[2]*force_data_.runtime.f_z;
    force_data_.runtime.f_y = force_y_axis_[0]*force_data_.runtime.f_x + force_y_axis_[1]*force_data_.runtime.f_y + force_y_axis_[2]*force_data_.runtime.f_z;
    force_data_.runtime.f_z = force_z_axis_[0]*force_data_.runtime.f_x + force_z_axis_[1]*force_data_.runtime.f_y + force_z_axis_[2]*force_data_.runtime.f_z;
    force_data_.runtime.m_x = torque_x_axis_[0]*force_data_.runtime.m_x + torque_x_axis_[1]*force_data_.runtime.m_y + torque_x_axis_[2]*force_data_.runtime.m_z;
    force_data_.runtime.m_y = torque_y_axis_[0]*force_data_.runtime.m_x + torque_y_axis_[1]*force_data_.runtime.m_y + torque_y_axis_[2]*force_data_.runtime.m_z;
    force_data_.runtime.m_z = torque_z_axis_[0]*force_data_.runtime.m_x + torque_z_axis_[1]*force_data_.runtime.m_y + torque_z_axis_[2]*force_data_.runtime.m_z;

    if(!init_)
    {
      if(init_count_ < init_count_max)
      {
        bias_fx_ = bias_fx_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.f_x / (double)(init_count_+1);
        bias_fy_ = bias_fy_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.f_y / (double)(init_count_+1);
        bias_fz_ = bias_fz_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.f_z / (double)(init_count_+1);
        bias_mx_ = bias_mx_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.m_x / (double)(init_count_+1);
        bias_my_ = bias_my_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.m_y / (double)(init_count_+1);
        bias_mz_ = bias_mz_*((double)init_count_/(double)(init_count_+1)) + force_data_.runtime.m_z / (double)(init_count_+1);
      }
      else
      {
        init_ = true;
      }
      init_count_++;
    }

    force_data_.runtime.f_x -= bias_fx_;
    force_data_.runtime.f_y -= bias_fy_;
    force_data_.runtime.f_z -= bias_fz_;
    force_data_.runtime.m_x -= bias_mx_;
    force_data_.runtime.m_y -= bias_my_;
    force_data_.runtime.m_z -= bias_mz_;
  }
  
  void* ForceSri6d::Output()
  {
    return &bus_send_data_;
  }

  void ForceSri6d::UpdateRuntimeData()
  {
    monitor_data_[0] = force_data_.runtime.f_x;
    monitor_data_[1] = force_data_.runtime.f_y;
    monitor_data_[2] = force_data_.runtime.f_z;
    monitor_data_[3] = force_data_.runtime.m_x;
    monitor_data_[4] = force_data_.runtime.m_y;
    monitor_data_[5] = force_data_.runtime.m_z;
  }
}