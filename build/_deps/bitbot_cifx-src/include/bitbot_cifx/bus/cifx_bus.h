#pragma once

#include "bitbot_kernel/bus/bus_manager.hpp"
#include "bitbot_cifx/device/cifx_device.hpp"
#include "bitbot_cifx/device/cifx_joint.hpp"

namespace bitbot
{
  class CifxBus : public BusManagerTpl<CifxBus, CifxDevice>
  {
  public:
    CifxBus(/* args */);
    virtual ~CifxBus();

    void ReadBus();
    void WriteBus();

    void doRegisterDevices();

    void Init()
    {
      ordered_devices_ = devices_;
      std::sort(ordered_devices_.begin(), ordered_devices_.end(),
                [](CifxDevice *a, CifxDevice *b)
                {
                  return (a->Id() < b->Id());
                });
      for (auto &device : ordered_devices_)
      {
        if (device->BasicType() == static_cast<uint32_t>(BasicDeviceType::MOTOR))
        {
          joints_.emplace_back(dynamic_cast<CifxJoint *>(device));
        }
      }

      joints_num_ = joints_.size();

      logger_->debug("cifx bus device number: {}", devices_num_);
      logger_->debug("cifx bus joint number: {}", joints_num_);

      InitParameter();
    }

    void InitParameter()
    {
      for (auto &device : ordered_devices_)
      {
        send_data_length_ += device->OutputDataLength();
        receive_data_length_ += device->InputDataLength();
      }
      send_data_.resize(send_data_length_, 0);
      receive_data_.resize(receive_data_length_, 0);

      logger_->debug("cifx bus send data length: {}", send_data_length_);
      logger_->debug("cifx bus receive data length: {}", receive_data_length_);
    }

    void InitDevices();
    JointPowerOnState JointsPowerOn();
    void JointsPowerOff();
    void PrintMotorsError();

  protected:
    void doConfigure(const pugi::xml_node &bus_node);

  private:
    void InitJoints();

    uint32_t send_data_length_ = 0;
    uint32_t receive_data_length_ = 0;
    std::vector<uint8_t> send_data_;
    std::vector<uint8_t> receive_data_;

    uint32_t joints_num_;

    std::vector<CifxDevice *> ordered_devices_;
    std::vector<CifxJoint *> joints_;
  };

}
