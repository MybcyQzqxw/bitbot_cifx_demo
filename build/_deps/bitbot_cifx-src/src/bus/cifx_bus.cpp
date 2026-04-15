#include "bitbot_cifx/bus/cifx_bus.h"
#include "bitbot_cifx/bus/cifx/cifx_driver.h"
#include "bitbot_cifx/bus/cifx/protocol_SDO.h"
#include "bitbot_cifx/device/cifx_device.hpp"
#include "bitbot_cifx/device/joint_elmo_old.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"
#include "bitbot_cifx/device/imu_mti300.h"
#include "bitbot_cifx/device/imu_optic.h"
#include "bitbot_cifx/device/ahrs_xsens.h"
#include "bitbot_cifx/device/force_sri6d.h"
#include "bitbot_cifx/device/encoder_3d.h"
#include "bitbot_cifx/device/kuafu_foot.h"
#include "bitbot_kernel/utils/time_func.h"

namespace bitbot
{

  CifxBus::CifxBus(/* args */) {}

  CifxBus::~CifxBus() {}

  void CifxBus::doRegisterDevices()
  {
    static DeviceRegistrar<CifxDevice, JointElmoOld> elmo_old(static_cast<uint32_t>(CifxDeviceType::JOINT_ELMO_OLD), "JointElmoOld");
    static DeviceRegistrar<CifxDevice, JointElmo> elmo(static_cast<uint32_t>(CifxDeviceType::JOINT_ELMO), "JointElmo");
    static DeviceRegistrar<CifxDevice, JointElmoPushrod> elmo_pushrod(static_cast<uint32_t>(CifxDeviceType::JOINT_ELMO_PUSHROD), "JointElmoPushrod");
    static DeviceRegistrar<CifxDevice, ImuMti300> imu_mti300(static_cast<uint32_t>(CifxDeviceType::IMU_XSENSE_MTI300), "ImuMti300");
    static DeviceRegistrar<CifxDevice, ImuOptic> imu_optic(static_cast<uint32_t>(CifxDeviceType::IMU_OPTIC), "ImuOptic");
    static DeviceRegistrar<CifxDevice, AhrsXsens> ahrs_xsens(static_cast<uint32_t>(CifxDeviceType::AHRS_XSENS), "AhrsXsens");
    static DeviceRegistrar<CifxDevice, ForceSri6d> force_sri6d(static_cast<uint32_t>(CifxDeviceType::FORCE_SRI6D), "ForceSri6d");
    static DeviceRegistrar<CifxDevice, Encoder3d> encoder_3d(static_cast<uint32_t>(CifxDeviceType::ENCODER_3D), "Encoder3d");
    static DeviceRegistrar<CifxDevice, KuafuFoot> kuafu_foot(static_cast<uint32_t>(CifxDeviceType::KUAFU_FOOT), "KuafuFoot");
  }

  void CifxBus::doConfigure(const pugi::xml_node &bus_node)
  {
    this->CreateDevices(bus_node);
  }

  void CifxBus::ReadBus()
  {
    cifx_return_val = xChannelIORead(cifx_channel, 0, 0, receive_data_length_, receive_data_.data(), 0);
    if (cifx_return_val != 0)
    {
      logger_->error("Error reading IO Data area!");
      ShowError(cifx_return_val);
    }
    else
    {
      uint32_t index = 0;
      for (unsigned int i = 0; i < this->devices_num_; i++)
      {
        ordered_devices_[i]->Input(&receive_data_[index]);
        index += ordered_devices_[i]->InputDataLength();
        // printf("receive: ");
        // for(auto i:receive_data_)
        // {
        //   printf("%02x ", i);
        // }
        // printf("\n");
      }
    }
  }

  void CifxBus::WriteBus()
  {
    uint32_t index = 0;
    for (unsigned int i = 0; i < devices_num_; i++)
    {
      memcpy(&send_data_[index], ordered_devices_[i]->Output(), ordered_devices_[i]->OutputDataLength());
      index += ordered_devices_[i]->OutputDataLength();
    }
    // printf("send: ");
    // for(auto i:send_data_)
    // {
    //   printf("%02x ", i);
    // }
    // printf("\n");
    if (CIFX_NO_ERROR != (cifx_return_val = xChannelIOWrite(cifx_channel, 0, 0, send_data_length_, send_data_.data(), 1)))
    {
      logger_->error("Error writing to IO Data area!");
      ShowError(cifx_return_val);
    }
  }

  void CifxBus::InitDevices()
  {
    for (auto &device : devices_)
    {
      device->InitSDO();
    }
    this->InitJoints();
  }

  void CifxBus::InitJoints()
  {
    for (uint32_t i = 0; i < joints_num_; i++)
    {
      this->ReadBus();
      SleepMS(10);
      this->ReadBus();
      // 错误复位
      joints_[i]->ResetFault();
      this->WriteBus();
      SleepMS(50);
      joints_[i]->ResetFault();
      this->WriteBus();
      SleepMS(50);
    }
    this->ReadBus();
  }

  JointPowerOnState CifxBus::JointsPowerOn()
  {
    static uint32_t joints_poweron_num = 0;
    bool error = false;

    if (joints_poweron_num < joints_num_)
    {
      auto &joint = joints_[joints_poweron_num];

      if (joint->isEnable())
      {
        auto state = joint->PowerOn();
        if (state == JointPowerOnState::Error)
        {
          logger_->error("joint {} power on fail", joint->Id());
          return JointPowerOnState::Error; // 执行出错
        }
        else if (state == JointPowerOnState::Finish)
        {
          joints_poweron_num++;
          logger_->info("joint {} power on finish", joint->Id());
        }
        return JointPowerOnState::Running; // running
      }
      else
      {
        joints_poweron_num++;
        logger_->debug("joint {} is not enabled", joint->Id());
        return JointPowerOnState::Running; // running
      }
    }
    else
    {
      return JointPowerOnState::Finish; // finish
    }
  }

  void CifxBus::JointsPowerOff()
  {
    for (auto joint : joints_)
    {
      joint->PowerOff();
      this->WriteBus();
      this->ReadBus();
      SleepMS(10);
      this->ReadBus();
    }
  }

  void CifxBus::PrintMotorsError()
  {
    for (auto &motor : joints_)
    {
      this->logger_->debug("motor(id {}) MF: 0x{:x}", motor->Id(), ReadMF(motor->Id()));
      this->logger_->debug("motor(id {}) EE: 0x{:x}", motor->Id(), ReadEE(motor->Id()));
      this->logger_->debug("motor(id {}) EC: 0x{:x}", motor->Id(), ReadEC(motor->Id()));
    }
    logger_->info("print motor error code finish");
  }

}
