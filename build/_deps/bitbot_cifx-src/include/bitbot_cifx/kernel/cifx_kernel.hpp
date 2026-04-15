#pragma once

#include "bitbot_kernel/kernel/kernel.hpp"
#include "bitbot_cifx/bus/cifx_bus.h"
#include "bitbot_cifx/bus/cifx/hardware_sys.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace bitbot
{
  namespace
  {
    std::atomic<bool> interrupt_flag = false;
    
    void InterruptCallback(uint32_t ulNotification, uint32_t ulDataLen, void* pvData, void* user_data)
    {
      interrupt_flag.store(true);
    }
  }

  enum class CifxKernelEvent : EventId
  {
    POWER_ON = 100,
    POWER_ON_FINISH,
  };

  enum class CifxKernelState : StateId
  {
    POWER_ON = 100,
    POWER_ON_FINISH,
  };

  template <typename UserData, CTString... CTSArray>
  class CifxKernel : public KernelTpl<CifxKernel<UserData, CTSArray...>, CifxBus, UserData, CTSArray...>
  {
  public:
    CifxKernel(std::string config_file)
      : KernelTpl<CifxKernel<UserData, CTSArray...>, CifxBus, UserData, CTSArray...>(config_file)
    {
      this->KernelRegisterEvent("power_on", static_cast<EventId>(CifxKernelEvent::POWER_ON), [this](EventValue, UserData&) -> std::optional<StateId>{
        this->logger_->info("joints power on");
        return static_cast<StateId>(CifxKernelState::POWER_ON);
      }, false);

      this->KernelRegisterEvent("power_on_finish", static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH), [this](EventValue, UserData&) -> std::optional<StateId>{
        this->logger_->info("joints power on finish");
        return static_cast<StateId>(CifxKernelState::POWER_ON_FINISH);
      }, false);

      this->InjectEventsToState(static_cast<StateId>(KernelState::IDLE), {static_cast<EventId>(CifxKernelEvent::POWER_ON)});

      this->KernelRegisterState("power on", static_cast<StateId>(CifxKernelState::POWER_ON),
        [this](const bitbot::KernelInterface& kernel, auto& extra_data, UserData& user_data){
          if(this->PowerOn())
          {
            this->EmitEvent(static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH), 0);
          }
        },
        {
          static_cast<EventId>(static_cast<EventId>(CifxKernelEvent::POWER_ON_FINISH))
        }
      );

      this->KernelRegisterState("power on finish", static_cast<StateId>(CifxKernelState::POWER_ON_FINISH),
        [this](const bitbot::KernelInterface& kernel, auto& extra_data, UserData& user_data){
          this->PowerOn();
        },
        {
          static_cast<EventId>(KernelEvent::START)
        }
      );
    }
    

  protected:

    void doStart()
    {
      HardwareEnableInterrupt(&InterruptCallback, nullptr);
      interrupt_mode_ = true;
      this->logger_->info("synchronous mode");
      SleepMS(100);
      this->busmanager_.ReadBus();
      this->busmanager_.WriteBus();
    }

    void doRun()
    {
      InitHardwareSys();
      this->busmanager_.Init();
      this->busmanager_.InitDevices();

      // this->busmanager_.PrintMotorsError();

      while (!this->kernel_config_data_.stop_flag)
      {
        constexpr double ns_to_ms = 1/1e6;
        constexpr double s_to_ms = 1e3;
        static struct timespec this_time, last_time, temp_time, end_time;

        if(interrupt_mode_)
        {
          while (!interrupt_flag.load())
          {
            SleepNS(10);
            //sched_yield();
          }
          interrupt_flag.store(false);
        }
        else
        {
          SleepMS(100);
        }

        this->kernel_runtime_data_.periods_count++;
        clock_gettime(CLOCK_MONOTONIC, &this_time);
        temp_time = DiffTime(last_time, this_time);
        this->kernel_runtime_data_.period = temp_time.tv_sec*s_to_ms+temp_time.tv_nsec*ns_to_ms;
        last_time = this_time;

        this->HandleEvents();
        this->KernelLoopTask();

        clock_gettime(CLOCK_MONOTONIC, &end_time);
        temp_time = DiffTime(this_time, end_time);
        this->kernel_runtime_data_.process_time = temp_time.tv_sec*s_to_ms+temp_time.tv_nsec*ns_to_ms;

        // Save log file
        this->KernelPrivateLoopEndTask();
      }
      if(interrupt_mode_)
      {
        HardwareDisableInterrupt();
      }

      // logger->info("kernel stop");

      this->busmanager_.JointsPowerOff();
      this->busmanager_.PrintMotorsError();
      
      HardwareExit();
    }

    bool PowerOn()
    {
      auto val = this->busmanager_.JointsPowerOn();
      if(val == JointPowerOnState::Finish)
      {
        return true; // 执行成功
      }
      else if(val == JointPowerOnState::Error)
      {
        this->logger_->error("motors power on error");
        this->kernel_config_data_.stop_flag = true;
        return false; // 执行失败
      }
      else
      {
        return false; // 未完成
      }
    }
    
  private:
    bool interrupt_mode_ = false;

  };
}
