#pragma once

#include "bitbot_cifx/device/cifx_joint.hpp"
#include "bitbot_cifx/device/canopen_motor.hpp"

namespace bitbot
{

  class JointElmoOld final: public CifxJoint
  {
  public:
    JointElmoOld(const pugi::xml_node& motor_node);
    virtual ~JointElmoOld() = default;

    virtual void InitSDO() final override;

    inline virtual void Input(void* bus_data) final override
    {
      static int32_t encoder = 0;
      static int16_t current = 0;
      static const float current_ratio = joint_data_.config.current_ratio / 1000.0 / 1000.0;
      // printf("receive: ");
      // for(uint16_t i = 0; i<16; i++)
      // {
      //   printf("%02x ", *((unsigned char*)(bus_data)+i));
      // }
      // printf("\n");
      std::memcpy(bus_receive_data_.data(), bus_data, 16);
      
      std::memcpy(&joint_data_.runtime.status, bus_receive_data_.data()+elmo_input_addr.status_word, sizeof(uint16_t));
      std::memcpy(&joint_data_.runtime.actual_mode, bus_receive_data_.data()+elmo_input_addr.mode, sizeof(int16_t));

      std::memcpy(&encoder, bus_receive_data_.data()+elmo_input_addr.position, sizeof(int32_t));
      joint_data_.runtime.actual_pos = EncoderCount2Angle(encoder);

      std::memcpy(&encoder, bus_receive_data_.data()+elmo_input_addr.velocity, sizeof(int32_t));
      joint_data_.runtime.actual_velocity = EncoderCounts2Rads(encoder);

      std::memcpy(&current, bus_receive_data_.data()+elmo_input_addr.torque, sizeof(int16_t));
      joint_data_.runtime.actual_torque = current * current_ratio;
      joint_data_.runtime.actual_current = joint_data_.runtime.actual_torque;
    }

    inline virtual void* Output() final override
    {
      return bus_send_data_.data();
    }

    virtual void ResetFault() final override;
    virtual JointPowerOnState PowerOn() final override;
    virtual void PowerOff() final override;

    void SetMode(CANopenMotorMode mode);
    virtual void SetMode(int mode) final override;

    virtual void SetTargetPosition(double position) final;

    void SetTargetVelocity(double velocity);

    /**
     * @brief 设置电机力矩。对于Elmo驱动器，目标力矩与目标电流相同，单位A
     * 
     * @param torque 
     */
    void SetTargetTorque(double torque);

    /**
     * @brief 设置电机电流，单位A
     * 
     * @param current 
     */
    void SetTargetCurrent(double current);

    virtual void UpdateRuntimeData() final override;

  private:
    void doSetMode(int16_t mode);
  
    void InitParams();

    void SetProfileVelocity(uint32_t profile_velocity);
    void SetEndVelocity(uint32_t end_velocity);
    void SetAcceleration(uint32_t acceleration);
    void SetDeceleration(uint32_t deceleration);

    /*
    uint16_t control_word
    int16_t mode
    int16_t  torque
    int32_t  position
    uint32_t  profile_velocity
    uint32_t  end_velocity
    uint32_t  acceleration
    uint32_t  deceleration
    int32_t  target_velocity
    */
    static struct ElmoSendAddr
    {
      static constexpr uint16_t control_word = 0;
      static constexpr uint16_t mode = 2;			//01-P/3-V/4-T/08-SP/09-SV/0A-ST
      static constexpr uint16_t target_torque = 4;
      static constexpr uint16_t target_position = 6;
      static constexpr uint16_t profile_velocity = 10;
      static constexpr uint16_t end_velocity = 14;
      static constexpr uint16_t acceleration = 18;
      static constexpr uint16_t deceleration = 22;
      static constexpr uint16_t target_velocity = 26;
    } elmo_send_addr;

    /*
    uint16_t status_word;
    int16_t  mode;			//01-P/3-V/4-T/08-SP/09-SV/0A-ST
    int32_t  position;
    int32_t  velocity;
    int16_t  torque;
    int16_t  current;
    */
    static struct ElmoInputAddr
    {
      static constexpr uint16_t status_word = 0;
      static constexpr uint16_t mode = 2;			
      static constexpr uint16_t position = 4;
      static constexpr uint16_t velocity = 8;
      static constexpr uint16_t torque = 12;
      static constexpr uint16_t current = 14;
    } elmo_input_addr;

    std::array<uint8_t, 16> bus_receive_data_{0};
    std::array<uint8_t, 30> bus_send_data_{0};

    int poweron_step_ = 0;
  };

}
