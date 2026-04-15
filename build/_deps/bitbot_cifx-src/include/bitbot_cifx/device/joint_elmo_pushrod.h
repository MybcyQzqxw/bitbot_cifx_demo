#pragma once

#include "bitbot_cifx/device/cifx_joint.hpp"
#include "bitbot_cifx/device/canopen_motor.hpp"

namespace bitbot
{

  class JointElmoPushrod final: public CifxJoint
  {
  public:
    JointElmoPushrod(const pugi::xml_node& motor_node);
    virtual ~JointElmoPushrod() = default;

    virtual void InitSDO() final override;

    inline virtual void Input(void* bus_data) final override
    {
      const float current_ratio = joint_data_.config.current_ratio / 1000.0 / 1000.0;
      // printf("receive: ");
      // for(uint16_t i = 0; i<16; i++)
      // {
      //   printf("%02x ", *((unsigned char*)(bus_data)+i));
      // }
      // printf("\n");

      std::memcpy(bus_receive_data_.data(), bus_data, 14);
      
      std::memcpy(&joint_data_.runtime.status, bus_receive_data_.data()+elmo_input_addr.status_word, sizeof(uint16_t));

      std::memcpy(&joint_data_.runtime.actual_mode, bus_receive_data_.data()+elmo_input_addr.mode, sizeof(int16_t));

      int32_t encoder;
      std::memcpy(&encoder, bus_receive_data_.data()+elmo_input_addr.position, sizeof(int32_t));
      motor_pos_ = EncoderCount2Angle(encoder);

      std::memcpy(&encoder, bus_receive_data_.data()+elmo_input_addr.velocity, sizeof(int32_t));
      motor_vel_ = EncoderCounts2Rads(encoder);

      int16_t current;
      std::memcpy(&current, bus_receive_data_.data()+elmo_input_addr.current, sizeof(int16_t));
      joint_data_.runtime.actual_current = current * current_ratio;
      
      motor_torque_ = Current2Torque(joint_data_.runtime.actual_current);

      MotorParams2JointParams();
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
    void InitParams();
    void doSetMode(int16_t target_mode);
    void MotorParams2JointParams()
    {
      MotorParams2PushrodParams();
      PushrodParams2JointParams();
    }
    void MotorParams2PushrodParams()
    {
      pushrod_len_ = pushrod_initial_len_ - motor_pos_ * n_1_2pi_ * pushrod_lead_;
      double costheta1 = (n_l0_2_r0_2_ - std::pow(pushrod_len_,2)) * n_1_2l0r0_;
      if(costheta1 > 1 || costheta1 < -1)
      {
        logger_->error("joint pushrod {} out of range", name_);
      }
      else
      {
        theta1_ = std::acos(costheta1);
      }
      pushrod_vel_ = motor_vel_*n_1_2pi_*pushrod_lead_;

      pushrod_force_ = motor_torque_*n_2pi_*pushrod_efficiency_*n_1_lead_;
    }
    void PushrodParams2JointParams()
    {
      joint_data_.runtime.actual_pos = M_PI - theta0_ - theta1_ - joint_offset_pos_;

      sin_alpha_ = l0_*std::sin(theta1_)/pushrod_len_;
      if(sin_alpha_ > 1 || sin_alpha_ < -1)
      {
        logger_->error("joint pushrod {} out of range", name_);
      }
      else
      {
        joint_data_.runtime.actual_velocity = pushrod_vel_ / sin_alpha_ * n_1_r0_;
      }

      joint_data_.runtime.actual_torque = pushrod_force_*sin_alpha_*r0_;
    }

    double JointPos2MotorPos(double joint_pos)
    {
      double pr_pos = std::sqrt(n_l0_2_r0_2_-n_2l0r0_*std::cos(M_PI-theta0_-joint_pos));
      return (pushrod_initial_len_ - pr_pos) * n_2pi_ * n_1_lead_;
    }
    double JointVel2MotorVel(double joint_vel)
    {
      double pr_vel = joint_vel*r0_*sin_alpha_;
      return pr_vel * n_2pi_ * n_1_lead_;
    }
    double JointTorque2MotorTorque(double joint_torque)
    {
      double pr_force = joint_torque*n_1_r0_/sin_alpha_;
      return pr_force * pushrod_lead_ * n_1_efficiency_ * n_1_2pi_;
    }

    /*
    uint16_t control_word
    int16_t mode
    int16_t  torque
    int32_t  position
    int32_t  target_velocity
    */

    static struct ElmoSendAddr
    {
      static constexpr uint16_t control_word = 0;
      static constexpr uint16_t mode = 2;			//01-P/3-V/4-T/08-SP/09-SV/0A-ST
      static constexpr uint16_t target_torque = 4;
      static constexpr uint16_t target_position = 6;
      static constexpr uint16_t target_velocity = 10;
    } elmo_send_addr;

    /*
    uint16_t status_word;
    int16_t  mode;			//01-P/3-V/4-T/08-SP/09-SV/0A-ST
    int32_t  position;
    int32_t  velocity;
    int16_t  current;
    */

    static struct ElmoInputAddr
    {
      static constexpr uint16_t status_word = 0;
      static constexpr uint16_t mode = 2;			
      static constexpr uint16_t position = 4;
      static constexpr uint16_t velocity = 8;
      static constexpr uint16_t current = 12;
    } elmo_input_addr;

    std::array<uint8_t, 14> bus_receive_data_{0};
    std::array<uint8_t, 14> bus_send_data_{0};

    int poweron_step_ = 0;

    double motor_pos_;
    double motor_vel_;
    double motor_torque_;
    double pushrod_lead_ = 0; // 导程
    double pushrod_len_;
    double pushrod_vel_;
    double pushrod_force_;
    double pushrod_efficiency_ = 0.95;
    double joint_initial_pos_ = 0; // 初始关节角
    double joint_offset_pos_ = 0; // 关节角偏置
    double pushrod_initial_len_ = 0; // 推杆初始长度
    double theta0_ = 0; // angle offset between the end of pushrod and the next link
    double theta1_; // angle offset between the end of pushrod and the previous link
    double sin_alpha_;
    double l0_ = 0; // length between the start of pushrod and joint
    double r0_ = 0; // length between the end of pushrod and joint
    double n_l0_2_r0_2_; // l0^2+r0^2
    double n_2l0r0_; // 2*l0*r0
    double n_1_2l0r0_; // 1/2*l0*r0
    double n_1_r0_; // 1/r0
    double n_1_lead_; // 1/pushrod_lead_;
    double n_1_efficiency_;

    static constexpr double n_2pi_ = 2*M_PI; // 2pi
    static constexpr double n_1_2pi_ = 0.5*M_1_PI; // 1/2pi
  };

}
