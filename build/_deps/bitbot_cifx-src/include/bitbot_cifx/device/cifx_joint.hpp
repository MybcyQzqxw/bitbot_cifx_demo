#pragma once

#include "bitbot_cifx/device/cifx_device.hpp"

#ifndef _USE_MATH_DEFINES
  #define _USE_MATH_DEFINES
#endif // _USE_MATH_DEFINES
#include <math.h>

namespace bitbot
{
  struct JointConfigData
  {
    bool enable = true;

    int mode = 0;
    int direction = 1;
    int torque_direction = 1;
    uint32_t current_ratio = 10000; // 电流比例 1000->1mA 10000->10mA
    double encoder_resolution = 1;
    double transmission_ratio = 1;
    double torque_constant = 0;
    double calibration_pos = 0;
    double current_min = -1;
    double current_max = 1;
    double pos_min = -1;
    double pos_max = 1;
  };

  struct JointRuntimeData
  {
    uint16_t status = 0;
    int actual_mode = 0;

    double actual_pos = 0;
    double actual_velocity = 0;
    double actual_torque = 0;
    double actual_current = 0;

    double target_pos = 0;
    double target_velocity = 0;
    double target_torque = 0;
    double target_current = 0;
  };

  struct JointData
  {
    JointConfigData config;
    JointRuntimeData runtime;
  };

  enum class JointPowerOnState
  {
    Null = 0,
    Running,
    Finish,
    Error
  };

  class CifxJoint : public CifxDevice
  {
  public:
    CifxJoint(const pugi::xml_node& device_node)
      : CifxDevice(device_node)
    {
      constexpr double deg2rad = M_PI / 180.0;

      ConfigParser::ParseAttribute2b(joint_data_.config.enable,              device_node.attribute("enable"));
      ConfigParser::ParseAttribute2i(joint_data_.config.direction,           device_node.attribute("motor_direction"));
      ConfigParser::ParseAttribute2i(joint_data_.config.torque_direction,    device_node.attribute("torque_direction"));
      ConfigParser::ParseAttribute2d(joint_data_.config.encoder_resolution,  device_node.attribute("encoder_resolution"));
      ConfigParser::ParseAttribute2d(joint_data_.config.transmission_ratio,  device_node.attribute("transmission_ratio"));
      ConfigParser::ParseAttribute2d(joint_data_.config.torque_constant,     device_node.attribute("torque_constant"));
      ConfigParser::ParseAttribute2d(joint_data_.config.calibration_pos,   device_node.attribute("calibration_pos"));
      joint_data_.config.calibration_pos = joint_data_.config.calibration_pos * deg2rad;
      ConfigParser::ParseAttribute2d(joint_data_.config.current_min,         device_node.attribute("current_min"));
      ConfigParser::ParseAttribute2d(joint_data_.config.current_max,         device_node.attribute("current_max"));
      ConfigParser::ParseAttribute2d(joint_data_.config.pos_min,           device_node.attribute("pos_min"));
      ConfigParser::ParseAttribute2d(joint_data_.config.pos_max,           device_node.attribute("pos_max"));
    }
    virtual ~CifxJoint() = default;

    virtual void ResetFault() = 0;
    virtual JointPowerOnState PowerOn() = 0;
    virtual void PowerOff() = 0;

    inline void Enable()
    {
      joint_data_.config.enable = true;
    }
    inline void Disable()
    {
      joint_data_.config.enable = false;
    }
    inline bool isEnable()
    {
      return joint_data_.config.enable;
    }

    /**
     * @brief 设置电机模式
     * 
     * @param mode 
     */
    virtual void SetMode(int mode) = 0;

    /**
     * @brief 设置电机目标位置 单位：弧度(rad)
     * 
     * @param angle 
     */
    virtual void SetTargetPosition(double position) = 0;
    /**
     * @brief 设置电机目标速度 单位：转/分(rpm)
     * 
     * @param velocity 
     */
    virtual void SetTargetVelocity(double velocity) = 0;

    /**
     * @brief 设置电机力矩
     * 
     * @param torque 
     */
    virtual void SetTargetTorque(double torque) = 0;

    /**
     * @brief 设置电机电流，单位A
     * 
     * @param current 
     */
    virtual void SetTargetCurrent(double current) = 0;

    inline uint16_t GetStatus()
    {
      return joint_data_.runtime.status;
    }
    inline int16_t GetMode()
    {
      return joint_data_.runtime.actual_mode;
    }
    inline double GetActualPosition()
    {
      return joint_data_.runtime.actual_pos;
    }
    inline double GetActualVelocity()
    {
      return joint_data_.runtime.actual_velocity;
    }
    inline double GetActualTorque()
    {
      return joint_data_.runtime.actual_torque;
    }
    inline double GetActualCurrent()
    {
      return joint_data_.runtime.actual_current;
    }
    inline double GetTargetPosition()
    {
      return joint_data_.runtime.target_pos;
    }
    inline double GetTargetVelocity()
    {
      return joint_data_.runtime.target_velocity;
    }
    inline double GetTargetTorque()
    {
      return joint_data_.runtime.target_torque;
    }
    inline double GetTargetCurrent()
    {
      return joint_data_.runtime.target_current;
    }

    inline uint32_t GetCurrentRatio()
    {
      return joint_data_.config.current_ratio;
    }

  protected:
    // Position count -> angle(rad)
    inline double EncoderCount2Angle(int32_t encoder)
    {
      return joint_data_.config.direction * encoder / joint_data_.config.encoder_resolution / joint_data_.config.transmission_ratio * 2 * M_PI + joint_data_.config.calibration_pos;
    }

    // Position angle(rad) -> count
    inline int32_t EncoderAngle2Count(double angle)
    {
      return static_cast<int32_t>(joint_data_.config.direction * (angle - joint_data_.config.calibration_pos) * joint_data_.config.encoder_resolution * joint_data_.config.transmission_ratio * 0.5 * M_1_PI);
    }

    // Velocity  count/s -> rpm
    inline double EncoderCounts2Rpm(int32_t encoder)
    {
      return joint_data_.config.direction * encoder / joint_data_.config.encoder_resolution / joint_data_.config.transmission_ratio * 60.0 ;
    }

    // Velocity  count/s -> rad/s
    inline double EncoderCounts2Rads(int32_t encoder)
    {
      return joint_data_.config.direction * encoder / joint_data_.config.encoder_resolution / joint_data_.config.transmission_ratio * 2 * M_PI ;
    }

    // Velocity  rpm -> count/s
    inline int32_t EncoderRpm2Counts(double rpm)
    {
      return static_cast<int32_t>(joint_data_.config.direction * rpm * joint_data_.config.encoder_resolution * joint_data_.config.transmission_ratio / 60.0);
    }

    // Torque Nm -> A
    inline double Torque2Current(double torque)
    {
      return torque / joint_data_.config.transmission_ratio / joint_data_.config.torque_constant;
    }

    // Current A -> Nm
    inline double Current2Torque(double current)
    {
      return current * joint_data_.config.transmission_ratio * joint_data_.config.torque_constant;
    }

    JointData joint_data_;
  };
  
}


