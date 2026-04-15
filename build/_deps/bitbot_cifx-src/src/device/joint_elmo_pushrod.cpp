#include "bitbot_cifx/device/joint_elmo_pushrod.h"
#include "bitbot_kernel/utils/time_func.h"
#include "bitbot_cifx/bus/canopen.h"
#include "bitbot_cifx/bus/cifx/protocol_SDO.h"

namespace bitbot
{
  JointElmoPushrod::JointElmoPushrod(const pugi::xml_node& motor_node)
    : CifxJoint(motor_node)
  {
    constexpr double deg2rad = M_PI/180.0;

    basic_type_ = static_cast<uint32_t>(BasicDeviceType::MOTOR);
    type_ = static_cast<uint32_t>(CifxDeviceType::JOINT_ELMO_PUSHROD);
    input_data_length_ = 14;
    output_data_length_ = 14;

    monitor_header_.headers = {"status","mode","actual_position", "target_position", "actual_velocity","target_velocity","actual_current","target_torque"};
    monitor_data_.resize(monitor_header_.headers.size());

    std::string mode;
    ConfigParser::ParseAttribute2s(mode, motor_node.attribute("mode"));
    if(mode.compare("CSP") == 0)
    {
      joint_data_.config.mode = static_cast<int>(CANopenMotorMode::CSP);
    }
    else if(mode.compare("CST") == 0)
    {
      joint_data_.config.mode = static_cast<int>(CANopenMotorMode::CST);
    }else if(mode.compare("CSV") == 0)
    {
      joint_data_.config.mode = static_cast<int>(CANopenMotorMode::CSV);
    }

    this->InitParams();

    ConfigParser::ParseAttribute2d(joint_initial_pos_, motor_node.attribute("initial_pos"));
    joint_initial_pos_ *= deg2rad;
    ConfigParser::ParseAttribute2d(joint_offset_pos_, motor_node.attribute("offset_pos"));
    joint_offset_pos_ *= deg2rad;
    ConfigParser::ParseAttribute2d(theta0_, motor_node.attribute("theta0"));
    theta0_ *= deg2rad;
    ConfigParser::ParseAttribute2d(pushrod_efficiency_, motor_node.attribute("efficiency"));

    ConfigParser::ParseAttribute2d(pushrod_lead_, motor_node.attribute("lead"));
    if(pushrod_lead_ == 0)
    {
      logger_->error("please set param lead of pushrod:{}", name_);
      abort();
    }
    ConfigParser::ParseAttribute2d(l0_, motor_node.attribute("l0"));
    if(l0_ == 0)
    {
      logger_->error("please set param l0 of pushrod:{}", name_);
      abort();
    }
    ConfigParser::ParseAttribute2d(r0_, motor_node.attribute("r0"));
    if(r0_ == 0)
    {
      logger_->error("please set param r0 of pushrod:{}", name_);
      abort();
    }
    n_1_r0_ = 1/r0_;
    n_2l0r0_ = 2*l0_*r0_;
    n_1_2l0r0_ = 1/(2*l0_*r0_);
    n_l0_2_r0_2_ = std::pow(l0_,2) + std::pow(r0_,2);
    n_1_lead_ = 1/pushrod_lead_;
    n_1_efficiency_ = 1/pushrod_efficiency_;

    // 计算零位时的初始推杆长度
    pushrod_initial_len_ = std::sqrt(n_l0_2_r0_2_-n_2l0r0_*std::cos(M_PI-theta0_-joint_initial_pos_));
    // logger_->debug("initial len: {}", pushrod_initial_len_);
  }

  void JointElmoPushrod::InitSDO()
  {
    SetMotorRateTorque(id_, joint_data_.config.current_ratio);
    SleepMS(50);
    logger_->debug("joint {}(id {}) TorqueRate: {}", name_, id_, ReadMotorRateTorque(id_));
    SleepMS(50);
    SetMotorRateCurrent(id_, joint_data_.config.current_ratio);
    SleepMS(50);
    logger_->debug("joint {}(id {}) CurrentRate: {}", name_, id_, ReadMotorRateCurrent(id_));
  }
  
  void JointElmoPushrod::InitParams()
  {
    joint_data_.config.current_ratio = 10000;
    PowerOff();
    SetMode(static_cast<CANopenMotorMode>(joint_data_.config.mode));
    SetTargetVelocity(0);
    SetTargetTorque(0);
  }

  JointPowerOnState JointElmoPushrod::PowerOn()
  {
    using namespace ds402;

    constexpr int step_start = 0;
    constexpr int step_ready_to_switch_on = 1;
    constexpr int step_switch_on = 2;
    constexpr int step_enable_oprational = 3;
    constexpr int step_finish = 4;

    static int waiting_count = 0;

    // 位置参数复位
    this->SetTargetPosition(this->GetActualPosition());

    switch (poweron_step_)
    {
    case step_start:
      poweron_step_++;
      waiting_count = 0;
      break;
    case step_ready_to_switch_on:
      ReadyToSwitchOn(reinterpret_cast<uint16_t*>(bus_send_data_.data()+elmo_send_addr.control_word));
      if(isReadyToSwitchOn(*reinterpret_cast<uint16_t*>(bus_receive_data_.data()+elmo_input_addr.status_word)))
      {
        poweron_step_++;
        waiting_count = 0;
      }
      else
      {
        waiting_count++;
        if(waiting_count > 30)
        {
          logger_->error("motor(id{}) is not ready to switch on", id_);
          return JointPowerOnState::Error;
        }
      }
      break;
    case step_switch_on:
      SwitchOn(reinterpret_cast<uint16_t*>(bus_send_data_.data()+elmo_send_addr.control_word));
      if(isSwitchedOn(*reinterpret_cast<uint16_t*>(bus_receive_data_.data()+elmo_input_addr.status_word)))
      {
        poweron_step_++;
        waiting_count = 0;
      }
      else
      {
        waiting_count++;
        if(waiting_count > 30)
        {
          logger_->error("motor(id{}) does not switch on", id_);
          return JointPowerOnState::Error;
        }
      }
      break;
    case step_enable_oprational:
      EnableOperational(reinterpret_cast<uint16_t*>(bus_send_data_.data()+elmo_send_addr.control_word));
      if(isOperationEnabled(*reinterpret_cast<uint16_t*>(bus_receive_data_.data()+elmo_input_addr.status_word)))
      {
        poweron_step_++;
        waiting_count = 0;
      }
      else
      {
        waiting_count++;
        if(waiting_count > 30)
        {
          logger_->error("motor(id{}) is not operation_enabled", id_);
          return JointPowerOnState::Error;
        }
      }
      break;
    case step_finish:
      return JointPowerOnState::Finish;
      break;
    default:
      break;
    }
    return JointPowerOnState::Running;
  }

  void JointElmoPushrod::ResetFault()
  {
    ds402::ResetFault(reinterpret_cast<uint16_t*>(bus_send_data_.data()+elmo_send_addr.control_word));
  }

  void JointElmoPushrod::PowerOff()
  {
    ds402::ShutDown(reinterpret_cast<uint16_t*>(bus_send_data_.data()+elmo_send_addr.control_word));
  }

  void JointElmoPushrod::SetMode(CANopenMotorMode mode)
  {
    this->doSetMode(static_cast<int16_t>(mode));
  }

  void JointElmoPushrod::SetMode(int mode)
  {
    this->doSetMode(static_cast<int16_t>(mode));
  }

  void JointElmoPushrod::doSetMode(int16_t target_mode)
  {
    joint_data_.config.mode = target_mode;

    std::memcpy(bus_send_data_.data()+elmo_send_addr.mode, &target_mode, sizeof(int16_t));
  }

  void JointElmoPushrod::SetTargetPosition(double position)
  {
    static int32_t count = 0;
    joint_data_.runtime.target_pos = position;

    if(position > joint_data_.config.pos_max)
      position = joint_data_.config.pos_max;
    else if(position < joint_data_.config.pos_min)
      position = joint_data_.config.pos_min;

    count = EncoderAngle2Count(JointPos2MotorPos(position + joint_offset_pos_));
    std::memcpy(bus_send_data_.data()+elmo_send_addr.target_position, &count, sizeof(int32_t));
  }

  void JointElmoPushrod::SetTargetVelocity(double velocity)
  {
    static int32_t count = 0;
    joint_data_.runtime.target_velocity = velocity;

    count = EncoderCounts2Rads(JointVel2MotorVel(velocity));
    std::memcpy(bus_send_data_.data()+elmo_send_addr.target_velocity, &count, sizeof(int32_t));
  }

  void JointElmoPushrod::SetTargetTorque(double torque)
  {
    joint_data_.runtime.target_torque = torque;
    
    SetTargetCurrent(Torque2Current(JointTorque2MotorTorque(torque)));
  }

  void JointElmoPushrod::SetTargetCurrent(double current)
  {
    static int16_t value = 0;
    static const double factor = 1000000/joint_data_.config.current_ratio;

    if(current > joint_data_.config.current_max)
      current = joint_data_.config.current_max;
    else if(current < joint_data_.config.current_min)
      current = joint_data_.config.current_min;

    joint_data_.runtime.target_current = current;

    value = current * factor;
    std::memcpy(bus_send_data_.data()+elmo_send_addr.target_torque, &value, sizeof(int16_t));
  }

  void JointElmoPushrod::UpdateRuntimeData()
  {
    constexpr double rad2deg = 180.0/M_PI;
    // "status","mode","actual_position","target_position","actual_velocity","target_velocity","actual_current","target_current"
    monitor_data_[0] = joint_data_.runtime.status;
    monitor_data_[1] = joint_data_.runtime.actual_mode;
    monitor_data_[2] = joint_data_.runtime.actual_pos * rad2deg;
    monitor_data_[3] = joint_data_.runtime.target_pos * rad2deg;
    monitor_data_[4] = joint_data_.runtime.actual_velocity * rad2deg;
    monitor_data_[5] = joint_data_.runtime.target_velocity * rad2deg;
    monitor_data_[6] = joint_data_.runtime.actual_current;
    monitor_data_[7] = joint_data_.runtime.target_torque;
  }

}
