#include "user_func.h"
#include "bitbot_cifx/device/joint_elmo.h"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"

#include <chrono>
#include <iostream>
#include <memory>

#define _ControlT 0.001

constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2deg = 180.0 / M_PI;

bitbot::JointElmo *joint1 = nullptr, *joint2 = nullptr, *joint3 = nullptr;
bitbot::JointElmoPushrod *joint4 = nullptr;

std::vector<std::vector<double>> traj_data;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &)
{
  joint1 = bus.GetDevice<bitbot::JointElmo>(3).value();
  joint2 = bus.GetDevice<bitbot::JointElmo>(2).value();
  joint3 = bus.GetDevice<bitbot::JointElmo>(1).value();
  joint4 = bus.GetDevice<bitbot::JointElmoPushrod>(0).value();
}

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::InitPos);
}

std::optional<bitbot::StateId> EventMaintainPos(bitbot::EventValue value, UserData &user_data)
{
  return static_cast<bitbot::StateId>(States::MaintainPos);
}

std::optional<bitbot::StateId> EventChangeMode(bitbot::EventValue value, UserData &user_data)
{
  return static_cast<bitbot::StateId>(States::ChangeMode);
}

void StateWaiting(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
}

void StateInitPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  static bool init = false;
  static double start_time = 0;
  static double initial_pos1 = 0;
  static double initial_pos2 = 0;
  static double initial_pos3 = 0;
  static double initial_pos4 = 0;
  static constexpr double final_pos1 = 31.6 * deg2rad;  // 目标位置 (rad)
  static constexpr double final_pos2 = 0.0 * deg2rad;
  static constexpr double final_pos3 = 0.0 * deg2rad;
  static constexpr double final_pos4 = 70.7 * deg2rad;
  static constexpr double duration = 20.0;   // 运动时长 (s)

  double time = kernel.GetPeriodsCount() * _ControlT;

  if (!init)
  {
    start_time = time;
    initial_pos1 = joint1->GetActualPosition();
    initial_pos2 = joint2->GetActualPosition();
    initial_pos3 = joint3->GetActualPosition();
    initial_pos4 = joint4->GetActualPosition();
    init = true;
  }
  double t = time - start_time;

  double pos1, pos2, pos3, pos4;
  if (t >= duration)
  {
    pos1 = final_pos1;
    pos2 = final_pos2;
    pos3 = final_pos3;
    pos4 = final_pos4;
  }
  else
  {
    // 余弦插值：起止速度为零，位置连续
    double s = (1.0 - std::cos(M_PI * t / duration)) / 2.0;
    pos1 = initial_pos1 + (final_pos1 - initial_pos1) * s;
    pos2 = initial_pos2 + (final_pos2 - initial_pos2) * s;
    pos3 = initial_pos3 + (final_pos3 - initial_pos3) * s;
    pos4 = initial_pos4 + (final_pos4 - initial_pos4) * s;
  }
  joint1->SetTargetPosition(pos1);
  joint2->SetTargetPosition(pos2);
  joint3->SetTargetPosition(pos3);
  joint4->SetTargetPosition(pos4);
}

void StateMaintainPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  static bool init = false;
  static double target_pos1 = 0;
  static double target_pos2 = 0;
  static double target_pos3 = 0;
  static double target_pos4 = 0;

  // PD 参数（根据实际电机调整）
  static constexpr double Kp1 = 50, Kd1 = 0.5;
  static constexpr double Kp2 = 50, Kd2 = 0.5;
  static constexpr double Kp3 = 50, Kd3 = 0.5;
  static constexpr double Kp4 = 50, Kd4 = 0.5;

  if (!init)
  {
    target_pos1 = joint1->GetActualPosition();
    target_pos2 = joint2->GetActualPosition();
    target_pos3 = joint3->GetActualPosition();
    target_pos4 = joint4->GetActualPosition();

    joint1->SetTargetCurrent(0);
    joint1->SetMode(bitbot::CANopenMotorMode::CST);
    joint2->SetTargetCurrent(0);
    joint2->SetMode(bitbot::CANopenMotorMode::CST);
    joint3->SetTargetCurrent(0);
    joint3->SetMode(bitbot::CANopenMotorMode::CST);
    joint4->SetTargetCurrent(0);
    joint4->SetMode(bitbot::CANopenMotorMode::CST);
    init = true;
  }

  // PD 控制: tau = Kp * (target - actual) - Kd * velocity
  double err1 = target_pos1 - joint1->GetActualPosition();
  double err2 = target_pos2 - joint2->GetActualPosition();
  double err3 = target_pos3 - joint3->GetActualPosition();
  double err4 = target_pos4 - joint4->GetActualPosition();

  double vel1 = joint1->GetActualVelocity();
  double vel2 = joint2->GetActualVelocity();
  double vel3 = joint3->GetActualVelocity();
  double vel4 = joint4->GetActualVelocity();

  double tau1 = Kp1 * err1 - Kd1 * vel1;
  double tau2 = Kp2 * err2 - Kd2 * vel2;
  double tau3 = Kp3 * err3 - Kd3 * vel3;
  double tau4 = Kp4 * err4 - Kd4 * vel4;

  joint1->SetTargetCurrent(tau1);
  joint2->SetTargetCurrent(tau2);
  joint3->SetTargetCurrent(tau3);
  joint4->SetTargetCurrent(tau4);
}

void StateChangeMode(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data)
{
  static bool once = false;
  if (!once)
  {
    // joint1->SetTargetCurrent(0);
    // joint1->SetMode(bitbot::CANopenMotorMode::CST);
    // joint2->SetTargetCurrent(0);
    // joint2->SetMode(bitbot::CANopenMotorMode::CST);
    once = true;
  }
}
