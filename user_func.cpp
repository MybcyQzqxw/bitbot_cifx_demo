#include "user_func.h"
#include "bitbot_cifx/device/joint_elmo.h"

#include <chrono>
#include <iostream>
#include <memory>

#define _ControlT 0.004

constexpr double deg2rad = M_PI / 180.0;
constexpr double rad2deg = 180.0 / M_PI;

bitbot::JointElmo *joint1, *joint2, *joint3, *joint4, *joint5 = nullptr;

std::vector<std::vector<double>> traj_data;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &)
{
  joint1 = bus.GetDevice<bitbot::JointElmo>(3).value();
  // joint2 = bus.GetDevice<bitbot::JointElmo>(1).value();
  // joint3 = bus.GetDevice<bitbot::JointElmo>(2).value();
  // joint4 = bus.GetDevice<bitbot::JointElmo>(3).value();
  // joint5 = bus.GetDevice<bitbot::JointElmo>(4).value();

  // traj_data = ReadCSV("../../cz/run55km_4ms.csv");
}

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue, UserData &)
{
  return static_cast<bitbot::StateId>(States::InitPos);
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
  static double initial_pos = 0;

  static int cnt = 0;
  static int max_cnt = traj_data.size();

  double time = kernel.GetPeriodsCount() * 0.001;

  if (!init)
  {
    start_time = time;
    initial_pos = joint1->GetActualPosition();
    init = true;
  }
  double t = time - start_time;
  // if (cnt < max_cnt)
  // {
  //   joint1->SetTargetPosition(traj_data[cnt][0]);
  //   joint2->SetTargetPosition(traj_data[cnt][1]);
  //   joint3->SetTargetPosition(traj_data[cnt][2]);
  //   joint4->SetTargetPosition(traj_data[cnt][3]);
  //   joint5->SetTargetPosition(traj_data[cnt][4]);

  //   cnt++;
  // }
  // joint1 = traj_data[cnt][0]
  joint1->SetTargetPosition(initial_pos + 10 * deg2rad * std::sin(0.1 * M_PI * t));
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
