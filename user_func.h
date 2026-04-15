#pragma once

#include "bitbot_cifx/kernel/cifx_kernel.hpp"
#include "bitbot_cifx/device/joint_elmo_pushrod.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <fstream>
#include <sstream>

enum Events
{
  InitPos = 1001,
  MaintainPos = 1002,
  ChangeMode = 1003,
};

enum class States : bitbot::StateId
{
  Waiting = 2001,
  InitPos = 2002,
  MaintainPos = 2003,
  ChangeMode = 2004,
};

extern bitbot::JointElmoPushrod *joint_x;

struct UserData
{
};

using CifxKernel = bitbot::CifxKernel<UserData, "setup_time", "solve_time", "pin_time">;

void ConfigFunc(const bitbot::CifxBus &bus, UserData &);

std::optional<bitbot::StateId> EventInitPos(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventMaintainPos(bitbot::EventValue value, UserData &user_data);
std::optional<bitbot::StateId> EventChangeMode(bitbot::EventValue value, UserData &user_data);

void StateWaiting(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateInitPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateMaintainPos(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);
void StateChangeMode(const bitbot::KernelInterface &kernel, CifxKernel::ExtraData &extra_data, UserData &user_data);

// void InitPos(double time);
// void JointSinPos(double time); // 关节空间sin

static std::vector<std::vector<double>> ReadCSV(std::string filename)
{
  using namespace std;
  vector<vector<double>> data;
  ifstream file(filename);

  string line;
  while (getline(file, line))
  {
    stringstream liness(line);
    string cell;
    vector<double> row;

    while (getline(liness, cell, ','))
    {
      row.push_back(stod(cell));
    }

    if (!row.empty())
      data.push_back(row);
  }

  return data;
}
