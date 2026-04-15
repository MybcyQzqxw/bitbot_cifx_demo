#pragma once

#include "bitbot_cifx/bus/canopen.h"

namespace bitbot
{
  enum class CANopenMotorMode : int16_t
  {
    PP = 1, //ProPositionMode
    PV = 3, //ProVelocityMode
    PT = 4, //ProTorqueMode	
    CSP = 8,  //SynPositionMode
    CSV = 9,  //SynVelocityMode
    CST = 10  //SynTorqueMode	
  };

  namespace ds402
  {
    inline void ShutDown(uint16_t* control_word)
    {
      *control_word = SHUT_DOWN;
    }
    inline void ResetFault(uint16_t* control_word)
    {
      *control_word |= CW_FalutReset;
    }
    inline void ReadyToSwitchOn(uint16_t* control_word)
    {
      *control_word = READY_TO_SWITCH_ON;
    }
    inline void SwitchOn(uint16_t* control_word)
    {
      *control_word = SWITCH_ON;
    }
    inline void EnableOperational(uint16_t* control_word)
    {
      *control_word = ENABLE_OPERATIONAL;
    }

    inline bool isReadyToSwitchOn(uint16_t status_word)
    {
      using namespace ds402;
      return status_word & SW_ReadyToSwitchOn == SW_ReadyToSwitchOn?true:false;
    }
    inline bool isSwitchedOn(uint16_t status_word)
    {
      using namespace ds402;
      return status_word & SW_SwitchedOn == SW_SwitchedOn?true:false;
    }
    inline bool isOperationEnabled(uint16_t status_word)
    {
      using namespace ds402;
      return status_word & SW_OperationEnabled == SW_OperationEnabled?true:false;
    }
    
  };
  
}