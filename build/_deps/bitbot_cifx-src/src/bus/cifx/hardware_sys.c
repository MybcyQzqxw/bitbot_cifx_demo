#include "bitbot_cifx/bus/cifx/hardware_sys.h"
#include "bitbot_cifx/bus/cifx/protocol_SDO.h"
#include "bitbot_kernel/utils/time_func.h"

#include <stdio.h>

void InitHardwareSys()
{
  InitCommunication();   //init cifx
  SleepS(1);
}

void HardwareEnableInterrupt(PFN_NOTIFY_CALLBACK function, void* user_data)
{
  RegisterInEvent(function, user_data);

  /* Set Sync mode */
  if(ECatMasterDev_SetSyncMode(cifx_channel) != 0)
  {
    printf("error during set sync mode\n");
  }

}

void HardwareDisableInterrupt()
{
  ECatMasterDev_UnRegisterInEvent(cifx_channel);
}


void HardwareExit()
{
  CloseCommunication();
}
