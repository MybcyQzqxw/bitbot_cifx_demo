#ifndef _HARDWARE_SYS_H
#define _HARDWARE_SYS_H

#include "bitbot_cifx/bus/cifx/cifx_driver.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

void InitHardwareSys();

void HardwareEnableInterrupt(PFN_NOTIFY_CALLBACK function, void* user_data);

void HardwareDisableInterrupt();

void HardwareExit();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _HARDWARE_SYS_H
