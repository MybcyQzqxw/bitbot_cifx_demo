#ifndef _ETHERCAT_H
#define _ETHERCAT_H

#define CIFX_DEV "cifX0"

#include "bitbot_cifx/bus/cifx/cifx_variable.h"

#include <cifx/cifXUser.h>
#include <cifx/cifXErrors.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct SYNC_CALLBACK_DATAtag
{
	uint8_t bSyncHSMode;
	CIFXHANDLE hDevice;
} SYNC_CALLBACK_DATA;

void InitCommunication();
void DumpPacket(CIFX_PACKET* ptPacket);
void DisplayDriverInformation (void);
long EnumBoardDemo(void);
int32_t SysdeviceDemo(void);
int32_t ChannelDemo(void);
void CloseCommunication();

void DumpData(unsigned char* pbData, unsigned long ulDataLen);
void ShowError(int32_t error);

void testInterruptCallback(uint32_t ulNotification, uint32_t ulDataLen, void* pvData, void* user_data);
void RegisterInEvent(PFN_NOTIFY_CALLBACK function, void* pvUser);
void ECatMasterDev_UnRegisterInEvent(CIFXHANDLE hChannel);
long ECatMasterDev_SetSyncMode(CIFXHANDLE cifx_channel);

void AckownledgeSyn();
#ifdef __cplusplus
}
#endif // __cplusplus

#endif
