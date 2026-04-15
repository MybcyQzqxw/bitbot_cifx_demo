#include "bitbot_cifx/bus/cifx/cifx_driver.h"
#include "bitbot_cifx/bus/cifx/ethercat_define.h"
#include "bitbot_cifx/bus/cifx/rcX_Public.h"
#include "bitbot_kernel/utils/time_func.h"

#include <cifx/cifxlinux.h>
#include <cifx/cifXEndianess.h>
#include <cifx/cifXErrors.h>
#include <cifx/Hil_SystemCmd.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <linux/time.h>

uint32_t    ulState = 0;

void InitCommunication()
{
	struct CIFX_LINUX_INIT init =
	{
		.init_options        = CIFX_DRIVER_INIT_AUTOSCAN,
		.base_dir            = NULL,
		.poll_interval       = 0,          //non-interrupt 0:500ms
		.trace_level         = 255,      /* form log */
		.user_card_cnt       = 0,
		.user_cards          = NULL,
		.iCardNumber         = 0,
		.fEnableCardLocking  = 0,
		.poll_StackSize      = 0,        /* set to 0 to use default */
	};

	#ifdef DEBUG
		printf("%s() called\n", __FUNCTION__);
	#endif

	/* First of all initialize toolkit */
	cifx_return_val = cifXDriverInit(&init);

	if(CIFX_NO_ERROR == cifx_return_val)
	{
		/* Display version of cifXRTXDrv and cifXToolkit */
		DisplayDriverInformation();

		/* Demonstrate the board/channel enumeration */
		EnumBoardDemo();

		/* Demonstrate system channel functionality */
		//SysdeviceDemo();

		/* Demonstrate communication channel functionality */
		//ChannelDemo();
	}
  else{
    ShowError(cifx_return_val);
  }

  if((cifx_return_val = xDriverOpen(&cifx_driver)) != CIFX_NO_ERROR)
    ShowError(cifx_return_val);

  if((cifx_return_val = xChannelOpen(cifx_driver, CIFX_DEV, 0, &cifx_channel)) != CIFX_NO_ERROR)
    ShowError(cifx_return_val);

  uint32_t actual_state;
	cifx_return_val =	xChannelBusState(cifx_channel, CIFX_BUS_STATE_ON, &actual_state, 10000);
	if (0 != cifx_return_val)
	{
		printf("Error while invoking ChannelBusState on!\n");
		ShowError(cifx_return_val);
	}
}

/*****************************************************************************/
/*! Dumps a rcX packet to debug console
*   \param ptPacket Pointer to packed being dumped                           */
/*****************************************************************************/
void DumpPacket(CIFX_PACKET* ptPacket)
{
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif
  printf("Dest   : 0x%08lX      ID   : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDest),  (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulId));
  printf("Src    : 0x%08lX      Sta  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrc),   (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulState));
  printf("DestID : 0x%08lX      Cmd  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulDestId),(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulCmd));
  printf("SrcID  : 0x%08lX      Ext  : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulSrcId), (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulExt));
  printf("Len    : 0x%08lX      Rout : 0x%08lX\r\n",(long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulLen),   (long unsigned int)HOST_TO_LE32(ptPacket->tHeader.ulRout));

  printf("Data:");
  DumpData(ptPacket->abData, HOST_TO_LE32(ptPacket->tHeader.ulLen));
}

/*****************************************************************************/
/*! Function to display driver information
 *   \param  cifx_driver  Handle to cifX driver
 *   \param  ptVTable Pointer to cifX API function table
 *   \return CIFX_NO_ERROR on success                                        */
/*****************************************************************************/
void DisplayDriverInformation (void)
{
  cifx_return_val = CIFX_NO_ERROR;
  DRIVER_INFORMATION tDriverInfo      = {{0}};
  char               szDrvVersion[32] = "";

  if (CIFX_NO_ERROR == (cifx_return_val = xDriverOpen(&cifx_driver)))
  {
    printf("\n---------- Display Driver Version ----------\n");
    if( CIFX_NO_ERROR != (cifx_return_val = xDriverGetInformation(NULL, sizeof(tDriverInfo), &tDriverInfo)) )
      ShowError( cifx_return_val);
    else if ( CIFX_NO_ERROR != (cifx_return_val = cifXGetDriverVersion( sizeof(szDrvVersion)/sizeof(*szDrvVersion), szDrvVersion)))
      ShowError( cifx_return_val);
    else
      printf("Driver Version: %s, based on %.32s \n\n", szDrvVersion, tDriverInfo.abDriverVersion);

    /* close previously opened driver */
    xDriverClose(cifx_driver);

  }

  printf(" State = 0x%08X\r\n", (unsigned int)cifx_return_val);
  printf("----------------------------------------------------\r\n");
}

/*****************************************************************************/
/*! Function to demonstrate the board/channel enumeration
*   \param  cifx_driver  Handle to cifX driver
*   \return CIFX_NO_ERROR on success                                         */
/*****************************************************************************/
long EnumBoardDemo(void)
{
	  unsigned long     ulBoard    = 0;
	  BOARD_INFORMATION tBoardInfo = {0};
	  cifx_return_val = xDriverOpen(&cifx_driver);
	  printf("\n---------- Board/Channel enumeration demo ----------\n");
	  if(CIFX_NO_ERROR == cifx_return_val)
	  {
      /* Iterate over all boards */
      while(CIFX_NO_ERROR == xDriverEnumBoards(cifx_driver, ulBoard, sizeof(tBoardInfo), &tBoardInfo))
      {
        printf("Found Board %s\n", tBoardInfo.abBoardName);
        if(strlen( (char*)tBoardInfo.abBoardAlias) != 0)
          printf(" Alias        : %s\n", tBoardInfo.abBoardAlias);
        printf(" DeviceNumber : %u\n", tBoardInfo.tSystemInfo.ulDeviceNumber);
        printf(" SerialNumber : %u\n", tBoardInfo.tSystemInfo.ulSerialNumber);
        printf(" Board ID     : %u\n", tBoardInfo.ulBoardID);
        printf(" System Error : 0x%08X\n", tBoardInfo.ulSystemError);
        printf(" Channels     : %u\n", tBoardInfo.ulChannelCnt);
        printf(" DPM Size     : %u\n", tBoardInfo.ulDpmTotalSize);

	    
	      unsigned long       ulChannel    = 0;
	      CHANNEL_INFORMATION tChannelInfo = {{0}};

	      /* iterate over all channels on the current board */
	      while(CIFX_NO_ERROR == xDriverEnumChannels(cifx_driver, ulBoard, ulChannel, sizeof(tChannelInfo), &tChannelInfo))
	      {
	        printf(" - Channel %lu:\n", ulChannel);
	        printf("    Firmware : %s\n", tChannelInfo.abFWName);
	        printf("    Version  : %u.%u.%u build %u\n",
	                tChannelInfo.usFWMajor,
	                tChannelInfo.usFWMinor,
	                tChannelInfo.usFWRevision,
	                tChannelInfo.usFWBuild);
	        printf("    Date     : %02u/%02u/%04u\n",
	                tChannelInfo.bFWMonth,
	                tChannelInfo.bFWDay,
	                tChannelInfo.usFWYear);

	        ++ulChannel;
	      }
	    
        ++ulBoard;
      }
	    xDriverClose(cifx_driver);
	  }
    else
    {
      ShowError(cifx_return_val);
    }
	  printf(" State = 0x%08X\n", cifx_return_val);
	  printf("----------------------------------------------------\n");

	  return cifx_return_val;
}

/*****************************************************************************/
/*! Function to demonstrate system channel functionality (PacketTransfer)
*   \param  cifx_driver  Handle to cifX driver
*   \return CIFX_NO_ERROR on success                                         */
/*****************************************************************************/
int32_t SysdeviceDemo(void)
{
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif
  cifx_return_val    = xDriverOpen(&cifx_driver);

  printf("---------- System Device handling demo ----------\r\n");

  if(CIFX_NO_ERROR == cifx_return_val)
  {
    /* Driver/Toolkit successfully opened */
    CIFXHANDLE hSys = NULL;
    cifx_return_val = xSysdeviceOpen(cifx_driver, CIFX_DEV, &hSys);

    if(CIFX_NO_ERROR != cifx_return_val)
    {
      printf("Error opening SystemDevice!\r\n");

    }
    else
    {
      SYSTEM_CHANNEL_SYSTEM_INFO_BLOCK    tSysInfoBlock = {{0}};
      SYSTEM_CHANNEL_SYSTEM_INFORMATION   tSysInfo      = {0};
      SYSTEM_CHANNEL_SYSTEM_CONTROL_BLOCK tControlBlock = {0};
      SYSTEM_CHANNEL_SYSTEM_STATUS_BLOCK  tStatusBlock  = {0};

      /* System channel successfully opened, try to read the System Info Block */
      if( CIFX_NO_ERROR != (cifx_return_val = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_INFO_BLOCK, sizeof(tSysInfoBlock), &tSysInfoBlock)))
      {
        printf("Error querying system information block\r\n");
      }
      else
      {
        printf("System Channel Info Block:\r\n");
        printf("==========================\r\n");
        printf("DPM Cookie       : %.4s\r\n",(char*)tSysInfoBlock.abCookie);
        printf("DPM Size         : %lu\r\n",(long unsigned int)tSysInfoBlock.ulDpmTotalSize);
        printf("Device Number    : %lu\r\n",(long unsigned int)tSysInfoBlock.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n",(long unsigned int)tSysInfoBlock.ulSerialNumber);
        printf("HW Options       : 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
               tSysInfoBlock.ausHwOptions[0], tSysInfoBlock.ausHwOptions[1],
               tSysInfoBlock.ausHwOptions[2], tSysInfoBlock.ausHwOptions[3]);
        printf("Manufacturer     : %u\r\n", tSysInfoBlock.usManufacturer);
        printf("Production Date  : %u\r\n", tSysInfoBlock.usProductionDate);
        printf("Device Class     : %u\r\n", tSysInfoBlock.usDeviceClass);
        printf("HW Revision      : %u\r\n", tSysInfoBlock.bHwRevision);
        printf("HW Compatibility : %u\r\n", tSysInfoBlock.bHwCompatibility);

        printf("License Flags 1  : 0x%08X\r\n", tSysInfoBlock.ulLicenseFlags1);
        printf("License Flags 2  : 0x%08X\r\n", tSysInfoBlock.ulLicenseFlags2);
        printf("LicenseID        : 0x%04X\r\n", tSysInfoBlock.usNetxLicenseID);
        printf("LicenseFlags     : 0x%04X\r\n", tSysInfoBlock.usNetxLicenseFlags);
        printf("==========================\r\n");
      }

      /* Try to read the System Information */
      if( CIFX_NO_ERROR != (cifx_return_val = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_INFORMATION, sizeof(tSysInfo), &tSysInfo)))
      {
        printf("Error querying system information\r\n");
      }
      else
      {
        printf("System Information:\r\n");
        printf("===================\r\n");
        printf("System Error     : 0x%08X\r\n", tSysInfo.ulSystemError);
        printf("DPM Size         : %lu\r\n",    (long unsigned int)tSysInfo.ulDpmTotalSize);
        printf("Mailbox size     : %lu\r\n",    (long unsigned int)tSysInfo.ulMBXSize);
        printf("Device Number    : %lu\r\n",    (long unsigned int)tSysInfo.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n",    (long unsigned int)tSysInfo.ulSerialNumber);
        printf("Open Count       : %lu\r\n",    (long unsigned int)tSysInfo.ulOpenCnt);
        printf("===================\r\n");
      }

      /* Try to read the System Control Block */
      if( CIFX_NO_ERROR != (cifx_return_val = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_CONTROL_BLOCK, sizeof(tControlBlock), &tControlBlock)))
      {
        printf("Error querying system control block\r\n");
      }
      else
      {
        printf("System Control Block:\r\n");
        printf("=====================\r\n");
        printf("Command COS      : 0x%08X\r\n", tControlBlock.ulSystemCommandCOS);
        printf("System Control   : 0x%08X\r\n", tControlBlock.ulSystemControl);
        printf("=====================\r\n");
      }

      printf("Waiting 2s to let cifX card calculate CPU load!\r\n");
      SleepS(2);

      /* Try to read the System Status Block */
      if( CIFX_NO_ERROR != (cifx_return_val = xSysdeviceInfo(hSys, CIFX_INFO_CMD_SYSTEM_STATUS_BLOCK, sizeof(tStatusBlock), &tStatusBlock)))
      {
        printf("Error querying system status block\r\n");
      }
      else
      {
        printf("System Status Block:\r\n");
        printf("====================\r\n");
        printf("System COS       : 0x%08X\r\n", tStatusBlock.ulSystemCOS);
        printf("System Status    : 0x%08X\r\n", tStatusBlock.ulSystemStatus);
        printf("System Error     : 0x%08X\r\n", tStatusBlock.ulSystemError);
        printf("Time since start : %lu\r\n",    (long unsigned int)tStatusBlock.ulTimeSinceStart);
        printf("CPU Load [%%]     : %.2f\r\n",  (float)tStatusBlock.usCpuLoad / 100);
        printf("====================\r\n");
      }

      unsigned long ulSendPktCount = 0;
      unsigned long ulRecvPktCount = 0;

      printf("\r\n");
      printf("Trying to read Security Eeprom:\r\n");
      printf("===============================\r\n");


      /* Read Security EEPROM zone 1*/
      xSysdeviceGetMBXState(hSys, (uint32_t*)&ulRecvPktCount, (uint32_t*)&ulSendPktCount);
      printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n",
             ulSendPktCount, ulRecvPktCount);

      HIL_SECURITY_EEPROM_READ_REQ_T tCryptoRead    = {{0}};
      HIL_SECURITY_EEPROM_READ_CNF_T tCryptoReadCnf = {{0}};

      tCryptoRead.tHead.ulDest   = HOST_TO_LE32(HIL_PACKET_DEST_SYSTEM);
      tCryptoRead.tHead.ulLen    = HOST_TO_LE32(sizeof(tCryptoRead.tData));
      tCryptoRead.tHead.ulCmd    = HOST_TO_LE32(HIL_SECURITY_EEPROM_READ_REQ);
      tCryptoRead.tData.ulZoneId = HOST_TO_LE32(1);

      if(CIFX_NO_ERROR != (cifx_return_val = xSysdevicePutPacket(hSys, (CIFX_PACKET*)&tCryptoRead, 10)))
      {
        printf("Error sending packet to device (0x%X)!\r\n", cifx_return_val);
      }
      else
      {
        printf("Send Packet (Read Crypto Flash Zone 1):\r\n");
        DumpPacket((CIFX_PACKET*)&tCryptoRead);

        xSysdeviceGetMBXState(hSys, (uint32_t*)&ulRecvPktCount, (uint32_t*)&ulSendPktCount);
        printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n",
              ulSendPktCount, ulRecvPktCount);

        if(CIFX_NO_ERROR != (cifx_return_val = xSysdeviceGetPacket(hSys, sizeof(tCryptoReadCnf), (CIFX_PACKET*)&tCryptoReadCnf, 20)) )
        {
          printf("Error getting packet from device! (cifx_return_val=0x%08X)\r\n",(unsigned int)cifx_return_val);
          ShowError(cifx_return_val);
        }
        else
        {
          printf("Received Packet (Read Crypto Flash Zone 1):\r\n");
          DumpPacket((CIFX_PACKET*)&tCryptoReadCnf);

          xSysdeviceGetMBXState(hSys, (uint32_t*)&ulRecvPktCount, (uint32_t*)&ulSendPktCount);
          printf("System Mailbox State: MaxSend = %lu, Pending Receive = %lu\r\n",
                ulSendPktCount, ulRecvPktCount);
        }
      }

      printf("===============================\r\n");
      printf("\r\n");

      xSysdeviceClose(hSys);
    }

    xDriverClose(cifx_driver);
  }

  printf(" State = 0x%08X\r\n", (unsigned int)cifx_return_val);
  printf("----------------------------------------------------\r\n");

  return cifx_return_val;
}

/*****************************************************************************/
/*! Function to demonstrate communication channel functionality
*   Packet Transfer and I/O Data exchange
*   \return CIFX_NO_ERROR on success                                         */
/*****************************************************************************/
int32_t ChannelDemo(void)
{
#ifdef DEBUG
	printf("%s() called\n", __FUNCTION__);
#endif
 // CIFXHANDLE cifx_driver = NULL;
  cifx_return_val = xDriverOpen(&cifx_driver);

  printf("---------- Communication Channel demo ----------\r\n");

  if(CIFX_NO_ERROR == cifx_return_val)
  {
    /* Driver/Toolkit successfully opened */
    cifx_return_val = xChannelOpen(cifx_driver, CIFX_DEV, 0, &cifx_channel);

    if(CIFX_NO_ERROR != cifx_return_val)
    {
      printf("Error opening Channel!");

    }
    else
    {
      CHANNEL_INFORMATION tChannelInfo = {{0}};

      /* Channel successfully opened, so query basic information */
      if( CIFX_NO_ERROR != (cifx_return_val = xChannelInfo(cifx_channel, sizeof(CHANNEL_INFORMATION), &tChannelInfo)))
      {
        printf("Error querying system information block\r\n");
      }
      else
      {
        printf("Communication Channel Info:\r\n");
        printf("Device Number    : %lu\r\n",(long unsigned int)tChannelInfo.ulDeviceNumber);
        printf("Serial Number    : %lu\r\n",(long unsigned int)tChannelInfo.ulSerialNumber);
        printf("Firmware         : %s\r\n", tChannelInfo.abFWName);
        printf("FW Version       : %u.%u.%u build %u\r\n",
                tChannelInfo.usFWMajor,
                tChannelInfo.usFWMinor,
                tChannelInfo.usFWRevision,
                tChannelInfo.usFWBuild);
        printf("FW Date          : %02u/%02u/%04u\r\n",
                tChannelInfo.bFWMonth,
                tChannelInfo.bFWDay,
                tChannelInfo.usFWYear);

        printf("Mailbox Size     : %lu\r\n",(long unsigned int)tChannelInfo.ulMailboxSize);
      }
      xChannelClose(cifx_channel);
    }
    xDriverClose(cifx_driver);
  }
  printf(" State = 0x%08X\r\n", (unsigned int)cifx_return_val);
  printf("----------------------------------------------------\r\n");

  return cifx_return_val;
}

void CloseCommunication()
{
  uint32_t actual_state;
	cifx_return_val =	xChannelBusState(cifx_channel, CIFX_BUS_STATE_OFF, &actual_state, 10000);
	if (0 != cifx_return_val)
	{
		printf("Error while invoking ChannelBusState off!\n");
		ShowError(cifx_return_val);
	}
  xChannelClose(cifx_channel);
	xDriverClose(cifx_driver);
}

/*****************************************************************************/
/*! Displays a hex dump on the debug console (16 bytes per line)
*   \param pbData     Pointer to dump data
*   \param ulDataLen  Length of data dump                                    */
/*****************************************************************************/
void DumpData(unsigned char* pbData, unsigned long ulDataLen)
{
  unsigned long ulIdx = 0;
  for(ulIdx = 0; ulIdx < ulDataLen; ++ulIdx)
  {
    if(0 == (ulIdx % 16))
      printf("\n");

    printf("%02X ", pbData[ulIdx]);
  }
  printf("\n");
}

/*****************************************************************************/
/*! Displays cifX error
*   \param lError     Error code
*****************************************************************************/
inline void ShowError(int32_t error)
{
  if(error != CIFX_NO_ERROR)
  {
    char szError[512] ={0};
    xDriverGetErrorDescription(error,  szError, sizeof(szError));
    printf("Error: 0x%08X, <%s>\n", (unsigned int)error, szError);
  }
}

/*****************************************************************************/
/*! Function to demonstrate event handling
*   \param  cifx_driver  Handle to cifX driver
*   \return CIFX_NO_ERROR on success                                         */
/*****************************************************************************/
void RegisterInEvent(PFN_NOTIFY_CALLBACK function, void* pvUser)
{
  // Register for events
  if ((CIFX_NO_ERROR != (cifx_return_val = xChannelRegisterNotification(cifx_channel, CIFX_NOTIFY_PD0_IN, function, pvUser))))
  {
    // Failed to register one of the events */
    // Read driver error description
    printf("Error while register input event callback function\r\n");
    ShowError(cifx_return_val);
  }
  else
  {
    /* Get actual host state */
    if ((cifx_return_val = xChannelHostState(cifx_channel, CIFX_HOST_STATE_READ, &ulState, 0L)) != CIFX_NO_ERROR)
    {
      // Read driver error description
      ShowError(cifx_return_val);
    }

    /* Set host ready */
    if ((cifx_return_val = xChannelHostState(cifx_channel, CIFX_HOST_STATE_READY, NULL, 2000L)) != CIFX_NO_ERROR)
    {
      // Read driver error description
      ShowError(cifx_return_val);
    }
  }
}

//解除注册
void ECatMasterDev_UnRegisterInEvent(CIFXHANDLE cifx_channel)
{
	long cifx_return_val = 0;

	cifx_return_val = xChannelUnregisterNotification(cifx_channel, CIFX_NOTIFY_PD0_IN);
	if (CIFX_NO_ERROR != cifx_return_val) {
		printf("Error while unregister input event callback function\r\n");
		ShowError(cifx_return_val);
	}
}

//zm 2020.2.19  用于中断
static unsigned long g_ulIdCnt = 0;
/***************************************************************************
* Helper functions
****************************************************************************/
static long SendPacket(CIFXHANDLE cifx_channel, CIFX_PACKET *ptPacket)
{
	int32_t cifx_return_val = CIFX_NO_ERROR;

	ptPacket->tHeader.ulId = g_ulIdCnt;
	ptPacket->tHeader.ulDest = 0x20;
	ptPacket->tHeader.ulState = 0;

	cifx_return_val = xChannelPutPacket(cifx_channel, ptPacket, 1000);

	if (cifx_return_val) {
		printf("Error during xChannelPutPacket(cifx_channel, &ptPacket, 1000)\r\n");
		ShowError(cifx_return_val);
	}
	return cifx_return_val;
}

static long RecvPacket(CIFXHANDLE cifx_channel, CIFX_PACKET *ptPacket)
{
	int32_t cifx_return_val = CIFX_NO_ERROR;
	uint32_t ulRecvCnt = 0, ulSendCnt = 0, ulWait = 20;

	while (--ulWait) {
		SleepMS(500);
		cifx_return_val = xChannelGetMBXState(cifx_channel, &ulRecvCnt, &ulSendCnt);
		if (cifx_return_val) 
		{
			printf("Error during xChannelGetMBXState(cifx_channel,&ulRecvCnt, &ulSendCnt)\r\n");
		}

		if (ulRecvCnt)
		{
			cifx_return_val = xChannelGetPacket(cifx_channel, sizeof(*ptPacket), ptPacket, 0);
			if (cifx_return_val) 
			{
				printf("Error during xChannelGetPacket(cifx_channel, sizeof(ptPacket), &ptPacket, 0)\r\n");
			}
			else 
			{
				if (ptPacket->tHeader.ulId == g_ulIdCnt) 
				{
					break;
				}
				else 
				{
					printf("Warning a recived packet is out of sequence!\n");
				}
			}
		}
	}

	if (ulWait) {
		if (ptPacket->tHeader.ulState) {
			printf("Packet Error: 0x%08X\r\n", ptPacket->tHeader.ulState);
			cifx_return_val = (int32_t)ptPacket->tHeader.ulState;
		}
	}
	return cifx_return_val;
}

static long SendRecv(CIFXHANDLE cifx_channel, CIFX_PACKET *ptPacket) {
	long cifx_return_val = 0;
	g_ulIdCnt += 1;

	cifx_return_val = SendPacket(cifx_channel, ptPacket);
	if (cifx_return_val == 0) {
		cifx_return_val = RecvPacket(cifx_channel, ptPacket);
	}
	return cifx_return_val;
}

//设置中断模式
#define RCX_IO_MODE_OEM_1 (0x5)
#define RCX_IO_MODE_OEM_2 (0x6)
long ECatMasterDev_SetSyncMode(CIFXHANDLE cifx_channel)
{
	CIFX_PACKET tPack = { 0 };
	RCX_SET_HANDSHAKE_CONFIG_REQ_T *ptPacket = (RCX_SET_HANDSHAKE_CONFIG_REQ_T*)&tPack;

	ptPacket->tHead.ulCmd = RCX_SET_HANDSHAKE_CONFIG_REQ;
	ptPacket->tHead.ulLen = RCX_SET_HANDSHAKE_CONFIG_REQ_SIZE;

	ptPacket->tData.bPDInHskMode = RCX_IO_MODE_OEM_1;
	ptPacket->tData.bPDInSource = 0;
	ptPacket->tData.usPDInErrorTh = 1;

	ptPacket->tData.bPDOutHskMode = RCX_IO_MODE_BUFF_HST_CTRL;
	ptPacket->tData.bPDOutSource = 0;
	ptPacket->tData.usPDOutErrorTh = 1;

	ptPacket->tData.bSyncHskMode = RCX_SYNC_MODE_OFF;
	ptPacket->tData.bSyncSource = 0;
	ptPacket->tData.usSyncErrorTh = 0;

	ptPacket->tData.aulReserved[0] = 0;
	ptPacket->tData.aulReserved[1] = 0;

	return SendRecv(cifx_channel, &tPack);
}

void AckownledgeSyn()
{
  static uint32_t error_count = 0;
  cifx_return_val = xChannelSyncState(cifx_channel, CIFX_SYNC_ACKNOWLEDGE_CMD, 10, &error_count);
  if(cifx_return_val != CIFX_NO_ERROR)
  {
    printf("xChannelSyncState Error: 0x%08X\r\n", cifx_return_val);
  }
}
