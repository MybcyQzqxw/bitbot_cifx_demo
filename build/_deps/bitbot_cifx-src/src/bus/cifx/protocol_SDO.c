#include "cifx/cifXUser.h"
#include "bitbot_cifx/bus/cifx/protocol_SDO.h"
#include "bitbot_cifx/bus/cifx/cifx_driver.h"
#include "bitbot_kernel/utils/time_func.h"

#include <cifx/cifXErrors.h>
#include <unistd.h>
#include <stdio.h>

uint32_t error_code;

/*****************************************************************************/
//下发读SDO包命令
//S_ID:电机序号；  S_ID：电机号；SDO_Up_Data:SDO读命令数据包--从机结点号、索引、子索引
//返回值：错误代码
// 祝 敏
//2019.06.11
/*****************************************************************************/
uint32_t SDOUploadCMD( uint8_t S_ID,CIFXHANDLE cifx_channel, CIFX_PACKET_SDO_Upload_Data SDO_Up_Data)
{
  CIFX_PACKET SDO_Up_PACKETCMD;
  CIFX_PACKET_HEADER SendPacketHead; 
  //包头10个32bits的数据
  SendPacketHead.ulDest = 0x00000020;     //通过通道邮箱传递数据包
  SendPacketHead.ulSrc=0;
  SendPacketHead.ulDestId=0;
  SendPacketHead.ulSrcId=0;
  SendPacketHead.ulLen=12;                //包数据部分字节数
  SendPacketHead.ulId=0;
  SendPacketHead.ulState=0;
  SendPacketHead.ulCmd=0x00650006;        //读取从站数据命令
  SendPacketHead.ulExt=0;
  SendPacketHead.ulRout=0;
  //包数据部分：node（代表不同的从站，从配置文件中可以看到固定值）+索引+子索引
  //node 32bit
  SDO_Up_Data.ulNodeID=0x0100+S_ID;
  SDO_Up_PACKETCMD.abData[0]=  SDO_Up_Data.ulNodeID&0x000000ff;
  SDO_Up_PACKETCMD.abData[1]=( SDO_Up_Data.ulNodeID&0x0000ff00)>>8;
  SDO_Up_PACKETCMD.abData[2]=( SDO_Up_Data.ulNodeID&0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[3]=( SDO_Up_Data.ulNodeID&0xff000000) >> 24;
  //索引32bit
  SDO_Up_PACKETCMD.abData[4]=  SDO_Up_Data.ulIndex&0x000000ff;
  SDO_Up_PACKETCMD.abData[5]=( SDO_Up_Data.ulIndex&0x0000ff00) >> 8;
  SDO_Up_PACKETCMD.abData[6]=( SDO_Up_Data.ulIndex&0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[7]=( SDO_Up_Data.ulIndex&0xff000000) >> 24;
  //子索引32bit
  SDO_Up_PACKETCMD.abData[8] =  SDO_Up_Data.ulSubIndex&0x000000ff;
  SDO_Up_PACKETCMD.abData[9] =( SDO_Up_Data.ulSubIndex&0x0000ff00) >> 8;
  SDO_Up_PACKETCMD.abData[10]=( SDO_Up_Data.ulSubIndex&0x00ff0000) >> 16;
  SDO_Up_PACKETCMD.abData[11]=( SDO_Up_Data.ulSubIndex&0xff000000) >> 24;

  SDO_Up_PACKETCMD.tHeader=SendPacketHead;
  return(xChannelPutPacket ( cifx_channel, &SDO_Up_PACKETCMD, 30));    //���ش������
}

/*****************************************************************************/
//发读命令后读取数据
//S_ID:电机序号； DataLength：接收数据字节数
//返回读取的数据
/*****************************************************************************/
uint32_t SDOReadData( uint8_t S_ID, CIFXHANDLE cifx_channel, uint8_t DataLength)
{
  CIFX_PACKET SDO_Download_PACKET;
  CIFX_PACKET_HEADER SDO_DownloadtHead;
 // uint8_t DataLength;
  uint32_t PacketLethTemp;
  uint8_t  i, abData2;
  uint32_t abData1 = 0;
  PacketLethTemp = (16 + DataLength) + 4 * 10;     //全部接收长度（加包头10*4）,数据：16+DataLength
  error_code=xChannelGetPacket(cifx_channel, PacketLethTemp, &SDO_Download_PACKET, 100);
  if(error_code==0)
  { 
    SDO_DownloadtHead= SDO_Download_PACKET.tHeader;
    if (SDO_DownloadtHead.ulCmd == 0x00650007)
    {
	  DataLength = SDO_Download_PACKET.abData[12] | (SDO_Download_PACKET.abData[13] << 8) | (SDO_Download_PACKET.abData[14] << 16) | (SDO_Download_PACKET.abData[15] << 24);
	  abData1 = 0;
	  //取数
	  for (i = DataLength; i >1; i--)
	  {
		  abData2 = SDO_Download_PACKET.abData[15 + i];
		  abData1 = (abData1|abData2 )<< 8;
	    }
	  abData2 = SDO_Download_PACKET.abData[16];
	  abData1 = abData1 | abData2;
	}
	return abData1;
  }
  else
  {
	  printf("SDOReadData Error error_code: 0x%08x\n", error_code);   //以十六进制输出
	  return -1;
  }
}
/*****************************************************************************/
//SDO读取数据
//S_ID:电机序号；SDO_Up_Data_Temp: SDO读命令数据包--从机结点号、索引、子索引;DataLength：接收数据字节数
//返回读取的数据
/*****************************************************************************/
uint32_t SDORead(uint8_t S_ID, CIFXHANDLE cifx_channel, CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp, uint8_t DataLength)
{
	uint32_t mm;
	//发读命令
	error_code = SDOUploadCMD(S_ID, cifx_channel, SDO_Up_Data_Temp);
	SleepMS(200);
	if (error_code == 0)      //发读命令正常
	{
		mm = SDOReadData(S_ID, cifx_channel, DataLength);
		while (error_code == 0x800c0019)
		{
			SleepMS(10);
			mm = SDOReadData(S_ID, cifx_channel, DataLength);          //读SDO数据
		}
		return(mm);
	}
	else
	{
		printf("SDOReadCMD Error error_code: %x/n", error_code);
		return -1;
	}
}
/*****************************************************************************/
//通过SDO包更新驱动器对象字典
//S_ID:电机序号； SDO__Download_Data:下发SDO数据包--结点号、索引、子索引、数据长度、数据
/*****************************************************************************/
uint32_t SDODownload(uint8_t S_ID, CIFXHANDLE cifx_channel, CIFX_PACKET_SDO_Download_Data SDO_Download_Data)
{
	uint32_t DataLength;
	CIFX_PACKET SDO_Download_PACKET;
	CIFX_PACKET_HEADER SendPacketHead;
	uint8_t   i;
	//包头
	DataLength = SDO_Download_Data.ulDataCnt;    //数据长度
	SendPacketHead.ulDest = 0x00000020;
	SendPacketHead.ulSrc = 0;
	SendPacketHead.ulDestId = 0;
	SendPacketHead.ulSrcId = 0;
	SendPacketHead.ulLen = 16+ DataLength;      //数据包长度
	SendPacketHead.ulId = 0;
	SendPacketHead.ulState = 0;
	SendPacketHead.ulCmd = 0x00650008;          //写入从站数据命令
	SendPacketHead.ulExt = 0;
	SendPacketHead.ulRout = 0;
	//包数据部分
	//node 32bit
	SDO_Download_Data.ulNodeID = 0x0100 + S_ID;
	SDO_Download_PACKET.abData[0] = SDO_Download_Data.ulNodeID & 0x000000ff;
	SDO_Download_PACKET.abData[1] = (SDO_Download_Data.ulNodeID & 0x0000ff00) >> 8;
	SDO_Download_PACKET.abData[2] = (SDO_Download_Data.ulNodeID & 0x00ff0000) >> 16;
	SDO_Download_PACKET.abData[3] = (SDO_Download_Data.ulNodeID & 0xff000000) >> 24;
	//索引32bit
	SDO_Download_PACKET.abData[4] = SDO_Download_Data.ulIndex & 0x000000ff;
	SDO_Download_PACKET.abData[5] = (SDO_Download_Data.ulIndex & 0x0000ff00) >> 8;
	SDO_Download_PACKET.abData[6] = (SDO_Download_Data.ulIndex & 0x00ff0000) >> 16;
	SDO_Download_PACKET.abData[7] = (SDO_Download_Data.ulIndex & 0xff000000) >> 24;
	//子引32bit
	SDO_Download_PACKET.abData[8] =   SDO_Download_Data.ulSubIndex & 0x000000ff;
	SDO_Download_PACKET.abData[9] =  (SDO_Download_Data.ulSubIndex & 0x0000ff00) >> 8;
	SDO_Download_PACKET.abData[10] = (SDO_Download_Data.ulSubIndex & 0x00ff0000) >> 16;
	SDO_Download_PACKET.abData[11] = (SDO_Download_Data.ulSubIndex & 0xff000000) >> 24;
	//数据长度32bit
	SDO_Download_PACKET.abData[12] =  SDO_Download_Data.ulDataCnt & 0x000000ff;
	SDO_Download_PACKET.abData[13] = (SDO_Download_Data.ulDataCnt & 0x0000ff00) >> 8;
	SDO_Download_PACKET.abData[14] = (SDO_Download_Data.ulDataCnt & 0x00ff0000) >> 16;
	SDO_Download_PACKET.abData[15] = (SDO_Download_Data.ulDataCnt & 0xff000000) >> 24;
	
	for (i = 0; i<DataLength; i++)
		SDO_Download_PACKET.abData[16 + i] = *(SDO_Download_Data.abSdoData+i);
	//包头
	SDO_Download_PACKET.tHeader = SendPacketHead;
	error_code = xChannelPutPacket(cifx_channel, &SDO_Download_PACKET, 0);
	if(error_code != CIFX_NO_ERROR)
	{
		printf("SDODownload Error error_code: 0x%x\n", error_code);   //以十六进制输出
	}
	return error_code;
}



//设置插值周期子索引1：插值时间值；子索引2：插值时间单位默认-3:10的-3次方表示ms;子索引1*10的子索引2次方
//从站结点号、索引、子索引
void SetInterpolationCycle(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	uint8_t  DataTemp[1];
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
	SDO_Download_Data_Temp.ulIndex = 0x60C2;               //索引
	SDO_Download_Data_Temp.ulSubIndex = 1;                 //子索引 周期数 
	SDO_Download_Data_Temp.ulDataCnt = 1;                  //写入数据字节数
	DataTemp[0] = InterpolationCycleValue;                  //2,0xfc--200us; 1,0xfd---1ms
	SDO_Download_Data_Temp.abSdoData = DataTemp;           //写入数据指针
	usleep(500000);
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	if(error_code != 0)
		printf("Error Set Interpolation Cycle Value:%d\n", error_code);
	//获取写入数据后从机返回的数据
	DataLength = 1;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	usleep(500000);
	//写插值周期子索引2
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
	SDO_Download_Data_Temp.ulIndex = 0x60C2;               //索引
	SDO_Download_Data_Temp.ulSubIndex = 2;                 //子索引 插值时间单位
	SDO_Download_Data_Temp.ulDataCnt = 1;                  //写入数据字节数
	DataTemp[0] = InterpolationCycleUint;                 //0xfc--100us;0xfd--ms;
	SDO_Download_Data_Temp.abSdoData = DataTemp;           //写入数据指针
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	if (error_code != 0)
		printf("Error Set Interpolation Cycle unit:%d\n", error_code);
	usleep(500000);
	//获取写入数据后从机返回的数据
	DataLength = 1;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	if (error_code != 0)
		printf("Error Read Interpolation Cycle unit  Code:%d\n", error_code);
}
//读插值周期值
uint32_t ReadInterpolationCycleValue(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //从站结点号
	SDO_Up_Data_Temp.ulIndex = 0x60C2;            //索引  ;
	SDO_Up_Data_Temp.ulSubIndex = 1;              //子索引
	DataLength = 4;                               //数据的字节数
	ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
	if (error_code != 0)
	{
		printf("Error Read Interpolation Cycle unit  Code:%d\n", error_code);
		return 0;
	}
	else
		return ReciveData;
}
//读插值周期单位
uint32_t ReadInterpolationCycleUnit(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //从站结点号
	SDO_Up_Data_Temp.ulIndex = 0x60C2;            //索引  ;
	SDO_Up_Data_Temp.ulSubIndex = 2;              //子索引
	DataLength = 4;                               //数据的字节数
	ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
	if (error_code != 0)
	{
		printf("Error Read Interpolation Cycle unit  Code:%d\n", error_code);
		return 0;
	}
	else
	 return ReciveData;
}
//设置0x605D --Halt option code	
void SetHaltoption(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength, i;
	uint8_t  DataTemp[2];
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
	uint16_t code = 2; //Halt option code	
//从站结点号、索引、子索引
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
	SDO_Download_Data_Temp.ulIndex = 0x605d;               //索引
	SDO_Download_Data_Temp.ulSubIndex = 0;                 //子索引
	SDO_Download_Data_Temp.ulDataCnt = 2;                  //写入数据字节数
	for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
	{
		DataTemp[i] = code & 0x00ff;
		code = code >> 8;
	}
	SDO_Download_Data_Temp.abSdoData = DataTemp;         //写入数据指针
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	usleep(500000);
	//获取写入数据后从机返回的数据
	DataLength = 2;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
}


//设置位置运动方式Positioning option code 主要针对PP模式
void SetPositionOption(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	uint32_t ReciveData;
	uint8_t  DataLength,i;
	uint8_t  DataTemp[2];
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
	uint16_t code = PositionOptionCodeBuffer; //不用bit4触发，0x20只要数据放入buffer触发,0x10数据到位触发
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
	SDO_Download_Data_Temp.ulIndex = 0x60F2;               //索引
	SDO_Download_Data_Temp.ulSubIndex = 0;                 //子索引
	SDO_Download_Data_Temp.ulDataCnt = 2;                  //写入数据字节数
	for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
	{
		DataTemp[i] = code & 0x000000ff;
		code = code >> 8;
	}
	SDO_Download_Data_Temp.abSdoData = DataTemp;         //写入数据指针
	usleep(5000000);
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	if (error_code != 0)
		printf("Error Positioning option code:%d\n", error_code);
	usleep(5000000);
	//获取写入数据后从机返回的数据
	DataLength = 2;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	if (error_code != 0)
		printf("Error Read Positioning option code:%d\n", error_code);
}
//读位置运动方式Positioning option code 主要针对PP模式
uint32_t ReadPositionOption(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //从站结点号
	SDO_Up_Data_Temp.ulIndex = 0x60F2;            //索引  Positioning option code 
	SDO_Up_Data_Temp.ulSubIndex = 0;              //子索引
	DataLength = 2;                               //数据的字节数
	ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
	if (error_code != 0)
	{
		printf("Error Read  Positioning option code:%d\n", error_code);
		return 0;
	}
	else
		return ReciveData;
}


//设置0x6075--Motor Send Rate Current	  电流输入值1000000---A，1000--mA
void SetMotorRateCurrent(uint32_t S_ID, uint32_t rate)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	uint32_t  DataTemp;
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
	//从站结点号、索引、子索引
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
	SDO_Download_Data_Temp.ulIndex = 0x6075;               //索引
	SDO_Download_Data_Temp.ulSubIndex = 0;                 //子索引
	SDO_Download_Data_Temp.ulDataCnt = 4;                  //写入数据字节数
	SDO_Download_Data_Temp.abSdoData = (uint8_t*)&rate; //DataTemp;       //写入数据指针
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	SleepMS(200);
	DataLength = 4;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
}

//读0x6075--Motor Send Rate Current	 电流输入值1000000---A，1000--mA
	uint32_t ReadMotorRateCurrent(uint32_t S_ID)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		// DataLength = 4;
		// ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
		//sleep(500);
		//通过SDO读数据
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //从站结点号
		SDO_Up_Data_Temp.ulIndex = 0x6075;            //索引 
		SDO_Up_Data_Temp.ulSubIndex = 0;              //子索引
		DataLength = 4;                              //数据的字节数
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}


	//设置0x6077--Motor Read Rate Current	  电流输入值1000000---A，1000--mA
	void SetMotorReadRateCurrent(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
	//	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		uint32_t  DataTemp;
		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x6077;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 4;                  //д�������ֽ���
		DataTemp = 1000;                                             //д������10000--10mA��1000--1mA	
		SDO_Download_Data_Temp.abSdoData = (uint8_t*)&DataTemp;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
		//2020.05.20
		usleep(100000);
		DataLength = 4;
		ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	}

	//读0x6077--Motor Read Rate Current	 电流输入值1000000---A，1000--mA
	uint32_t ReadMotorReadRateCurrent(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		/*DataLength = 4;
		ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
		sleep(500);*/
		//通过SDO读数据
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x6077;            //���� 
		SDO_Up_Data_Temp.ulSubIndex = 0;              //������
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}


	//设置0x6076--Motor Send Rate Torque	 力矩模式输入值1---A，1000--mA
	void SetMotorRateTorque(uint32_t S_ID, uint32_t ratio)
	{
		//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;

		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x6076;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 4;                  //写入数据字节数
		SDO_Download_Data_Temp.abSdoData = (uint8_t*)&ratio;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
		SleepMS(200);

		ReciveData = SDOReadData(S_ID, cifx_channel, sizeof(ratio));
	}

	//读0x6076--Motor Send Rate Torque	 力矩模式输入值1000000---A，1000--mA
	uint32_t ReadMotorRateTorque(uint32_t S_ID)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;

		//通过SDO读数据
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x6076;            //���� 
		SDO_Up_Data_Temp.ulSubIndex = 0;              //������

		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, sizeof(ReciveData));
		return ReciveData;
	}


	//设置0x6078--Motor Read Rate Torque	 力矩模式输入值1000000---A，1000--mA
	void SetMotorReadRateTorque(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength, i;
		uint32_t  DataTemp;
		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x6078;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 4;                  //д�������ֽ���
		DataTemp = 1000;                                             //д������д������10000--10mA��1000--mA
		SDO_Download_Data_Temp.abSdoData = (uint8_t*)&DataTemp;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
		//2020.05.20
		usleep(100000);
		DataLength = 4;
		ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	}

	//读0x6078--Motor Read Rate Torque	 力矩模式输入值1000000---A，1000--mA
	uint32_t ReadMotorReadRateTorque(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		/*DataLength = 4;
		ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
		sleep(500);*/
		//通过SDO读数据
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x6078;            //���� 
		SDO_Up_Data_Temp.ulSubIndex = 0;              //������
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}



	//设置0x0x6502--驱动器支持的模式选择
	void SetSupDriveModes(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  i;
		uint8_t  DataTemp[4];
		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		uint32_t code = 0x3ED;  
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x6502;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 4;                  //д�������ֽ���
		for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
		{
			DataTemp[i] = code & 0x000000ff;
			code = code >> 8;
		}
		SDO_Download_Data_Temp.abSdoData = DataTemp;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	}

	//读0x6502--驱动器支持的模式选择
	uint32_t ReaSupDriveModes(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		// 设置0x6502--Supportde drive modes   驱动器支持的模式选择
		//通过SDO读数据
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x6502;            //����
		SDO_Up_Data_Temp.ulSubIndex = 0;              //������
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}

	//读MF(Elmo-OX313D)
	uint32_t ReadMF(uint8_t S_ID)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x313D;            //����  0x313D  MF;
		SDO_Up_Data_Temp.ulSubIndex = 1;              //������ ��1��ʼ
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}
	//读EC(Elmo-OX306A)
	uint32_t ReadEC(uint8_t S_ID)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x306A;            //����  0x306A  EC;
		SDO_Up_Data_Temp.ulSubIndex = 1;              //������ ��1��ʼ
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}

	//读EE(Elmo-OX306C)
	uint32_t ReadEE(uint8_t S_ID)
	{
		CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  DataLength;
		SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
		SDO_Up_Data_Temp.ulIndex = 0x306C;            //����  0x306C EE;
		SDO_Up_Data_Temp.ulSubIndex = 1;              //������ ��1��ʼ
		DataLength = 4;                               //���ݵ��ֽ���
		ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
		return ReciveData;
	}


	//设置0x0x6007--Abort connect option code  驱动器心跳停止功能取消
	void SetHeartBeat(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  i;
		uint8_t  DataTemp[1];
		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x6007;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 1;                  //д�������ֽ���
		uint8_t mm = 0;                                            //д������	
		for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
		{
			DataTemp[i] = mm & 0xff;
			mm = mm >> 8;
		}
		SDO_Download_Data_Temp.abSdoData = DataTemp;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	}
	//设置0x0x3146--MO=1 清除MF错误码
	void SetM0(uint8_t S_ID, CIFXHANDLE cifx_channel)
	{
		//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
		//uint32_t error_code;
		uint32_t ReciveData;
		uint8_t  i,mm;
		uint8_t  DataTemp[1];
		CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
		//从站结点号、索引、子索引
		SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;
		SDO_Download_Data_Temp.ulIndex = 0x3146;               //����
		SDO_Download_Data_Temp.ulSubIndex = 0;                 //������
		SDO_Download_Data_Temp.ulDataCnt = 1;                  //д�������ֽ���
		mm = 0;                                            //д������	
		for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
		{
			DataTemp[i] = mm & 0x000000ff;
			mm = mm >> 8;
		}
		SDO_Download_Data_Temp.abSdoData = DataTemp;         //д������ָ��
		error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	}



/*****************************************************************************/
//SDO初始化
/*****************************************************************************/
void SDODInit(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	uint32_t val;
	SetMotorRateTorque(S_ID, 10000);
	SleepMS(50);
	val = ReadMotorRateTorque(S_ID);
	printf("RateTorque:%d\n",val);
	SleepMS(50);
	SetMotorRateCurrent(S_ID, 10000);
	SleepMS(50);
	val = ReadMotorRateCurrent(S_ID);
	printf("RateCurrent:%d\n",val);
}
//  ok
void ZeroClearSDOD(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	uint8_t  DataTemp[4];
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;
  SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;       //ÿ���������ǹ̶��ģ��������ļ�
	SDO_Download_Data_Temp.ulIndex = 0x319D;               //����
	SDO_Download_Data_Temp.ulSubIndex = 1;                 //������
	SDO_Download_Data_Temp.ulDataCnt = 4;                  //д�������ֽ���
	DataTemp[0] = 0;
	DataTemp[1] = 0;
	DataTemp[2] = 0;
	DataTemp[3] = 0;
	SDO_Download_Data_Temp.abSdoData = DataTemp;           //д������ָ��
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	usleep(150000);
	//获取写入数据后从机返回的数据
	DataLength = 4;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	usleep(100000);
	
}
//读零点
uint32_t ReadZero(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
	SDO_Up_Data_Temp.ulIndex = 0x319D;            //����  0x319��ǰλ��;
	SDO_Up_Data_Temp.ulSubIndex = 1;              //������
	DataLength = 4;                               //���ݵ��ֽ���
	ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
	return ReciveData;
  }
//读UI
uint32_t ReadUi(uint8_t S_ID, CIFXHANDLE cifx_channel)
{
	CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength;
	SDO_Up_Data_Temp.ulNodeID = 0x0100 + S_ID;    //��վ����
	SDO_Up_Data_Temp.ulIndex = 0x3210;            //����  0x3210  UI;
	SDO_Up_Data_Temp.ulSubIndex = 1;              //������ ��1��ʼ
	DataLength = 4;                               //���ݵ��ֽ���
	ReciveData = SDORead(S_ID, cifx_channel, SDO_Up_Data_Temp, DataLength);
	return ReciveData;
}

//写ui
void WriteUiSDOD(uint8_t S_ID, CIFXHANDLE cifx_channel, uint32_t datatemp)
{
	//CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp;
	//uint32_t error_code;
	uint32_t ReciveData;
	uint8_t  DataLength, i;
	uint8_t  DataTemp[4];
	CIFX_PACKET_SDO_Download_Data  SDO_Download_Data_Temp;

	//设置插值周期子索引1：插值时间值；子索引2：插值时间单位默认-3:10的-3次方表示ms;子索引1*10的子索引2次方
	//从站结点号、索引、子索引
	SDO_Download_Data_Temp.ulNodeID = 0x0100 + S_ID;       //ÿ���������ǹ̶��ģ��������ļ�
	SDO_Download_Data_Temp.ulIndex = 0x3210;               //����
	SDO_Download_Data_Temp.ulSubIndex = 1;                 //������
	SDO_Download_Data_Temp.ulDataCnt = 4;                  //д�������ֽ���
	uint32_t mm = datatemp;
	for (i = 0; i < SDO_Download_Data_Temp.ulDataCnt; i++)
	{
		DataTemp[i] = mm & 0x000000ff;
		mm = mm >> 8;
	}                                     //200,0xfc--200us; 1,0xfd---1ms
	SDO_Download_Data_Temp.abSdoData = DataTemp;           //д������ָ��
	error_code = SDODownload(S_ID, cifx_channel, SDO_Download_Data_Temp);
	//sleep(500);
	usleep(1000000);
	//获取写入数据后从机返回的数据
	DataLength = 4;
	ReciveData = SDOReadData(S_ID, cifx_channel, DataLength);
	//sleep(500);
	usleep(1000000);
}

void SDODInit_All(CIFXHANDLE cifx_channel, uint16_t Mot_NumTemp)
{
	uint16_t i;
	for (i = 0; i<Mot_NumTemp; i++)
	{
		printf("SDODInit %d\n",i);
		SDODInit(i, cifx_channel);
	}
	printf("SDODInit Finished\n");
}
