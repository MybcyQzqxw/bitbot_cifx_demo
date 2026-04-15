#ifndef _PROTOCOL_SDO_H
#define _PROTOCOL_SDO_H

#include "cifx/cifXUser.h"

#define   InterpolationPeriodIndx       0x60c2
#define   InterpolationPeriodSubIndx    2

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//写SDO数据包
typedef  struct EherCar_Master_Packet_SDO_Download_Req_Data_Ttag
{
 uint32_t ulNodeID;
 uint32_t ulIndex;
 uint32_t ulSubIndex;
 uint32_t ulDataCnt;
 uint8_t  *abSdoData;
 } CIFX_PACKET_SDO_Download_Data;

/*typedef struct Master_Package_SDO_Download_Req_Ttag
{
  CIFX_PACKET_HEADER  tHeader;
  CIFX_PACKET_SDO_Download_Data;
 } CIFX_SDO_Download_PACKET;
 */
//读SDO数据包
typedef  struct EherCar_Master_Packet_SDO_Upload_Req_Data_Ttag
{
 uint32_t ulNodeID;
 uint32_t ulIndex;
 uint32_t ulSubIndex;
  } CIFX_PACKET_SDO_Upload_Data;

/*typedef struct Master_Package_SDO_Upload_Req_Ttag
{
  CIFX_PACKET_HEADER  tHeader;
  CIFX_PACKET_SDO_Upload_Data;
 } CIFX_SDO_Upload_PACKET;*/
 /*****************************************************************************/


 //下发读SDO包命令
 //S_ID:电机序号；  S_ID：电机号；SDO_Up_Data:SDO数据包--结点号、索引、子索引
 /*****************************************************************************/
//uint32_t SDOUploadCMD(uint8_t S_ID, CIFXHANDLE hChannel, CIFX_PACKET_SDO_Upload_Data SDO_Up_Data);
/*****************************************************************************/
//发读命令后读取数据
//S_ID:电机序号； PacketLeth：接收包缓冲器长度
/*****************************************************************************/
uint32_t SDOReadData(uint8_t S_ID, CIFXHANDLE hChannel, uint8_t DataLength);

uint32_t SDORead(uint8_t S_ID, CIFXHANDLE hChannel, CIFX_PACKET_SDO_Upload_Data SDO_Up_Data_Temp, uint8_t DataLength);
/*****************************************************************************/
//通过SDO包更新驱动器对象字典
//S_ID:电机序号； SDO__Download_Data:下发SDO数据包--结点号、索引、子索引、数据长度、数据
/*****************************************************************************/
uint32_t SDODownload(uint8_t S_ID, CIFXHANDLE hChannel, CIFX_PACKET_SDO_Download_Data SDO_Download_Data);
/*****************************************************************************/
//SDO测试
/*****************************************************************************/
//void SDODTest(uint8_t S_ID, CIFXHANDLE hChannel);

/*****************************************************************************/
//SDO初始化
/*****************************************************************************/
void SDODInit(uint8_t S_ID, CIFXHANDLE hChannel);
//清零
void ZeroClearSDOD(uint8_t S_ID, CIFXHANDLE hChannel);
//读零点
uint32_t ReadZero(uint8_t S_ID, CIFXHANDLE hChannel);
//读UI
uint32_t  ReadUi(uint8_t S_ID, CIFXHANDLE hChannel);
//写ui
void WriteUiSDOD(uint8_t S_ID, CIFXHANDLE hChannel, uint32_t datatemp);
//插值周期值2,0xfc--200us; 1,0xfd---1ms
#define InterpolationCycleValue    1
#define InterpolationCycleUint     0xfd    //10的-3次方（ms）0xfc10的-4次方（）
#define   PositionOptionCodeBuffer      0x20   //不用应答状态字bit4 buffer中有数就按新点执行
#define   PositionOptionCodePreCon      0x10   //不用应答状态字bit4 原来点执行完成立即执行新点

//设置插值周期子索引1：插值时间值；子索引2：插值时间单位默认-3:10的-3次方表示ms;子索引1*10的子索引2次方
//从站结点号、索引、子索引
extern void SetInterpolationCycle(uint8_t S_ID, CIFXHANDLE hChannel);
//读插值周期值
extern uint32_t ReadInterpolationCycleValue(uint8_t S_ID, CIFXHANDLE hChannel);
//读插值周期单位
extern uint32_t ReadInterpolationCycleUnit(uint8_t S_ID, CIFXHANDLE hChannel);

//设置0x6075--Motor Rate Current	  电流输入值1000000---A，1000--mA
extern void SetMotorRateCurrent(uint32_t S_ID, uint32_t rate);
//读0x6075--Motor Rate Current	 电流输入值1000000---A，1000--mA
extern uint32_t ReadMotorRateCurrent(uint32_t S_ID);
//设置0x6077--Motor Read Rate Current	  电流输入值1000000---A，1000--mA
extern void SetMotorReadRateCurrent(uint8_t S_ID, CIFXHANDLE hChannel);
//读0x6077--Motor Read Rate Current	 电流输入值1---A，1000--mA
extern uint32_t ReadMotorReadRateCurrent(uint8_t S_ID, CIFXHANDLE hChannel);
//设置0x6076--Motor Rate Torque	 力矩模式输入值1---A，1000--mA
extern void SetMotorRateTorque(uint32_t S_ID, uint32_t rate);
//读0x6076--Motor Rate Torque	 力矩模式输入值1---A，1000--mA
extern uint32_t ReadMotorRateTorque(uint32_t S_ID);
//设置0x6078--Motor Read Rate Torque	 力矩模式输入值1000000---A，1000--mA
extern void SetMotorReadRateTorque(uint8_t S_ID, CIFXHANDLE hChannel);
//读0x6076--Motor Read Rate Torque	 力矩模式输入值1000000---A，1000--mA
extern uint32_t ReadMotorReadRateTorque(uint8_t S_ID, CIFXHANDLE hChannel);
//设置0x3146--MO=1
extern uint32_t ReadMF(uint8_t S_ID);
extern uint32_t ReadEC(uint8_t S_ID);
extern uint32_t ReadEE(uint8_t S_ID);
//设置0x0x3146--MO=1 清除MF错误码
//设置0x0x6007--Abort connect option code  驱动器心跳停止功能取消
extern void SetHeartBeat(uint8_t S_ID, CIFXHANDLE hChannel);
extern void SetM0(uint8_t S_ID, CIFXHANDLE hChannel);
extern void SDODInit_All(CIFXHANDLE hChannel, uint16_t Mot_NumTemp);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _PROTOCOL_SDO_H
