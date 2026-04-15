#ifndef _ETHERCAT_DEFINE_H
#define _ETHERCAT_DEFINE_H

//电机工作模式
#define   ProPositionMode					1
#define   ProVelocityMode					3
#define   ProTorqueMode						4
#define   InterpolatedMode				7
#define   SynPositionMode					8
#define   SynVelocityMode					9
#define   SynTorqueMode						10
//控制字
#define   SwitchOff							0x06
#define   SwitchOnVoltageEn					0x07
#define   EnOperation						0x0f
//连续位置模式控制字
#define   PositionConSwithOff				0x56    //连续位置模式
#define   PositionConVoltageEn				0x57    //连续位置模式
#define   PositionConEnOperation			0x5f    //连续位置模式
//单点位置模式控制字
#define   PositionSigSwithOff				0x76    //单点位置模式
#define   PositionSigVoltageEn				0x77    //单点位置模式
#define   PositionSigEnOperation			0x7f    //单点位置模式
//设置状态字bit12位set_point_acnolwafe
#define   set_point_acknoledge				0x00001000    //
//检测状态位
#define   set_point_acknoledge				0x00001000    //
#define   PowerOffStatusFlag				0x0001    //  掉电
#define   VoltageStatusEnFlag				0x0003    //  上电 无力
#define   PowerOnStatusFlag					0x0007    //  上电 伺服


//D:/PCIToEcat卡设置5ElmoCSPALL-2090618.spj
#define   ControlWordRel						0
#define   ControlWordNum					2
#define   OperationModeRel						2
#define   OperationModeNum					1        //后面结尾有一个NC
#define   TargetTorqueRel						4
#define   TargetTorqueTypeNum				2
#define   TargetPositionRel						6         //位置
#define   TargetPositionTypeNum				4

#define   ProfileVelocityRel					10 
#define   ProfileVelocityTypeNum			4

#define   EndVelocityRel						14
#define   EndVelocityTypeNum				4
#define   AccelerationRel						18     //加速度
#define   AccelerationTypeNum				4
#define   DecelerationRel						22     //加速度
#define   DecelerationTypeNum				4
#define   TargetVelocityRel						26
#define   TargetVelocityTypeNum				4
//以上共30

//Input(工控机)量相对位置（offset)及占字节数
#define   StatusWordRel							0
#define   StatusWordNum						2
#define   OperationModeDisRel					2
#define   OperationModeDisNum				1      //后面结尾有一个NC

#define   ActualPositionRel						4         //位置
#define   ActualPositionTypeNum				4
#define   ActualVelocityRel						8
#define   ActualVelocityTypeNum				4
#define   ActualTorqueRel						12
#define   ActualTorqueTypeNum				2
#define   ActualCurrentRel						14
#define   ActualCurrentTypeNum				2
//码盘
#define   CodePositionRel						16         //位置
#define   CodePositionTypeNum				4
//以上共16
/** Output process data ******/
#define  CW_SWITCH_ON						0x0001
#define  CW_ENABLE_VOLTAGE					0x0002
#define  CW_QUICK_STOP						0x0004
#define  CW_ENABLE_OPERATION				0x0008
#define  CW_RESET_FAULT						0x0080
#define  CW_MANDONTARY_MASK					0x008F
#define  CW_Relative						0x0040      //0:绝对运动，1：相对运动
#define  CW_New_Set_PP						0x0010      //new set_point
#define  CW_New_Clr_PP						0xFF7f      //new set_point clr

/* Device control commands */
#define  SHUTDOWN(cw)             (CW_QUICK_STOP | CW_ENABLE_VOLTAGE  /*| ((cw) | (CW_ENABLE_OPERATION))*/)  //6
#define  READYTOSWITCHON(cw)			SHUTDOWN(cw)
#define  SWITCH_ON(cw)            (CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)                         //7
#define  DIASABLE_VOLTAGE(cw)     (cw & 0xFFFD)
#define  QUICK_STOP(cw)           (CW_ENABLE_VOLTAGE | ((cw) | (CW_ENABLE_OPERATION | CW_SWITCH_ON)))
#define  DISABLE_OPERATIONAL(cw)  (CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)                        //7 
#define  ENABLE_OPERATIONAL(cw)   (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)  //F
#define  FAULT_RESET(cw)          (CW_RESET_FAULT | ((cw) | (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)))   //0X8F
#define  ENABLE_OPERATION_Rel(cw) (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON | CW_Relative )   //0X40(bit6)
//PP模式
#define  ENABLE_OPERATION_Abs_PP(cw) (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON |CW_New_Set_PP )   //0X40(bit6)
#define  ENABLE_OPERATION_Rel_PP(cw) (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON | CW_Relative|CW_New_Set_PP )   //0X40(bit6)
#define  AllSet      (CW_ENABLE_OPERATION | CW_QUICK_STOP | CW_ENABLE_VOLTAGE | CW_SWITCH_ON)
#define  DisENABLE_OPERATION_Abs_PP(cw)  (AllSet & CW_New_Clr_PP)

/** Input process data ***************************************************/
#define  SW_READY_TO_SWITCH_ON         0x0001
#define  SW_SWITCH_ON                  0x0002
#define  SW_OPERATION_ENABLE           0x0004
#define  SW_FAULT                      0x0008
#define  SW_VOLTAGE_DISABLED           0x0010
#define  SW_QUICK_STOP                 0x0020
#define  SW_SWITCH_ON_DISABLED         0x0040
#define  SW_REMOTE                     0x0200
#define  SW_TARGET_REACHED             0x0400
#define  SW_INTERANL_LIMIT_ACTIVE      0x0800

#define  SW_MANDONTARY_MASK            0x0E7F

/* The following macros can be used to indicate the status of the device */
#define  NOT_READY_TO_SWITCH_ON(sw)     (((sw) & 0x004F) == 0x0000)
#define  SWITCH_ON_DISABLED(sw)         (((sw) & 0x004F) == 0x0040)
#define  READY_TO_SWITCH_ON(sw)         (((sw) & 0x0021) == 0x0021)
#define  SWITCHED_ON(sw)                (((sw) & 0x0023) == 0x0023)
#define  OPERATION_ENABLED(sw)         	(((sw) & 0x0027) == 0x0027)
#define  QUICK_STOP_ACTIVE(sw)          (((sw) & 0x006F) == 0x0007)
#define  FAULT_REACTION_ACTIVE(sw)      (((sw) & 0x004F) == 0x000F)
#define  FAULT(sw)                      (((sw) & 0x004F) == 0x0008)


#endif // _ETHERCAT_DATA_H
