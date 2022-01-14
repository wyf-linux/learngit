/*************************************************
  Copyright (C), 2021-4-3, EI41s 3driveBoard Ltd
  File name:       motor.h
  Author: wyf      Version:V1.0        Date:2021-4-3
  Description:     电机变量以及函数声明
  兼容所有终端控制板，部分设备并非电机，后期没有修改，可以全部抽象成电机；
  Others:         
  Function List: 
    1. 
  History:        
                  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H_
#define __MOTOR_H_

#include <stdint.h>
#include "fsl_gpio.h"

extern void Delay(uint32_t t);//延时


#ifdef MOTOR_CTRL_BOARD
#define _FanMotorRun				{\
										GPIO_PinWrite(GPIO, 0, 21, 1);\
										struMotorCtrl.RelayStatus = RELAY_STATUS_RUN;\
									}	//其他主板电机继电器吸合
#define _FanMotorStop				{\
										GPIO_PinWrite(GPIO, 0, 21, 0);\
										struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;\
									}	//其他主板电机继电器断开
#endif //MOTOR_CTRL_BOARD							

									
#ifdef VENTSHADE_CTRL_BOARD								
/////////////////////////////////////////////////////小窗控制板///////////////////////////////////////////////////////
//侧窗电机向大指示灯
#define _MotorOpenLedOn		{ ( GPIO_PinWrite(GPIO, 0, 0, 1)); }
#define _MotorOpenLedOff	{ ( GPIO_PinWrite(GPIO, 0, 0, 0)); }
//侧窗电机向小指示灯
#define _MotorCloseLedOn	{ ( GPIO_PinWrite(GPIO, 0, 9, 1)); }	
#define _MotorCloseLedOff	{ ( GPIO_PinWrite(GPIO, 0, 9, 0)); }


#define RUN_RELAY_ON			{ ( GPIO_PinWrite(GPIO, 0, 22, 1)); }	//动作继电器吸合
#define RUN_RELAY_OFF			{ ( GPIO_PinWrite(GPIO, 0, 22, 0)); }	//动作继电器断开	
#define DIRECTION_RELAY_ON		{ ( GPIO_PinWrite(GPIO, 0, 21, 1)); }	//方向继电器吸合	
#define DIRECTION_RELAY_OFF		{ ( GPIO_PinWrite(GPIO, 0, 21, 0)); }	//方向继电器断开

//小窗控制信号
#define _ShadeMotorStop				{\
										RUN_RELAY_OFF;\
										DIRECTION_RELAY_OFF;\
										_MotorOpenLedOff;\
										_MotorCloseLedOff;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;\
									}	//侧窗停止 

#define _ShadeMotorOpen				{\
										DIRECTION_RELAY_OFF;\
										RUN_RELAY_ON;\
										_MotorOpenLedOn;\
										_MotorCloseLedOff;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_OPEN;\
									}	//侧窗向大开   方向继电器引脚为低电平

#define _ShadeMotorClose				{\
										DIRECTION_RELAY_ON;\
										RUN_RELAY_ON;\
										_MotorOpenLedOff;\
										_MotorCloseLedOn;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_CLOSE;\
									}	//侧窗向小开   方向继电器引脚为高电平 
/////////////////////////////////////////////////////小窗控制板///////////////////////////////////////////////////////	
#endif //VENTSHADE_CTRL_BOARD
				

#ifdef AIRINTAKE_CTRL_BOARD	
/////////////////////////////////////////////////////进风口控制板///////////////////////////////////////////////////////
//电机控制信号
#define _ShadeMotorStop			{\
									GPIO_PinWrite(GPIO, 0, 21, 0);\
									struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;\
								}
								

#define _ShadeMotorOpen			{\
									GPIO_PinWrite(GPIO, 0, 22, 1);\
									GPIO_PinWrite(GPIO, 0, 21, 1);\
									struMotorCtrl.RelayStatus = RELAY_STATUS_OPEN;\
								}

#define _ShadeMotorClose		{\
									GPIO_PinWrite(GPIO, 0, 22, 0);\
									GPIO_PinWrite(GPIO, 0, 21, 1);\
									struMotorCtrl.RelayStatus = RELAY_STATUS_CLOSE;\
								}
/////////////////////////////////////////////////////进风口控制板///////////////////////////////////////////////////////								
#endif //AIRINTAKE_CTRL_BOARD								
									
////////////////////////////////////////	电机总电压电流	////////////////////////////////////////
#define PHASE_A				0//A相
#define PHASE_B				1//B相
#define PHASE_C				2//C相									
#define MOTOR_PHASE_CNT		3
extern  uint64_t    phaseVoltageRegister[MOTOR_PHASE_CNT];//相电压寄存器值	27位
extern  uint64_t    phaseCurrentRegister[MOTOR_PHASE_CNT];//相电流寄存器值	27位
extern	uint64_t	phaseVoltage[MOTOR_PHASE_CNT];//相电压
extern	uint64_t	lineVoltage[MOTOR_PHASE_CNT];//线电压	
extern  uint64_t    phaseCurrent[MOTOR_PHASE_CNT];//相电流
extern uint64_t     phaseCurrentdisplay[MOTOR_PHASE_CNT];//相电流显示值
extern volatile uint8_t flag_Read_Current_250ms;//250ms读取电流电压值时间标志位
////////////////////////////////////////	电机总电压电流	////////////////////////////////////////



#define THREE_PHASE_MOTOR_RATED_VOLTAGE					3800//三相电机额定电压值380V
#define SINGLE_PHASE_MOTOR_RATED_VOLTAGE				2200//单相电机额定电压值220V
#define OVER_VOLT_SCALE			(112)//1.12倍额定电压
#define UNDER_VOLT_SCALE		(88) //0.88倍额定电压

#define PHASE_LOSS_ALARM_VOLTAGE			(1000) //电机缺相报警电压值,100V
#define SHORT_CIRCUIT_CURRENT_SCALE			(80000)//电机短路电流系数，8倍额定电流认定为电机短路，立刻脱扣

#define NULL_LOADING_CURRENT_LOW			(struMotorCtrl.RatedCurrent*5500)//空载电流下限
#define NULL_LOADING_CURRENT_HIGH			(struMotorCtrl.RatedCurrent*6000)//空载电流上限
#define UNDER_LOADING_CURRENT_LOW			(struMotorCtrl.RatedCurrent*6000)//轻载电流下限 
#define UNDER_LOADING_CURRENT_HIGH			(struMotorCtrl.RatedCurrent*7500)//轻载电流上限 

#define MOTOR_MIN_RATED_CURRENT				(1)//电机最小额定电流 5->1 modified by wyf 20211124
#define MOTOR_STOP_CURRENT					(10000)//电机停转电流 modified 500mA->100mA 20211217
//考虑是否设定一个固定值，比如100mA

//add by wyf 20220107 750->1000
#define INVALID_OVER_CURRENT_TIME 	1000 //单位：ms 无效过载时间，add by wyf 20211217

//电机启动时间一般5~10s，按照10s判断
#define MOTOR_MAX_START_TIME		10//电机停转控制时间 modified 2->10 by wyf 20211217


enum MOTOR_TYPE   //电机类型
{
	MOTOR_TYPE_SINGLE_PHASE = 0x5A,//单相电机
	MOTOR_TYPE_THREE_PHASE = 0xA5,//三相电机
};
enum MOTOR_STATUS //电机状态
{
	MOTOR_STATUS_RUN   = 0x5A,//运行
	MOTOR_STATUS_STOP  = 0xA5,//停止
};
#if defined(MOTOR_CTRL_BOARD) || defined(LIGHT_CTRL_BOARD)
enum RELAY_ORDER //继电器控制指令
{
	RELAY_ORDER_RUN   = 0x5A,//吸合
	RELAY_ORDER_STOP  = 0xA5,//断开
};
enum RELAY_STATUS //继电器状态
{
	RELAY_STATUS_RUN   = 0x5A,//吸合
	RELAY_STATUS_STOP  = 0xA5,//断开
};
#endif //MOTOR_CTRL_BOARD


#if defined(VENTSHADE_CTRL_BOARD) || defined(AIRINTAKE_CTRL_BOARD)
enum RELAY_ORDER //继电器控制指令
{
	RELAY_ORDER_STOP  = 0x00,//停止
	RELAY_ORDER_OPEN  = 0x5A,//向大
	RELAY_ORDER_CLOSE = 0xA5,//向小
};
enum RELAY_STATUS //继电器状态
{
	RELAY_STATUS_STOP  = 0x00,//停止
	RELAY_STATUS_OPEN  = 0x5A,//向大
	RELAY_STATUS_CLOSE = 0xA5,//向小
};
#endif

//enum MOTOR_MODE //电机工作模式 add by wyf 20211108 触摸屏增加测试页，用于测试外设
//{
//	MOTOR_MODE_TEST = 0x5A,//测试模式
//	MOTOR_MODE_NORMAL =0xA5,//正常工作模式
//};


#define OVER_CURRENT_SECTION_COUNT_THERMAL	18//热过载热态区间数 modified 17->18 by wyf 20211217
#define OVER_CURRENT_SECTION_COUNT_COLD		17//热过载冷态区间数 modified 16->17 by wyf 20211217
#define OVER_CURRENT_SECTION_COUNT_MAX		18//热过载最大区间数 modified 17->18 by wyf 20211217


//小型电动机整定电流一般为额定1.05到1.1，整定电流这里取1.1倍，控制器内部固化
#define OVER_CURRENT_SCALE_SET	110//电机整定电流比例
#ifdef DEBUG_FOR_TEST
#define MOTOR_OVER_CURRENT_CLEAR_TIME	(uint32_t)1800
#else
#define MOTOR_OVER_CURRENT_CLEAR_TIME	(uint32_t)1800
#endif


//热过载模式，分为冷态和热态
enum OVER_CURRENT_MODE
{
	OVER_CURRENT_MODE_THERMAL = 0x5A,//热态
	OVER_CURRENT_MODE_COLD 	= 0xA5,//冷态
};
//定义电机结构类型
typedef struct MotorStructType
{
	uint16_t 	RatedCurrent;//额定电流						MCU 10倍数存储
	uint16_t    RatedVolt;//额定电压 add by wyf 20211123	MCU 10倍存储
	uint64_t 	LineVoltage[MOTOR_PHASE_CNT];//线电压		MCU 10倍数存储
	uint64_t 	PhaseCurrent[MOTOR_PHASE_CNT];//相电流	MCU 100倍数存储
	uint16_t 	StartDelay;//启动延时
	uint16_t	OverCurrentClearTime;//不处在任何一个热过载区间时间累计
	uint8_t     OverCurrentMode;//热过载模式
	uint64_t    OverCurrentTime[OVER_CURRENT_SECTION_COUNT_MAX];//不同区间热过载计时
	uint8_t     OverCurrentSection;//当前所处的热过载区间
	uint8_t     Mode;//工作模式 for 触摸屏测试模式
	uint16_t	ReverseDelay;//反向延时时间		add by wyf 20211110 对于侧窗/幕帘
	uint16_t 	RunLong;//运行时间长度
	uint8_t		Type;//类型
	uint8_t     OldStatus;//电机之前的运行状态
	uint8_t 	Status;//电机当前状态
	uint8_t		RelayOrder;//继电器控制指令
	uint8_t		RelayStatus;//继电器状态
	uint16_t    ActivePower;//有功功率
	uint16_t    Lifetime;//电机寿命
} MOTORSTRUCTTYPE;
extern volatile MOTORSTRUCTTYPE struMotorCtrl;//电机控制结构体

extern volatile uint8_t flag_UpdateMotorStatus_1s;//1s更新一次设备运行状态

extern volatile uint8_t flag_Push_OverLoad_Time_1s;//add by wyf 20211216 for debug


extern void MotorCtrlInit(void);//电机控制初始化
extern void MotorFaultMonitor(void);//电机故障监控主函数
extern void MotorExceptionCtrl(void);//电机异常情况控制，主要针对热过载、短路，当且仅当重新复位后，允许控制;
extern void UpdateMotorRelay(void);//更新电机继电器
extern void UpdateMotorStatus(void);//更新电机工作状态
extern void UpdateMotorRatedVoltage(void);//更新电机额定电压


#endif // __MOTOR_H_

