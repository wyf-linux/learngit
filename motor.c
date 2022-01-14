/*************************************************
  Copyright (C), 2021-4-3, EI41s 3driveBoard Ltd
  File name:       motor.c
  Author: wyf      Version:V1.0        Date:2021-4-3
  Description:     电机变量以及函数定义
  Others:         
  Function List: 
	1.
  History:        
                  
    1. Date:
       Author:
       Modification:
    2. ...
*************************************************/
#include "hw.h"
#include "alarm.h"
#include "motor.h"
#include "global.h"
#include "device.h"
#include "AD5302.h"


volatile uint8_t flag_UpdateMotorStatus_1s;//1s更新一次设备运行状态
////////////////////////////////////////	电机总电压电流	////////////////////////////////////////
uint64_t    phaseVoltageRegister[MOTOR_PHASE_CNT];//相电压寄存器值	27位
uint64_t    phaseCurrentRegister[MOTOR_PHASE_CNT];//相电流寄存器值	27位
uint64_t	phaseVoltage[MOTOR_PHASE_CNT];//相电压
uint64_t	lineVoltage[MOTOR_PHASE_CNT];//线电压	
uint64_t    phaseCurrent[MOTOR_PHASE_CNT];//相电流
uint64_t    phaseCurrentdisplay[MOTOR_PHASE_CNT];//相电流显示值
volatile uint8_t flag_Read_Current_250ms;//250ms读取电流电压值时间标志位
////////////////////////////////////////	电机总电压电流	////////////////////////////////////////



volatile MOTORSTRUCTTYPE struMotorCtrl;//电机控制结构体

volatile uint8_t flag_Push_OverLoad_Time_1s;//add by wyf 20211216 for debug



//热过载热态不同区间对应的整定电流比例
const uint32_t gThermalOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_THERMAL] = 
{
	110,		//热过载比例1对应的整定电流系数1.1
	120,		//热过载比例2对应的整定电流系数1.2	
	130,		//热过载比例3对应的整定电流系数1.3
	140,		//热过载比例4对应的整定电流系数1.4
	150,		//热过载比例5对应的整定电流系数1.5
	160,		//热过载比例6对应的整定电流系数1.6
	170,		//热过载比例7对应的整定电流系数1.7
	180,		//热过载比例8对应的整定电流系数1.8
	190,		//热过载比例9对应的整定电流系数1.9
	200,		//热过载比例10对应的整定电流系数2.0
	245,		//热过载比例11对应的整定电流系数2.45
	250,		//热过载比例12对应的整定电流系数2.5
	300,		//热过载比例13对应的整定电流系数3.0
	350,		//热过载比例14对应的整定电流系数3.5
	400,		//热过载比例15对应的整定电流系数4.0
	500,		//热过载比例16对应的整定电流系数5.0
	600,		//热过载比例17对应的整定电流系数6.0
	720,		//热过载比例18对应的整定电流系数7.0
};

//热过载热态不同区间对应的脱扣时间 单位ms
const uint32_t gThermalOverCurrentTableTime[OVER_CURRENT_SECTION_COUNT_THERMAL] = 
{
	400000,		//热过载区间1对应的脱扣时间 400s [1.1,1.2)
	244000,		//热过载区间2对应的脱扣时间 244s [1.2,1.3)
	116000,		//热过载区间3对应的脱扣时间 116s [1.3,1.4)
	82000,		//热过载区间4对应的脱扣时间 82s  [1.4,1.5)
	78000,		//热过载区间5对应的脱扣时间 78s  [1.5,1.6)
	74000,		//热过载区间6对应的脱扣时间 74s  [1.6,1.7)
	72000,		//热过载区间7对应的脱扣时间 72s  [1.7,1.8)	
	71000,		//热过载区间8对应的脱扣时间 71s  [1.8,1.9)
	70000,		//热过载区间9对应的脱扣时间 70s  [1.9,2.0)
	66000,		//热过载区间10对应的脱扣时间 66s  [2.0,2.45)
	52000,		//热过载区间11对应的脱扣时间 52s  [2.45,2.5)
	27000,		//热过载区间12对应的脱扣时间 27s  [2.5,3.0)	
	18000,		//热过载区间13对应的脱扣时间 18s  [3.0,3.5)	
	14000,		//热过载区间14对应的脱扣时间 14s  [3.5,4.0)
	8000,		//热过载区间15对应的脱扣时间 8s  [4.0,5.0)	
	6000,		//热过载区间16对应的脱扣时间 6s  [5.0,6.0)
	4000,		//热过载区间17对应的脱扣时间 4s  [6.0,7.2)
	2000,		//热过载区间18对应的脱扣时间 2s  [7.2, 无穷大）
};



//热过载冷态不同区间对应的整定电流比例
const uint32_t gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD] = 
{
	110,		//热过载比例1对应的整定电流系数1.1
	120,		//热过载比例2对应的整定电流系数1.2	
	130,		//热过载比例3对应的整定电流系数1.3
	140,		//热过载比例4对应的整定电流系数1.4
	150,		//热过载比例5对应的整定电流系数1.5
	160,		//热过载比例6对应的整定电流系数1.6
	170,		//热过载比例7对应的整定电流系数1.7
	180,		//热过载比例8对应的整定电流系数1.8
	190,		//热过载比例9对应的整定电流系数1.9
	200,		//热过载比例10对应的整定电流系数2.0
	250,		//热过载比例11对应的整定电流系数2.5
	300,		//热过载比例12对应的整定电流系数3.0
	350,		//热过载比例13对应的整定电流系数3.5
	400,		//热过载比例14对应的整定电流系数4.0
	500,		//热过载比例15对应的整定电流系数5.0
	600,		//热过载比例16对应的整定电流系数6.0
	720,		//热过载比例17对应的整定电流系数7.2
};
//热过载冷态不同区间对应的脱扣时间 单位ms
const uint32_t gColdOverCurrentTableTime[OVER_CURRENT_SECTION_COUNT_COLD] = 
{
	700000,		//热过载区间1对应的脱扣时间 700s [1.1,1.2)
	460000,		//热过载区间2对应的脱扣时间 460s [1.2,1.3)
	280000,		//热过载区间3对应的脱扣时间 280s [1.3,1.4)
	208000,		//热过载区间4对应的脱扣时间 208s  [1.4,1.5)
	182000,		//热过载区间5对应的脱扣时间 182s  [1.5,1.6)
	162000,		//热过载区间6对应的脱扣时间 162s  [1.6,1.7)
	145000,		//热过载区间7对应的脱扣时间 145s  [1.7,1.8)	
	129000,		//热过载区间8对应的脱扣时间 129s  [1.8,1.9)
	116000,		//热过载区间9对应的脱扣时间 116s  [1.9,2.0)
	82000,		//热过载区间10对应的脱扣时间 82s  [2.0,2.5)
	49000,		//热过载区间11对应的脱扣时间 49s  [2.5,3.0)
	34000,		//热过载区间12对应的脱扣时间 34s  [3.0,3.5)	
	28000,		//热过载区间13对应的脱扣时间 28s  [3.5,4.0)	
	20000,		//热过载区间14对应的脱扣时间 20s  [4.0,5.0)
	12000,		//热过载区间15对应的脱扣时间 12s  [5.0,6.0)	
	7000,		//热过载区间16对应的脱扣时间 7s  [6.0,7.2)	
	3000,		//热过载区间17对应的脱扣时间 3s  [7.2,无穷大)	
};



/**
 * @brief	延时
 * @param	无
 * @return	无
 */
void Delay(uint32_t t)//延时
{
	if(t==0)
		return;
	while(t--);
}


/**
 * @brief	电机控制初始化
 * @param	无
 * @return	无
 */
void MotorCtrlInit(void)
{
	//电机类型未设置或者错误，执行默认初始化程序
	if ((struMotorCtrl.Type != MOTOR_TYPE_SINGLE_PHASE) &&
	(struMotorCtrl.Type != MOTOR_TYPE_THREE_PHASE))
	{
		//风机
		if ( struDeviceCtrl.Type == DEV_TYPE_FAN )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//小窗
		else if ( struDeviceCtrl.Type == DEV_TYPE_VENTSHADE )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}
		//加热
		else if ( struDeviceCtrl.Type == DEV_TYPE_ADDTEMP )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}
		//进风口
		else if ( struDeviceCtrl.Type == DEV_TYPE_AIRINTAKE )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//湿帘
		else if ( struDeviceCtrl.Type == DEV_TYPE_WETPUMP )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//供料
		else if ( struDeviceCtrl.Type == DEV_TYPE_MALESUPPLY )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//喂料
		else if ( struDeviceCtrl.Type == DEV_TYPE_FEEDING )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//调光
		else if ( struDeviceCtrl.Type == DEV_TYPE_LIGHT )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;//modified by wyf Flash三次备份  修改为单相电机	
		}
		//加药
		else if ( struDeviceCtrl.Type == DEV_TYPE_DOSING )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}	
		//喷雾
		else if ( struDeviceCtrl.Type == DEV_TYPE_FOG )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//清粪
		else if ( struDeviceCtrl.Type == DEV_TYPE_SHIT )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//默认单相电机 只检测1相故障 wyf add 2021/10/26
		else
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;
		}
	}
	//默认额定电流如果为0，则初始化为500mA，最小电流，不要出现0值，热过载报警电流值通过额定电流得到；
	if ( struMotorCtrl.RatedCurrent < MOTOR_MIN_RATED_CURRENT )
	{
		struMotorCtrl.RatedCurrent = MOTOR_MIN_RATED_CURRENT;//modified by wyf 20211106
	}

	struMotorCtrl.Status = MOTOR_STATUS_STOP;
	struMotorCtrl.OldStatus = MOTOR_STATUS_STOP;
	struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;	
	struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
	
	//判断热过载冷态还是热态
	if (  struDeviceCtrl.Type == DEV_TYPE_SHIT  )
	{
		struMotorCtrl.OverCurrentMode = OVER_CURRENT_MODE_COLD;	//清粪按照冷态运行
	}
	else
	{
		struMotorCtrl.OverCurrentMode = OVER_CURRENT_MODE_THERMAL;	//其他主板按照热态运行
	}
	
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		struMotorCtrl.RatedVolt	= SINGLE_PHASE_MOTOR_RATED_VOLTAGE;
		phaseVoltage[PHASE_A] = struMotorCtrl.RatedVolt;
	}
	//三相设备
	else
	{
		struMotorCtrl.RatedVolt	= THREE_PHASE_MOTOR_RATED_VOLTAGE;	
		lineVoltage[PHASE_A] = struMotorCtrl.RatedVolt;
		lineVoltage[PHASE_B] = struMotorCtrl.RatedVolt;
		lineVoltage[PHASE_C] = struMotorCtrl.RatedVolt;
	}
}






#ifndef LIGHT_CTRL_BOARD
/**
 * @brief	电机过载控制(冷态)
 * @param	无
 * @return	无
 */
static void MotorOverCurrentColdMonitor(void)
{
	uint64_t ISet = 0;//电机整定电流设定值
	uint8_t hasLevel = 0;//处在某一个热过载范围区间
	uint64_t ratio = 0;	
	uint8_t overCurrentSection = 0;
	
	//计算当前热过载整定电流值
	ISet = struMotorCtrl.RatedCurrent*OVER_CURRENT_SCALE_SET;
	//判断当前热过载等级
	//任意1相相电流
	for ( uint8_t phase = PHASE_A; phase <= PHASE_C; phase++)
	{
		//单相设备   注释 by wyf 20211230
//		if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE ) 
//		{
//			//B相或者C相
//			if ( phase != PHASE_A )
//			{
//				break;
//			}
//		}		
		//判断当前电流所处的热过载区间
		for ( uint8_t scale = 0; scale < OVER_CURRENT_SECTION_COUNT_COLD-1; scale++)
		{
			if ( (phaseCurrent[phase] >= ISet*gColdOverCurrentTableScale[scale])	&&
				(phaseCurrent[phase] < ISet*gColdOverCurrentTableScale[scale+1]))
			{
				if ((scale+1) > overCurrentSection)
				{
					overCurrentSection = scale+1;
				}
			}
		}
		//判断是否处于最后一个热过载区间  add by wy 20211217
		if ( phaseCurrent[phase] >= ISet*gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD-1]) 
		{
			overCurrentSection = OVER_CURRENT_SECTION_COUNT_COLD;
		}
	}
	struMotorCtrl.OverCurrentSection = overCurrentSection;//确定当前热过载区间,区间范围1~n
	//不处在任何一个热过载区间
	if ( (!struMotorCtrl.OverCurrentSection) && (struMotorCtrl.OverCurrentClearTime >= MOTOR_OVER_CURRENT_CLEAR_TIME))
	{
		struMotorCtrl.OverCurrentClearTime = 0;
		//各区间热过载时间清0
		for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_COLD; section++)
		{
			struMotorCtrl.OverCurrentTime[section] = 0;//热过载时间清0
		}				
	}

	
	//判断热过载报警
	for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_COLD; section++)
	{
		//停止电机，过载电流时间低于500ms，认定为启动电流或者干扰，时间清0 add by wyf 20211217
		if ( (struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) && 
			(struMotorCtrl.OverCurrentTime[section] <= INVALID_OVER_CURRENT_TIME) )
		{
			struMotorCtrl.OverCurrentTime[section] = 0;
		}
		ratio += struMotorCtrl.OverCurrentTime[section] *100 / gColdOverCurrentTableTime[section];	
	}
	if ( ratio >= 100 )
	{
		bmAlarmWord |= constbmawOverCurrentAlarm;	//热过载	
	}
}




/**
 * @brief	电机过载控制(热态)
 * @param	无
 * @return	无
 */
static void MotorOverCurrentThermalMonitor(void)
{
	uint64_t ISet = 0;//电机整定电流设定值
	uint8_t hasLevel = 0;//处在某一个热过载范围区间
	uint64_t ratio = 0;	
	uint8_t overCurrentSection = 0;
	//计算当前热过载整定电流值
	ISet = struMotorCtrl.RatedCurrent*OVER_CURRENT_SCALE_SET;
	//判断当前热过载等级
	//任意1相相电流
	for ( uint8_t phase = PHASE_A; phase <= PHASE_C; phase++)
	{
		//单相设备     注释 by wyf 20211230
//		if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE ) 
//		{
//			//B相或者C相
//			if ( phase != PHASE_A )
//			{
//				break;
//			}
//		}		
		//判断当前电流所处的热过载区间
		for ( uint8_t scale = 0; scale < OVER_CURRENT_SECTION_COUNT_THERMAL; scale++)
		{
			if ( (phaseCurrent[phase] >= ISet*gThermalOverCurrentTableScale[scale])	&&
				(phaseCurrent[phase] < ISet*gThermalOverCurrentTableScale[scale+1]))
			{
				if ((scale+1) > overCurrentSection)
				{
					overCurrentSection = scale+1;
				}
		
			}
		}
		//判断是否处于最后一个热过载区间  add by wy 20211217
		if ( phaseCurrent[phase] >= ISet*gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD-1]) 
		{
			overCurrentSection = OVER_CURRENT_SECTION_COUNT_THERMAL;
		}
	}
	struMotorCtrl.OverCurrentSection = overCurrentSection;//确定当前热过载区间,区间范围1~n
	//不处在任何一个热过载区间
	if ( (!struMotorCtrl.OverCurrentSection) && (struMotorCtrl.OverCurrentClearTime >= MOTOR_OVER_CURRENT_CLEAR_TIME))
	{
		struMotorCtrl.OverCurrentClearTime = 0;
		//各区间热过载时间清0
		for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_THERMAL; section++)
		{
			struMotorCtrl.OverCurrentTime[section] = 0;//热过载时间清0
		}				
	}

	
	//判断热过载报警
	for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_THERMAL; section++)
	{
		//停止电机，过载电流时间低于500ms，认定为启动电流或者干扰，时间清0 add by wyf 20211217
		if ( (struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) && 
			(struMotorCtrl.OverCurrentTime[section] <= INVALID_OVER_CURRENT_TIME) )
		{
			struMotorCtrl.OverCurrentTime[section] = 0;
		}
		//四舍五入
		ratio += ((struMotorCtrl.OverCurrentTime[section] *1000 / gThermalOverCurrentTableTime[section]+5)/10);	
	}
	if ( ratio >= 100 )
	{
		bmAlarmWord |= constbmawOverCurrentAlarm;	//热过载	
	}
}
#endif //LIGHT_CTRL_BOARD




/**
 * @brief	电机欠压故障监控
 * @param	无
 * @return	无
 */
void MotorLowVoltageMonitor(void)
{
	//限定风机1和小窗1判断超压 add by wyf 20211215
	if ( (struDeviceCtrl.Number != DEV_START_ADDR_FAN) && 
		(struDeviceCtrl.Number != DEV_START_ADDR_VENTSHADE))
	{
		return ;
	}
	
	
	uint32_t underVoltValue = struMotorCtrl.RatedVolt*UNDER_VOLT_SCALE/100;
	 //单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	
	{
		if ( ((phaseVoltage[PHASE_A] <= underVoltValue) &&
				(phaseVoltage[PHASE_A] > PHASE_LOSS_ALARM_VOLTAGE)))	//A相
		{
			//认为欠压
			bmAlarmWord |= constbmawUnderVoltageAlarm;	//不明白为什么报欠压
		}	
		else
		{
			bmAlarmWord &= ~constbmawUnderVoltageAlarm;
		}		
	}
	//三相设备
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( ((lineVoltage[PHASE_A] <= underVoltValue) &&
				(lineVoltage[PHASE_A] > PHASE_LOSS_ALARM_VOLTAGE))	||		//A相
			((lineVoltage[PHASE_B] <= underVoltValue) &&
				(lineVoltage[PHASE_B] > PHASE_LOSS_ALARM_VOLTAGE))	||		//B相		
			((lineVoltage[PHASE_C] <= underVoltValue) &&		
				(lineVoltage[PHASE_C] > PHASE_LOSS_ALARM_VOLTAGE)))		//C相
		{
			//认为欠压
			bmAlarmWord |= constbmawUnderVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawUnderVoltageAlarm;
		}	
	}
}



/**
 * @brief	电机超压故障监控
 * @param	无
 * @return	无
 */
void MotorOverVoltageMonitor(void)
{
	//限定风机1和小窗1判断超压 add by wyf 20211215
	if ( (struDeviceCtrl.Number != DEV_START_ADDR_FAN) && 
		(struDeviceCtrl.Number != DEV_START_ADDR_VENTSHADE))
	{
		return ;
	}
	
	
	uint32_t overVoltValue = struMotorCtrl.RatedVolt*OVER_VOLT_SCALE/100;
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseVoltage[PHASE_A] >= overVoltValue ) 	//A相
		{
			//认为超压
			bmAlarmWord |= constbmawOverVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawOverVoltageAlarm;
		}		
	}
	//三相设备
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( (lineVoltage[PHASE_A] >= overVoltValue) ||		//AB
			(lineVoltage[PHASE_B] >= overVoltValue) ||		//BC
			(lineVoltage[PHASE_C] >= overVoltValue) )		//AC
		{
			//认为超压
			bmAlarmWord |= constbmawOverVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawOverVoltageAlarm;
		}	
	}		
}



/**
 * @brief	电机断相故障监控
 * @param	无
 * @return	无
 */
void MotorPhaseLossMonitor(void)
{
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseVoltage[PHASE_A] <= PHASE_LOSS_ALARM_VOLTAGE )
		{
			bmAlarmWord |= constbmawPhaseLossAlarm;	
			//停止电机运行
			struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
//			#ifdef LIGHT_CTRL_BOARD
//			//调光控制
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);	
//			#endif
		}	
		else
		{
			bmAlarmWord &= ~constbmawPhaseLossAlarm;	
		}		
	}
	//三相设备
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( (lineVoltage[PHASE_A] <= PHASE_LOSS_ALARM_VOLTAGE) ||
			(lineVoltage[PHASE_B] <= PHASE_LOSS_ALARM_VOLTAGE)  ||
			(lineVoltage[PHASE_C] <= PHASE_LOSS_ALARM_VOLTAGE))
		{
			bmAlarmWord |= constbmawPhaseLossAlarm;	
			//停止电机运行
			struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
//			#ifdef LIGHT_CTRL_BOARD
//			//调光控制
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);	
//			#endif			
		}	
		else
		{
			bmAlarmWord &= ~constbmawPhaseLossAlarm;	
		}	
	}
}






#ifdef LIGHT_CTRL_BOARD
/**
 * @brief	电机短路故障监控
 * @param	无
 * @return	无
 */
static void MotorShortCircuitMonitor(void)
{
	uint64_t shortCircuitCurrent = 0;
//	//短路电流等于额定电流的8倍，认定为短路
//	shortCircuitCurrent = struMotorCtrl.RatedCurrent*SHORT_CIRCUIT_CURRENT_SCALE;

	if ( struDeviceCtrl.Type == DEV_TYPE_LIGHT )
	{
		shortCircuitCurrent = struMotorCtrl.RatedCurrent*10000;
	}

	
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseCurrent[PHASE_A] > shortCircuitCurrent )
		{
			bmAlarmWord |= constbmawShortCircuitAlarm;	
			//调光控制
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);
			//停止电机运行，当且仅当复位，报警消除。
//			struMotorCtrl.Order = MOTOR_ORDER_STOP;
		}	
		else
		{
			bmAlarmWord &= ~constbmawShortCircuitAlarm;		
		}
	}
	//三相设备
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{	
		if ( (phaseCurrent[PHASE_A] > shortCircuitCurrent) ||
			(phaseCurrent[PHASE_B] > shortCircuitCurrent)  ||
			(phaseCurrent[PHASE_C] > shortCircuitCurrent))
		{
			bmAlarmWord |= constbmawShortCircuitAlarm;
			//调光控制
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);			
			//停止电机运行，当且仅当复位，报警消除。
//			struMotorCtrl.Order = MOTOR_ORDER_STOP;
		}	
		else
		{
			bmAlarmWord &= ~constbmawShortCircuitAlarm;
		}			
	}
}
#endif



/**
 * @brief	电机轻载故障监控
 * @param	无
 * @return	无
 */
static void MotorUnderLoadingMonitor(void)
{
	//风机启动时长<10s，风机在启动过程中，运行电流处于空载电流范围内 add by wyf 20211229
	if ( (struMotorCtrl.RunLong < MOTOR_MAX_START_TIME) ||
		(struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) )
	{
		bmAlarmWord &= ~constbmawNullLoadingAlarm;	
		return ;
	}
	
	
	if ( DBTW(phaseCurrent[PHASE_A], UNDER_LOADING_CURRENT_LOW, UNDER_LOADING_CURRENT_HIGH) ||
		DBTW(phaseCurrent[PHASE_B], UNDER_LOADING_CURRENT_LOW, UNDER_LOADING_CURRENT_HIGH)  ||
		DBTW(phaseCurrent[PHASE_C], UNDER_LOADING_CURRENT_LOW, UNDER_LOADING_CURRENT_HIGH))
	{
		bmAlarmWord |= constbmawUnderLoadingAlarm;	//只报警不控制
	}
	else
	{
		bmAlarmWord &= ~constbmawUnderLoadingAlarm;	
	}
}



/**
 * @brief	电机空载故障监控
 * @param	无
 * @return	无
 */
static void MotorNullLoadingMonitor(void)
{
	//风机启动时长<10s，风机在启动过程中，运行电流处于空载电流范围内 add by wyf 20211229
	if ( (struMotorCtrl.RunLong < MOTOR_MAX_START_TIME) ||
		(struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) )
	{
		bmAlarmWord &= ~constbmawNullLoadingAlarm;	
		return ;
	}
	
	
	if ( DBTW(phaseCurrent[PHASE_A], NULL_LOADING_CURRENT_LOW, NULL_LOADING_CURRENT_HIGH) ||
		DBTW(phaseCurrent[PHASE_B], NULL_LOADING_CURRENT_LOW, NULL_LOADING_CURRENT_HIGH)  ||
		DBTW(phaseCurrent[PHASE_C], NULL_LOADING_CURRENT_LOW, NULL_LOADING_CURRENT_HIGH))
	{
		bmAlarmWord |= constbmawNullLoadingAlarm;	//只报警不控制
	}	
	else
	{
		bmAlarmWord &= ~constbmawNullLoadingAlarm;	
	}
}





/**
 * @brief	电机停转故障监控
 * @param	无
 * @return	无
 */
void MotorStopExceptionMonitor(void)
{
	if ( struDeviceCtrl.Type != DEV_TYPE_FAN )
	{
		return ;
	}
	
	if ( (struMotorCtrl.RelayStatus != RELAY_STATUS_STOP) &&
		(struMotorCtrl.Status == MOTOR_STATUS_STOP))
	{
		//电机停转报警
		bmAlarmWord |= constbmawMotorStopAlarm;	//只报警不控制
	}
	else
	{
	 	bmAlarmWord &= ~constbmawMotorStopAlarm;	
	}
}






/**
 * @brief	电机异常情况控制，主要针对热过载、短路，当且仅当重新复位后，允许控制;
 * @param	无
 * @return	无
 */
void MotorExceptionCtrl(void)
{
	//电机出现如下故障，短路、热过载，电机停止
	if ( (bmAlarmWord & constbmawShortCircuitAlarm) ||
		(bmAlarmWord & constbmawOverCurrentAlarm))
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;	
	}
}






/**
 * @brief	更新电机工作状态
 * @param	无
 * @return	无
 */
void UpdateMotorStatus(void)
{	
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseCurrent[PHASE_A] > MOTOR_STOP_CURRENT )
		{
			struMotorCtrl.Status = MOTOR_STATUS_RUN;				
		}	
		else
		{
			struMotorCtrl.Status = MOTOR_STATUS_STOP;				
		}
	}
	//三相设备
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{	
		if ( (phaseCurrent[PHASE_A] > MOTOR_STOP_CURRENT) &&
			(phaseCurrent[PHASE_B] > MOTOR_STOP_CURRENT)  &&
			(phaseCurrent[PHASE_C] > MOTOR_STOP_CURRENT))
		{
			struMotorCtrl.Status = MOTOR_STATUS_RUN;			
		}		
		else
		{
			struMotorCtrl.Status = MOTOR_STATUS_STOP;			
		}
	}
}



#ifdef MOTOR_CTRL_BOARD
/**
 * @brief	更新电机控制主板电机继电器
 * @param	无
 * @return	无
 */
void UpdateMotorRelay(void)
{	
	//发生热过载报警，必须手动复位 add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_FanMotorStop;
		return ;
	}

	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//电机停止
	{
		_FanMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_RUN )	//电机运行
	{
		_FanMotorRun;	
	}
	else												//异常状态
	{
		_FanMotorStop;	
	}
	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
}
#endif //MOTOR_CTRL_BOARD



#ifdef VENTSHADE_CTRL_BOARD
/**
 * @brief	更新小窗/进风口电机继电器
 * @param	无
 * @return	无
 */
void UpdateMotorRelay(void)
{
	//发生热过载报警，必须手动复位 add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_ShadeMotorStop;
		return ;
	}
	
	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//电机停止
	{
		_ShadeMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_OPEN )	//电机向大
	{
		_ShadeMotorOpen;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_CLOSE )//电机向小
	{
		_ShadeMotorClose;
	}
	else												//异常状态
	{
		_ShadeMotorStop;	
	}
	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
}
#endif //defined(VENTSHADE_CTRL_BOARD) || defined(AIRINTAKE_CTRL_BOARD)




#ifdef AIRINTAKE_CTRL_BOARD
/**
 * @brief	更新小窗/进风口电机继电器
 * @param	无
 * @return	无
 */
void UpdateMotorRelay(void)
{
	//发生热过载报警，必须手动复位 add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_ShadeMotorStop;
		return ;
	}

	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//电机停止
	{
		_ShadeMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_OPEN )	//电机向大
	{
		_ShadeMotorOpen;	
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_CLOSE )//电机向小
	{
		_ShadeMotorClose;
	}
	else													//异常状态
	{
		//不建议直接停止继电器	
	}
	///////////////////////////////////////////////电机 继电器更新////////////////////////////////////////////////
}
#endif 


/**
 * @brief	更新电机额定电压
 * @param	无
 * @return	无
 */
void UpdateMotorRatedVoltage(void)
{
	//单相设备
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		struMotorCtrl.RatedVolt	= SINGLE_PHASE_MOTOR_RATED_VOLTAGE;
	}
	//三相设备
	else
	{
		struMotorCtrl.RatedVolt	= THREE_PHASE_MOTOR_RATED_VOLTAGE;	
	}
}





/**
 * @brief	电机故障监控主函数
 * @param	无
 * @return	无
 */
void MotorFaultMonitor(void)
{		
	UpdateMotorRatedVoltage();//更新电机额定电压 add by wyf 20211213
	//基于电压检测的故障监控
	MotorOverVoltageMonitor();//超压故障监控  annotation by wyf 20210708 防止频繁报警
	MotorLowVoltageMonitor();//欠压故障监控	  annotation by wyf 20210708 防止频繁报警

	//风机皮带松/皮带断故障监控
	if ( struDeviceCtrl.Type == DEV_TYPE_FAN )
	{
		MotorNullLoadingMonitor();//空载故障监控-》风机皮带断 额定电流的55%~60%
		MotorUnderLoadingMonitor();//轻载故障监控-》风机皮带松	丢转10%认定为皮带松  额定电流的60%~75%
	}

	#ifndef	LIGHT_CTRL_BOARD	
	//判断当前热过载模式
	//冷态
	if ( struMotorCtrl.OverCurrentMode == OVER_CURRENT_MODE_COLD )
	{
		MotorOverCurrentColdMonitor();//电机过载故障监控-》(冷态)
	}
	//热态
	else
	{
		MotorOverCurrentThermalMonitor();//电机过载故障监控-》(热态)		
	}
	
	//风机启动时长>=10s
	if ( struMotorCtrl.RunLong >= MOTOR_MAX_START_TIME )
	{
		MotorStopExceptionMonitor();
	}
	else
	{
		bmAlarmWord &= ~constbmawMotorStopAlarm;	//停转报警消除	，触摸屏报警改为电机停转
	}
	#endif

	MotorPhaseLossMonitor();//断相故障监控
	//2021/10/23 测试时发现连接下载器时，一直出现短路报警；比较危险，建议取消；原因：复位后，额定电流为0
	//播种机怀疑是电机启动瞬间，RAM或者flash中的额定电流值变为0值
	#ifdef	LIGHT_CTRL_BOARD
	MotorShortCircuitMonitor();//电机短路故障电流监控-》热继电器只能起到延时脱扣作用，模拟断路器实现电机短路电流瞬时脱扣
	#endif
}




