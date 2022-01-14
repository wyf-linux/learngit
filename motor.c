/*************************************************
  Copyright (C), 2021-4-3, EI41s 3driveBoard Ltd
  File name:       motor.c
  Author: wyf      Version:V1.0        Date:2021-4-3
  Description:     ��������Լ���������
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


volatile uint8_t flag_UpdateMotorStatus_1s;//1s����һ���豸����״̬
////////////////////////////////////////	����ܵ�ѹ����	////////////////////////////////////////
uint64_t    phaseVoltageRegister[MOTOR_PHASE_CNT];//���ѹ�Ĵ���ֵ	27λ
uint64_t    phaseCurrentRegister[MOTOR_PHASE_CNT];//������Ĵ���ֵ	27λ
uint64_t	phaseVoltage[MOTOR_PHASE_CNT];//���ѹ
uint64_t	lineVoltage[MOTOR_PHASE_CNT];//�ߵ�ѹ	
uint64_t    phaseCurrent[MOTOR_PHASE_CNT];//�����
uint64_t    phaseCurrentdisplay[MOTOR_PHASE_CNT];//�������ʾֵ
volatile uint8_t flag_Read_Current_250ms;//250ms��ȡ������ѹֵʱ���־λ
////////////////////////////////////////	����ܵ�ѹ����	////////////////////////////////////////



volatile MOTORSTRUCTTYPE struMotorCtrl;//������ƽṹ��

volatile uint8_t flag_Push_OverLoad_Time_1s;//add by wyf 20211216 for debug



//�ȹ�����̬��ͬ�����Ӧ��������������
const uint32_t gThermalOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_THERMAL] = 
{
	110,		//�ȹ��ر���1��Ӧ����������ϵ��1.1
	120,		//�ȹ��ر���2��Ӧ����������ϵ��1.2	
	130,		//�ȹ��ر���3��Ӧ����������ϵ��1.3
	140,		//�ȹ��ر���4��Ӧ����������ϵ��1.4
	150,		//�ȹ��ر���5��Ӧ����������ϵ��1.5
	160,		//�ȹ��ر���6��Ӧ����������ϵ��1.6
	170,		//�ȹ��ر���7��Ӧ����������ϵ��1.7
	180,		//�ȹ��ر���8��Ӧ����������ϵ��1.8
	190,		//�ȹ��ر���9��Ӧ����������ϵ��1.9
	200,		//�ȹ��ر���10��Ӧ����������ϵ��2.0
	245,		//�ȹ��ر���11��Ӧ����������ϵ��2.45
	250,		//�ȹ��ر���12��Ӧ����������ϵ��2.5
	300,		//�ȹ��ر���13��Ӧ����������ϵ��3.0
	350,		//�ȹ��ر���14��Ӧ����������ϵ��3.5
	400,		//�ȹ��ر���15��Ӧ����������ϵ��4.0
	500,		//�ȹ��ر���16��Ӧ����������ϵ��5.0
	600,		//�ȹ��ر���17��Ӧ����������ϵ��6.0
	720,		//�ȹ��ر���18��Ӧ����������ϵ��7.0
};

//�ȹ�����̬��ͬ�����Ӧ���ѿ�ʱ�� ��λms
const uint32_t gThermalOverCurrentTableTime[OVER_CURRENT_SECTION_COUNT_THERMAL] = 
{
	400000,		//�ȹ�������1��Ӧ���ѿ�ʱ�� 400s [1.1,1.2)
	244000,		//�ȹ�������2��Ӧ���ѿ�ʱ�� 244s [1.2,1.3)
	116000,		//�ȹ�������3��Ӧ���ѿ�ʱ�� 116s [1.3,1.4)
	82000,		//�ȹ�������4��Ӧ���ѿ�ʱ�� 82s  [1.4,1.5)
	78000,		//�ȹ�������5��Ӧ���ѿ�ʱ�� 78s  [1.5,1.6)
	74000,		//�ȹ�������6��Ӧ���ѿ�ʱ�� 74s  [1.6,1.7)
	72000,		//�ȹ�������7��Ӧ���ѿ�ʱ�� 72s  [1.7,1.8)	
	71000,		//�ȹ�������8��Ӧ���ѿ�ʱ�� 71s  [1.8,1.9)
	70000,		//�ȹ�������9��Ӧ���ѿ�ʱ�� 70s  [1.9,2.0)
	66000,		//�ȹ�������10��Ӧ���ѿ�ʱ�� 66s  [2.0,2.45)
	52000,		//�ȹ�������11��Ӧ���ѿ�ʱ�� 52s  [2.45,2.5)
	27000,		//�ȹ�������12��Ӧ���ѿ�ʱ�� 27s  [2.5,3.0)	
	18000,		//�ȹ�������13��Ӧ���ѿ�ʱ�� 18s  [3.0,3.5)	
	14000,		//�ȹ�������14��Ӧ���ѿ�ʱ�� 14s  [3.5,4.0)
	8000,		//�ȹ�������15��Ӧ���ѿ�ʱ�� 8s  [4.0,5.0)	
	6000,		//�ȹ�������16��Ӧ���ѿ�ʱ�� 6s  [5.0,6.0)
	4000,		//�ȹ�������17��Ӧ���ѿ�ʱ�� 4s  [6.0,7.2)
	2000,		//�ȹ�������18��Ӧ���ѿ�ʱ�� 2s  [7.2, �����
};



//�ȹ�����̬��ͬ�����Ӧ��������������
const uint32_t gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD] = 
{
	110,		//�ȹ��ر���1��Ӧ����������ϵ��1.1
	120,		//�ȹ��ر���2��Ӧ����������ϵ��1.2	
	130,		//�ȹ��ر���3��Ӧ����������ϵ��1.3
	140,		//�ȹ��ر���4��Ӧ����������ϵ��1.4
	150,		//�ȹ��ر���5��Ӧ����������ϵ��1.5
	160,		//�ȹ��ر���6��Ӧ����������ϵ��1.6
	170,		//�ȹ��ر���7��Ӧ����������ϵ��1.7
	180,		//�ȹ��ر���8��Ӧ����������ϵ��1.8
	190,		//�ȹ��ر���9��Ӧ����������ϵ��1.9
	200,		//�ȹ��ر���10��Ӧ����������ϵ��2.0
	250,		//�ȹ��ر���11��Ӧ����������ϵ��2.5
	300,		//�ȹ��ر���12��Ӧ����������ϵ��3.0
	350,		//�ȹ��ر���13��Ӧ����������ϵ��3.5
	400,		//�ȹ��ر���14��Ӧ����������ϵ��4.0
	500,		//�ȹ��ر���15��Ӧ����������ϵ��5.0
	600,		//�ȹ��ر���16��Ӧ����������ϵ��6.0
	720,		//�ȹ��ر���17��Ӧ����������ϵ��7.2
};
//�ȹ�����̬��ͬ�����Ӧ���ѿ�ʱ�� ��λms
const uint32_t gColdOverCurrentTableTime[OVER_CURRENT_SECTION_COUNT_COLD] = 
{
	700000,		//�ȹ�������1��Ӧ���ѿ�ʱ�� 700s [1.1,1.2)
	460000,		//�ȹ�������2��Ӧ���ѿ�ʱ�� 460s [1.2,1.3)
	280000,		//�ȹ�������3��Ӧ���ѿ�ʱ�� 280s [1.3,1.4)
	208000,		//�ȹ�������4��Ӧ���ѿ�ʱ�� 208s  [1.4,1.5)
	182000,		//�ȹ�������5��Ӧ���ѿ�ʱ�� 182s  [1.5,1.6)
	162000,		//�ȹ�������6��Ӧ���ѿ�ʱ�� 162s  [1.6,1.7)
	145000,		//�ȹ�������7��Ӧ���ѿ�ʱ�� 145s  [1.7,1.8)	
	129000,		//�ȹ�������8��Ӧ���ѿ�ʱ�� 129s  [1.8,1.9)
	116000,		//�ȹ�������9��Ӧ���ѿ�ʱ�� 116s  [1.9,2.0)
	82000,		//�ȹ�������10��Ӧ���ѿ�ʱ�� 82s  [2.0,2.5)
	49000,		//�ȹ�������11��Ӧ���ѿ�ʱ�� 49s  [2.5,3.0)
	34000,		//�ȹ�������12��Ӧ���ѿ�ʱ�� 34s  [3.0,3.5)	
	28000,		//�ȹ�������13��Ӧ���ѿ�ʱ�� 28s  [3.5,4.0)	
	20000,		//�ȹ�������14��Ӧ���ѿ�ʱ�� 20s  [4.0,5.0)
	12000,		//�ȹ�������15��Ӧ���ѿ�ʱ�� 12s  [5.0,6.0)	
	7000,		//�ȹ�������16��Ӧ���ѿ�ʱ�� 7s  [6.0,7.2)	
	3000,		//�ȹ�������17��Ӧ���ѿ�ʱ�� 3s  [7.2,�����)	
};



/**
 * @brief	��ʱ
 * @param	��
 * @return	��
 */
void Delay(uint32_t t)//��ʱ
{
	if(t==0)
		return;
	while(t--);
}


/**
 * @brief	������Ƴ�ʼ��
 * @param	��
 * @return	��
 */
void MotorCtrlInit(void)
{
	//�������δ���û��ߴ���ִ��Ĭ�ϳ�ʼ������
	if ((struMotorCtrl.Type != MOTOR_TYPE_SINGLE_PHASE) &&
	(struMotorCtrl.Type != MOTOR_TYPE_THREE_PHASE))
	{
		//���
		if ( struDeviceCtrl.Type == DEV_TYPE_FAN )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//С��
		else if ( struDeviceCtrl.Type == DEV_TYPE_VENTSHADE )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}
		//����
		else if ( struDeviceCtrl.Type == DEV_TYPE_ADDTEMP )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}
		//�����
		else if ( struDeviceCtrl.Type == DEV_TYPE_AIRINTAKE )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//ʪ��
		else if ( struDeviceCtrl.Type == DEV_TYPE_WETPUMP )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//����
		else if ( struDeviceCtrl.Type == DEV_TYPE_MALESUPPLY )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//ι��
		else if ( struDeviceCtrl.Type == DEV_TYPE_FEEDING )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//����
		else if ( struDeviceCtrl.Type == DEV_TYPE_LIGHT )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;//modified by wyf Flash���α���  �޸�Ϊ������	
		}
		//��ҩ
		else if ( struDeviceCtrl.Type == DEV_TYPE_DOSING )
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;		
		}	
		//����
		else if ( struDeviceCtrl.Type == DEV_TYPE_FOG )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//���
		else if ( struDeviceCtrl.Type == DEV_TYPE_SHIT )
		{
			struMotorCtrl.Type = MOTOR_TYPE_THREE_PHASE;		
		}
		//Ĭ�ϵ����� ֻ���1����� wyf add 2021/10/26
		else
		{
			struMotorCtrl.Type = MOTOR_TYPE_SINGLE_PHASE;
		}
	}
	//Ĭ�϶�������Ϊ0�����ʼ��Ϊ500mA����С��������Ҫ����0ֵ���ȹ��ر�������ֵͨ��������õ���
	if ( struMotorCtrl.RatedCurrent < MOTOR_MIN_RATED_CURRENT )
	{
		struMotorCtrl.RatedCurrent = MOTOR_MIN_RATED_CURRENT;//modified by wyf 20211106
	}

	struMotorCtrl.Status = MOTOR_STATUS_STOP;
	struMotorCtrl.OldStatus = MOTOR_STATUS_STOP;
	struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;	
	struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
	
	//�ж��ȹ�����̬������̬
	if (  struDeviceCtrl.Type == DEV_TYPE_SHIT  )
	{
		struMotorCtrl.OverCurrentMode = OVER_CURRENT_MODE_COLD;	//��ఴ����̬����
	}
	else
	{
		struMotorCtrl.OverCurrentMode = OVER_CURRENT_MODE_THERMAL;	//�������尴����̬����
	}
	
	//�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		struMotorCtrl.RatedVolt	= SINGLE_PHASE_MOTOR_RATED_VOLTAGE;
		phaseVoltage[PHASE_A] = struMotorCtrl.RatedVolt;
	}
	//�����豸
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
 * @brief	������ؿ���(��̬)
 * @param	��
 * @return	��
 */
static void MotorOverCurrentColdMonitor(void)
{
	uint64_t ISet = 0;//������������趨ֵ
	uint8_t hasLevel = 0;//����ĳһ���ȹ��ط�Χ����
	uint64_t ratio = 0;	
	uint8_t overCurrentSection = 0;
	
	//���㵱ǰ�ȹ�����������ֵ
	ISet = struMotorCtrl.RatedCurrent*OVER_CURRENT_SCALE_SET;
	//�жϵ�ǰ�ȹ��صȼ�
	//����1�������
	for ( uint8_t phase = PHASE_A; phase <= PHASE_C; phase++)
	{
		//�����豸   ע�� by wyf 20211230
//		if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE ) 
//		{
//			//B�����C��
//			if ( phase != PHASE_A )
//			{
//				break;
//			}
//		}		
		//�жϵ�ǰ�����������ȹ�������
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
		//�ж��Ƿ������һ���ȹ�������  add by wy 20211217
		if ( phaseCurrent[phase] >= ISet*gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD-1]) 
		{
			overCurrentSection = OVER_CURRENT_SECTION_COUNT_COLD;
		}
	}
	struMotorCtrl.OverCurrentSection = overCurrentSection;//ȷ����ǰ�ȹ�������,���䷶Χ1~n
	//�������κ�һ���ȹ�������
	if ( (!struMotorCtrl.OverCurrentSection) && (struMotorCtrl.OverCurrentClearTime >= MOTOR_OVER_CURRENT_CLEAR_TIME))
	{
		struMotorCtrl.OverCurrentClearTime = 0;
		//�������ȹ���ʱ����0
		for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_COLD; section++)
		{
			struMotorCtrl.OverCurrentTime[section] = 0;//�ȹ���ʱ����0
		}				
	}

	
	//�ж��ȹ��ر���
	for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_COLD; section++)
	{
		//ֹͣ��������ص���ʱ�����500ms���϶�Ϊ�����������߸��ţ�ʱ����0 add by wyf 20211217
		if ( (struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) && 
			(struMotorCtrl.OverCurrentTime[section] <= INVALID_OVER_CURRENT_TIME) )
		{
			struMotorCtrl.OverCurrentTime[section] = 0;
		}
		ratio += struMotorCtrl.OverCurrentTime[section] *100 / gColdOverCurrentTableTime[section];	
	}
	if ( ratio >= 100 )
	{
		bmAlarmWord |= constbmawOverCurrentAlarm;	//�ȹ���	
	}
}




/**
 * @brief	������ؿ���(��̬)
 * @param	��
 * @return	��
 */
static void MotorOverCurrentThermalMonitor(void)
{
	uint64_t ISet = 0;//������������趨ֵ
	uint8_t hasLevel = 0;//����ĳһ���ȹ��ط�Χ����
	uint64_t ratio = 0;	
	uint8_t overCurrentSection = 0;
	//���㵱ǰ�ȹ�����������ֵ
	ISet = struMotorCtrl.RatedCurrent*OVER_CURRENT_SCALE_SET;
	//�жϵ�ǰ�ȹ��صȼ�
	//����1�������
	for ( uint8_t phase = PHASE_A; phase <= PHASE_C; phase++)
	{
		//�����豸     ע�� by wyf 20211230
//		if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE ) 
//		{
//			//B�����C��
//			if ( phase != PHASE_A )
//			{
//				break;
//			}
//		}		
		//�жϵ�ǰ�����������ȹ�������
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
		//�ж��Ƿ������һ���ȹ�������  add by wy 20211217
		if ( phaseCurrent[phase] >= ISet*gColdOverCurrentTableScale[OVER_CURRENT_SECTION_COUNT_COLD-1]) 
		{
			overCurrentSection = OVER_CURRENT_SECTION_COUNT_THERMAL;
		}
	}
	struMotorCtrl.OverCurrentSection = overCurrentSection;//ȷ����ǰ�ȹ�������,���䷶Χ1~n
	//�������κ�һ���ȹ�������
	if ( (!struMotorCtrl.OverCurrentSection) && (struMotorCtrl.OverCurrentClearTime >= MOTOR_OVER_CURRENT_CLEAR_TIME))
	{
		struMotorCtrl.OverCurrentClearTime = 0;
		//�������ȹ���ʱ����0
		for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_THERMAL; section++)
		{
			struMotorCtrl.OverCurrentTime[section] = 0;//�ȹ���ʱ����0
		}				
	}

	
	//�ж��ȹ��ر���
	for ( uint8_t section = 0; section < OVER_CURRENT_SECTION_COUNT_THERMAL; section++)
	{
		//ֹͣ��������ص���ʱ�����500ms���϶�Ϊ�����������߸��ţ�ʱ����0 add by wyf 20211217
		if ( (struMotorCtrl.RelayStatus == RELAY_STATUS_STOP) && 
			(struMotorCtrl.OverCurrentTime[section] <= INVALID_OVER_CURRENT_TIME) )
		{
			struMotorCtrl.OverCurrentTime[section] = 0;
		}
		//��������
		ratio += ((struMotorCtrl.OverCurrentTime[section] *1000 / gThermalOverCurrentTableTime[section]+5)/10);	
	}
	if ( ratio >= 100 )
	{
		bmAlarmWord |= constbmawOverCurrentAlarm;	//�ȹ���	
	}
}
#endif //LIGHT_CTRL_BOARD




/**
 * @brief	���Ƿѹ���ϼ��
 * @param	��
 * @return	��
 */
void MotorLowVoltageMonitor(void)
{
	//�޶����1��С��1�жϳ�ѹ add by wyf 20211215
	if ( (struDeviceCtrl.Number != DEV_START_ADDR_FAN) && 
		(struDeviceCtrl.Number != DEV_START_ADDR_VENTSHADE))
	{
		return ;
	}
	
	
	uint32_t underVoltValue = struMotorCtrl.RatedVolt*UNDER_VOLT_SCALE/100;
	 //�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	
	{
		if ( ((phaseVoltage[PHASE_A] <= underVoltValue) &&
				(phaseVoltage[PHASE_A] > PHASE_LOSS_ALARM_VOLTAGE)))	//A��
		{
			//��ΪǷѹ
			bmAlarmWord |= constbmawUnderVoltageAlarm;	//������Ϊʲô��Ƿѹ
		}	
		else
		{
			bmAlarmWord &= ~constbmawUnderVoltageAlarm;
		}		
	}
	//�����豸
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( ((lineVoltage[PHASE_A] <= underVoltValue) &&
				(lineVoltage[PHASE_A] > PHASE_LOSS_ALARM_VOLTAGE))	||		//A��
			((lineVoltage[PHASE_B] <= underVoltValue) &&
				(lineVoltage[PHASE_B] > PHASE_LOSS_ALARM_VOLTAGE))	||		//B��		
			((lineVoltage[PHASE_C] <= underVoltValue) &&		
				(lineVoltage[PHASE_C] > PHASE_LOSS_ALARM_VOLTAGE)))		//C��
		{
			//��ΪǷѹ
			bmAlarmWord |= constbmawUnderVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawUnderVoltageAlarm;
		}	
	}
}



/**
 * @brief	�����ѹ���ϼ��
 * @param	��
 * @return	��
 */
void MotorOverVoltageMonitor(void)
{
	//�޶����1��С��1�жϳ�ѹ add by wyf 20211215
	if ( (struDeviceCtrl.Number != DEV_START_ADDR_FAN) && 
		(struDeviceCtrl.Number != DEV_START_ADDR_VENTSHADE))
	{
		return ;
	}
	
	
	uint32_t overVoltValue = struMotorCtrl.RatedVolt*OVER_VOLT_SCALE/100;
	//�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseVoltage[PHASE_A] >= overVoltValue ) 	//A��
		{
			//��Ϊ��ѹ
			bmAlarmWord |= constbmawOverVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawOverVoltageAlarm;
		}		
	}
	//�����豸
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( (lineVoltage[PHASE_A] >= overVoltValue) ||		//AB
			(lineVoltage[PHASE_B] >= overVoltValue) ||		//BC
			(lineVoltage[PHASE_C] >= overVoltValue) )		//AC
		{
			//��Ϊ��ѹ
			bmAlarmWord |= constbmawOverVoltageAlarm;		
		}	
		else
		{
			bmAlarmWord &= ~constbmawOverVoltageAlarm;
		}	
	}		
}



/**
 * @brief	���������ϼ��
 * @param	��
 * @return	��
 */
void MotorPhaseLossMonitor(void)
{
	//�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseVoltage[PHASE_A] <= PHASE_LOSS_ALARM_VOLTAGE )
		{
			bmAlarmWord |= constbmawPhaseLossAlarm;	
			//ֹͣ�������
			struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
//			#ifdef LIGHT_CTRL_BOARD
//			//�������
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);	
//			#endif
		}	
		else
		{
			bmAlarmWord &= ~constbmawPhaseLossAlarm;	
		}		
	}
	//�����豸
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{
		if ( (lineVoltage[PHASE_A] <= PHASE_LOSS_ALARM_VOLTAGE) ||
			(lineVoltage[PHASE_B] <= PHASE_LOSS_ALARM_VOLTAGE)  ||
			(lineVoltage[PHASE_C] <= PHASE_LOSS_ALARM_VOLTAGE))
		{
			bmAlarmWord |= constbmawPhaseLossAlarm;	
			//ֹͣ�������
			struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
//			#ifdef LIGHT_CTRL_BOARD
//			//�������
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
 * @brief	�����·���ϼ��
 * @param	��
 * @return	��
 */
static void MotorShortCircuitMonitor(void)
{
	uint64_t shortCircuitCurrent = 0;
//	//��·�������ڶ������8�����϶�Ϊ��·
//	shortCircuitCurrent = struMotorCtrl.RatedCurrent*SHORT_CIRCUIT_CURRENT_SCALE;

	if ( struDeviceCtrl.Type == DEV_TYPE_LIGHT )
	{
		shortCircuitCurrent = struMotorCtrl.RatedCurrent*10000;
	}

	
	//�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		if ( phaseCurrent[PHASE_A] > shortCircuitCurrent )
		{
			bmAlarmWord |= constbmawShortCircuitAlarm;	
			//�������
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);
			//ֹͣ������У����ҽ�����λ������������
//			struMotorCtrl.Order = MOTOR_ORDER_STOP;
		}	
		else
		{
			bmAlarmWord &= ~constbmawShortCircuitAlarm;		
		}
	}
	//�����豸
	else if ( struMotorCtrl.Type == MOTOR_TYPE_THREE_PHASE )
	{	
		if ( (phaseCurrent[PHASE_A] > shortCircuitCurrent) ||
			(phaseCurrent[PHASE_B] > shortCircuitCurrent)  ||
			(phaseCurrent[PHASE_C] > shortCircuitCurrent))
		{
			bmAlarmWord |= constbmawShortCircuitAlarm;
			//�������
//			WriteAD5302(LIGHT_CHANNEL, LIGHTOFF);			
			//ֹͣ������У����ҽ�����λ������������
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
 * @brief	������ع��ϼ��
 * @param	��
 * @return	��
 */
static void MotorUnderLoadingMonitor(void)
{
	//�������ʱ��<10s����������������У����е������ڿ��ص�����Χ�� add by wyf 20211229
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
		bmAlarmWord |= constbmawUnderLoadingAlarm;	//ֻ����������
	}
	else
	{
		bmAlarmWord &= ~constbmawUnderLoadingAlarm;	
	}
}



/**
 * @brief	������ع��ϼ��
 * @param	��
 * @return	��
 */
static void MotorNullLoadingMonitor(void)
{
	//�������ʱ��<10s����������������У����е������ڿ��ص�����Χ�� add by wyf 20211229
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
		bmAlarmWord |= constbmawNullLoadingAlarm;	//ֻ����������
	}	
	else
	{
		bmAlarmWord &= ~constbmawNullLoadingAlarm;	
	}
}





/**
 * @brief	���ͣת���ϼ��
 * @param	��
 * @return	��
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
		//���ͣת����
		bmAlarmWord |= constbmawMotorStopAlarm;	//ֻ����������
	}
	else
	{
	 	bmAlarmWord &= ~constbmawMotorStopAlarm;	
	}
}






/**
 * @brief	����쳣������ƣ���Ҫ����ȹ��ء���·�����ҽ������¸�λ���������;
 * @param	��
 * @return	��
 */
void MotorExceptionCtrl(void)
{
	//����������¹��ϣ���·���ȹ��أ����ֹͣ
	if ( (bmAlarmWord & constbmawShortCircuitAlarm) ||
		(bmAlarmWord & constbmawOverCurrentAlarm))
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;	
	}
}






/**
 * @brief	���µ������״̬
 * @param	��
 * @return	��
 */
void UpdateMotorStatus(void)
{	
	//�����豸
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
	//�����豸
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
 * @brief	���µ�������������̵���
 * @param	��
 * @return	��
 */
void UpdateMotorRelay(void)
{	
	//�����ȹ��ر����������ֶ���λ add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_FanMotorStop;
		return ;
	}

	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//���ֹͣ
	{
		_FanMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_RUN )	//�������
	{
		_FanMotorRun;	
	}
	else												//�쳣״̬
	{
		_FanMotorStop;	
	}
	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
}
#endif //MOTOR_CTRL_BOARD



#ifdef VENTSHADE_CTRL_BOARD
/**
 * @brief	����С��/����ڵ���̵���
 * @param	��
 * @return	��
 */
void UpdateMotorRelay(void)
{
	//�����ȹ��ر����������ֶ���λ add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_ShadeMotorStop;
		return ;
	}
	
	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//���ֹͣ
	{
		_ShadeMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_OPEN )	//������
	{
		_ShadeMotorOpen;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_CLOSE )//�����С
	{
		_ShadeMotorClose;
	}
	else												//�쳣״̬
	{
		_ShadeMotorStop;	
	}
	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
}
#endif //defined(VENTSHADE_CTRL_BOARD) || defined(AIRINTAKE_CTRL_BOARD)




#ifdef AIRINTAKE_CTRL_BOARD
/**
 * @brief	����С��/����ڵ���̵���
 * @param	��
 * @return	��
 */
void UpdateMotorRelay(void)
{
	//�����ȹ��ر����������ֶ���λ add by wyf 20211120
	if ( bmAlarmWord & constbmawOverCurrentAlarm )
	{
		struMotorCtrl.RelayOrder = RELAY_ORDER_STOP;
		_ShadeMotorStop;
		return ;
	}

	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
	if ( struMotorCtrl.RelayOrder == RELAY_ORDER_STOP )		//���ֹͣ
	{
		_ShadeMotorStop;
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_OPEN )	//������
	{
		_ShadeMotorOpen;	
	}
	else if ( struMotorCtrl.RelayOrder == RELAY_ORDER_CLOSE )//�����С
	{
		_ShadeMotorClose;
	}
	else													//�쳣״̬
	{
		//������ֱ��ֹͣ�̵���	
	}
	///////////////////////////////////////////////��� �̵�������////////////////////////////////////////////////
}
#endif 


/**
 * @brief	���µ�����ѹ
 * @param	��
 * @return	��
 */
void UpdateMotorRatedVoltage(void)
{
	//�����豸
	if ( struMotorCtrl.Type == MOTOR_TYPE_SINGLE_PHASE )
	{
		struMotorCtrl.RatedVolt	= SINGLE_PHASE_MOTOR_RATED_VOLTAGE;
	}
	//�����豸
	else
	{
		struMotorCtrl.RatedVolt	= THREE_PHASE_MOTOR_RATED_VOLTAGE;	
	}
}





/**
 * @brief	������ϼ��������
 * @param	��
 * @return	��
 */
void MotorFaultMonitor(void)
{		
	UpdateMotorRatedVoltage();//���µ�����ѹ add by wyf 20211213
	//���ڵ�ѹ���Ĺ��ϼ��
	MotorOverVoltageMonitor();//��ѹ���ϼ��  annotation by wyf 20210708 ��ֹƵ������
	MotorLowVoltageMonitor();//Ƿѹ���ϼ��	  annotation by wyf 20210708 ��ֹƵ������

	//���Ƥ����/Ƥ���Ϲ��ϼ��
	if ( struDeviceCtrl.Type == DEV_TYPE_FAN )
	{
		MotorNullLoadingMonitor();//���ع��ϼ��-�����Ƥ���� �������55%~60%
		MotorUnderLoadingMonitor();//���ع��ϼ��-�����Ƥ����	��ת10%�϶�ΪƤ����  �������60%~75%
	}

	#ifndef	LIGHT_CTRL_BOARD	
	//�жϵ�ǰ�ȹ���ģʽ
	//��̬
	if ( struMotorCtrl.OverCurrentMode == OVER_CURRENT_MODE_COLD )
	{
		MotorOverCurrentColdMonitor();//������ع��ϼ��-��(��̬)
	}
	//��̬
	else
	{
		MotorOverCurrentThermalMonitor();//������ع��ϼ��-��(��̬)		
	}
	
	//�������ʱ��>=10s
	if ( struMotorCtrl.RunLong >= MOTOR_MAX_START_TIME )
	{
		MotorStopExceptionMonitor();
	}
	else
	{
		bmAlarmWord &= ~constbmawMotorStopAlarm;	//ͣת��������	��������������Ϊ���ͣת
	}
	#endif

	MotorPhaseLossMonitor();//������ϼ��
	//2021/10/23 ����ʱ��������������ʱ��һֱ���ֶ�·�������Ƚ�Σ�գ�����ȡ����ԭ�򣺸�λ�󣬶����Ϊ0
	//���ֻ������ǵ������˲�䣬RAM����flash�еĶ����ֵ��Ϊ0ֵ
	#ifdef	LIGHT_CTRL_BOARD
	MotorShortCircuitMonitor();//�����·���ϵ������-���ȼ̵���ֻ������ʱ�ѿ����ã�ģ���·��ʵ�ֵ����·����˲ʱ�ѿ�
	#endif
}




