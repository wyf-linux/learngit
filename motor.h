/*************************************************
  Copyright (C), 2021-4-3, EI41s 3driveBoard Ltd
  File name:       motor.h
  Author: wyf      Version:V1.0        Date:2021-4-3
  Description:     ��������Լ���������
  ���������ն˿��ư壬�����豸���ǵ��������û���޸ģ�����ȫ������ɵ����
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

extern void Delay(uint32_t t);//��ʱ


#ifdef MOTOR_CTRL_BOARD
#define _FanMotorRun				{\
										GPIO_PinWrite(GPIO, 0, 21, 1);\
										struMotorCtrl.RelayStatus = RELAY_STATUS_RUN;\
									}	//�����������̵�������
#define _FanMotorStop				{\
										GPIO_PinWrite(GPIO, 0, 21, 0);\
										struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;\
									}	//�����������̵����Ͽ�
#endif //MOTOR_CTRL_BOARD							

									
#ifdef VENTSHADE_CTRL_BOARD								
/////////////////////////////////////////////////////С�����ư�///////////////////////////////////////////////////////
//�ര������ָʾ��
#define _MotorOpenLedOn		{ ( GPIO_PinWrite(GPIO, 0, 0, 1)); }
#define _MotorOpenLedOff	{ ( GPIO_PinWrite(GPIO, 0, 0, 0)); }
//�ര�����Сָʾ��
#define _MotorCloseLedOn	{ ( GPIO_PinWrite(GPIO, 0, 9, 1)); }	
#define _MotorCloseLedOff	{ ( GPIO_PinWrite(GPIO, 0, 9, 0)); }


#define RUN_RELAY_ON			{ ( GPIO_PinWrite(GPIO, 0, 22, 1)); }	//�����̵�������
#define RUN_RELAY_OFF			{ ( GPIO_PinWrite(GPIO, 0, 22, 0)); }	//�����̵����Ͽ�	
#define DIRECTION_RELAY_ON		{ ( GPIO_PinWrite(GPIO, 0, 21, 1)); }	//����̵�������	
#define DIRECTION_RELAY_OFF		{ ( GPIO_PinWrite(GPIO, 0, 21, 0)); }	//����̵����Ͽ�

//С�������ź�
#define _ShadeMotorStop				{\
										RUN_RELAY_OFF;\
										DIRECTION_RELAY_OFF;\
										_MotorOpenLedOff;\
										_MotorCloseLedOff;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_STOP;\
									}	//�രֹͣ 

#define _ShadeMotorOpen				{\
										DIRECTION_RELAY_OFF;\
										RUN_RELAY_ON;\
										_MotorOpenLedOn;\
										_MotorCloseLedOff;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_OPEN;\
									}	//�ര���   ����̵�������Ϊ�͵�ƽ

#define _ShadeMotorClose				{\
										DIRECTION_RELAY_ON;\
										RUN_RELAY_ON;\
										_MotorOpenLedOff;\
										_MotorCloseLedOn;\
										struMotorCtrl.RelayStatus = RELAY_STATUS_CLOSE;\
									}	//�ര��С��   ����̵�������Ϊ�ߵ�ƽ 
/////////////////////////////////////////////////////С�����ư�///////////////////////////////////////////////////////	
#endif //VENTSHADE_CTRL_BOARD
				

#ifdef AIRINTAKE_CTRL_BOARD	
/////////////////////////////////////////////////////����ڿ��ư�///////////////////////////////////////////////////////
//��������ź�
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
/////////////////////////////////////////////////////����ڿ��ư�///////////////////////////////////////////////////////								
#endif //AIRINTAKE_CTRL_BOARD								
									
////////////////////////////////////////	����ܵ�ѹ����	////////////////////////////////////////
#define PHASE_A				0//A��
#define PHASE_B				1//B��
#define PHASE_C				2//C��									
#define MOTOR_PHASE_CNT		3
extern  uint64_t    phaseVoltageRegister[MOTOR_PHASE_CNT];//���ѹ�Ĵ���ֵ	27λ
extern  uint64_t    phaseCurrentRegister[MOTOR_PHASE_CNT];//������Ĵ���ֵ	27λ
extern	uint64_t	phaseVoltage[MOTOR_PHASE_CNT];//���ѹ
extern	uint64_t	lineVoltage[MOTOR_PHASE_CNT];//�ߵ�ѹ	
extern  uint64_t    phaseCurrent[MOTOR_PHASE_CNT];//�����
extern uint64_t     phaseCurrentdisplay[MOTOR_PHASE_CNT];//�������ʾֵ
extern volatile uint8_t flag_Read_Current_250ms;//250ms��ȡ������ѹֵʱ���־λ
////////////////////////////////////////	����ܵ�ѹ����	////////////////////////////////////////



#define THREE_PHASE_MOTOR_RATED_VOLTAGE					3800//���������ѹֵ380V
#define SINGLE_PHASE_MOTOR_RATED_VOLTAGE				2200//���������ѹֵ220V
#define OVER_VOLT_SCALE			(112)//1.12�����ѹ
#define UNDER_VOLT_SCALE		(88) //0.88�����ѹ

#define PHASE_LOSS_ALARM_VOLTAGE			(1000) //���ȱ�౨����ѹֵ,100V
#define SHORT_CIRCUIT_CURRENT_SCALE			(80000)//�����·����ϵ����8��������϶�Ϊ�����·�������ѿ�

#define NULL_LOADING_CURRENT_LOW			(struMotorCtrl.RatedCurrent*5500)//���ص�������
#define NULL_LOADING_CURRENT_HIGH			(struMotorCtrl.RatedCurrent*6000)//���ص�������
#define UNDER_LOADING_CURRENT_LOW			(struMotorCtrl.RatedCurrent*6000)//���ص������� 
#define UNDER_LOADING_CURRENT_HIGH			(struMotorCtrl.RatedCurrent*7500)//���ص������� 

#define MOTOR_MIN_RATED_CURRENT				(1)//�����С����� 5->1 modified by wyf 20211124
#define MOTOR_STOP_CURRENT					(10000)//���ͣת���� modified 500mA->100mA 20211217
//�����Ƿ��趨һ���̶�ֵ������100mA

//add by wyf 20220107 750->1000
#define INVALID_OVER_CURRENT_TIME 	1000 //��λ��ms ��Ч����ʱ�䣬add by wyf 20211217

//�������ʱ��һ��5~10s������10s�ж�
#define MOTOR_MAX_START_TIME		10//���ͣת����ʱ�� modified 2->10 by wyf 20211217


enum MOTOR_TYPE   //�������
{
	MOTOR_TYPE_SINGLE_PHASE = 0x5A,//������
	MOTOR_TYPE_THREE_PHASE = 0xA5,//������
};
enum MOTOR_STATUS //���״̬
{
	MOTOR_STATUS_RUN   = 0x5A,//����
	MOTOR_STATUS_STOP  = 0xA5,//ֹͣ
};
#if defined(MOTOR_CTRL_BOARD) || defined(LIGHT_CTRL_BOARD)
enum RELAY_ORDER //�̵�������ָ��
{
	RELAY_ORDER_RUN   = 0x5A,//����
	RELAY_ORDER_STOP  = 0xA5,//�Ͽ�
};
enum RELAY_STATUS //�̵���״̬
{
	RELAY_STATUS_RUN   = 0x5A,//����
	RELAY_STATUS_STOP  = 0xA5,//�Ͽ�
};
#endif //MOTOR_CTRL_BOARD


#if defined(VENTSHADE_CTRL_BOARD) || defined(AIRINTAKE_CTRL_BOARD)
enum RELAY_ORDER //�̵�������ָ��
{
	RELAY_ORDER_STOP  = 0x00,//ֹͣ
	RELAY_ORDER_OPEN  = 0x5A,//���
	RELAY_ORDER_CLOSE = 0xA5,//��С
};
enum RELAY_STATUS //�̵���״̬
{
	RELAY_STATUS_STOP  = 0x00,//ֹͣ
	RELAY_STATUS_OPEN  = 0x5A,//���
	RELAY_STATUS_CLOSE = 0xA5,//��С
};
#endif

//enum MOTOR_MODE //�������ģʽ add by wyf 20211108 ���������Ӳ���ҳ�����ڲ�������
//{
//	MOTOR_MODE_TEST = 0x5A,//����ģʽ
//	MOTOR_MODE_NORMAL =0xA5,//��������ģʽ
//};


#define OVER_CURRENT_SECTION_COUNT_THERMAL	18//�ȹ�����̬������ modified 17->18 by wyf 20211217
#define OVER_CURRENT_SECTION_COUNT_COLD		17//�ȹ�����̬������ modified 16->17 by wyf 20211217
#define OVER_CURRENT_SECTION_COUNT_MAX		18//�ȹ������������ modified 17->18 by wyf 20211217


//С�͵綯����������һ��Ϊ�1.05��1.1��������������ȡ1.1�����������ڲ��̻�
#define OVER_CURRENT_SCALE_SET	110//���������������
#ifdef DEBUG_FOR_TEST
#define MOTOR_OVER_CURRENT_CLEAR_TIME	(uint32_t)1800
#else
#define MOTOR_OVER_CURRENT_CLEAR_TIME	(uint32_t)1800
#endif


//�ȹ���ģʽ����Ϊ��̬����̬
enum OVER_CURRENT_MODE
{
	OVER_CURRENT_MODE_THERMAL = 0x5A,//��̬
	OVER_CURRENT_MODE_COLD 	= 0xA5,//��̬
};
//�������ṹ����
typedef struct MotorStructType
{
	uint16_t 	RatedCurrent;//�����						MCU 10�����洢
	uint16_t    RatedVolt;//���ѹ add by wyf 20211123	MCU 10���洢
	uint64_t 	LineVoltage[MOTOR_PHASE_CNT];//�ߵ�ѹ		MCU 10�����洢
	uint64_t 	PhaseCurrent[MOTOR_PHASE_CNT];//�����	MCU 100�����洢
	uint16_t 	StartDelay;//������ʱ
	uint16_t	OverCurrentClearTime;//�������κ�һ���ȹ�������ʱ���ۼ�
	uint8_t     OverCurrentMode;//�ȹ���ģʽ
	uint64_t    OverCurrentTime[OVER_CURRENT_SECTION_COUNT_MAX];//��ͬ�����ȹ��ؼ�ʱ
	uint8_t     OverCurrentSection;//��ǰ�������ȹ�������
	uint8_t     Mode;//����ģʽ for ����������ģʽ
	uint16_t	ReverseDelay;//������ʱʱ��		add by wyf 20211110 ���ڲര/Ļ��
	uint16_t 	RunLong;//����ʱ�䳤��
	uint8_t		Type;//����
	uint8_t     OldStatus;//���֮ǰ������״̬
	uint8_t 	Status;//�����ǰ״̬
	uint8_t		RelayOrder;//�̵�������ָ��
	uint8_t		RelayStatus;//�̵���״̬
	uint16_t    ActivePower;//�й�����
	uint16_t    Lifetime;//�������
} MOTORSTRUCTTYPE;
extern volatile MOTORSTRUCTTYPE struMotorCtrl;//������ƽṹ��

extern volatile uint8_t flag_UpdateMotorStatus_1s;//1s����һ���豸����״̬

extern volatile uint8_t flag_Push_OverLoad_Time_1s;//add by wyf 20211216 for debug


extern void MotorCtrlInit(void);//������Ƴ�ʼ��
extern void MotorFaultMonitor(void);//������ϼ��������
extern void MotorExceptionCtrl(void);//����쳣������ƣ���Ҫ����ȹ��ء���·�����ҽ������¸�λ���������;
extern void UpdateMotorRelay(void);//���µ���̵���
extern void UpdateMotorStatus(void);//���µ������״̬
extern void UpdateMotorRatedVoltage(void);//���µ�����ѹ


#endif // __MOTOR_H_

