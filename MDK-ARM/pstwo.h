#ifndef __PSTWO_H
#define __PSTWO_H
#include "main.h"
/*********************************************************
Copyright (C), 2015-2025, YFRobot.
www.yfrobot.com
File��PS2��������
Author��pinggai    Version:1.1     Data:2015/10/20
Description: PS2��������
             ���ӹ��ܣ�
			 1��������á����ģʽ�������̵�ģʽ�������������á����桱��ͨ���ֱ���ģʽ�������޷��ı�
			 2�������ֱ��𶯣�ͨ����ֵ�����ã��ı������𶯵����Ƶ�ʡ�
			                  ͨ����ֵ�����ã������Ҳ�С�𶯵����
History:  
V1.0: 	2015/05/16
1���ֱ����룬ʶ�𰴼�ֵ����ȡģ��ֵ��       
**********************************************************/	 
#define DI   HAL_GPIO_ReadPin(DI_GPIO_Port,DI_Pin)           //PB12  ����

#define DO_H  HAL_GPIO_WritePin(DO_GPIO_Port, DO_Pin, GPIO_PIN_SET)       //����λ��
#define DO_L  HAL_GPIO_WritePin(DO_GPIO_Port, DO_Pin, GPIO_PIN_RESET)        //����λ��

#define CS_H HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)       //CS����
#define CS_L HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)       //CS����

#define CLK_H HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET)      //ʱ������
#define CLK_L HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET)      //ʱ������


//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5				//��ҡ��X������
#define PSS_RY 6				//��ҡ��Y������	
#define PSS_LX 7				//��ҡ��X������
#define PSS_LY 8				//��ҡ��Y������

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;

void PS2_Init(void);
uint8_t PS2_RedLight(void);   //�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void); //���ֱ�����
void PS2_Cmd(uint8_t CMD);		  //���ֱ���������
uint8_t PS2_DataKey(void);		  //����ֵ��ȡ
uint8_t PS2_AnologData(uint8_t button); //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  //������ݻ�����
void PS2_Vibration(uint8_t motor1, uint8_t motor2);//������motor1  0xFF���������أ�motor2  0x40~0xFF

void PS2_EnterConfing(void);	 //��������
void PS2_TurnOnAnalogMode(void); //����ģ����
void PS2_VibrationMode(void);    //������
void PS2_ExitConfing(void);	     //�������
void PS2_SetInit(void);		     //���ó�ʼ��

#endif
