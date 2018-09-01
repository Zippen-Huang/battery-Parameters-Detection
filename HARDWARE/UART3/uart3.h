#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u16 USART3_RX_STA;         		//����״̬���

//extern u8 USART_TX_STA;         		//����״̬���

//��ز���
extern float Work_full;//��λwh


extern u8 copy_rxrefreshdata[15];//��Դ�������� ����

extern u8 powerSta[2];//�洢��Դ״̬��Ϣ����ѹ-����

//�洢����ʱ��ĵ�Դ��Ϣ
struct PowerOnStaArgStruct
{
	float voltagePowerOnArg;//������ѹ
	float qValuePowerOnArg;//��������
	float workValuePowerOnArg;//������
};
extern struct PowerOnStaArgStruct powerOnStaArg;

//��ʼ��IO ����3 
void uart3_init(u32 bound);

//��ȡ��Դģ������Ĳ������ڴ����ж��з��ظ��µ�����-��ʽ��16����
void uart3_send_refreshMsgCmd(void);

//���ͳ�ʼ����Դģ��ָ��
void uart3_send_initPowerMsgCmd(void);

//���������Դģ���ۼ�����ָ��
void uart3_send_ClearEnergyMsgCmd(void);

//���뵱ǰ��ѹV�����ص�ǰ����Ah�����ڿ�����⵱ǰ�ĵ�ѹ��������������⣩
float Q_sta_cal_powerOn(float v); 

//ͨ����ǰ�ĵ�ѹ�͵������㿪��ʱ����ܹ�
float W_powerOn_cal(float Q_poweron,float U_poweron);

//�ӷ����ĵ�Դ���������еõ���ǰ��ѹֵ ��λV
float current_voltage_read(void);

//�ӷ����ĵ�Դ���������еõ���ǰ���ĵ����� ��λwh
unsigned long current_work_read(void);

//�ӷ����ĵ�Դ���������еõ���ǰ�ĵ��� ��λA
float DC_current_read(void);

//�ӷ����ĵ�Դ���������еõ���ǰ�Ĺ��� ��λw
float DC_power_read(void);
	
//��Դģ�鿪����ʼ��
void powerOn_init(void);

//��Դģ�������ȡ
void powerSta_measure(void);
	
#endif
