#ifndef __USART3_H
#define __USART3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 


#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u16 USART3_RX_STA;         		//接收状态标记

//extern u8 USART_TX_STA;         		//发送状态标记

//电池参数
extern float Work_full;//单位wh


extern u8 copy_rxrefreshdata[15];//电源测量数据 副本

extern u8 powerSta[2];//存储电源状态信息：电压-电量

//存储开机时候的电源信息
struct PowerOnStaArgStruct
{
	float voltagePowerOnArg;//开机电压
	float qValuePowerOnArg;//开机电量
	float workValuePowerOnArg;//开机功
};
extern struct PowerOnStaArgStruct powerOnStaArg;

//初始化IO 串口3 
void uart3_init(u32 bound);

//读取电源模块测量的参数，在串口中断中返回更新的数据-格式：16进制
void uart3_send_refreshMsgCmd(void);

//发送初始化电源模块指令
void uart3_send_initPowerMsgCmd(void);

//发送清除电源模块累计数据指令
void uart3_send_ClearEnergyMsgCmd(void);

//输入当前电压V，返回当前电量Ah（用于开机检测当前的电压，方法：开环检测）
float Q_sta_cal_powerOn(float v); 

//通过当前的电压和电量计算开机时候的总功
float W_powerOn_cal(float Q_poweron,float U_poweron);

//从反馈的电源测量数组中得到当前电压值 单位V
float current_voltage_read(void);

//从反馈的电源测量数组中得到当前消耗的能量 单位wh
unsigned long current_work_read(void);

//从反馈的电源测量数组中得到当前的电流 单位A
float DC_current_read(void);

//从反馈的电源测量数组中得到当前的功率 单位w
float DC_power_read(void);
	
//电源模块开机初始化
void powerOn_init(void);

//电源模块参数读取
void powerSta_measure(void);
	
#endif
