#include "sys.h"
#include "delay.h"
#include "uart3.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"


int main(void)
{  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		//延时初始化 
	uart_init(4800);   
	uart3_init(4800);	//串口初始化波特率默认为4800

	powerOn_init();//电源模块开机初始化
	delay_ms(1500);
	while(1)
	{
		powerSta_measure();//读取电源模块状态参数【当前电压-当前电量】
		delay_ms(1000);//电源模块状态读取周期大于1s
	}
}