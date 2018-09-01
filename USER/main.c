#include "sys.h"
#include "delay.h"
#include "uart3.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"


int main(void)
{  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		//��ʱ��ʼ�� 
	uart_init(4800);   
	uart3_init(4800);	//���ڳ�ʼ��������Ĭ��Ϊ4800

	powerOn_init();//��Դģ�鿪����ʼ��
	delay_ms(1500);
	while(1)
	{
		powerSta_measure();//��ȡ��Դģ��״̬��������ǰ��ѹ-��ǰ������
		delay_ms(1000);//��Դģ��״̬��ȡ���ڴ���1s
	}
}