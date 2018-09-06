abstract: read bateery parateters, inculding voltage, electric current, the rest of energy and power, form LT-211 module by stm32f407. 

hardware:stm32f407; lith-baterry; baterry measuerment module LT-211: https://item.taobao.com/item.htm?spm=a1z09.2.0.0.f02f2e8dyfBUSX&id=17807208004&_u=61sqvu1j8b97

brief theory: LT-211 support MODBUS communication protocol, and just send pre-defined cmd char array to LT-211 by stm32f407 usart(default: usart3), and read returned cmd char array by stm32 usart buffer, and then extract message from char array by bit movement operation. 

comment: I just use MODBUS communication protocol simplely, and did not use CRC check code。


实验器材:
	探索者STM32F4开发板
	
实验目的:
	学习串口的使用(接收与发送)
	
硬件资源:
	1,DS0(连接在PF9) 
	2,串口1(波特率:115200,PA9/PA10连接在板载USB转串口芯片CH340上面)
	
	
实验现象:
	本实验,STM32通过串口1和上位机对话，STM32在收到上位机发过来的字符串(以回车换
	行结束)后，原原本本的返回给上位机。下载后，DS0闪烁，提示程序在运行，同时每隔
	一定时间，通过串口1输出一段信息到电脑。 
	
注意事项:
	1,电脑端串口调试助手波特率必须是115200.
	2,请使用XCOM/SSCOM串口调试助手,其他串口助手可能控制DTR/RTS导致MCU复位/程序不运行
	3,串口输入字符串以回车换行结束.
	4,请用USB线连接在USB_232,找到USB转串口后测试本例程.
	5,P6的PA9/PA10必须通过跳线帽连接在RXD/TXD上.

					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
