#include "sys.h"
#include "uart3.h"
#include "string.h"
#include "math.h"
#include "delay.h"

#if EN_USART3_RX   //如果使能了接收	

u16 USART3_RX_STA=0;       //接收状态标记	

//u8 USART_TX_STA=0;  //串口1发送标志位

float batteryRatio = 0.8823;//由于放电曲线数据是8.5AH，但是我们现在用的是7.5AH，因此用7.5/8.5作为比例系数

u8 txinitdata[8]={0x00,0x03,0x00,0x01,0x00,0x04,0x14,0x18}; //初始化电源测量模块
u8 txrefreshdata[8]={0x01,0x03,0x00,0x48,0x00,0x05,0x05,0xDF}; //更新读取电源模块测量的参数
u8 txcleardata[13]={0x01,0x10,0x00,0x0C,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0xF3,0xFA}; //清除电源模块累计的能耗
u8 rxrefreshdata[15];//接收电源更新的数据
u8 copy_rxrefreshdata[15];//电源测量数据 副本

//电池参数
#define U_full  54.2//单位v
#define Q_full  7.5//单位Ah
float Work_full = U_full*Q_full;//电池充满电后的总电量

//存储电源状态信息：电压-电量
u8 powerSta[2]={0,0};

//存储电池通电瞬间的状态参数
struct PowerOnStaArgStruct powerOnStaArg;


//初始化IO 串口3 
//bound:波特率
void uart3_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
  USART_Cmd(USART3, ENABLE);  //使能串口3
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART3_RX	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}

void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)   RESET是0
		//如果接收到数据
	{
		rxrefreshdata[USART3_RX_STA++] = USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		if(USART3_RX_STA == 15){
			memcpy(&copy_rxrefreshdata,&rxrefreshdata[0],15);
			USART3_RX_STA = 0;
//			USART_TX_STA = 1;
		}
  } 
} 

void uart3_send_refreshMsgCmd()//读取电源模块测量的参数，在串口中断中返回更新的数据-格式：16进制
{
	for(char i=0;i<sizeof(txrefreshdata);i++)
	{
		USART_SendData(USART3, txrefreshdata[i]);         //向串口3发送数据
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

void uart3_send_initPowerMsgCmd()//发送初始化电源模块指令
{
	for(char i=0;i<sizeof(txinitdata);i++)
	{
		USART_SendData(USART3, txinitdata[i]);         //向串口3发送数据
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

void uart3_send_ClearEnergyMsgCmd()//发送清除电源模块累计数据指令
{
	for(char i=0;i<sizeof(txcleardata);i++)
	{
		USART_SendData(USART3, txcleardata[i]);         //向串口3发送数据
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//等待发送结束
	}
}

float Q_sta_cal_powerOn(float v) //输入当前电压V，返回当前电量Ah（用于开机检测当前的电压，方法：开环检测）
{
	float q;
	double voltage = v;
	//通过实验数据和曲线拟合，生成电压-电量分段函数
	if((voltage<=52.82)&&(voltage>47.64))  //电压在[47.64,52.82]
	{
		q = 9.179*sin(0.1*voltage+27.32);
	}
	else if((voltage<=47.64)&&(voltage>=39.08))  //电压在[39.08,47.64]
	{
		q = 23.85*sin(0.03089*voltage+11.31)+0.8929*sin(0.8383*voltage-6.089);
	}
	else
	{
		q = 0;
	}
	return q;
}

//通过当前的电压和电量计算开机时候的总功
float W_powerOn_cal(float Q_poweron,float U_poweron)
{
	return Q_poweron*U_poweron*batteryRatio;
}

//从反馈的电源测量数组中得到当前电压值 单位V
float current_voltage_read()
{
	short u = (short)(copy_rxrefreshdata[3])<<8|copy_rxrefreshdata[4];
  float U = (float)u/100;
	return U;
}

//从反馈的电源测量数组中得到当前消耗的能量 单位wh
unsigned long current_work_read()
{
	unsigned long e = (unsigned long)(copy_rxrefreshdata[9])<<24|(copy_rxrefreshdata[10]<<16)|(copy_rxrefreshdata[11]<<8)|copy_rxrefreshdata[12];
  unsigned long E = e/100;
	return E;
}

//从反馈的电源测量数组中得到当前的电流 单位A
float DC_current_read()
{
	 short i = (short)(rxrefreshdata[5])<<8|rxrefreshdata[6];
   float I = (float)i/1000;
	 return I;//单位A
}

//从反馈的电源测量数组中得到当前的功率 单位w
float DC_power_read()
{
	 short p = (short)(copy_rxrefreshdata[7])<<8|copy_rxrefreshdata[8];
   float P = (float)p/10;
	 return P;//单位w
}


//电源模块开机初始化
void powerOn_init()
{
	uart3_send_initPowerMsgCmd();//发送模块初始化命令
	delay_ms(100);
//  uart3_send_ClearEnergyMsgCmd();//发送清除电源模块累计数据指令
//	delay_ms(1000);
//	uart3_send_initPowerMsgCmd();//发送模块初始化命令
//	delay_ms(1000);
	uart3_send_refreshMsgCmd();//读取电源模块测量的参数数组
	delay_ms(100);
	
	float voltageInitSta = current_voltage_read();//从反馈的电源测量数组中得到当前电压值 单位V
	float qValueInitSta = Q_sta_cal_powerOn(voltageInitSta);//输入当前电压V，返回当前电量Ah
	float workValueInitSta = W_powerOn_cal(qValueInitSta,voltageInitSta);//通过当前的电压和电量计算开机时候的总功(即总电量，单位wh）
	float workInitPercent = workValueInitSta*100/Work_full;//计算开机状态下的电池电量百分比，已经乘以100，因此值在0-100之间
	
	//将得到的初始数据存入数组中
	powerSta[0] = (unsigned char)voltageInitSta;
	powerSta[1] = (unsigned char)workInitPercent;
	
	printf("%d ",*powerSta);
	printf("%d ",*(powerSta+1));
	
	//将得到的初始数据存入结构体中，不得改变
	(&powerOnStaArg)->voltagePowerOnArg = voltageInitSta;
	(&powerOnStaArg)->qValuePowerOnArg = qValueInitSta;
	(&powerOnStaArg)->workValuePowerOnArg = workValueInitSta;
}

//电源模块参数读取
void powerSta_measure()
{
	uart3_send_refreshMsgCmd();//读取电源模块测量的参数数组
//	delay_ms(1000);
	
	float voltageCurrentSta = current_voltage_read();//从反馈的电源测量数组中得到当前电压值 单位V
	
	//从反馈的电源测量数组中得到当前消耗的能量 单位wh
  unsigned long workedElectricalEnergy = current_work_read();
	if(workedElectricalEnergy>=340)
	workedElectricalEnergy = workedElectricalEnergy-340;
	
	//求解剩下的电能
	unsigned long restofElectricalEnergy = (&powerOnStaArg)->workValuePowerOnArg - workedElectricalEnergy;
	//求解剩余电量百分比
	float workCurrentPercent = restofElectricalEnergy*100/Work_full;
	
	//将得到的初始数据存入数组中
	powerSta[0] = (unsigned char)voltageCurrentSta;
	powerSta[1] = (unsigned char)workCurrentPercent;
	
	printf("%d ",*powerSta);
	printf("%d ",*(powerSta+1));
	
	printf("0x%.2x  ",*powerSta);
	printf("0x%.2x ",*(powerSta+1));
}

#endif	

 



