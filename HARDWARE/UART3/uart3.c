#include "sys.h"
#include "uart3.h"
#include "string.h"
#include "math.h"
#include "delay.h"

#if EN_USART3_RX   //���ʹ���˽���	

u16 USART3_RX_STA=0;       //����״̬���	

//u8 USART_TX_STA=0;  //����1���ͱ�־λ

float batteryRatio = 0.8823;//���ڷŵ�����������8.5AH���������������õ���7.5AH�������7.5/8.5��Ϊ����ϵ��

u8 txinitdata[8]={0x00,0x03,0x00,0x01,0x00,0x04,0x14,0x18}; //��ʼ����Դ����ģ��
u8 txrefreshdata[8]={0x01,0x03,0x00,0x48,0x00,0x05,0x05,0xDF}; //���¶�ȡ��Դģ������Ĳ���
u8 txcleardata[13]={0x01,0x10,0x00,0x0C,0x00,0x02,0x04,0x00,0x00,0x00,0x00,0xF3,0xFA}; //�����Դģ���ۼƵ��ܺ�
u8 rxrefreshdata[15];//���յ�Դ���µ�����
u8 copy_rxrefreshdata[15];//��Դ�������� ����

//��ز���
#define U_full  54.2//��λv
#define Q_full  7.5//��λAh
float Work_full = U_full*Q_full;//��س��������ܵ���

//�洢��Դ״̬��Ϣ����ѹ-����
u8 powerSta[2]={0,0};

//�洢���ͨ��˲���״̬����
struct PowerOnStaArgStruct powerOnStaArg;


//��ʼ��IO ����3 
//bound:������
void uart3_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10��GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PB10��PB11

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART3_RX	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}

void USART3_IRQHandler(void)                	//����3�жϷ������
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)   RESET��0
		//������յ�����
	{
		rxrefreshdata[USART3_RX_STA++] = USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
		if(USART3_RX_STA == 15){
			memcpy(&copy_rxrefreshdata,&rxrefreshdata[0],15);
			USART3_RX_STA = 0;
//			USART_TX_STA = 1;
		}
  } 
} 

void uart3_send_refreshMsgCmd()//��ȡ��Դģ������Ĳ������ڴ����ж��з��ظ��µ�����-��ʽ��16����
{
	for(char i=0;i<sizeof(txrefreshdata);i++)
	{
		USART_SendData(USART3, txrefreshdata[i]);         //�򴮿�3��������
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
}

void uart3_send_initPowerMsgCmd()//���ͳ�ʼ����Դģ��ָ��
{
	for(char i=0;i<sizeof(txinitdata);i++)
	{
		USART_SendData(USART3, txinitdata[i]);         //�򴮿�3��������
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
}

void uart3_send_ClearEnergyMsgCmd()//���������Դģ���ۼ�����ָ��
{
	for(char i=0;i<sizeof(txcleardata);i++)
	{
		USART_SendData(USART3, txcleardata[i]);         //�򴮿�3��������
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	}
}

float Q_sta_cal_powerOn(float v) //���뵱ǰ��ѹV�����ص�ǰ����Ah�����ڿ�����⵱ǰ�ĵ�ѹ��������������⣩
{
	float q;
	double voltage = v;
	//ͨ��ʵ�����ݺ�������ϣ����ɵ�ѹ-�����ֶκ���
	if((voltage<=52.82)&&(voltage>47.64))  //��ѹ��[47.64,52.82]
	{
		q = 9.179*sin(0.1*voltage+27.32);
	}
	else if((voltage<=47.64)&&(voltage>=39.08))  //��ѹ��[39.08,47.64]
	{
		q = 23.85*sin(0.03089*voltage+11.31)+0.8929*sin(0.8383*voltage-6.089);
	}
	else
	{
		q = 0;
	}
	return q;
}

//ͨ����ǰ�ĵ�ѹ�͵������㿪��ʱ����ܹ�
float W_powerOn_cal(float Q_poweron,float U_poweron)
{
	return Q_poweron*U_poweron*batteryRatio;
}

//�ӷ����ĵ�Դ���������еõ���ǰ��ѹֵ ��λV
float current_voltage_read()
{
	short u = (short)(copy_rxrefreshdata[3])<<8|copy_rxrefreshdata[4];
  float U = (float)u/100;
	return U;
}

//�ӷ����ĵ�Դ���������еõ���ǰ���ĵ����� ��λwh
unsigned long current_work_read()
{
	unsigned long e = (unsigned long)(copy_rxrefreshdata[9])<<24|(copy_rxrefreshdata[10]<<16)|(copy_rxrefreshdata[11]<<8)|copy_rxrefreshdata[12];
  unsigned long E = e/100;
	return E;
}

//�ӷ����ĵ�Դ���������еõ���ǰ�ĵ��� ��λA
float DC_current_read()
{
	 short i = (short)(rxrefreshdata[5])<<8|rxrefreshdata[6];
   float I = (float)i/1000;
	 return I;//��λA
}

//�ӷ����ĵ�Դ���������еõ���ǰ�Ĺ��� ��λw
float DC_power_read()
{
	 short p = (short)(copy_rxrefreshdata[7])<<8|copy_rxrefreshdata[8];
   float P = (float)p/10;
	 return P;//��λw
}


//��Դģ�鿪����ʼ��
void powerOn_init()
{
	uart3_send_initPowerMsgCmd();//����ģ���ʼ������
	delay_ms(100);
//  uart3_send_ClearEnergyMsgCmd();//���������Դģ���ۼ�����ָ��
//	delay_ms(1000);
//	uart3_send_initPowerMsgCmd();//����ģ���ʼ������
//	delay_ms(1000);
	uart3_send_refreshMsgCmd();//��ȡ��Դģ������Ĳ�������
	delay_ms(100);
	
	float voltageInitSta = current_voltage_read();//�ӷ����ĵ�Դ���������еõ���ǰ��ѹֵ ��λV
	float qValueInitSta = Q_sta_cal_powerOn(voltageInitSta);//���뵱ǰ��ѹV�����ص�ǰ����Ah
	float workValueInitSta = W_powerOn_cal(qValueInitSta,voltageInitSta);//ͨ����ǰ�ĵ�ѹ�͵������㿪��ʱ����ܹ�(���ܵ�������λwh��
	float workInitPercent = workValueInitSta*100/Work_full;//���㿪��״̬�µĵ�ص����ٷֱȣ��Ѿ�����100�����ֵ��0-100֮��
	
	//���õ��ĳ�ʼ���ݴ���������
	powerSta[0] = (unsigned char)voltageInitSta;
	powerSta[1] = (unsigned char)workInitPercent;
	
	printf("%d ",*powerSta);
	printf("%d ",*(powerSta+1));
	
	//���õ��ĳ�ʼ���ݴ���ṹ���У����øı�
	(&powerOnStaArg)->voltagePowerOnArg = voltageInitSta;
	(&powerOnStaArg)->qValuePowerOnArg = qValueInitSta;
	(&powerOnStaArg)->workValuePowerOnArg = workValueInitSta;
}

//��Դģ�������ȡ
void powerSta_measure()
{
	uart3_send_refreshMsgCmd();//��ȡ��Դģ������Ĳ�������
//	delay_ms(1000);
	
	float voltageCurrentSta = current_voltage_read();//�ӷ����ĵ�Դ���������еõ���ǰ��ѹֵ ��λV
	
	//�ӷ����ĵ�Դ���������еõ���ǰ���ĵ����� ��λwh
  unsigned long workedElectricalEnergy = current_work_read();
	if(workedElectricalEnergy>=340)
	workedElectricalEnergy = workedElectricalEnergy-340;
	
	//���ʣ�µĵ���
	unsigned long restofElectricalEnergy = (&powerOnStaArg)->workValuePowerOnArg - workedElectricalEnergy;
	//���ʣ������ٷֱ�
	float workCurrentPercent = restofElectricalEnergy*100/Work_full;
	
	//���õ��ĳ�ʼ���ݴ���������
	powerSta[0] = (unsigned char)voltageCurrentSta;
	powerSta[1] = (unsigned char)workCurrentPercent;
	
	printf("%d ",*powerSta);
	printf("%d ",*(powerSta+1));
	
	printf("0x%.2x  ",*powerSta);
	printf("0x%.2x ",*(powerSta+1));
}

#endif	

 



