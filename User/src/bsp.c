
#include "stdio.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "common.h"
#include "MPU6050.h"
#include "oled.h"
#include "bsp.h"
#include "ADC.h"
#include "usart.h"
#include "motor.h"
#include "I2C.h"
#include "ultrasonic.h"
#include "infrare.h"


/* 
	ʹ��SWD�� ʧ��JTAG
	PB3,PB4,PA15����ͨIOʹ�ã�����ʧ��JTAG�� 
*/
void SWDConfig(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
}



void LEDInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

  /*GPIOB Configuration*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // �����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void LEDToggle(void)
{
	static char flag=0;
	if(flag==0)
	{
		flag = 1;
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
	}
	else{
		flag = 0;
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	}
}
/*
	��ȡ��Ƭ��12�ֽ� ID
*/
#define UniqueID_Addr 0x1FFFF7E8U
void GetUniqueID(char* ID)
{
	char i;
	char *p = (char*)UniqueID_Addr; 
	for(i=0; i<12; i++){
		*ID++ = *p++;
	}
}


/*
	������������
*/
void SetBlueToothName(void)
{
	char temp[32];
	char check;
	GetUniqueID(temp);
	check = XOR_Get(temp, 12);
	sprintf(temp, "AT+NAMEMiaowLabs-%04X\r\n",(int)check);
	Uart3SendStr(temp);
	delay_ms(100);
}


/*
	bsp��ʼ��
*/
void BspInit(void)
{
	SWDConfig();
	
	ADCInit();				//ADC��ʼ��

	USART1Init();			//����1��ʼ��-�װ�Ԥ�����ؼ�������
	USART3Init(0);			//����3��ʼ��-��������

	TIM1_Cap_Init();		//TIM1��ʼ��-���ڳ��������湦��
	TIM3_PWM_Init(); 		//PWM��ʼ��
	TIM2_Encoder_Init();	//TIM2���������ʼ��-���ڲ���
	TIM4_Encoder_Init();	//TIM4���������ʼ��-���ڲ���
	
	i2cInit();	 			//I2C��ʼ��
	
	InfraredIOInit();		//����IO�ڳ�ʼ��

	OLED_Init();			//OLED��ʼ��
	delay_ms(20);
	MPU6050_Init();		    //MPU6050��ʼ��
	
	LEDInit();				//ָʾ�Ƴ�ʼ��
	
	UltraSelfCheck();							//	����ģ�鿪���Լ�
	InfrareSelfCheck();						//  ����ģ�鿪���Լ�
	
	delay_ms(500);			//��ʱ0.5s���ȴ�����ģ������
	Uart3SendStr("AT+BAUD8\r\n"); //�����������ڲ�����Ϊ115200 ( ԭ������9600 ) 
	delay_ms(20);
	Uart3SendStr("AT+RESET\r\n");  //������λ
	USART3Init(1);					//����UART3������Ϊ115200
	delay_ms(20);
	SetBlueToothName();		//��������ģ������
	Uart3SendStr("AT+RESET\r\n");  //������λ
}

