
#include "stm32f10x.h"
#include "usart.h"
#include "SysTick.h"
#include "control.h"
#include "debug.h"
#include "communicate.h"
#include "dataflash.h"
#include "common.h"
#include "motor.h"
#include "display.h"
#include "bsp.h"
#include "ADC.h"
#include "ultrasonic.h"
#include "infrare.h"
#include "manage.h"


//�뼶����
void SecTask()
{
	if(SoftTimer[0])return;
	else{
		SoftTimer[0] = 1000;
	}
	g_RunTime++; 			// ��¼����ʱ��
	g_BatVolt = GetBatVoltage(); // ��ȡ��ص�ѹ
	
	if(StatusFlag)ResponseStatus();
	
	LEDToggle();
}


/*
	���������ţ����⣬���ƹ��ܺ�����stm32f10x_it.cִ���ļ��ĵδ�ʱ���жϷ�������ѭ��ִ�С�
*/
int main(void)
{	
	
	BspInit();				//��ʼ��BSP

	PIDInit(); 				//��ʼ��PID
	
	CarUpstandInit(); 	//��ʼ��ϵͳ����
	
	SysTick_Init();			//��ʼ����ʱ��	
	
	if(IsInfrareOK())
		g_iGravity_Offset = 5; //������⵽���Һ���ģ�飬�����ƫ��ֵ��
	
	ShowHomePageInit();
 
	while (1)
	{
		
		SecTask();			//�뼶����

		if(SoftTimer[1] == 0)//�����ϴ����ֻ�����λ��
		{// ÿ��20ms ִ��һ��
			SoftTimer[1] = 20;
			ResponseIMU();			
			DebugService();			
			Parse(Uart3Buffer);
			
		}			
  	
		if(SoftTimer[2] == 0)
		{
			SoftTimer[2] = 20;
			ShowHomePage();
	
			Read_Distane();

			if(g_CarRunningMode == ULTRA_FOLLOW_MODE){
				if(IsUltraOK())UltraControl(0);	//����������ģʽ
	 		}
			if(g_CarRunningMode == ULTRA_AVOID_MODE){
				if(IsUltraOK())UltraControl(1);	//����������ģʽ
	 		}
			else if(g_CarRunningMode == INFRARED_TRACE_MODE){
				TailingControl();
			}
		}			
	}
}


/******************* (C) COPYRIGHT 2016 MiaowLabs Team *****END OF FILE************/


