

#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "communicate.h"
#include "mpu6050.h"
#include "control.h"
#include "debug.h"
#include "usart.h"
#include "dataflash.h"
#include "common.h"
#include "manage.h"

char StatusFlag=0;

/*
	�ϱ���̬����
*/
void ResponseIMU(void)
{
	char temp[32];	
	sprintf(temp, "V,%.0f,%.0f,%.0f\r\n", 
		g_fGravityAngle+180,
		g_fGyroAngleSpeed+180,
		g_fCarAngle+180);
	BluetoothOut(temp);
}

/*
	�ϱ�PIDֵ
*/
void ResponsePID(PIDType type)
{
	char temp[32];
	if(type==AnglePID)
		sprintf(temp, "AP,%.2f,%.2f,%.2f\r\n", 
		g_tCarAnglePID.P,
		g_tCarAnglePID.I,
		g_tCarAnglePID.D);
	else if(type == SpeedPID)
		sprintf(temp, "SP,%.2f,%.2f,%.2f\r\n", 
		g_tCarSpeedPID.P,
		g_tCarSpeedPID.I,
		g_tCarSpeedPID.D);
	BluetoothOut(temp);
}

/*
	�ϱ��汾��Ϣ
*/
void ResponseInfo(void)
{
	char temp[32];
	sprintf(temp, "I,%s,%s,%s\r\n", 
		FirmwareVer,
		EEPROMVer,
		MCUVer);
	BluetoothOut(temp);
}


/*
	�ϱ�״̬��Ϣ
*/
void ResponseStatus(void)
{
	char temp[32];
	sprintf(temp, "R,%d,%d\r\n", 
		g_BatVolt*10,
		g_RunTime);
	BluetoothOut(temp);
}

/*
	APP���ݽ�������
	Э��淶:֡ͷ+����+����+У��
	��:#17,CM,-27.314,-5.716*56;  ֡ͷ# ����17  ����CM,-27.314,-5.716 У��56������Ϊ�����ֽ���
*/
void Parse(char *dataInput)
{
	unsigned char len,check,ret;
	char *p = dataInput;
	float fTemp1, fTemp2;

#if COMMUNICATE_DEBUG_EN > 0
	if(Uart3Index){
		DebugOutBuff(Uart3Buffer, Uart3Index);
		DebugOutStr("\r\n");
	}
#endif
	while(((p-dataInput)<Uart3Index))
	{
		if(*p == '#')// 
		{
			len = (unsigned char)atof(strtok(p+1, ","));//length of data
			if((len>0)&&(len<100))//���ȴ���100�����ݺ���
			{
				if(len<10){
					check = (unsigned char)atof(strtok(p+len+4, ";"));
					ret = XOR_Check(p+3, len, check);
					p += 3;
				}
				else{
					check = (unsigned char)atof(strtok(p+len+5, ";"));
					ret = XOR_Check(p+4, len, check);
					p += 4;
				}
				if(ret){//  У����ȷ����ʼ��������
					if(g_CarRunningMode == CONTROL_MODE)
					{
						if(strstr(p , "CS")){Steer(0, 0);}//ֹͣ
						else if(strstr(p, "CJ")){// ҡ�˿���
							strtok(p, ",");
							fTemp1= atof(strtok(NULL,","));	// X������
							fTemp2 = atof(strtok(NULL,"*"));// Y������
							Steer(fTemp1 * 10, fTemp2 * 10);
						}
						else if(strstr(p, "CM")){//������Ӧ����
							strtok(p, ",");
							fTemp1 = atof(strtok(NULL, ",")); 		// Pitch
	      						fTemp2 = atof(strtok(NULL, "*")); 	// Roll
	      						Steer(fTemp2 / 5, fTemp1 / 2);
						}
					}
					
					if(strstr(p,"SNK")){// ����Ϊ��ͨң��ģʽ
						g_CarRunningMode = CONTROL_MODE;
					}
					else if(strstr(p,"SUF")){// ����Ϊ����������ģʽ
						g_CarRunningMode = ULTRA_FOLLOW_MODE;
					}
					else if(strstr(p,"SUA")){// ����Ϊ����������ģʽ
						g_CarRunningMode = ULTRA_AVOID_MODE;
						Steer(0, 5);
					}
					else if(strstr(p,"SIT")){// ����Ϊ����Ѱ��ģʽ
						g_CarRunningMode = INFRARED_TRACE_MODE;
					}
					
					else if(strstr(p,"SAP")){// ���ýǶȻ�PID
						strtok(p, ",");
						g_tCarAnglePID.P = atof(strtok(NULL,","));
						g_tCarAnglePID.I = atof(strtok(NULL,","));
						g_tCarAnglePID.D = atof(strtok(NULL,"*"));
						PIDWrite(AnglePID);
						ResponsePID(AnglePID);
					}
					else if(strstr(p,"SSP")){// �����ٶȻ�PID
						strtok(p, ",");
						g_tCarSpeedPID.P = atof(strtok(NULL,","));
						g_tCarSpeedPID.I = atof(strtok(NULL,","));
						g_tCarSpeedPID.D = atof(strtok(NULL,"*"));
						PIDWrite(SpeedPID);
						ResponsePID(SpeedPID);
					}
					else if(strstr(p, "GAP")){//��ѯ�ǶȻ�PID
						ResponsePID(AnglePID);
					}
					else if(strstr(p, "GSP")){//��ѯ�ٶȻ�PID
						ResponsePID(SpeedPID);
					}
					else if(strstr(p,"RA")){// �ǶȻ�pid�ָ�Ĭ��ֵ
						PIDReset(AnglePID);
						ResponsePID(AnglePID);
					}
					else if(strstr(p,"RV")){//�ٶȻ�pid�ָ�Ĭ��ֵ
						PIDReset(SpeedPID);
						ResponsePID(SpeedPID);
					}
					else if(strstr(p,"RB")){//��ѯ�汾������״̬�ϱ���Ϣ
						StatusFlag = 1;
						ResponseInfo();
					}
					else if(strstr(p,"RS")){//�ر�״̬�ϱ�
						StatusFlag = 0;
					}
				}
			}
			
		}
		p++;
	}
	Uart3Index=0;
	memset(Uart3Buffer, 0, Uart3BufferSize);
}



