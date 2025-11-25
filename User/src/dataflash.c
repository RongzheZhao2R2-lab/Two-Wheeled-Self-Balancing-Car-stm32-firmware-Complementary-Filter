

#include "stm32f10x_flash.h"
#include "control.h"
#include "dataflash.h"

static char FlashWBuffer[128]; 

#define FLASH_START_ADDR	(0x08000000+1024*60)// 

#define original_PID_Addr	0x0
#define Angle_PID_Addr	0x32
#define Speed_PID_Addr	0x48


/*
	����˳��:
	- ÿ��д����֮ǰ����Ҷ���ݶ�ȡ��������
	- ��Ҫ�洢�����ݰ�ָ����ַд��������
	- Ҷ����
	- ����������������д��flash

	��������СΪ128�ֽڣ����ʵ����Ч�洢��ַ��Χ0x00-0x7F
	���Ҫ���Ӵ洢��������ͨ�����󻺳���ʵ��
*/
void ProgramFlash(uint32_t addr, char* data, uint16_t len)
{
	uint8_t i=0;
	
	FLASH_Unlock();  //
     	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//

	for(i=0; i<128; i ++){
		FlashWBuffer[i] = *((char*)(FLASH_START_ADDR+i));
	}

	for(i=0; i< len; i++){
		FlashWBuffer[addr+i] = *data++;
	}

	FLASH_ErasePage(FLASH_START_ADDR); 	//
	for(i=0; i<128; i+=4)
		FLASH_ProgramWord(FLASH_START_ADDR+i, *((uint32_t*)&FlashWBuffer[i])); //

	FLASH_ClearFlag(FLASH_FLAG_BSY|FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);//
	FLASH_Lock();    //
}

/*
	flash����������ַ��Χ0x00--0x7F
*/
void ReadFlash(uint32_t addr, char* data, uint16_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)
	 	*data++ = *((char*)(FLASH_START_ADDR+addr+i));
}

/*
	PID ������ʼ��
	������Щ��Ĭ��ֵд��flash
	�ǳ����������flash�ж�ȡpid����
*/
void PIDInit()
{
	char flag[2];
	
	ReadFlash(original_PID_Addr, flag, 2);
	if((flag[0] == 0xa5)&&(flag[1] == 0x5a))
	{// �ǳ�������
		PIDRead();
	}
	else{// ��������
		flag[0] = 0xa5;
		flag[1] = 0x5a;
		ProgramFlash(original_PID_Addr, flag, 2);
		ProgramFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));// ����ǶȻ�Ĭ��pid����
		ProgramFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));// �����ٶȻ�Ĭ��PID����
	}
}

/*
	�����޸ĵ�PID����
	flag��0--���ֽǶȻ�pid��1--�����ٶȻ�pid
*/
void PIDWrite(char flag)
{
	char i=1;
	if(flag==0){
		ProgramFlash(Angle_PID_Addr, &i, 1);	
		ProgramFlash(Angle_PID_Addr+1,(char*)&g_tCarAnglePID, sizeof(PID_t));
	}
	else{
		ProgramFlash(Speed_PID_Addr, &i ,1);	
		ProgramFlash(Speed_PID_Addr+1,(char*)&g_tCarSpeedPID, sizeof(PID_t));
	}
		
}

/*
	��ȡPID����������û�и������ȡĬ��ֵ
*/
void PIDRead()
{
	char i;
	
	ReadFlash(Angle_PID_Addr, &i, 1);
	if(i==1)      
		ReadFlash(Angle_PID_Addr+1, (char*)&g_tCarAnglePID, sizeof(PID_t));
	else
		ReadFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));
	
	ReadFlash(Speed_PID_Addr, &i, 1);
	if(i==1)      
		ReadFlash(Speed_PID_Addr+1, (char*)&g_tCarSpeedPID, sizeof(PID_t));
	else
		ReadFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));
}
/*
	��PID�����ָ���Ĭ��ֵ
*/
void PIDReset(char flag)
{
	char i=0;
	if(flag==0){
		ProgramFlash(Angle_PID_Addr,  &i, 1);
		ReadFlash(original_PID_Addr+2, (char*)&g_tCarAnglePID, sizeof(PID_t));
	}
	else{
		ProgramFlash(Speed_PID_Addr, &i, 1);
		ReadFlash(original_PID_Addr+2+16, (char*)&g_tCarSpeedPID, sizeof(PID_t));
	}
}


