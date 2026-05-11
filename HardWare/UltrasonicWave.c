#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Timer.h"
#include "PetAction.h"
#include "Variable.h"
//***超声避障***//
extern uint16_t T;  //记录距离
extern uint16_t Time;
int cf=0;//记录重复次数 解决bug用
void UltrasonicWave_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//开启GPIOA的时钟
	/*GPIO初始化*/ 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB12 TRIG
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);					//将PB13 ECHO

	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	
	
}

void UltrasonicWave_Start(void)
{
	//给触发信号
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	Delay_us(45);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	Timer_Init();  //计时 返回时间计算距离
}

uint16_t UltrasonicWave_Getvalue(void)
{
	UltrasonicWave_Start();
	Delay_ms(100);
	return ((Time * 0.0001) * 34000) / 2;
}

void Bizhang(void)
{
	T=UltrasonicWave_Getvalue();
	// Delay_ms(10);
	if(T<=10)
	{
		Action_Mode=5;
		cf++;
		
	}
}


