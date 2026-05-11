#include "stm32f10x.h"

extern uint16_t Time;
extern uint8_t pb12_state;
extern uint8_t pb13_state;
extern uint16_t T;
extern float ttt;
void Timer_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 1. 使能GPIOB时钟（用于PB13-ECHO）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 2. 配置PB13为输入模式（ECHO引脚）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 3. 初始化TIM4定时器
    Time = 0;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    TIM_InternalClockConfig(TIM4);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 7200-1;        // ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;          // PSC
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        ttt=T;
		pb12_state = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
		pb13_state = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
        
        // 读取PB13
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 1)
        {
            Time++;
        }
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
