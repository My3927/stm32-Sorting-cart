#include <timer.h>

void TIM1_Int_Init(u16 arr,u16 psc) //定时器3定时中断
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period = arr;     
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc; 
	TIM_TimeBaseInitStruct.TIM_ClockDivision =0; 
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);  
	
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_IRQn;   
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;   
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; //最高抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;   
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM1,ENABLE);	  
}


