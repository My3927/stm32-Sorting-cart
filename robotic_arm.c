#include <robotic_arm.h>
#include <delay.h>
#include <turn.h>
void Robot_arm_base_init(u16 arr,u16 psc)   //µ××ù
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO ,ENABLE);//ÖØÓ³ÉäÁË
	
	GPIO_PinRemapConfig(GPIO_Remap_TIM4 , ENABLE);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD,&GPIO_InitStruct);
	

	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM4, ENABLE);    
}

void Robot_bigarm_init(u16 arr,u16 psc)  //´ó±Û
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC ,ENABLE);//ÖØÓ³ÉäÁË
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  

	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM8, ENABLE);          

  TIM_CtrlPWMOutputs(TIM8,ENABLE);
}


void Robot_smallarm_init(u16 arr,u16 psc)  //Ð¡±Û
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC ,ENABLE);//ÖØÓ³ÉäÁË
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  

	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM8, ENABLE);          

  TIM_CtrlPWMOutputs(TIM8,ENABLE);
}


void Robot_hand_init(u16 arr,u16 psc)  //¼Ð×¦
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC ,ENABLE);//ÖØÓ³ÉäÁË
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;        
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; 
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  

	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); 

	TIM_Cmd(TIM8, ENABLE);          

  TIM_CtrlPWMOutputs(TIM8,ENABLE);
}

void Robot_Arm_Init(void)
{

	 Robot_arm_base_init(1999,719);
   Robot_bigarm_init(1999,719);
   Robot_smallarm_init(1999,719);
   Robot_hand_init(1999,719) ;
	 
	
	delay_ms(20);
	TIM_SetCompare1(TIM4,1850);  //µ××ù
	delay_ms(100);
	TIM_SetCompare2(TIM8,1745);  //´ó±Û
	delay_ms(100);
	TIM_SetCompare3(TIM8,1745);  //Ð¡±Û
	delay_ms(100);
	TIM_SetCompare4(TIM8,1900);  //¼Ð×¦
	delay_ms(100);

}


void Gain_Ripe_Fruit(int big,int small)
{

 int i,j,k,m;

 
 for(k=1745;k<=small;k++)
 {
   TIM_SetCompare3(TIM8,k);//Ð¡±Û
   delay_ms(10);
 }
 delay_ms(500);

 
 for(j=1745;j<=big;j++)
 {
  TIM_SetCompare2(TIM8,j);//´ó±Û
  delay_ms(10);
 }
 delay_ms(1000);
 
 for(m=1900;m>=1865;m--)
 {
  TIM_SetCompare4(TIM8,m);//¼Ð×¦
  delay_ms(10);
 }
 delay_ms(1000);
 

 for(j=big;j>=1753;j--)
 {
  TIM_SetCompare2(TIM8,j);//´ó±Û
  delay_ms(10);
 }
 delay_ms(500);
 
 for(k=small;k>=1745;k--)
 {
  TIM_SetCompare3(TIM8,1745);//Ð¡±Û
  delay_ms(20);
 }
 delay_ms(500);
 
 
 for(i=1850;i<=1880;i++)
 {
  TIM_SetCompare1(TIM4,i);//µ××ù
  delay_ms(10);
 }
 
 delay_ms(500);
 
 for(m=1865;m<=1900;m++)
 {
  TIM_SetCompare4(TIM8,m);//¼Ð×¦
  delay_ms(10);
 }
 delay_ms(500);
 

 for(i=1880;i>=1850;i--)
 {
  TIM_SetCompare1(TIM4,i);//µ××ù
  delay_ms(10);
 }
 delay_ms(500);
 
 TIM_SetCompare2(TIM8,1745);//´ó±Û
}


void Gain_Unripe_Fruit(int big,int small)
{

 int i,j,k,m;
 

 for(k=1745;k<=small;k++)
 {
   TIM_SetCompare3(TIM8,k);//Ð¡±Û
   delay_ms(10);
 }
 delay_ms(500);
 

 
 for(j=1745;j<=big;j++)
 {
  TIM_SetCompare2(TIM8,j);//´ó±Û
  delay_ms(10);
 }
 delay_ms(1000);
 
 for(m=1900;m>=1865;m--)
 {
  TIM_SetCompare4(TIM8,m);//¼Ð×¦
  delay_ms(10);
 }
 delay_ms(1000);
 

 for(j=big;j>=1753;j--)
 {
  TIM_SetCompare2(TIM8,j);//´ó±Û
  delay_ms(10);
 }
 delay_ms(500);
 
 for(k=small;k>=1745;k--)
 {
  TIM_SetCompare3(TIM8,1745);//Ð¡±Û
  delay_ms(20);
 }
 delay_ms(500);
 
 
 for(i=1850;i>=1820;i--)
 {
  TIM_SetCompare1(TIM4,i);//µ××ù
  delay_ms(10);
 }
 
 delay_ms(500);
 
 for(m=1865;m<=1900;m++)
 {
  TIM_SetCompare4(TIM8,m);//¼Ð×¦
  delay_ms(10);
 }
 delay_ms(1000);
 

 for(i=1820;i<=1850;i++)
 {
  TIM_SetCompare1(TIM4,i);//µ××ù
  delay_ms(10);
 }
 delay_ms(500);
 
 TIM_SetCompare2(TIM8,1745);//´ó±Û
 
}

