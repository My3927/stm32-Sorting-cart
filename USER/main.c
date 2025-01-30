#include <data.h>
#include <robotic_arm.h>
#include <oled.h>
#include <usart.h>
#include <timer.h>
#include <motor.h>
#include <encoder.h>
#include <control.h>
#include <beep.h>
#include <TCRC5000.h>
#include <relay.h>
#include <turn.h>
#include <GY_30.h>
#include <hcsr04.h>
#include "led.h"
#include "dht11.h"

int tmp;
int main(void)
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   uart_init(115200);
   delay_init();
	 TCRC5000_init();        //循迹初始化
	 Motor_Init();       //电机的初始化
	 Robot_Arm_Init();      //机械臂初始化
   Turn_init();          //转向初始化
	 UART3_Init();          //与openmv通信
	 UART4_Config();     //语音播报

	 OLED_Init();       //oled初始化
   OLED_ColorTurn(0);//0正常显示，1 反色显示
   OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
	
   DHT11_Init();      //温湿度
   TIM5_Init();
	 HCSR04_Init();     //超声波
	
	 BH1750_Init();     //光照传感器
	 LED_Init();        //LED灯
	 Openmv_init();      //摄像头初始化
	
	 TIM1_Int_Init(99,7199); 
	 TIM6_Int_Init(99,7199);
		
	 playsong(1);       //系统初始化完成了
   delay_ms(30);
	
	while(1)
	{
		oled_show_temperture();
		
	}
}

