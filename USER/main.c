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
	 TCRC5000_init();        //ѭ����ʼ��
	 Motor_Init();       //����ĳ�ʼ��
	 Robot_Arm_Init();      //��е�۳�ʼ��
   Turn_init();          //ת���ʼ��
	 UART3_Init();          //��openmvͨ��
	 UART4_Config();     //��������

	 OLED_Init();       //oled��ʼ��
   OLED_ColorTurn(0);//0������ʾ��1 ��ɫ��ʾ
   OLED_DisplayTurn(0);//0������ʾ 1 ��Ļ��ת��ʾ
	
   DHT11_Init();      //��ʪ��
   TIM5_Init();
	 HCSR04_Init();     //������
	
	 BH1750_Init();     //���մ�����
	 LED_Init();        //LED��
	 Openmv_init();      //����ͷ��ʼ��
	
	 TIM1_Int_Init(99,7199); 
	 TIM6_Int_Init(99,7199);
		
	 playsong(1);       //ϵͳ��ʼ�������
   delay_ms(30);
	
	while(1)
	{
		oled_show_temperture();
		
	}
}

