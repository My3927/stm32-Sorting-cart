#ifndef _control_H
#define _control_H

#include <stm32f10x.h>
#include <delay.h>
#include <sys.h>
#include <motor.h>
#include <encoder.h>
#include <timer.h>
#define AIN1   PEout(10)
#define AIN2   PEout(11)  //左轮

#define BIN1   PEout(12)
#define BIN2   PEout(13)  //右轮

void Motor_Init(void);
void Xianfu_Pwm_R(void); //限制PWM幅值
void Xianfu_Pwm_L(void);
int myabs(int a);   //取绝对值
void Set_Pwm_Right(int moto);
void Set_Pwm_Left(int moto); 
int Position_PID (float Encoder,float Target);
float Read_EncoderA(void);
void Turn_Left(void);
void Turn_Right(void);
void Go_back(void);
void Go_ahead(void);
void Car_Stop(void);
void Set_Pwm_Right_Back(int moto);
void Set_Pwm_Left_Back(int moto);
void Car_stop(void);
void Car_Star(void);
void Set_Pwm_Right_turn(int moto);
void Set_Pwm_Left_turn(int moto);
#endif

