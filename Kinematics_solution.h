#ifndef _Kinematics_solution_H
#define _Kinematics_solution_H
#include <delay.h>
#include <stm32f10x.h>
#include <math.h>

#define  PI 3.1415926

float Radian_angle(float Radian);
double Sqrt(double n);
void Angle_solving(float x);
int big_arm_work(int angle);
int small_arm_work(int angle);




#endif

