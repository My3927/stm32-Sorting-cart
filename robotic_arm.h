#ifndef _robotic_arm_H
#define _robotic_arm_H

#include <stm32f10x.h>
void Robot_arm_base_init(u16 arr,u16 psc);
void Robot_bigarm_init(u16 arr,u16 psc);
void Robot_smallarm_init(u16 arr,u16 psc);
void Robot_hand_init(u16 arr,u16 psc) ;
void Robot_Arm_Init(void);
void Gain_Unripe_Fruit(int big,int small);
void Gain_Ripe_Fruit(int big,int small);
#endif

