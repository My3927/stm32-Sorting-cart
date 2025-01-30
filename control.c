#include <control.h>
#include <usart.h>
#include <relay.h>
#include <JQ8400.h>
#include <oled.h>
#include <robotic_arm.h>
#include <Kinematics_solution.h>
#include <hcsr04.h>
#include <turn.h>
#include <TCRC5000.h>


static float Target_velocity=25.0; //给定目标值
float Encoder_R=0.0,Encoder_L=0.0; //给定目标值

int Moto_L,Moto_R;
u8 moto_time=0;


void TIM1_UP_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM1,TIM_FLAG_Update)==SET)	 
 {

	 moto_time=0;
	 Encoder_L=Read_Encoder(2);   
	 Encoder_R=Read_Encoder(3);
	 
	 Moto_L=Position_PID(Encoder_L,Target_velocity); 
	 Moto_R=Position_PID(Encoder_R,Target_velocity); 
	 
	 Xianfu_Pwm_L();
	 Xianfu_Pwm_R();
	 //对小车进行限幅，计算和pid控制

	 printf("%.2f,%.2f,%.2f,%d,%d \n",Encoder_L,Encoder_R,Target_velocity,Moto_R,Moto_L);
	 TIM_ClearITPendingBit(TIM1,TIM_IT_Update);  
	 


 }
}


//  float Position_KP=1.8,Position_KI=0.0013,Position_KD=0.5;
int Position_PID (float Encoder,float Target) //66mm的轮 速度约为0.5m每秒
{
    float Position_KP=1.3,Position_KI=0.0,Position_KD=0.4;
	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	
	 Bias=Encoder-Target;         
	
	 Integral_bias+=Bias;	      
	 
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);   
	 
	 Last_Bias=Bias;                
	 
	 return Pwm;   
}





extern int turn_sign;
extern int line_sign;
//static u8 sign=0;


extern float big_angle,small_angle;
float distance=0;
int fruit_count=0,count=0,unfruit_count=0;

void USART3_IRQHandler(void) //接收到数据开启中断
{
	static u8 k=0;
  static u8 RxBuffer1[4]={0};
  static u8 RxState = 0;
	static u8 Not_detected,i;
	static u8 Cx=0,Cy=0,Cw=0,Ch=0;
	u8 com_data=0;
  int big,small,big_rally_angle,small_rally_angle,a;
  float dis,sum;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//判断是否接收到数据
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	 
		com_data=USART_ReceiveData(USART3); //接收到的数据传到dat
		
		if(RxState==0&&com_data==0x2c)  //0x2c??
				{
					RxState=1;
					RxBuffer1[k++]=com_data;

				//	printf(" 第一个数据com_data=%x \n",com_data);
				}
		
		else if(RxState==1&&com_data==0x12)  //0x12??
		    {
					RxState=2;
					RxBuffer1[k++]=com_data;	
					
				//	printf("  第二个数据com_data=%x  \n",com_data);
				}
				
		else if(RxState==2)
				{
					RxBuffer1[k++]=com_data;

				//	printf("  com_dat=%x\n  ",com_data);
					
					if( k>=4 || com_data==0x5b)
					{
					RxState=3;					
					Cx=RxBuffer1[0],
		      Cy=RxBuffer1[1],
			    Cw=RxBuffer1[2],
			    Ch=RxBuffer1[3];
						
				//  sign=RxBuffer1[k-2];
				 // printf(" 结果= %d  \n  ",sign);
	
					}
				}		
		else if(RxState==3)
		{
			if (Ch==0x5b)
			{
			USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
				
			k = 0;
			RxState = 0;
			
		printf("cx= %x  cy= %x  cw= %x  ch= %x ",Cx,Cy,Cw,Ch);

			for(a=0;a<5;a++)
			{
			dis=GetDistance();
      sum+=dis;			
			}
			
			distance=sum/5+1.0;
			sum=0;                      //超声波测量横向距离
			
			Angle_solving(distance);   //抓取算法计算大臂小臂抓取角度 
			
			big_rally_angle=180-(int)big_angle;
			big=big_arm_work(big_rally_angle);      //计算大臂转动所需占空比
			
			small_rally_angle=180-(int)small_angle;
			small=small_arm_work(small_rally_angle); //计算小臂转动所需占空
	//		printf("dis= %.2f   big=%d  small=%d ",distance,big_rally_angle,small_rally_angle);
			
			delay_ms(10);
			                //机械臂运动学逆解

			OLED_Clear();
			oled_show_furit_count();
				if(Cw==0xa2)
				{
				  playsong(2);       
          delay_ms(10);
					
		      Gain_Ripe_Fruit(big,small);      //机械臂抓取
					delay_ms(100);
					
					fruit_count++;

					printf("检测到成熟果实");
					
				
					delay_ms(10);
					Not_detected=0;  
          count=0;          
					delay_ms(10);
					USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
				}			
				
				else if(Cw==0xa1)
				{
					playsong(3);
					delay_ms(10);
							
          Gain_Unripe_Fruit(big,small);
					delay_ms(100);
					
					unfruit_count++;
		
					printf("未成熟");

					delay_ms(10);
					Not_detected=0; 
					count=0;
					delay_ms(10);
					USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
				}
				else if(Cw==0xa3)
				{
					Not_detected=1;
					if(Not_detected==1)
					{
						count++;
   //         printf("没有检测到");
						
						if(count>=30)
                {
									turn_sign=1;
									line_sign=0;
									delay_ms(10);
									TIM_Cmd(TIM6,ENABLE);	    //开启循迹中断
								  count=0;
									Openmv_off();
									OLED_Clear();
									for(i=0;i<=4;i++){RxBuffer1[i]=0;}
			            k = 0;
			            RxState = 0;
			           // sign=0;
								}
						else {USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);}
					}	
					  //Not_detected变量用于判断是否检测到了果实，count变量用于累计没有检测到果实的数目

				}	
			 else 
       { 
        for(i=0;i<=4;i++){RxBuffer1[i]=0;}
			  k = 0;
			  RxState = 0;
			//  sign=0;		 
			  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
			 }	          //接收有异常
			}
		}
			 }	  
}


int turn_sign=0;
extern int line_sign;
int i;
void TIM6_IRQHandler(void) 
{

	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
{
   TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	 Follow_the_trail();
	
	 		if(line_sign==1) //检测到了停止线
		{
			delay_ms(2);
			if(line_sign==1)
			{
			TIM_SetCompare2(TIM4,1845);
			Car_stop();
			Openmv_on(); 
		  }
			
	 }
		if(turn_sign==1)
		{

			    TIM_SetCompare2(TIM4,1845);
			  	Car_Star();
			    delay_ms(80);
			
		    	Go_back();
			    delay_ms(1350);


				  Car_stop();
				  delay_ms(200);
				 
		  	  TIM_SetCompare2(TIM4,1920);
				 
				  delay_ms(500);
				 
				 	line_sign=0;
			    turn_sign=0;
			
				  Car_Star();
			  	delay_ms(80);
				
				  Go_ahead();
				  delay_ms(800);
					TIM_SetCompare2(TIM4,1845);
					Go_ahead();   
					delay_ms(800);
		}
}
}

void Motor_Init(void)
{
	 MOTO_Init();   // 电机初始化
	 PWM_Init_TIM4_CH3(0,7199);
	 PWM_Init_TIM4_CH4(0,7199);
	 Encoder_TIM2_Init(); //编码器1初始化
	 Encoder_TIM3_Init();//编码器2初始化
	 delay_ms(50);
	 
}

void Car_stop(void)
{
	AIN1=1,   AIN2=0;
  BIN1=1,   BIN2=0;
	delay_ms(110);
	AIN1=0,   AIN2=0;
  BIN1=0,   BIN2=0;
	TIM_Cmd(TIM1,DISABLE);

}
void Car_Star(void)
{
  TIM_Cmd(TIM1,ENABLE);
}
void Go_ahead(void)
{
	 Set_Pwm_Left(Moto_L);
	 Set_Pwm_Right(Moto_R);
}

void Go_back(void)
{
		TIM_SetCompare3(TIM4,Moto_L);
    AIN1=1,   AIN2=0;
		TIM_SetCompare4(TIM4,Moto_R);
    BIN1=1,   BIN2=0;

}
void Set_Pwm_Left(int moto) //驱动函数  前进
{
	TIM_SetCompare3(TIM4,moto);
  AIN1=0,   AIN2=1;
}


void Set_Pwm_Right(int moto) //驱动函数  //前进
{
	TIM_SetCompare4(TIM4,moto);
  BIN1=0,   BIN2=1;
}
void Set_Pwm_Left_turn(int moto) //驱动函数  前进
{
	TIM_SetCompare3(TIM4,moto);
  AIN1=1,   AIN2=0;
}

void Set_Pwm_Right_turn(int moto) //驱动函数  //前进
{
	TIM_SetCompare4(TIM4,moto);
  BIN1=1,   BIN2=0;
}

void Turn_Left(void)
{
	  Set_Pwm_Right(Moto_R);
		TIM_SetCompare3(TIM4,Moto_L);
	  AIN1=1,   AIN2=0;
    AIN1=0,   AIN2=0;

}

void Turn_Right(void)
{
	 Set_Pwm_Left(Moto_L);
	 TIM_SetCompare4(TIM4,Moto_R);
	 BIN1=1,   BIN2=0;
   BIN1=0,   BIN2=0;

}

 
int myabs(int a)  //取绝对值函数
{ 		   
	 int temp;
	 if(a<0)  temp=-a;  
	 else temp=a;
	 return temp;
}

void Set_Pwm_Right_Back(int moto) //驱动函数 
{
	TIM_SetCompare4(TIM4,moto);
  BIN1=1,   BIN2=0;
}

void Xianfu_Pwm_L(void)  //pwm限幅 因为给定pwm最大值为7199
 {
   int Amplitude=7100;  

	 if(Moto_L<-Amplitude)  Moto_L = -Amplitude;
	 if(Moto_L>Amplitude)   Moto_L =  Amplitude;
 }
void Xianfu_Pwm_R(void)  //pwm限幅 因为给定pwm最大值为7199
 {
   int Amplitude=7100;  
	 if(Moto_R<-Amplitude)  Moto_R = -Amplitude;
	 if(Moto_R>Amplitude)   Moto_R =  Amplitude;

 }
