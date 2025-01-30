#include <Kinematics_solution.h>

float Radian_angle(float Radian)  //弧度转角度
{
	float angle;
	angle = Radian * 180 / PI;
	return angle;
}

double Sqrt(double n)  //开平方公式
{
		double low, x2, mid, tmp;
		if (n > 1) 
	{
		low = 1;
		x2 = n;
  } 
	else {
    low = n;
		x2 = 1;
}
while (low <= x2) 
{
		mid = (low + x2) / 2.000;
		tmp = mid * mid;
		if (tmp - n <= 0.001 && tmp -n >= 0.001 * -1)
			{
		return mid;
		} 
		else if (tmp > n) 
			{
    x2 = mid;
			} 
			else 
	{
		low = mid;
   }
}
return -1.000;
}

float big_angle=0.0,small_angle=0.0;
void Angle_solving(float x)
{
	double y=7.0;
	double l=12.5,L2=((x*x)+(y*y));  // 第三边的平方;
	double theta_1,theta_2;
	double alpha,beta,gama;
	double data1,data2;

	data1=(((x*x)+(y*y))/(2*l*Sqrt(L2)))  , data2=(L2-2*l*l)/(-2*l*l);
	alpha=atan2(y,x);
	gama=acos(data1);   //余弦定理求gama
	
	theta_1=90-Radian_angle(gama)+Radian_angle(alpha);
	
	beta=acos(data2);
	
	theta_2=Radian_angle(beta)-theta_1-20.0;
	
  big_angle=theta_1;
	
	small_angle=theta_2;
	
	delay_ms(10);

}
int big_arm_work(int angle)
{
 int pwmval;
  pwmval = (int)(2000*(1-((0.5+angle/90.0)/20.0))) -5;  
 return pwmval;
}
int small_arm_work(int angle)
{
 int pwmval;
  pwmval = (int)(2000*(1-((0.5+angle/90.0)/20.0))) -5;  
 return pwmval;
}
	
