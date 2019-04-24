#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "hc05.h"
#include "usart3.h"
#include "hc05.h"
#include "timer.h"
#define STD (T / 2)

#define STEP 3
#define T 30
void MPU6050_Report(short pitch, short roll, short yaw);
//PA5 SCL PA7 SDA
//PA3 RX2 PA2 TX2
//PA6 PWM1 PA8 PWM2 PA11 PWM3 PB1 PWM4

float pitch = 0, roll = 0, yaw = 0; //欧拉角
//
volatile u16 level1 = T / 2, level2 = T / 2, level3 = T / 2, level4 = T / 2;

int main(void)
{

	// short aacx=0,aacy=0,aacz=0;		//加速度传感器原始数据
	// short gyrox=0,gyroy=0,gyroz=0;	//陀螺仪原始数据
	u8 feedback_flag =0;
	u8 modify_flag =0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);								//串口初始化为500000
	delay_init();									//延时初始化
	delay_ms(1000);
	while (MPU_Init())
		; //初始化MPU6050
	USART3_RX_STA = 0;
	usart2_init(9600);
	u2_printf("mpu6050 initializing...\n");
	delay_ms(1000);
	while (mpu_dmp_init())
		;

	// HC05_Set_Cmd("AT+ROLE=0");
	// HC05_Set_Cmd("AT+RESET");	//复位ATK-HC05模块
	// delay_ms(200);
	u2_printf("pwm initializing...\n");

	PWMIO_Init();
	TIM3_Int_Init(20, 7200);
	// TIM3_PWM_Init(T,7200);
	// TIM1_PWM_Init(T,7200);
	// TIM_SetCompare1(TIM3,level1);
	// TIM_SetCompare4(TIM1,level2);
	// TIM_SetCompare1(TIM1,level3);
	// TIM_SetCompare4(TIM3,level4);

	while (1)
	{
		mpu_dmp_get_data(&pitch, &roll, &yaw);
		//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		//MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		//IMUupdate(gyrox,gyroy,gyroz,aacx,aacy,aacz);
		if(feedback_flag) u2_printf("yaw:%.2f  pitch:%.2f  roll:%.2f\n", yaw, pitch, roll);
		delay_ms(50);
		//if(modify_flag)
		{
			volatile short M1=STD,M2=STD,M3=STD,M4=STD;
			volatile short yaw_modify=0,pitch_modify=0,roll_modify=0;
			pitch_modify=pitch/10;

			if(roll>0)
				roll_modify=10-(roll-90)/10;
			else
				roll_modify=-10-(roll+90)/10;

			if(yaw>0)
				yaw_modify=10-yaw/20;
			else
				yaw_modify=-10-yaw/20;
			
			M1=STD+pitch_modify-roll_modify-yaw_modify;
			M2=STD+pitch_modify+roll_modify+yaw_modify;
			M3=STD-pitch_modify+roll_modify-yaw_modify;
			M4=STD-pitch_modify-roll_modify+yaw_modify;
			u2_printf("M1:%d M2:%d M3:%d M4:%d\n",M1,M2,M3,M4);
			if(M2>T-STEP)	M2=T-STEP;
			if(M1>T-STEP)	M1=T-STEP;
			if(M4>T-STEP)	M4=T-STEP;
			if(M3>T-STEP)	M3=T-STEP;
				level3=M2;
				level4=M4;
				level1=M1;
				level2=M3;
				
			
			
		}
		// u2_printf("ax:%.3f ay:%.3f az:%.3f",aacx,aacy,aacz);
		// delay_ms(100);
		// u2_printf("gx:%.3f gy:%.3f gz:%.3f",gyrox,gyroy,gyroz);

		if (USART3_RX_STA & 0X8000) //接收到一次数据了
		{

			u8 flag = 0;
			u2_printf("your keyval is '%c' !!!\n", USART3_RX_BUF[0]);
			switch (USART3_RX_BUF[0])
			{

			case '1':
				if (level1 < T - STEP)
					level1 = level1 + STEP;
				else
					flag = 1;
				break;
			case '2':
				if (level1 > STEP)
					level1 = level1 - STEP;
				else
					flag = 1;
				break;
			case '3':
				if (level2 < T - STEP)
					level2 = level2 + STEP;
				else
					flag = 1;
				break;
			case '4':
				if (level2 > STEP)
					level2 = level2 - STEP;
				else
					flag = 1;
				break;
			case '5':
				if (level3 < T - STEP)
					level3 = level3 + STEP;
				else
					flag = 1;
				break;
			case '6':
				if (level3 > STEP)
					level3 = level3 - STEP;
				else
					flag = 1;
				break;
			case 'A':
				if (level4 < T - STEP)
					level4 = level4 + STEP;
				else
					flag = 1;
				break;
			case 'B':
				if (level4 > STEP)
					level4 = level4 - STEP;
				else
					flag = 1;
				break;
			case 'C':
				if (level4 < T - STEP && level1 < T - STEP && level2 < T - STEP && level3 < T - STEP)
				{
					level1 = level1 + STEP;
					level2 = level2 + STEP;
					level3 = level3 + STEP;
					level4 = level4 + STEP;
				}
				else
					flag = 1;
				break;
			case 'D':
				if (level1 > STEP && level2 > STEP && level3 > STEP && level4 > STEP)
				{
					level1 = level1 - STEP;
					level2 = level2 - STEP;
					level3 = level3 - STEP;
					level4 = level4 - STEP;
				}
				else
					flag = 1;
				break;
			case 'F':
				feedback_flag =!feedback_flag;
			case 'M':
				modify_flag =!modify_flag;
			default:
				u2_printf("error !!! invalid keyval !!!\n");
				break;
			}

			// TIM_SetCompare1(TIM3,level1);
			// TIM_SetCompare4(TIM1,level2);
			// TIM_SetCompare1(TIM1,level3);
			// TIM_SetCompare4(TIM3,level4);
			USART3_RX_STA = 0;
			if (flag)
				u2_printf("pwm adjustment is out of range !!!\n");
			else
			{
				u2_printf("pwm adjustment is made !!!\n");
				u2_printf("current pwm 1 %d0%%\n", level1);
				u2_printf("current pwm 2 %d0%%\n", level2);
				u2_printf("current pwm 3 %d0%%\n", level3);
				u2_printf("current pwm 4 %d0%%\n", level4);
			}
		}
		// if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		// {

		// 	//

		// 	// if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
		// 	// if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));

		// }
	}
}

// void MPU6050_Report(short pitch,short roll,short yaw)
// {
// 	u8 sendbuf[15]={0};
// 	char sign='+';
// 	if (pitch<0) sign='-';
// 	u2_printf("pitch:%c%d",sign,pitch);
// 	delay_ms(50);

// 	if (roll<0) sign='-';
// 	else  sign='+';
// 	u2_printf("roll:%c%d",sign,roll);
// 	delay_ms(50);

// 	if (yaw<0) sign='-';
// 	else  sign='+';
// 	u2_printf("yaw:%c%d",sign,yaw);
// 	delay_ms(50);
// }

//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
// void usart1_niming_report(u8 fun,u8*data,u8 len)
// {
// 	u8 send_buf[32];
// 	u8 i;
// 	if(len>28)return;	//最多28字节数据
// 	send_buf[len+3]=0;	//校验数置零
// 	send_buf[0]=0X88;	//帧头
// 	send_buf[1]=fun;	//功能字
// 	send_buf[2]=len;	//数据长度
// 	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
// 	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和
// 	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1
// }
// //发送加速度传感器数据和陀螺仪数据
// //aacx,aacy,aacz:x,y,z三个方向上面的加速度值
// //gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
// void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
// {
// 	u8 tbuf[12];
// 	tbuf[0]=(aacx>>8)&0XFF;
// 	tbuf[1]=aacx&0XFF;
// 	tbuf[2]=(aacy>>8)&0XFF;
// 	tbuf[3]=aacy&0XFF;
// 	tbuf[4]=(aacz>>8)&0XFF;
// 	tbuf[5]=aacz&0XFF;
// 	tbuf[6]=(gyrox>>8)&0XFF;
// 	tbuf[7]=gyrox&0XFF;
// 	tbuf[8]=(gyroy>>8)&0XFF;
// 	tbuf[9]=gyroy&0XFF;
// 	tbuf[10]=(gyroz>>8)&0XFF;
// 	tbuf[11]=gyroz&0XFF;
// 	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
// }
// //通过串口1上报结算后的姿态数据给电脑
// //aacx,aacy,aacz:x,y,z三个方向上面的加速度值
// //gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
// //roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
// //pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
// //yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
// void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
// {
// 	u8 tbuf[28];
// 	u8 i;
// 	for(i=0;i<28;i++)tbuf[i]=0;//清0
// 	tbuf[0]=(aacx>>8)&0XFF;
// 	tbuf[1]=aacx&0XFF;
// 	tbuf[2]=(aacy>>8)&0XFF;
// 	tbuf[3]=aacy&0XFF;
// 	tbuf[4]=(aacz>>8)&0XFF;
// 	tbuf[5]=aacz&0XFF;
// 	tbuf[6]=(gyrox>>8)&0XFF;
// 	tbuf[7]=gyrox&0XFF;
// 	tbuf[8]=(gyroy>>8)&0XFF;
// 	tbuf[9]=gyroy&0XFF;
// 	tbuf[10]=(gyroz>>8)&0XFF;
// 	tbuf[11]=gyroz&0XFF;
// 	tbuf[18]=(roll>>8)&0XFF;
// 	tbuf[19]=roll&0XFF;
// 	tbuf[20]=(pitch>>8)&0XFF;
// 	tbuf[21]=pitch&0XFF;
// 	tbuf[22]=(yaw>>8)&0XFF;
// 	tbuf[23]=yaw&0XFF;
// 	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
// }

//float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
// float exInt = 0, eyInt = 0, ezInt = 0;    //当前加计测得的重力加速度在三轴上的分量
//                                 //与用当前姿态计算得来的重力在三轴上的分量的误差的积分
// void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
// {

//   float q0temp,q1temp,q2temp,q3temp;//四元数暂存变量，求解微分方程时要用
//   float norm; //矢量的模或四元数的范数
//   float vx, vy, vz;//当前姿态计算得来的重力在三轴上的分量
//   float ex, ey, ez;//当前加计测得的重力加速度在三轴上的分量
//               //与用当前姿态计算得来的重力在三轴上的分量的误差

//   // 先把这些用得到的值算好
//   float q0q0 = q0*q0;
//   float q0q1 = q0*q1;
//   float q0q2 = q0*q2;
//   float q1q1 = q1*q1;
//   float q1q3 = q1*q3;
//   float q2q2 = q2*q2;
//   float q2q3 = q2*q3;
//   float q3q3 = q3*q3;
//   if(ax*ay*az==0)//加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
//         return;
//   norm = sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，
//   ax = ax /norm;// 这样变更了量程也不需要修改KP参数，因为这里归一化了
//   ay = ay / norm;
//   az = az / norm;
//   //用当前姿态计算出重力在三个轴上的分量，
//   //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
//   vx = 2*(q1q3 - q0q2);
//   vy = 2*(q0q1 + q2q3);
//   vz = q0q0 - q1q1 - q2q2 + q3q3 ;
//   //计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
//   //原因我理解是因为两个向量是单位向量且sin0等于0
//   //不过要是夹角是180度呢~这个还没理解
//   ex = (ay*vz - az*vy) ;
//   ey = (az*vx - ax*vz) ;
//   ez = (ax*vy - ay*vx) ;

//   exInt = exInt + ex * Ki;                                           //对误差进行积分
//   eyInt = eyInt + ey * Ki;
//   ezInt = ezInt + ez * Ki;
//   // adjusted gyroscope measurements
//   gx = gx + Kp*ex + exInt;  //将误差PI后补偿到陀螺仪，即补偿零点漂移
//   gy = gy + Kp*ey + eyInt;
//   gz = gz + Kp*ez + ezInt;    //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
//   //下面进行姿态的更新，也就是四元数微分方程的求解
//   q0temp=q0;//暂存当前值用于计算
//   q1temp=q1;//网上传的这份算法大多没有注意这个问题，在此更正
//   q2temp=q2;
//   q3temp=q3;
//   //采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
//   q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*halfT;
//   q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*halfT;
//   q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*halfT;
//   q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*halfT;
//   //单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
//   norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
//   q0 = q0 / norm;
//   q1 = q1 / norm;
//   q2 = q2 / norm;
//   q3 = q3 / norm;
//   //四元数到欧拉角的转换，公式推导见博文一
//   //其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替
//   //Q_ANGLE.Z = GYRO_I.Z; // yaw
//   yaw = gz;
//   pitch = asin(-2 * q1 * q3 + 2 * q0* q2)*57.3; // pitch
//   roll = atan2(2 * q2 * q3 + 2 * q0 * q1,-2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
// }
