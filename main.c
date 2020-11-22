/*********************************************************************************
 * 文件名  ：main.c
 * 描述    ：无人机      
 * 实验平台： STM32开发板
 * 库版本  ：ST3.5.0
 * 作者    ：  零
**********************************************************************************/
#include "stm32f10x.h"
#include "pwm_output.h"
#include "key.h"
#include "delay.h"
#include "QDTFT_demo.h"
#include "led.h"
#include "Lcd_Driver.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "usart.h"	
#include "delay.h"
#include "misc.h"
#include "GUI.h"
#include "pid_1.h"
#include "math.h"
#include "control.h"
#include "string.h"
#include "usmart.h"
#include "stm32f10x_usart.h"
#include "beep.h"
#include "comand.h"
/************************************************/
u8 rec_flag=0;	//是否接收完数据的标志
u8 handlle_flag=1; //一次数据有没有处理完的标记
u8 i=0;			//循环显示到屏幕
u8 y=1;
int data=0;
u8 count = 0;  //串口接收数据位数
u8 Data[8];
u8 Data1[8];
char res;

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART3,c);  
}
/* 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
} 
************************************************/




//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
    u8 send_buf[32];
    u8 i;
    if(len>28)return;    //最多28字节数据
    send_buf[len+3]=0;  //校验数置零
    send_buf[0]=0X88;   //帧头
    send_buf[1]=fun;    //功能字
    send_buf[2]=len;    //数据长度
    for(i=0;i<len;i++)send_buf[3+i]=data[i];         //复制数据
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];    //计算校验和
    for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);   //发送数据到串口1
}
void mpu6050_send_data(float pitch,float roll,float yaw)
{
    u8 tbuf[16];
    unsigned char *p;
    p=(unsigned char *)&pitch;
    tbuf[0]=(unsigned char)(*(p+3));
    tbuf[1]=(unsigned char)(*(p+2));
    tbuf[2]=(unsigned char)(*(p+1));
    tbuf[3]=(unsigned char)(*(p+0));   
    p=(unsigned char *)&roll;
    tbuf[4]=(unsigned char)(*(p+3));
    tbuf[5]=(unsigned char)(*(p+2));
    tbuf[6]=(unsigned char)(*(p+1));
    tbuf[7]=(unsigned char)(*(p+0));    
    p=(unsigned char *)&yaw;
    tbuf[8]=(unsigned char)(*(p+3));
    tbuf[9]=(unsigned char)(*(p+2));
    tbuf[10]=(unsigned char)(*(p+1));
    tbuf[11]=(unsigned char)(*(p+0));     
    usart1_niming_report(0XA2,tbuf,12);//自定义帧,0XA2
}  
u16 pwm1=2000,pwm2=2000,pwm3=2000,pwm4=2000;


/************************************************/
/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
int main(void)
{	
	char  str[20] = "assdadaw";
	u8 i = 0;
	u8 report=1; //默认开启上报				
	u8 temp_value[20];			//存储陀螺仪的临时值
	u8 temp_value2[20];			//存储pwm输出的临时值
	u8 temp_value1[20];
	u8 t=0;						//页面循环次数
	u8 key;						//按键值
	u8 key_status=0;			//按键状态
	u8 keystatus=0;				//按键s4的状态值
	u8 Control = 1;					//失控标记
	float temp;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据	
	int Motor1=0;		//电机1输出
	int Motor2=0;		//电机2输出
	int Motor3=0;		//电机3输出
	int Motor4=0;		//电机4输出	
	float Pitch;
	float Roll;
	float Yaw;
	PIDx_init();
	PIDy_init();
	PIDz_init();
	pitch_init();
	roll_init();
	yaw_init();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级	
	delay_init();
	KEY_Init();
	BEEP_Init();
	uart_init(9600);  //初始化串口
	usmart_dev.init(72);		//初始化USMART
	pwm_init();/* TIM3 PWM波输出初始化，并使能TIM3 PWM输出 */
	Lcd_Init();
	LCD_LED_SET;//通过IO控制背光亮	
	Lcd_Clear(GRAY0);
	delay_ms(1000);	
	MPU_Init();					//初始化MPU6050
	Gui_DrawFont_GBK16(25,30,YELLOW,GRAY0,"Start check");
	while(mpu_dmp_init())Gui_DrawFont_GBK16(16,50,YELLOW,GRAY0,"MPU6050 Error");
	Gui_DrawFont_GBK16(25,50,YELLOW,GRAY0,"MPU6050 OK");
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(25,30,YELLOW,GRAY0,"S3->shuju");
	Gui_DrawFont_GBK16(25,50,BLUE,GRAY0,"S4->PWM");
	/*
	while(1)
	{	
		//for(;i<6;i++)
		//{
			//usart1_send_char(testchar[i]);
		//}
		//i = 0;
		printf("BLUE,GRAY0,temp_value2\r\n");
		delay_ms(600);
	}
	*/
	//BEEP=1;  	
	while (1)
		{ 
	
	
			
		if(keystatus!=1&&key!=2)
		key=KEY_Scan(0);    //得到键值
        if(key)
        {                           
            switch(key)
            {                 
                case KEY0_PRES:    //
				{					
					Lcd_Clear(GRAY0);
					Gui_DrawFont_GBK16(25,30,YELLOW,GRAY0,"M1:      	");
					Gui_DrawFont_GBK16(25,50,BLUE,GRAY0,"M2:      	 ");
					Gui_DrawFont_GBK16(25,70,RED,GRAY0, "M3:      	 ");
					Gui_DrawFont_GBK16(25,90,BLUE,GRAY0,"M4:      	 ");
					while (1)
					{
						key = KEY_Scan(1);
						if (key)
							switch(key)
								{ 
								case KEY1_PRES:
									key_status=1;
									break;
								}						
						if(key_status==1) break;
						//有没有更新值，有更新新的值就会继续往下执行
						if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
						{ 								
							//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
							if(pitch>25||pitch<-25||roll>25||roll<-25||yaw>25||yaw<-25)
							{
							Control = 0;
							}							
							if(Control)
						{
							MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
									
						
							//外环角度 角度-角速度
							Pitch = PIDx_out_realize(pitch);
							Roll = PIDy_out_realize(roll);
							Yaw = PIDz_out_realize(yaw);
							//内环角速度 角速度-PWM
							Pitch =PIDx_inner_realize(Pitch,gyrox);
							Roll =PIDy_inner_realize(Roll,gyroy);
							Yaw =PIDz_inner_realize(Yaw,gyroz);

							//Motor1 = (int)(2200 - Pitch + Roll - Yaw);
							//Motor2 = (int)(2200 + Pitch + Roll + Yaw);
							//Motor3 = (int)(2200 + Pitch - Roll - Yaw);
							//Motor4 = (int)(2200 - Pitch - Roll + Yaw);
				
							Motor1 = (int)(2300 - Pitch + Roll);//9
							Motor2 = (int)(2300 + Pitch + Roll);//11
							Motor3 = (int)(2300 + Pitch - Roll);//3
							Motor4 = (int)(2300 - Pitch - Roll);//10
						}
						else
						{
							TIM_SetCompare1(TIM3,0);//	 	
							TIM_SetCompare2(TIM3,0);//	
							TIM_SetCompare3(TIM3,0);//	 
							TIM_SetCompare4(TIM3,0);//
							break;
							
						}
							//TIM_SetCompare1(TIM3,Motor4);//	 	
							//TIM_SetCompare2(TIM3,Motor1);//	
							//TIM_SetCompare3(TIM3,Motor2);//	 
							//TIM_SetCompare4(TIM3,Motor3);//	
						
							if(t%1==0)
							{
							sprintf(temp_value2,"%4d",Motor4);
							Gui_DrawFont_GBK16(50,30,BLUE,GRAY0,temp_value2);	
							sprintf(temp_value2,"%4d",Motor1);
							Gui_DrawFont_GBK16(50,50,BLUE,GRAY0,temp_value2);	
							sprintf(temp_value2,"%4d",Motor2);
							Gui_DrawFont_GBK16(50,70,BLUE,GRAY0,temp_value2);	
							sprintf(temp_value2,"%4d",Motor3);
							Gui_DrawFont_GBK16(50,90,BLUE,GRAY0,temp_value2);
							
												
							if(rec_flag)
							{	
								Lcd_Clear(GRAY0);
								/*
								for(;i<count;i++)
								{
								res = Data[i];
								sprintf(temp_value1,"%d",(int)(res)-48);
								Gui_DrawFont_GBK16(25+i*15,110,BLUE,GRAY0,temp_value1);
								}
								sprintf(temp_value1,"%d",(count));
								Gui_DrawFont_GBK16(25,130,BLUE,GRAY0,temp_value1);
								*/
								
								
								handlle_flag=0;
								//for(;i<count;i++)
									//usart1_send_char(Data[i]);
								comand_handle(i,Data,count);
								
								
								for(;count-1>=0;count--)
								{
									//最大发送255
								if(y==1)
								{									
								data = (int)(USART_RX_BUF[count-1])-48;
								//USART_RX_BUF[count-1]= 0;
								}
								else if(y==2)
								{
								data += ((int)(USART_RX_BUF[count-1])-48)*10;
								//USART_RX_BUF[count-1]= 0;
								}
								else if(y==3)
								{
								data += ((int)(USART_RX_BUF[count-1])-48)*100;
								//USART_RX_BUF[count-1]= 0;
								}
								else data = 0;
								y++;
								}			
								sprintf(temp_value1,"%d",data);
								Gui_DrawFont_GBK16(50,130,BLUE,GRAY0,temp_value1);
								//usart1_send_char(data);
							i=0;
							count = 0;
							y=1;
							data = 0;
							//while((USART3->SR&0X40)==0);
							USART_RX_STA=0;
							rec_flag=0;
							handlle_flag=1;	
							}
							for(;i<8;i++)
								usart1_send_char(str[i]);
							i =0;
							//if(report)mpu6050_send_data(pitch,roll,yaw);//用自定义帧发送加速度和陀螺仪原始数据
			                //if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
							
						}
					t++;
				}
				key_status=0;				
			}
		}
			
                case KEY1_PRES:    //
				{
				if(Control==0)
				{
					Control=1;
					break;
				}
				keystatus = 0;
				Lcd_Clear(GRAY0);
				Gui_DrawFont_GBK16(25,50,BLUE,GRAY0,"Pitch:    .C");
				Gui_DrawFont_GBK16(25,70,RED,GRAY0, "Roll:    .C");
				Gui_DrawFont_GBK16(25,90,BLUE,GRAY0,"Yaw :    .C");		
				while(1){
				key = KEY_Scan(0);
						if (key)
							switch(key)
								{ 
								case KEY0_PRES:
									key_status=1;
									break;
								}
				if(key_status==1) break;
				if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
						{ 
							//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
							MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据										
							Pitch = PIDx_out_realize(pitch);
							Roll = PIDy_out_realize(roll);
							Yaw = PIDz_out_realize(yaw);
							Pitch =PIDx_inner_realize(Pitch,gyrox);
							Roll =PIDy_inner_realize(Roll,gyroy);
							Yaw =PIDz_inner_realize(Yaw,gyroz);
							//Motor1 = (int)(2200 - Pitch + Roll - Yaw);
							//Motor2 = (int)(2200 + Pitch + Roll + Yaw);
							//Motor3 = (int)(2200 + Pitch - Roll - Yaw);
							//Motor4 = (int)(2200 - Pitch - Roll + Yaw);
							
							Motor1 = (int)(2300 - Pitch + Roll);//9
							Motor2 = (int)(2300 + Pitch + Roll);//11
							Motor3 = (int)(2300 + Pitch - Roll);//3
							Motor4 = (int)(2300 - Pitch - Roll);//10
							
							//TIM_SetCompare1(TIM3,Motor4);//	 	
							//TIM_SetCompare2(TIM3,Motor1);//	
							//TIM_SetCompare3(TIM3,Motor2);//	 
							//TIM_SetCompare4(TIM3,Motor3);//	
							
				temp=pitch;
				if(temp<0)
				{
					Gui_DrawFont_GBK16(10,50,BLUE,GRAY0,"-");
					temp=-temp;		//转为正数
				}else Gui_DrawFont_GBK16(10,50,BLUE,GRAY0,"  ");//去掉负号 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(70,50,BLUE,GRAY0,temp_value);	
				temp=roll;
				if(temp<0)
				{
					Gui_DrawFont_GBK16(10,70,BLUE,GRAY0,"-");
					temp=-temp;		//转为正数
				}else Gui_DrawFont_GBK16(10,70,BLUE,GRAY0," ");//去掉负号 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(65,70,BLUE,GRAY0,temp_value);	
				temp=yaw;
				//z轴
				if(temp<0)
				{
					Gui_DrawFont_GBK16(10,90,BLUE,GRAY0,"-");
					temp=-temp;		//转为正数
				}else Gui_DrawFont_GBK16(10,90,BLUE,GRAY0," ");//去掉负号 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(65,90,BLUE,GRAY0,temp_value);	
				}
				if(report)mpu6050_send_data(pitch,roll,yaw);//用自定义帧发送加速度和陀螺仪原始数据
				//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			}
				key_status=0;
				keystatus=1;
		}
				
                case KEY2_PRES:    //
                    break;
                case KEY3_PRES:    //    
                    break;
				case KEY4_PRES:    //
                    break;
            }
        }else delay_ms(10);	
	}			
}
