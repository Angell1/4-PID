/*********************************************************************************
 * �ļ���  ��main.c
 * ����    �����˻�      
 * ʵ��ƽ̨�� STM32������
 * ��汾  ��ST3.5.0
 * ����    ��  ��
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
u8 rec_flag=0;	//�Ƿ���������ݵı�־
u8 handlle_flag=1; //һ��������û�д�����ı��
u8 i=0;			//ѭ����ʾ����Ļ
u8 y=1;
int data=0;
u8 count = 0;  //���ڽ�������λ��
u8 Data[8];
u8 Data1[8];
char res;

//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART3,c);  
}
/* 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	

//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
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
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
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
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
} 
************************************************/




//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
    u8 send_buf[32];
    u8 i;
    if(len>28)return;    //���28�ֽ�����
    send_buf[len+3]=0;  //У��������
    send_buf[0]=0X88;   //֡ͷ
    send_buf[1]=fun;    //������
    send_buf[2]=len;    //���ݳ���
    for(i=0;i<len;i++)send_buf[3+i]=data[i];         //��������
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];    //����У���
    for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);   //�������ݵ�����1
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
    usart1_niming_report(0XA2,tbuf,12);//�Զ���֡,0XA2
}  
u16 pwm1=2000,pwm2=2000,pwm3=2000,pwm4=2000;


/************************************************/
/*
 * ��������main
 * ����  ��������
 * ����  ����
 * ���  ����
 */
int main(void)
{	
	char  str[20] = "assdadaw";
	u8 i = 0;
	u8 report=1; //Ĭ�Ͽ����ϱ�				
	u8 temp_value[20];			//�洢�����ǵ���ʱֵ
	u8 temp_value2[20];			//�洢pwm�������ʱֵ
	u8 temp_value1[20];
	u8 t=0;						//ҳ��ѭ������
	u8 key;						//����ֵ
	u8 key_status=0;			//����״̬
	u8 keystatus=0;				//����s4��״ֵ̬
	u8 Control = 1;					//ʧ�ر��
	float temp;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����	
	int Motor1=0;		//���1���
	int Motor2=0;		//���2���
	int Motor3=0;		//���3���
	int Motor4=0;		//���4���	
	float Pitch;
	float Roll;
	float Yaw;
	PIDx_init();
	PIDy_init();
	PIDz_init();
	pitch_init();
	roll_init();
	yaw_init();
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�	
	delay_init();
	KEY_Init();
	BEEP_Init();
	uart_init(9600);  //��ʼ������
	usmart_dev.init(72);		//��ʼ��USMART
	pwm_init();/* TIM3 PWM�������ʼ������ʹ��TIM3 PWM��� */
	Lcd_Init();
	LCD_LED_SET;//ͨ��IO���Ʊ�����	
	Lcd_Clear(GRAY0);
	delay_ms(1000);	
	MPU_Init();					//��ʼ��MPU6050
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
		key=KEY_Scan(0);    //�õ���ֵ
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
						//��û�и���ֵ���и����µ�ֵ�ͻ��������ִ��
						if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
						{ 								
							//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
							if(pitch>25||pitch<-25||roll>25||roll<-25||yaw>25||yaw<-25)
							{
							Control = 0;
							}							
							if(Control)
						{
							MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
									
						
							//�⻷�Ƕ� �Ƕ�-���ٶ�
							Pitch = PIDx_out_realize(pitch);
							Roll = PIDy_out_realize(roll);
							Yaw = PIDz_out_realize(yaw);
							//�ڻ����ٶ� ���ٶ�-PWM
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
									//�����255
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
							//if(report)mpu6050_send_data(pitch,roll,yaw);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
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
							//MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
							MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������										
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
					temp=-temp;		//תΪ����
				}else Gui_DrawFont_GBK16(10,50,BLUE,GRAY0,"  ");//ȥ������ 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(70,50,BLUE,GRAY0,temp_value);	
				temp=roll;
				if(temp<0)
				{
					Gui_DrawFont_GBK16(10,70,BLUE,GRAY0,"-");
					temp=-temp;		//תΪ����
				}else Gui_DrawFont_GBK16(10,70,BLUE,GRAY0," ");//ȥ������ 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(65,70,BLUE,GRAY0,temp_value);	
				temp=yaw;
				//z��
				if(temp<0)
				{
					Gui_DrawFont_GBK16(10,90,BLUE,GRAY0,"-");
					temp=-temp;		//תΪ����
				}else Gui_DrawFont_GBK16(10,90,BLUE,GRAY0," ");//ȥ������ 
				sprintf(temp_value,"%.2f",temp);
				Gui_DrawFont_GBK16(65,90,BLUE,GRAY0,temp_value);	
				}
				if(report)mpu6050_send_data(pitch,roll,yaw);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
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
