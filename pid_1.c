#include<stdio.h>
#include<stdlib.h>
#include "pid_1.h"






        ////////////////////////�⻷�ǶȻ�(PD)///////////////////////////////   
struct pid{
	float SetSpeed;   				//�����趨ֵ
	float core_out;					//�����⻷�����
	float Kp,Ki;					//������������֡�΢��ϵ��
	float err;        //����ƫ��ֵ
}pidx_out,pidy_out,pidz_out;
void pitch_init(){
	pidx_out.SetSpeed = 0;
	pidx_out.core_out=0.0;
	pidx_out.Kp=0.0;
	pidx_out.Ki=0.0;
}
void roll_init(){
	pidy_out.SetSpeed = 0;
	pidy_out.core_out=0.0;
	pidy_out.Kp=0.0;
	pidy_out.Ki=0.0;
}
void yaw_init(){
	pidz_out.SetSpeed = 0;
	pidz_out.core_out=0.0;
	pidz_out.Kp=0.0;
	pidz_out.Ki=0.0;
}

float PIDx_out_realize(float pitch){
	pidx_out.err=pidx_out.SetSpeed-pitch;
	pidx_out.core_out = pidx_out.Kp *pidx_out.err;
	return pidx_out.core_out;
}
float PIDy_out_realize(float roll){
	pidy_out.err=pidy_out.SetSpeed-roll;
	pidy_out.core_out = pidy_out.Kp *pidy_out.err;
	return pidy_out.core_out;
}
float PIDz_out_realize(float yaw){
	pidz_out.err=pidz_out.SetSpeed-yaw;
	pidz_out.core_out = pidz_out.Kp *pidz_out.err;
	return pidz_out.core_out;
}

//
//Iʹ�ζ����ȼ�С
//D���ƹ���ı仯��Ƶ�ʿ�
//0.3-0.005-0.05

        ////////////////////////�ڻ��Ƕ��ٻ�(PID)///////////////////////////////   
struct _pid{
	float ActualSpeed;//�����⻷���ֵ
	float err;        //����ƫ��ֵ
	float err_next;   //�����ϴ�ƫ��ֵ
	float err_last;   //�������ϴε�ƫ��ֵ
	float Kp,Ki,Kd;   //������������֡�΢��ϵ��
}pidx,pidy,pidz;


void PIDx_init(){
	pidx.ActualSpeed=0.0;    //�����⻷���ֵ
	pidx.err=0.0;            //����ƫ��ֵ
	pidx.err_last=0.0;       //�����ϴ�ƫ��ֵ
	pidx.err_next=0.0;       //�������ϴε�ƫ��ֵ
	pidx.Kp=0.5;             //������������֡�΢��ϵ��
	pidx.Ki=0.005;
	pidx.Kd=0.1;
}
void PIDy_init(){
	pidy.ActualSpeed=0.0;   //�����⻷���ֵ
	pidy.err=0.0;           //����ƫ��ֵ
	pidy.err_last=0.0;      //�����ϴ�ƫ��ֵ
	pidy.err_next=0.0;      //�������ϴε�ƫ��ֵ
	pidy.Kp=0.5;             //������������֡�΢��ϵ��
	pidy.Ki=0.005;
	pidy.Kd=0.1;
}
void PIDz_init(){
	pidz.ActualSpeed=0.0; //�����⻷���ֵ
	pidz.err=0.0;         //����ƫ��ֵ
	pidz.err_last=0.0;    //�����ϴ�ƫ��ֵ
	pidz.err_next=0.0;    //�������ϴε�ƫ��ֵ
	pidz.Kp=0.5;           //������������֡�΢��ϵ��
	pidz.Ki=0.005;
	pidz.Kd=0.1;
}

float PIDx_inner_realize(float speed,short gyrox){
	float incrementSpeed;
	//pidx.ActualSpeed=speed;
    pidx.err=speed - gyrox/16.4;
    incrementSpeed=pidx.Kp*(pidx.err-pidx.err_next)+pidx.Ki*pidx.err+pidx.Kd*(pidx.err-2*pidx.err_next+pidx.err_last);//ֻ��ǰ�����ε����ֵ�йأ�Ҳ�������
    pidx.ActualSpeed+=incrementSpeed;
    pidx.err_last=pidx.err_next;
    pidx.err_next=pidx.err;
	/*
	if(pidx.ActualSpeed>1000)
		pidx.ActualSpeed=1000;
	if(pidx.ActualSpeed<1000)
		pidx.ActualSpeed=1000;
	*/
    return pidx.ActualSpeed;
}


float PIDy_inner_realize(float speed,short gyroy){
	float incrementSpeed;
	//pidy.ActualSpeed=speed;
    pidy.err=speed - gyroy/16.4;
    incrementSpeed=pidy.Kp*(pidy.err-pidy.err_next)+pidy.Ki*pidy.err+pidy.Kd*(pidy.err-2*pidy.err_next+pidy.err_last);//ֻ��ǰ�����ε����ֵ�йأ�Ҳ�������
    pidy.ActualSpeed+=incrementSpeed;
    pidy.err_last=pidy.err_next;
    pidy.err_next=pidy.err;
	/*
	if(pidy.ActualSpeed1000)
		pidy.ActualSpeed=1000;
	if(pidy.ActualSpeed<1000)
		pidy.ActualSpeed=1000;
	*/
    return pidy.ActualSpeed;
}

float PIDz_inner_realize(float speed,short gyroz){
	float incrementSpeed;
	//pidz.ActualSpeed=speed;
    pidz.err=speed - gyroz/16.4;
    incrementSpeed=pidz.Kp*(pidz.err-pidz.err_next)+pidz.Ki*pidz.err+pidz.Kd*(pidz.err-2*pidz.err_next+pidz.err_last);//ֻ��ǰ�����ε����ֵ�йأ�Ҳ�������
    pidz.ActualSpeed+=incrementSpeed;
    pidz.err_last=pidz.err_next;
    pidz.err_next=pidz.err;
	/*
	if(pidz.ActualSpeed>1000)
		pidz.ActualSpeed=1000;
	if(pidz.ActualSpeed<1000)
		pidz.ActualSpeed=1000;
	*/
    return pidz.ActualSpeed;
}


