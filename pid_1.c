#include<stdio.h>
#include<stdlib.h>
#include "pid_1.h"






        ////////////////////////外环角度环(PD)///////////////////////////////   
struct pid{
	float SetSpeed;   				//定义设定值
	float core_out;					//定义外环总输出
	float Kp,Ki;					//定义比例、积分、微分系数
	float err;        //定义偏差值
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
//I使晃动幅度减小
//D抑制过快的变化，频率快
//0.3-0.005-0.05

        ////////////////////////内环角度速环(PID)///////////////////////////////   
struct _pid{
	float ActualSpeed;//定义外环输出值
	float err;        //定义偏差值
	float err_next;   //定义上次偏差值
	float err_last;   //定义最上次的偏差值
	float Kp,Ki,Kd;   //定义比例、积分、微分系数
}pidx,pidy,pidz;


void PIDx_init(){
	pidx.ActualSpeed=0.0;    //定义外环输出值
	pidx.err=0.0;            //定义偏差值
	pidx.err_last=0.0;       //定义上次偏差值
	pidx.err_next=0.0;       //定义最上次的偏差值
	pidx.Kp=0.5;             //定义比例、积分、微分系数
	pidx.Ki=0.005;
	pidx.Kd=0.1;
}
void PIDy_init(){
	pidy.ActualSpeed=0.0;   //定义外环输出值
	pidy.err=0.0;           //定义偏差值
	pidy.err_last=0.0;      //定义上次偏差值
	pidy.err_next=0.0;      //定义最上次的偏差值
	pidy.Kp=0.5;             //定义比例、积分、微分系数
	pidy.Ki=0.005;
	pidy.Kd=0.1;
}
void PIDz_init(){
	pidz.ActualSpeed=0.0; //定义外环输出值
	pidz.err=0.0;         //定义偏差值
	pidz.err_last=0.0;    //定义上次偏差值
	pidz.err_next=0.0;    //定义最上次的偏差值
	pidz.Kp=0.5;           //定义比例、积分、微分系数
	pidz.Ki=0.005;
	pidz.Kd=0.1;
}

float PIDx_inner_realize(float speed,short gyrox){
	float incrementSpeed;
	//pidx.ActualSpeed=speed;
    pidx.err=speed - gyrox/16.4;
    incrementSpeed=pidx.Kp*(pidx.err-pidx.err_next)+pidx.Ki*pidx.err+pidx.Kd*(pidx.err-2*pidx.err_next+pidx.err_last);//只和前后三次的误差值有关，也方便计算
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
    incrementSpeed=pidy.Kp*(pidy.err-pidy.err_next)+pidy.Ki*pidy.err+pidy.Kd*(pidy.err-2*pidy.err_next+pidy.err_last);//只和前后三次的误差值有关，也方便计算
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
    incrementSpeed=pidz.Kp*(pidz.err-pidz.err_next)+pidz.Ki*pidz.err+pidz.Kd*(pidz.err-2*pidz.err_next+pidz.err_last);//只和前后三次的误差值有关，也方便计算
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


