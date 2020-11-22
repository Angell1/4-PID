#ifndef __PID_H
#define __PID_H
#include "sys.h"



float PIDx_out_realize(float pitch);
float PIDy_out_realize(float roll);
float PIDz_out_realize(float yaw);
float PIDx_inner_realize(float pitch,short gyrox);
float PIDy_inner_realize(float roll,short gyroy);
float PIDz_inner_realize(float yaw,short gyroz);
void PIDx_init();
void PIDy_init();
void PIDz_init();
void pitch_init();
void roll_init();
void yaw_init();
#endif