#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;
    float last_err;
    float last_last_err;
    float out;

    float out_min;
    float out_max;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd, float target, float out_min, float out_max);
float PID_Update(PID *pid, float feedback);
void PID_Reset(PID *pid);

#endif
