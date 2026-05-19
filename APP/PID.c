#include "PID.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float target, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->target = target;

    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->last_err = 0.0f;
    pid->last_last_err = 0.0f;
    pid->out = 0.0f;
}

float PID_Update(PID *pid, float feedback)
{
    float err = pid->target - feedback;
    float delta = pid->Kp * (err - pid->last_err)
                + pid->Ki * err
                + pid->Kd * (err - 2.0f * pid->last_err + pid->last_last_err);

    pid->out += delta;

    if (pid->out > pid->out_max) {
        pid->out = pid->out_max;
    } else if (pid->out < pid->out_min) {
        pid->out = pid->out_min;
    }

    pid->last_last_err = pid->last_err;
    pid->last_err = err;

    return pid->out;
}

void PID_Reset(PID *pid)
{
    pid->last_err = 0.0f;
    pid->last_last_err = 0.0f;
    pid->out = 0.0f;
}
