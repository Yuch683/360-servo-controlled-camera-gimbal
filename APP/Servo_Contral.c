#include "Servo_Contral.h"
#define ABSF(x) ((x) < 0.0f ? -(x) : (x))

static PID g_pan_pid;
static PID g_tilt_pid;

static float g_pan_speed = 0.0f;
static float g_tilt_speed = 0.0f;
static float g_pan_error = 0.0f;
static float g_tilt_error = 0.0f;
static ServoMode g_pan_mode = SERVO_MODE_STOP;
static ServoMode g_tilt_mode = SERVO_MODE_STOP;


static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/* ----- PWM_PulseSet ----- */
static void Servo_SetPulseUs(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us)
{
    if (pulse_us < SERVO_PULSE_MIN_US) {
        pulse_us = SERVO_PULSE_MIN_US;
    } else if (pulse_us > SERVO_PULSE_MAX_US) {
        pulse_us = SERVO_PULSE_MAX_US;
    }

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
}

/* ----- low-level output (speed in normalized -1.0 ~ 1.0) ----- */
static void Servo_OutputPan(float speed)
{
    float pulse_us = SERVO_PULSE_NEUTRAL_US + speed * (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US);
    Servo_SetPulseUs(&htim9, TIM_CHANNEL_1, (uint16_t)pulse_us);
    g_pan_speed = speed;
}

static void Servo_OutputTilt(float speed)
{
    float pulse_us = SERVO_PULSE_NEUTRAL_US + speed * (SERVO_PULSE_MAX_US - SERVO_PULSE_NEUTRAL_US);
    Servo_SetPulseUs(&htim12, TIM_CHANNEL_1, (uint16_t)pulse_us);
    g_tilt_speed = speed;
}

/* ----- public API ----- */

void Servo_Init(void)
{
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    Servo_SetPulseUs(&htim9, TIM_CHANNEL_1, SERVO_PULSE_NEUTRAL_US);
    Servo_SetPulseUs(&htim12, TIM_CHANNEL_1, SERVO_PULSE_NEUTRAL_US);

    PID_Init(&g_pan_pid,  PAN_KP,  PAN_KI,  PAN_KD, 0.0f, -PAN_MID_SPEED_MAX,  PAN_MID_SPEED_MAX);
    PID_Init(&g_tilt_pid, TILT_KP, TILT_KI, TILT_KD, 0.0f, -TILT_MID_SPEED_MAX, TILT_MID_SPEED_MAX);
}

void Servo_SetPanSpeed(float speed)
{
    g_pan_mode = SERVO_MODE_DIRECT;
    Servo_OutputPan(ClampFloat(speed, PAN_SPEED_MIN, PAN_SPEED_MAX));
}

void Servo_SetTiltSpeed(float speed)
{
    g_tilt_mode = SERVO_MODE_DIRECT;
    Servo_OutputTilt(ClampFloat(speed, TILT_SPEED_MIN, TILT_SPEED_MAX));
}

void Servo_SetPanTilt(float pan_speed, float tilt_speed)
{
    Servo_SetPanSpeed(pan_speed);
    Servo_SetTiltSpeed(tilt_speed);
}

/* ----- core multi-zone control for one axis ----- */
static void Servo_AxisControl(float error, float min_speed, float max_speed,
                               float far_speed, float direction,
                               float min_effective_speed,
                               PID *pid, ServoMode *mode, float *stored_error,
                               void (*output_func)(float))
{
    float abs_err = ABSF(error);
    float sign = (error * direction > 0.0f) ? 1.0f : -1.0f;
    float speed;

    if (abs_err > ZONE_FAR) {
        speed = sign * far_speed;
        *mode = SERVO_MODE_DIRECT;
        PID_Reset(pid);
        output_func(speed);
    } else if (abs_err > ZONE_MID) {
        speed = PID_Update(pid, error);
        speed = ClampFloat(speed, min_speed, max_speed);
        if (ABSF(speed) < min_effective_speed && ABSF(speed) > 0.001f) {
            speed = (speed > 0.0f) ? min_effective_speed : -min_effective_speed;
        }
        *mode = SERVO_MODE_DIRECT;
        output_func(speed);
    } else if (abs_err > ZONE_NEAR) {
        *mode = SERVO_MODE_DUTY_CYCLE;
        *stored_error = error;
    } else {
        *mode = SERVO_MODE_STOP;
        PID_Reset(pid);
        output_func(0.0f);
    }
}

/* ----- main d ----- */
void Servo_Contral(float x, float y)
{
    Servo_AxisControl(x, PAN_SPEED_MIN, PAN_SPEED_MAX, PAN_FAR_SPEED, PAN_DIR,
                      PAN_MIN_EFFECTIVE_SPEED, &g_pan_pid,
                      &g_pan_mode, &g_pan_error, Servo_OutputPan);

    Servo_AxisControl(y, TILT_SPEED_MIN, TILT_SPEED_MAX, TILT_FAR_SPEED, TILT_DIR,
                      TILT_MIN_EFFECTIVE_SPEED, &g_tilt_pid,
                      &g_tilt_mode, &g_tilt_error, Servo_OutputTilt);
}

/* ----- duty-cycle modulation for near zone (called every main-loop tick) ----- */    
static void Servo_DutyCycleAxis(float error, float direction,
                                 float min_effective_speed,
                                 ServoMode mode, void (*output_func)(float))
{
    if (mode != SERVO_MODE_DUTY_CYCLE) return;

    float abs_err = ABSF(error);
    float sign = (error * direction > 0.0f) ? 1.0f : -1.0f;

    /* duty: 0 at ZONE_NEAR, 1.0 at ZONE_MID */
    float duty = (abs_err - ZONE_NEAR) / (ZONE_MID - ZONE_NEAR);
    duty = ClampFloat(duty, 0.0f, 1.0f);

    uint32_t now = HAL_GetTick();
    uint32_t phase = now % DUTY_CYCLE_PERIOD_MS;
    uint32_t on_time = (uint32_t)(duty * DUTY_CYCLE_PERIOD_MS);

    if (phase < on_time) {
        output_func(sign * min_effective_speed);
    } else {
        output_func(0.0f);
    }
}

/* ----- Servo Contral Entry ----- */ 
void Servo_Update(void)
{
    Servo_DutyCycleAxis(g_pan_error, PAN_DIR, PAN_MIN_EFFECTIVE_SPEED, g_pan_mode, Servo_OutputPan);
    Servo_DutyCycleAxis(g_tilt_error, TILT_DIR, TILT_MIN_EFFECTIVE_SPEED, g_tilt_mode, Servo_OutputTilt);
}

/* ----- getters for STATUS command ----- */
float Servo_GetPanSpeed(void)
{
    return g_pan_speed;
}

float Servo_GetTiltSpeed(void)
{
    return g_tilt_speed;
}

ServoMode Servo_GetPanMode(void)
{
    return g_pan_mode;
}

ServoMode Servo_GetTiltMode(void)
{
    return g_tilt_mode;
}
