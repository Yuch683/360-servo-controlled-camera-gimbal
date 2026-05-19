#ifndef __SERVO_CONTRAL_H
#define __SERVO_CONTRAL_H

#include "tim.h"
#include "PID.h"

/* Zone thresholds (pixels from image center) */
#define ZONE_FAR         320.0f
#define ZONE_MID         160.0f
#define ZONE_NEAR         60.0f

/* Normalized speed limits (-1.0 ~ 1.0 maps to full PWM range 500~2500 us) */
#define PAN_SPEED_MAX     1.0f
#define PAN_SPEED_MIN    -1.0f
#define TILT_SPEED_MAX    1.0f
#define TILT_SPEED_MIN   -1.0f

/* Far-zone max speed (reduced to avoid overshoot / target loss at long range) */
#define PAN_FAR_SPEED   0.50f
#define TILT_FAR_SPEED  0.50f

/* Minimum effective speed — the smallest PWM offset that actually moves the servo. */
#define PAN_MIN_EFFECTIVE_SPEED   0.06f
#define TILT_MIN_EFFECTIVE_SPEED  0.06f

/* Direction: +1.0 = normal, -1.0 = reverse */
#define PAN_DIR     -1.0f
#define TILT_DIR    -1.0f

/* PID controller gains for mid zone */
#define PAN_KP  0.008f
#define PAN_KI  0.0004f
#define PAN_KD  0.004f
#define TILT_KP 0.003f
#define TILT_KI 0.0005f
#define TILT_KD 0.004f

/* Mid-zone speed limit (keep output modest to reduce overshoot) */
#define PAN_MID_SPEED_MAX  0.15f
#define TILT_MID_SPEED_MAX 0.12f

/* Duty cycle modulation period in ms for near zone */
#define DUTY_CYCLE_PERIOD_MS  60U

/* PWM parameters (unit: us) */
#define SERVO_PWM_PERIOD_US      20000U
#define SERVO_PULSE_MIN_US       500U
#define SERVO_PULSE_NEUTRAL_US   1500U
#define SERVO_PULSE_MAX_US       2500U

typedef enum {
    SERVO_MODE_STOP = 0,
    SERVO_MODE_DIRECT,
    SERVO_MODE_DUTY_CYCLE
} ServoMode;

void Servo_Init(void);
void Servo_SetPanSpeed(float speed);
void Servo_SetTiltSpeed(float speed);
void Servo_SetPanTilt(float pan_speed, float tilt_speed);
void Servo_Contral(float x, float y);
void Servo_Update(void);

float Servo_GetPanSpeed(void);
float Servo_GetTiltSpeed(void);
ServoMode Servo_GetPanMode(void);
ServoMode Servo_GetTiltMode(void);

#endif
