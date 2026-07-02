#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "app_config.h"

typedef struct
{
    motor_mode_t mode;
    motor_direction_t direction;
    int32_t target_rpm;
    int32_t ramped_target_rpm;
    int32_t measured_erpm;
    uint32_t sinus_angle_q16;
    uint32_t sinus_step_q16;
    uint16_t sinus_pwm_limit_ticks;
    uint16_t sinus_table_index;
    uint8_t in_rpm;
    uint8_t bemf_readable;
    uint8_t sixstep_step;
} motor_control_state_t;

extern motor_control_state_t g_motor;

void MOTOR_Init(void);
void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction);
void MOTOR_ControlTick10kHz(void);
void MOTOR_SinusStepIrq(void);
void MOTOR_BemfZeroCrossIrq(void);

#endif
