#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "app_config.h"

#define MOTOR_BEMF_LED_PHASE_NONE       0u
#define MOTOR_BEMF_LED_PHASE_A          1u
#define MOTOR_BEMF_LED_PHASE_B          2u
#define MOTOR_BEMF_LED_PHASE_C          3u
#define MOTOR_BEMF_PHASE_A_MASK         0x01u
#define MOTOR_BEMF_PHASE_B_MASK         0x02u
#define MOTOR_BEMF_PHASE_C_MASK         0x04u

typedef struct
{
    motor_mode_t mode;
    motor_direction_t direction;
    int32_t target_rpm;
    int32_t ramped_target_rpm;
    int32_t measured_erpm;
    uint32_t sinus_angle_q16;
    uint32_t sinus_step_q16;
    uint32_t open_loop_tick;
    uint32_t sinus_at_target_ticks;
    uint32_t sixstep_run_ticks;
    uint16_t sixstep_commutation_count;
    uint16_t sinus_pwm_limit_ticks;
    uint16_t sixstep_pwm_limit_ticks;
    uint16_t sixstep_pwm_ticks;
    uint16_t sixstep_interval_ticks;
    uint16_t sixstep_tick;
    uint16_t sixstep_speed_control_tick;
    uint16_t sixstep_bemf_delay_ticks;
    volatile uint16_t bemf_interval_ticks;
    volatile uint16_t bemf_average_interval_ticks;
    uint16_t bemf_last_zc_ticks;
    uint16_t bemf_this_zc_ticks;
    uint16_t bemf_led_hold_ticks;
    uint16_t sinus_table_index;
    uint8_t in_rpm;
    uint8_t bemf_readable;
    uint8_t bemf_blank_ticks;
    uint8_t bemf_poll_count;
    uint8_t bemf_zero_cross_count;
    uint8_t bemf_edge_seen;
    uint8_t bemf_armed;
    uint8_t bemf_phase_mask;
    uint8_t sixstep_bemf_closed_loop;
    uint8_t sixstep_commutation_pending;
    uint8_t sixstep_closed_loop_handoff_steps;
    uint8_t sixstep_step;
    uint8_t bemf_led_phase;
    uint8_t bemf_led_phase_mask;
    uint8_t bemf_diag_phase;
    uint8_t bemf_diag_initialized;
    uint8_t bemf_diag_level_a;
    uint8_t bemf_diag_level_b;
    uint8_t bemf_diag_level_c;
} motor_control_state_t;

extern volatile motor_control_state_t g_motor;

void MOTOR_Init(void);
void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction);
void MOTOR_ControlTick10kHz(void);
void MOTOR_SinusStepIrq(void);
void MOTOR_BemfZeroCrossIrq(void);
void MOTOR_BemfCommutationTimerIrq(void);
void MOTOR_BemfPwmSampleIrq(void);

#endif
