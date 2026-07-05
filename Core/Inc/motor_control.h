#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#include "app_config.h"

#define MOTOR_BEMF_PWM_SAMPLE_OFF_TIME  0u
#define MOTOR_BEMF_PWM_SAMPLE_ON_TIME   1u
#define MOTOR_SIXSTEP_PHASE_OPEN_LOOP   0u
#define MOTOR_SIXSTEP_PHASE_SIN_HANDOFF 1u
#define MOTOR_SIXSTEP_PHASE_CLOSED_LOOP 2u
#define MOTOR_SIXSTEP_PHASE_RECOVERY_OFF 3u
#define MOTOR_SIXSTEP_PHASE_CURRENT_FAULT 4u

typedef struct
{
    motor_mode_t mode;
    motor_direction_t direction;
    int32_t target_rpm;
    int32_t ramped_target_rpm;
    int32_t measured_erpm;
    uint32_t pwm_carrier_hz;
    uint32_t sinus_angle_q16;
    uint32_t sinus_step_q16;
    uint32_t open_loop_tick;
    uint32_t sinus_at_target_ticks;
    uint32_t sixstep_run_ticks;
    uint32_t sin_current_target_ma;
    uint32_t sixstep_current_limit_ma;
    uint32_t hard_current_limit_ma;
    uint16_t hard_current_adc_threshold;
    uint32_t measured_current_ma;
    uint32_t control_tick_count;
    uint32_t sin_current_update_count;
    uint32_t sixstep_speed_update_count;
    uint32_t sixstep_current_limit_update_count;
    uint32_t hard_current_fault_count;
    int32_t sin_current_error_adc;
    int32_t sixstep_target_rpm;
    int32_t sixstep_measured_rpm;
    int32_t sixstep_rpm_error;
    int32_t sixstep_current_error_ma;
    uint16_t sixstep_commutation_count;
    uint16_t current_adc_raw;
    uint16_t current_adc_filtered;
    uint16_t current_adc_signal;
    uint16_t sin_current_target_adc;
    uint16_t sinus_pwm_max_ticks;
    uint16_t sinus_pwm_ticks;
    uint16_t sin_current_pi_output_permille;
    uint16_t sin_current_target_pwm_ticks;
    uint16_t sixstep_pwm_limit_ticks;
    uint16_t sixstep_pwm_ticks;
    uint16_t sixstep_speed_pid_target_permille;
    uint16_t sixstep_speed_pid_output_permille;
    uint32_t sixstep_speed_pid_output_q16;
    uint32_t sixstep_speed_pid_rise_step_q16;
    uint32_t sixstep_speed_pid_fall_step_q16;
    uint16_t sixstep_current_limit_reduction_permille;
    uint16_t sixstep_final_pwm_permille;
    uint16_t sixstep_interval_ticks;
    uint16_t sixstep_fallback_interval_ticks;
    uint16_t sixstep_tick;
    uint16_t sixstep_bemf_delay_ticks;
    uint16_t sixstep_recovery_off_tick;
    volatile uint16_t bemf_interval_ticks;
    volatile uint16_t bemf_average_interval_ticks;
    uint16_t bemf_last_zc_ticks;
    uint16_t bemf_this_zc_ticks;
    uint16_t bemf_pwm_gating_open_ticks;
    uint16_t bemf_pwm_gating_close_ticks;
    uint16_t sinus_table_index;
    uint16_t sin_current_control_tick;
    uint16_t sixstep_speed_control_tick;
    uint16_t sixstep_current_limit_control_tick;
    uint8_t in_rpm;
    uint8_t bemf_readable;
    uint8_t bemf_blank_ticks;
    uint8_t bemf_poll_count;
    uint8_t bemf_zero_cross_count;
    uint8_t bemf_slow_update_tick;
    uint8_t bemf_edge_seen;
    uint8_t bemf_armed;
    uint8_t bemf_pwm_gating_active_window;
    uint8_t bemf_pwm_gating_close_pending;
    uint8_t bemf_pwm_sample_valid;
    uint8_t bemf_pwm_last_level;
    uint8_t sixstep_bemf_closed_loop;
    uint8_t hard_current_fault;
    uint8_t sixstep_commutation_pending;
    uint8_t sixstep_missed_zc_count;
    uint8_t sixstep_virtual_zc_pending;
    uint8_t sixstep_phase;
    uint8_t sixstep_step;
    uint8_t pwm_pulses_per_sector;
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
