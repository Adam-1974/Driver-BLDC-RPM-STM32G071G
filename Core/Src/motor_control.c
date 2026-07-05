#include "motor_control.h"

#include <limits.h>

#include "bemf_am32.h"
#include "board.h"
#include "nvm_config.h"
#include "pid.h"
#include "sinus_table.h"

#define MOTOR_SIXSTEP_STEP_COUNT        6u
#define MOTOR_BEMF_TIMER_HZ             2000000u
#define MOTOR_BEMF_START_INTERVAL_TICKS 10000u
#define MOTOR_BEMF_FILTER_MIN_TICKS     100u
#define MOTOR_BEMF_FILTER_MAX_TICKS     500u
#define MOTOR_BEMF_SLOW_UPDATE_DIVIDER  16u
#define MOTOR_BEMF_ADVANCE_RECIP_Q16    109u
#define MOTOR_BEMF_DIRECT_COMMUTATION_TICKS 2u
#define MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS 800u
#define MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS 1u
#define MOTOR_BEMF_FAST_FILTER_LEVEL    4u
#define MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS 320u
#define MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS 0u
#define MOTOR_BEMF_ULTRA_FAST_FILTER_LEVEL 2u
#define MOTOR_BEMF_ERPM_NUMERATOR       ((MOTOR_BEMF_TIMER_HZ * 60u) / MOTOR_SIXSTEP_STEP_COUNT)
#if DRIVER_SIXSTEP_BEMF_INTERVAL_IIR_SHIFT > 8u
#define MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT 8u
#else
#define MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT DRIVER_SIXSTEP_BEMF_INTERVAL_IIR_SHIFT
#endif
#define MOTOR_PWM_PULSES_MAX_DISPLAY    255u

#if defined(__GNUC__)
#define MOTOR_FAST_CODE                 __attribute__((optimize("O2")))
#else
#define MOTOR_FAST_CODE
#endif

volatile motor_control_state_t g_motor;
static pid_state_t s_sin_current_pid;
static pid_state_t s_sixstep_rpm_pid;
static pid_state_t s_sixstep_current_limit_pid;

static MOTOR_FAST_CODE void MOTOR_HandleBemfZeroCross(void);
static MOTOR_FAST_CODE void MOTOR_ServiceBemfPwmGatingIrq(void);
static MOTOR_FAST_CODE uint8_t MOTOR_StartBemfPwmGating(void);
static MOTOR_FAST_CODE void MOTOR_StopBemfPwmGating(void);
static MOTOR_FAST_CODE void MOTOR_CommitBemfCommutation(void);
static MOTOR_FAST_CODE void MOTOR_EnableBemfClosedLoop(void);
static MOTOR_FAST_CODE void MOTOR_ApplySixStepBridge(void);
static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfRisingForStep(void);
static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStepIndex(void);
static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStep(void);
static MOTOR_FAST_CODE void MOTOR_ArmBemfForSixStep(void);
static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfRecoveryOff(void);
static MOTOR_FAST_CODE void MOTOR_EnterHardCurrentFault(void);
static MOTOR_FAST_CODE uint8_t MOTOR_ServiceHardCurrentProtection(void);
static void MOTOR_ResetBemfValidation(void);
static void MOTOR_UpdateBemfMeasuredErpm(void);
static void MOTOR_UpdateStatusLed(void);

static uint32_t MOTOR_GetAbsRpm(int32_t rpm)
{
    if (rpm < 0)
    {
        return (uint32_t)(-rpm);
    }

    return (uint32_t)rpm;
}

static uint16_t MOTOR_GetPolePairs(void)
{
    if (g_driver_config.pole_pairs > 0u)
    {
        return g_driver_config.pole_pairs;
    }

    return 1u;
}

static uint16_t MOTOR_GetSinusSample(uint16_t index)
{
    return g_sinus_table[index % DRIVER_SIN_TABLE_STEPS];
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfMonitorOnlyMode(void)
{
#if DRIVER_SIXSTEP_BEMF_MONITOR_ONLY
    return 1u;
#else
    return 0u;
#endif
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfPwmGatingEnabled(void)
{
    if (DRIVER_BEMF_PWM_GATING_MODE == DRIVER_BEMF_PWM_GATING_NONE)
    {
        return 0u;
    }

    return 1u;
}

static uint16_t MOTOR_GetPwmTicksFromPermille(uint32_t duty_permille)
{
    if (duty_permille > 1000u)
    {
        duty_permille = 1000u;
    }

    return (uint16_t)(((uint32_t)BOARD_GetPwmPeriodTicks() * duty_permille) / 1000u);
}

static void MOTOR_UpdateSinusPwmMax(void)
{
    g_motor.sinus_pwm_max_ticks =
        MOTOR_GetPwmTicksFromPermille(DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE);

    if (g_motor.sinus_pwm_ticks > g_motor.sinus_pwm_max_ticks)
    {
        g_motor.sinus_pwm_ticks = g_motor.sinus_pwm_max_ticks;
    }
}

static uint16_t MOTOR_GetCurrentAdcSignal(uint16_t adc_raw)
{
    if (DRIVER_CURRENT_ADC_ZERO_OFFSET > DRIVER_CURRENT_ADC_MAX_VALUE)
    {
        return 0u;
    }

    if (adc_raw <= DRIVER_CURRENT_ADC_ZERO_OFFSET)
    {
        return 0u;
    }

    return (uint16_t)(adc_raw - DRIVER_CURRENT_ADC_ZERO_OFFSET);
}

static uint16_t MOTOR_GetCurrentAdcFromMilliAmps(uint32_t current_ma)
{
    const uint64_t denominator =
        (uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MA *
        DRIVER_CURRENT_ADC_REFERENCE_MV;
    uint64_t current_adc;

    if (denominator == 0u)
    {
        return 0u;
    }

    current_adc = ((uint64_t)current_ma *
                  DRIVER_CURRENT_ADC_FULL_SCALE_MV *
                  DRIVER_CURRENT_ADC_MAX_VALUE) +
                 (denominator >> 1u);
    current_adc /= denominator;

    if (current_adc > DRIVER_CURRENT_ADC_MAX_VALUE)
    {
        return (uint16_t)DRIVER_CURRENT_ADC_MAX_VALUE;
    }

    return (uint16_t)current_adc;
}

static uint32_t MOTOR_GetCurrentMilliAmpsFromAdc(uint16_t current_adc_signal)
{
    return (uint32_t)((((uint64_t)current_adc_signal *
                        DRIVER_CURRENT_ADC_MA_PER_ADC_Q16) + 32768u) >> 16u);
}

static void MOTOR_UpdateCurrentMeasurementFromAdc(void)
{
    g_motor.current_adc_raw = BOARD_GetCurrentAdcRaw();
    g_motor.current_adc_filtered = BOARD_GetCurrentAdcFiltered();
    g_motor.current_adc_signal =
        MOTOR_GetCurrentAdcSignal(g_motor.current_adc_filtered);
    g_motor.measured_current_ma =
        MOTOR_GetCurrentMilliAmpsFromAdc(g_motor.current_adc_signal);
    g_motor.sin_current_error_adc =
        (int32_t)g_motor.sin_current_target_adc -
        (int32_t)g_motor.current_adc_signal;
}

static void MOTOR_UpdateStatusLed(void)
{
    if (g_motor.hard_current_fault != 0u)
    {
        BOARD_SetStatusLed(BOARD_STATUS_LED_RED);
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SINUS)
    {
        BOARD_SetStatusLed(BOARD_STATUS_LED_GREEN);
        return;
    }

    if ((g_motor.mode == MOTOR_MODE_SIXSTEP) &&
        (g_motor.sixstep_bemf_closed_loop != 0u))
    {
        BOARD_SetStatusLed(BOARD_STATUS_LED_GREEN);
    }
}

static MOTOR_FAST_CODE void MOTOR_EnterHardCurrentFault(void)
{
    MOTOR_ResetBemfValidation();
    g_motor.hard_current_fault = 1u;
    g_motor.hard_current_fault_count++;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_CURRENT_FAULT;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    g_motor.sixstep_pwm_ticks = 0u;
    g_motor.sixstep_speed_pid_target_permille = 0u;
    g_motor.sixstep_speed_pid_output_permille = 0u;
    g_motor.sixstep_speed_pid_output_q16 = 0u;
    g_motor.sixstep_speed_pid_rise_step_q16 = 0u;
    g_motor.sixstep_speed_pid_fall_step_q16 = 0u;
    g_motor.sixstep_final_pwm_permille = 0u;

    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_StopBemfPwmGating();
    BOARD_DisableBemfCommutationTimer();
    BOARD_ResetBemfIntervalTimer();
    BOARD_AllPhasesOff();
    BOARD_SetStatusLed(BOARD_STATUS_LED_RED);
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceHardCurrentProtection(void)
{
    g_motor.hard_current_limit_ma = DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA;
    g_motor.hard_current_adc_threshold =
        MOTOR_GetCurrentAdcFromMilliAmps(g_motor.hard_current_limit_ma);

    if (g_motor.hard_current_fault != 0u)
    {
        BOARD_AllPhasesOff();
        BOARD_SetStatusLed(BOARD_STATUS_LED_RED);
        return 1u;
    }

    if (g_motor.hard_current_limit_ma == 0u)
    {
        return 0u;
    }

    if (g_motor.measured_current_ma > g_motor.hard_current_limit_ma)
    {
        MOTOR_EnterHardCurrentFault();
        return 1u;
    }

    return 0u;
}

static int32_t MOTOR_LimitSinusCurrentOutputPermille(int32_t output_permille)
{
    int32_t min_permille = s_sin_current_pid.config.out_min;
    int32_t max_permille = s_sin_current_pid.config.out_max;
    const int32_t duty_limit = (DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE > 1000u) ?
                               1000 : (int32_t)DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE;

    if (min_permille < 0)
    {
        min_permille = 0;
    }

    if ((max_permille <= 0) || (max_permille > duty_limit))
    {
        max_permille = duty_limit;
    }

    if (min_permille > max_permille)
    {
        min_permille = max_permille;
    }

    if (output_permille > max_permille)
    {
        return max_permille;
    }

    if (output_permille < min_permille)
    {
        return min_permille;
    }

    return output_permille;
}

static int32_t MOTOR_UpdateSinusCurrentPiPermille(int32_t current_error_adc)
{
    int64_t next_integrator = (int64_t)s_sin_current_pid.integrator +
                              current_error_adc;
    int64_t output_q16;
    int64_t output_permille_64;
    int32_t output_permille;
    int32_t limited_permille;

    if (next_integrator > INT32_MAX)
    {
        next_integrator = INT32_MAX;
    }

    if (next_integrator < INT32_MIN)
    {
        next_integrator = INT32_MIN;
    }

    output_q16 = ((int64_t)s_sin_current_pid.config.kp_q16 * current_error_adc);
    output_q16 += ((int64_t)s_sin_current_pid.config.ki_q16 * (int32_t)next_integrator);
    output_permille_64 = output_q16 >> 16;

    if (output_permille_64 > INT32_MAX)
    {
        output_permille = INT32_MAX;
    }
    else if (output_permille_64 < INT32_MIN)
    {
        output_permille = INT32_MIN;
    }
    else
    {
        output_permille = (int32_t)output_permille_64;
    }

    limited_permille = MOTOR_LimitSinusCurrentOutputPermille(output_permille);

    if ((output_permille == limited_permille) ||
        ((output_permille > limited_permille) && (current_error_adc < 0)) ||
        ((output_permille < limited_permille) && (current_error_adc > 0)))
    {
        s_sin_current_pid.integrator = (int32_t)next_integrator;
    }

    s_sin_current_pid.last_error = current_error_adc;

    return limited_permille;
}

static void MOTOR_MoveSinusPwmToward(uint16_t target_ticks)
{
    uint16_t step_ticks = DRIVER_SIN_CURRENT_MAX_PWM_STEP_TICKS;
    uint16_t actual_ticks = g_motor.sinus_pwm_ticks;

    if (step_ticks == 0u)
    {
        step_ticks = 1u;
    }

    if (target_ticks > g_motor.sinus_pwm_max_ticks)
    {
        target_ticks = g_motor.sinus_pwm_max_ticks;
    }

    if (actual_ticks < target_ticks)
    {
        const uint16_t delta = target_ticks - actual_ticks;
        actual_ticks += (delta > step_ticks) ? step_ticks : delta;
    }
    else if (actual_ticks > target_ticks)
    {
        const uint16_t delta = actual_ticks - target_ticks;
        actual_ticks -= (delta > step_ticks) ? step_ticks : delta;
    }

    g_motor.sinus_pwm_ticks = actual_ticks;
}

static void MOTOR_ResetSinusCurrentControl(void)
{
    pid_config_t config = {
        .kp_q16 = DRIVER_SIN_CURRENT_PID_KP_ADC_Q16,
        .ki_q16 = DRIVER_SIN_CURRENT_PID_KI_ADC_Q16,
        .kd_q16 = 0,
        .out_min = DRIVER_SIN_CURRENT_PID_OUT_MIN_PERMILLE,
        .out_max = DRIVER_SIN_CURRENT_PID_OUT_MAX_PERMILLE
    };

    BOARD_AllPhasesOff();

    if (config.out_min < 0)
    {
        config.out_min = 0;
    }

    if ((config.out_max <= 0) || (config.out_max > 1000))
    {
        config.out_max = 1000;
    }

    config.kd_q16 = 0;
    PID_Init(&s_sin_current_pid, &config);

    g_motor.sin_current_target_ma = DRIVER_SIN_CURRENT_TARGET_MA;
    g_motor.sin_current_target_adc =
        MOTOR_GetCurrentAdcFromMilliAmps(g_motor.sin_current_target_ma);
    g_motor.sin_current_control_tick = 0u;
    g_motor.sinus_pwm_ticks = 0u;
    g_motor.sin_current_pi_output_permille = 0u;
    g_motor.sin_current_target_pwm_ticks = 0u;
    g_motor.sin_current_error_adc = 0;
    MOTOR_UpdateSinusPwmMax();

    MOTOR_UpdateCurrentMeasurementFromAdc();
}

static void MOTOR_ServiceSinusCurrentControl(void)
{
    uint16_t divider_ticks = DRIVER_SIN_CURRENT_CONTROL_DIVIDER_TICKS;
    int32_t target_permille;
    uint16_t target_ticks;

    if (divider_ticks == 0u)
    {
        divider_ticks = 1u;
    }

    if (g_motor.sin_current_control_tick < 0xFFFFu)
    {
        g_motor.sin_current_control_tick++;
    }

    if (g_motor.sin_current_control_tick < divider_ticks)
    {
        return;
    }

    g_motor.sin_current_control_tick = 0u;
    g_motor.sin_current_update_count++;
    MOTOR_UpdateCurrentMeasurementFromAdc();

    target_permille =
        MOTOR_UpdateSinusCurrentPiPermille(g_motor.sin_current_error_adc);
    target_ticks = MOTOR_GetPwmTicksFromPermille((uint32_t)target_permille);
    g_motor.sin_current_pi_output_permille = (uint16_t)target_permille;
    g_motor.sin_current_target_pwm_ticks = target_ticks;
    MOTOR_MoveSinusPwmToward(target_ticks);
}

static int32_t MOTOR_UpdatePidByError(pid_state_t *pid, int32_t error)
{
    int64_t next_integrator = (int64_t)pid->integrator + error;
    int64_t output_q16;
    int64_t output_64;
    int32_t output;
    int32_t limited_output;
    const int32_t derivative = error - pid->last_error;

    if (next_integrator > INT32_MAX)
    {
        next_integrator = INT32_MAX;
    }

    if (next_integrator < INT32_MIN)
    {
        next_integrator = INT32_MIN;
    }

    output_q16 = ((int64_t)pid->config.kp_q16 * error);
    output_q16 += ((int64_t)pid->config.ki_q16 * (int32_t)next_integrator);
    output_q16 += ((int64_t)pid->config.kd_q16 * derivative);
    output_64 = output_q16 >> 16;

    if (output_64 > INT32_MAX)
    {
        output = INT32_MAX;
    }
    else if (output_64 < INT32_MIN)
    {
        output = INT32_MIN;
    }
    else
    {
        output = (int32_t)output_64;
    }

    limited_output = output;

    if (limited_output > pid->config.out_max)
    {
        limited_output = pid->config.out_max;
    }

    if (limited_output < pid->config.out_min)
    {
        limited_output = pid->config.out_min;
    }

    if ((output == limited_output) ||
        ((output > limited_output) && (error < 0)) ||
        ((output < limited_output) && (error > 0)))
    {
        pid->integrator = (int32_t)next_integrator;
    }

    pid->last_error = error;

    return limited_output;
}

static uint16_t MOTOR_GetSixStepMaxDutyPermille(void)
{
    uint32_t max_permille = DRIVER_SIXSTEP_MAX_DUTY_PERMILLE;

    if (max_permille > 1000u)
    {
        max_permille = 1000u;
    }

    return (uint16_t)max_permille;
}

static uint16_t MOTOR_LimitSixStepDutyPermille(uint32_t duty_permille)
{
    const uint16_t max_permille = MOTOR_GetSixStepMaxDutyPermille();

    if (duty_permille > max_permille)
    {
        return max_permille;
    }

    return (uint16_t)duty_permille;
}

static uint16_t MOTOR_GetSixStepStartDutyPermille(void)
{
    return MOTOR_LimitSixStepDutyPermille(DRIVER_SIXSTEP_RPM_PID_START_DUTY_PERMILLE);
}

static uint32_t MOTOR_GetSixStepRpmPidSlewStepQ16(uint32_t slew_permille_per_sec,
                                                  uint16_t divider_ticks)
{
    uint64_t step_q16;

    if (slew_permille_per_sec == 0u)
    {
        return 0u;
    }

    if (divider_ticks == 0u)
    {
        divider_ticks = 1u;
    }

    step_q16 =
        ((((uint64_t)slew_permille_per_sec * divider_ticks) << 16u) +
         (DRIVER_CONTROL_LOOP_HZ - 1u)) / DRIVER_CONTROL_LOOP_HZ;

    if (step_q16 == 0u)
    {
        step_q16 = 1u;
    }

    if (step_q16 > (1000u << 16u))
    {
        return (1000u << 16u);
    }

    return (uint32_t)step_q16;
}

static uint16_t MOTOR_MoveSixStepRpmPidOutputToward(uint16_t target_permille,
                                                    uint32_t rise_step_q16,
                                                    uint32_t fall_step_q16)
{
    uint32_t actual_q16 = g_motor.sixstep_speed_pid_output_q16;
    uint32_t target_q16;

    target_permille = MOTOR_LimitSixStepDutyPermille(target_permille);
    target_q16 = (uint32_t)target_permille << 16u;

    if (actual_q16 > (1000u << 16u))
    {
        actual_q16 = 1000u << 16u;
    }

    if (actual_q16 < target_q16)
    {
        const uint32_t delta_q16 = target_q16 - actual_q16;

        if ((rise_step_q16 == 0u) || (delta_q16 <= rise_step_q16))
        {
            actual_q16 = target_q16;
        }
        else
        {
            actual_q16 += rise_step_q16;
        }
    }
    else if (actual_q16 > target_q16)
    {
        const uint32_t delta_q16 = actual_q16 - target_q16;

        if ((fall_step_q16 == 0u) || (delta_q16 <= fall_step_q16))
        {
            actual_q16 = target_q16;
        }
        else
        {
            actual_q16 -= fall_step_q16;
        }
    }

    g_motor.sixstep_speed_pid_output_q16 = actual_q16;

    return MOTOR_LimitSixStepDutyPermille((actual_q16 + 32768u) >> 16u);
}

static uint16_t MOTOR_GetSixStepPwmTicksFromPermille(uint16_t duty_permille)
{
    uint16_t pwm_ticks =
        MOTOR_GetPwmTicksFromPermille(MOTOR_LimitSixStepDutyPermille(duty_permille));

    if (pwm_ticks > g_motor.sixstep_pwm_limit_ticks)
    {
        pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
    }

    return pwm_ticks;
}

static int32_t MOTOR_GetSixStepMeasuredMechanicalRpm(void)
{
    const uint16_t pole_pairs = MOTOR_GetPolePairs();

    if (pole_pairs == 0u)
    {
        return g_motor.measured_erpm;
    }

    return g_motor.measured_erpm / (int32_t)pole_pairs;
}

static void MOTOR_ResetSixStepClosedLoopControl(void)
{
    const uint16_t start_permille = MOTOR_GetSixStepStartDutyPermille();
    const uint16_t max_permille = MOTOR_GetSixStepMaxDutyPermille();
    pid_config_t rpm_config = {
        .kp_q16 = DRIVER_SIXSTEP_RPM_PID_KP_Q16,
        .ki_q16 = DRIVER_SIXSTEP_RPM_PID_KI_Q16,
        .kd_q16 = DRIVER_SIXSTEP_RPM_PID_KD_Q16,
        .out_min = -(int32_t)start_permille,
        .out_max = (int32_t)max_permille - (int32_t)start_permille
    };
    pid_config_t current_limit_config = {
        .kp_q16 = DRIVER_SIXSTEP_CURRENT_LIMIT_PID_KP_Q16,
        .ki_q16 = DRIVER_SIXSTEP_CURRENT_LIMIT_PID_KI_Q16,
        .kd_q16 = 0,
        .out_min = 0,
        .out_max = max_permille
    };

    PID_Init(&s_sixstep_rpm_pid, &rpm_config);
    PID_Init(&s_sixstep_current_limit_pid, &current_limit_config);

    g_motor.sixstep_target_rpm = DRIVER_SIXSTEP_TARGET_RPM;
    g_motor.sixstep_measured_rpm = MOTOR_GetSixStepMeasuredMechanicalRpm();
    g_motor.sixstep_rpm_error =
        g_motor.sixstep_target_rpm - g_motor.sixstep_measured_rpm;
    g_motor.sixstep_current_limit_ma = DRIVER_SIXSTEP_CURRENT_LIMIT_MA;
    g_motor.hard_current_limit_ma = DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA;
    g_motor.hard_current_adc_threshold = MOTOR_GetCurrentAdcFromMilliAmps(g_motor.hard_current_limit_ma);
    g_motor.sixstep_current_error_ma = 0;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_current_limit_control_tick = 0u;
    g_motor.sixstep_speed_update_count = 0u;
    g_motor.sixstep_current_limit_update_count = 0u;
    g_motor.sixstep_speed_pid_target_permille = start_permille;
    g_motor.sixstep_speed_pid_output_permille = start_permille;
    g_motor.sixstep_speed_pid_output_q16 = (uint32_t)start_permille << 16u;
    g_motor.sixstep_speed_pid_rise_step_q16 = 0u;
    g_motor.sixstep_speed_pid_fall_step_q16 = 0u;
    g_motor.sixstep_current_limit_reduction_permille = 0u;
    g_motor.sixstep_final_pwm_permille = start_permille;
    g_motor.sixstep_pwm_ticks =
        MOTOR_GetSixStepPwmTicksFromPermille(g_motor.sixstep_final_pwm_permille);
}

static uint8_t MOTOR_ServiceSixStepRpmPid(void)
{
    uint16_t divider_ticks = DRIVER_SIXSTEP_RPM_PID_UPDATE_DIVIDER_TICKS;
    const uint16_t start_permille = MOTOR_GetSixStepStartDutyPermille();
    uint16_t target_permille;
    int32_t pid_delta_permille;
    int32_t requested_permille;

    if (divider_ticks == 0u)
    {
        divider_ticks = 1u;
    }

    if (g_motor.sixstep_speed_control_tick < 0xFFFFu)
    {
        g_motor.sixstep_speed_control_tick++;
    }

    if (g_motor.sixstep_speed_control_tick < divider_ticks)
    {
        return 0u;
    }

    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_speed_update_count++;
    MOTOR_UpdateBemfMeasuredErpm();
    g_motor.sixstep_target_rpm = DRIVER_SIXSTEP_TARGET_RPM;
    g_motor.sixstep_measured_rpm = MOTOR_GetSixStepMeasuredMechanicalRpm();
    g_motor.sixstep_rpm_error =
        g_motor.sixstep_target_rpm - g_motor.sixstep_measured_rpm;

    pid_delta_permille =
        MOTOR_UpdatePidByError(&s_sixstep_rpm_pid, g_motor.sixstep_rpm_error);
    requested_permille = (int32_t)start_permille + pid_delta_permille;

    if (requested_permille < 0)
    {
        requested_permille = 0;
    }

    target_permille =
        MOTOR_LimitSixStepDutyPermille((uint32_t)requested_permille);

    g_motor.sixstep_speed_pid_target_permille = target_permille;
    g_motor.sixstep_speed_pid_rise_step_q16 =
        MOTOR_GetSixStepRpmPidSlewStepQ16(
            DRIVER_SIXSTEP_RPM_PID_MAX_RISE_PER_SEC_PERMILLE,
            divider_ticks);
    g_motor.sixstep_speed_pid_fall_step_q16 =
        MOTOR_GetSixStepRpmPidSlewStepQ16(
            DRIVER_SIXSTEP_RPM_PID_MAX_FALL_PER_SEC_PERMILLE,
            divider_ticks);
    g_motor.sixstep_speed_pid_output_permille =
        MOTOR_MoveSixStepRpmPidOutputToward(
            target_permille,
            g_motor.sixstep_speed_pid_rise_step_q16,
            g_motor.sixstep_speed_pid_fall_step_q16);

    if (MOTOR_GetAbsRpm(g_motor.sixstep_rpm_error) <= g_driver_config.in_rpm_window)
    {
        g_motor.in_rpm = 1u;
    }
    else
    {
        g_motor.in_rpm = 0u;
    }

    return 1u;
}

static uint8_t MOTOR_ServiceSixStepCurrentLimiter(void)
{
    uint16_t divider_ticks = DRIVER_SIXSTEP_CURRENT_LIMIT_UPDATE_DIVIDER_TICKS;
    uint16_t reduction_permille;

    if (divider_ticks == 0u)
    {
        divider_ticks = 1u;
    }

    if (g_motor.sixstep_current_limit_control_tick < 0xFFFFu)
    {
        g_motor.sixstep_current_limit_control_tick++;
    }

    if (g_motor.sixstep_current_limit_control_tick < divider_ticks)
    {
        return 0u;
    }

    g_motor.sixstep_current_limit_control_tick = 0u;
    g_motor.sixstep_current_limit_update_count++;
    MOTOR_UpdateCurrentMeasurementFromAdc();
    g_motor.sixstep_current_limit_ma = DRIVER_SIXSTEP_CURRENT_LIMIT_MA;
    g_motor.sixstep_current_error_ma =
        (int32_t)g_motor.measured_current_ma -
        (int32_t)g_motor.sixstep_current_limit_ma;

    if (g_motor.sixstep_current_error_ma <= 0)
    {
        uint16_t release_step = DRIVER_SIXSTEP_CURRENT_LIMIT_RELEASE_STEP_PERMILLE;

        PID_Reset(&s_sixstep_current_limit_pid);

        if (release_step == 0u)
        {
            release_step = 1u;
        }

        if (g_motor.sixstep_current_limit_reduction_permille > release_step)
        {
            g_motor.sixstep_current_limit_reduction_permille =
                (uint16_t)(g_motor.sixstep_current_limit_reduction_permille -
                           release_step);
        }
        else
        {
            g_motor.sixstep_current_limit_reduction_permille = 0u;
        }

        return 1u;
    }

    reduction_permille =
        (uint16_t)MOTOR_UpdatePidByError(&s_sixstep_current_limit_pid,
                                         g_motor.sixstep_current_error_ma);
    g_motor.sixstep_current_limit_reduction_permille =
        MOTOR_LimitSixStepDutyPermille(reduction_permille);

    return 1u;
}

static void MOTOR_ApplySixStepClosedLoopPwmControl(void)
{
    uint16_t final_permille;
    uint16_t pwm_ticks;

    if (g_motor.sixstep_current_limit_reduction_permille >=
        g_motor.sixstep_speed_pid_output_permille)
    {
        final_permille = 0u;
    }
    else
    {
        final_permille =
            (uint16_t)(g_motor.sixstep_speed_pid_output_permille -
                       g_motor.sixstep_current_limit_reduction_permille);
    }

    final_permille = MOTOR_LimitSixStepDutyPermille(final_permille);
    pwm_ticks = MOTOR_GetSixStepPwmTicksFromPermille(final_permille);

    g_motor.sixstep_final_pwm_permille = final_permille;

    if (pwm_ticks == g_motor.sixstep_pwm_ticks)
    {
        return;
    }

    g_motor.sixstep_pwm_ticks = pwm_ticks;
    MOTOR_ApplySixStepBridge();
}

static void MOTOR_ServiceSixStepClosedLoopPwmControl(void)
{
    (void)MOTOR_ServiceSixStepRpmPid();
    (void)MOTOR_ServiceSixStepCurrentLimiter();

    if (g_motor.sixstep_commutation_pending != 0u)
    {
        return;
    }

    MOTOR_ApplySixStepClosedLoopPwmControl();
}

static uint16_t MOTOR_ScaleSinusDuty(uint16_t sample)
{
    return (uint16_t)(((uint32_t)sample * g_motor.sinus_pwm_ticks) /
                      DRIVER_SIN_TABLE_MAX_VALUE);
}

static void MOTOR_UpdateSixStepPwmLimit(void)
{
    g_motor.sixstep_pwm_limit_ticks =
        MOTOR_GetPwmTicksFromPermille(DRIVER_SIXSTEP_MAX_DUTY_PERMILLE);
}

static uint16_t MOTOR_GetSinToSixStepPwmTicks(void)
{
    uint32_t pwm_ticks =
        ((uint32_t)g_motor.sinus_pwm_ticks *
         DRIVER_SIN_TO_6STEP_PWM_SCALE_PERMILLE) / 1000u;

    if (pwm_ticks > g_motor.sixstep_pwm_limit_ticks)
    {
        pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
    }

    return (uint16_t)pwm_ticks;
}

static uint16_t MOTOR_RescalePwmTicks(uint16_t ticks, uint16_t old_period, uint16_t new_period)
{
    uint32_t scaled_ticks;

    if (old_period == 0u)
    {
        return ticks;
    }

    scaled_ticks = (((uint32_t)ticks * new_period) + (old_period >> 1u)) / old_period;

    if (scaled_ticks > new_period)
    {
        scaled_ticks = new_period;
    }

    return (uint16_t)scaled_ticks;
}

static uint32_t MOTOR_ClampPwmCarrierHz(uint32_t carrier_hz)
{
    if (carrier_hz < DRIVER_PWM_6STEP_MIN_CARRIER_HZ)
    {
        return DRIVER_PWM_6STEP_MIN_CARRIER_HZ;
    }

    if (carrier_hz > DRIVER_PWM_6STEP_MAX_CARRIER_HZ)
    {
        return DRIVER_PWM_6STEP_MAX_CARRIER_HZ;
    }

    return carrier_hz;
}

static void MOTOR_UpdatePwmCarrierDebug(uint16_t sector_ticks)
{
    uint32_t pulses;

    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();

    if (sector_ticks == 0u)
    {
        g_motor.pwm_pulses_per_sector = 0u;
        return;
    }

    pulses = (((uint32_t)g_motor.pwm_carrier_hz * sector_ticks) +
              (MOTOR_BEMF_TIMER_HZ / 2u)) / MOTOR_BEMF_TIMER_HZ;

    if (pulses > MOTOR_PWM_PULSES_MAX_DISPLAY)
    {
        pulses = MOTOR_PWM_PULSES_MAX_DISPLAY;
    }

    g_motor.pwm_pulses_per_sector = (uint8_t)pulses;
}

static void MOTOR_RecalculateSixStepPwmAfterCarrierChange(uint16_t old_period, uint16_t new_period)
{
    g_motor.sixstep_pwm_ticks =
        MOTOR_RescalePwmTicks(g_motor.sixstep_pwm_ticks, old_period, new_period);
    MOTOR_UpdateSixStepPwmLimit();

    if (g_motor.sixstep_pwm_ticks > g_motor.sixstep_pwm_limit_ticks)
    {
        g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
    }
}

static void MOTOR_SetPwmCarrierHz(uint32_t carrier_hz)
{
    const uint16_t old_period = BOARD_GetPwmPeriodTicks();
    uint16_t new_period;

    carrier_hz = MOTOR_ClampPwmCarrierHz(carrier_hz);
    new_period = BOARD_SetPwmCarrierHz(carrier_hz);
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();

    if (new_period == old_period)
    {
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        MOTOR_RecalculateSixStepPwmAfterCarrierChange(old_period, new_period);

        if ((g_motor.sixstep_step != 0u) &&
            (g_motor.sixstep_step <= MOTOR_SIXSTEP_STEP_COUNT))
        {
            MOTOR_ApplySixStepBridge();
        }

        return;
    }

    MOTOR_UpdateSinusPwmMax();
    MOTOR_UpdateSixStepPwmLimit();
}

static MOTOR_FAST_CODE void MOTOR_UpdateAdaptiveSixStepPwmCarrier(void)
{
#if DRIVER_PWM_DYNAMIC_6STEP_ENABLE
    const uint16_t sector_ticks = g_motor.bemf_average_interval_ticks;
    const uint32_t target_pulses =
        (DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR == 0u) ?
        1u : DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR;
    uint32_t carrier_hz;

    if (sector_ticks == 0u)
    {
        return;
    }

    carrier_hz = (((uint32_t)MOTOR_BEMF_TIMER_HZ * target_pulses) +
                  (sector_ticks >> 1u)) / sector_ticks;

    MOTOR_SetPwmCarrierHz(carrier_hz);
    MOTOR_UpdatePwmCarrierDebug(sector_ticks);
#else
    MOTOR_UpdatePwmCarrierDebug(g_motor.bemf_average_interval_ticks);
#endif
}

static void MOTOR_RestoreDefaultPwmCarrier(void)
{
    BOARD_SetPwmCarrierHz(DRIVER_PWM_CARRIER_HZ);
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.pwm_pulses_per_sector = 0u;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetPwmDutyPermilleFromTicks(uint16_t duty_ticks)
{
    const uint16_t period_ticks = BOARD_GetPwmPeriodTicks();

    if (period_ticks == 0u)
    {
        return 0u;
    }

    if (duty_ticks >= period_ticks)
    {
        return 1000u;
    }

    return (uint16_t)((((uint32_t)duty_ticks * 1000u) +
                       (period_ticks >> 1u)) / period_ticks);
}

static MOTOR_FAST_CODE uint8_t MOTOR_UpdateBemfPwmGatingActiveWindow(uint16_t duty_ticks)
{
    const uint8_t mode = DRIVER_BEMF_PWM_GATING_MODE;
    uint16_t duty_permille;
    uint16_t on_enter_permille = DRIVER_BEMF_PWM_SAMPLE_ON_ENTER_PERMILLE;
    uint16_t on_exit_permille = DRIVER_BEMF_PWM_SAMPLE_ON_EXIT_PERMILLE;

    if (mode == DRIVER_BEMF_PWM_GATING_OFF_TIME)
    {
        g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
        return g_motor.bemf_pwm_gating_active_window;
    }

    if (mode == DRIVER_BEMF_PWM_GATING_ON_TIME)
    {
        g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_ON_TIME;
        return g_motor.bemf_pwm_gating_active_window;
    }

    if (on_enter_permille > 1000u)
    {
        on_enter_permille = 1000u;
    }

    if (on_exit_permille > on_enter_permille)
    {
        on_exit_permille = on_enter_permille;
    }

    duty_permille = MOTOR_GetPwmDutyPermilleFromTicks(duty_ticks);

    if (g_motor.bemf_pwm_gating_active_window == MOTOR_BEMF_PWM_SAMPLE_ON_TIME)
    {
        if (duty_permille <= on_exit_permille)
        {
            g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
        }

        return g_motor.bemf_pwm_gating_active_window;
    }

    if (duty_permille >= on_enter_permille)
    {
        g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_ON_TIME;
    }
    else
    {
        g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
    }

    return g_motor.bemf_pwm_gating_active_window;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfPwmGatingTicksForWindow(uint8_t active_window,
                                                                     uint16_t duty_ticks,
                                                                     uint16_t *open_ticks,
                                                                     uint16_t *close_ticks)
{
    const uint16_t period_ticks = BOARD_GetPwmPeriodTicks();
    uint16_t edge_settle_ticks = DRIVER_BEMF_PWM_SAMPLE_EDGE_SETTLE_TICKS;
    uint16_t close_margin_ticks = DRIVER_BEMF_PWM_SAMPLE_CLOSE_MARGIN_TICKS;
    uint32_t open_tick;
    uint32_t close_tick;

    if ((open_ticks == 0) || (close_ticks == 0) || (period_ticks <= 4u))
    {
        return 0u;
    }

    if (duty_ticks > period_ticks)
    {
        duty_ticks = period_ticks;
    }

    if (edge_settle_ticks >= period_ticks)
    {
        edge_settle_ticks = (uint16_t)(period_ticks - 1u);
    }

    if (close_margin_ticks >= period_ticks)
    {
        close_margin_ticks = (uint16_t)(period_ticks - 1u);
    }

    if (active_window == MOTOR_BEMF_PWM_SAMPLE_ON_TIME)
    {
        open_tick = edge_settle_ticks;

        if (duty_ticks <= close_margin_ticks)
        {
            return 0u;
        }

        close_tick = (uint32_t)duty_ticks - close_margin_ticks;
    }
    else
    {
        open_tick = (uint32_t)duty_ticks + edge_settle_ticks;

        if (period_ticks <= close_margin_ticks)
        {
            return 0u;
        }

        close_tick = (uint32_t)period_ticks - close_margin_ticks;
    }

    if (open_tick == 0u)
    {
        open_tick = 1u;
    }

    if (close_tick >= period_ticks)
    {
        close_tick = (uint32_t)period_ticks - 1u;
    }

    if ((open_tick >= period_ticks) || (close_tick <= open_tick))
    {
        return 0u;
    }

    *open_ticks = (uint16_t)open_tick;
    *close_ticks = (uint16_t)close_tick;

    return 1u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_PrepareBemfPwmGatingTicks(uint16_t duty_ticks)
{
    const uint8_t mode = DRIVER_BEMF_PWM_GATING_MODE;
    uint16_t open_ticks;
    uint16_t close_ticks;
    uint8_t active_window;
    uint8_t alternate_window;

    if (MOTOR_isBemfPwmGatingEnabled() == 0u)
    {
        return 0u;
    }

    active_window = MOTOR_UpdateBemfPwmGatingActiveWindow(duty_ticks);

    if (MOTOR_GetBemfPwmGatingTicksForWindow(active_window,
                                             duty_ticks,
                                             &open_ticks,
                                             &close_ticks) != 0u)
    {
        g_motor.bemf_pwm_gating_open_ticks = open_ticks;
        g_motor.bemf_pwm_gating_close_ticks = close_ticks;
        return 1u;
    }

    if (mode != DRIVER_BEMF_PWM_GATING_AUTO)
    {
        return 0u;
    }

    alternate_window =
        (active_window == MOTOR_BEMF_PWM_SAMPLE_ON_TIME) ?
        MOTOR_BEMF_PWM_SAMPLE_OFF_TIME : MOTOR_BEMF_PWM_SAMPLE_ON_TIME;

    if (MOTOR_GetBemfPwmGatingTicksForWindow(alternate_window,
                                             duty_ticks,
                                             &open_ticks,
                                             &close_ticks) == 0u)
    {
        return 0u;
    }

    g_motor.bemf_pwm_gating_active_window = alternate_window;
    g_motor.bemf_pwm_gating_open_ticks = open_ticks;
    g_motor.bemf_pwm_gating_close_ticks = close_ticks;

    return 1u;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfPwmGatingOpenTicks(uint16_t duty_ticks)
{
    if (MOTOR_PrepareBemfPwmGatingTicks(duty_ticks) != 0u)
    {
        return g_motor.bemf_pwm_gating_open_ticks;
    }

    return 1u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfExpectedLevelBeforeZc(void)
{
    return MOTOR_GetBemfRisingForStep();
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfExpectedLevelAfterZc(void)
{
    if (MOTOR_GetBemfExpectedLevelBeforeZc() == 0u)
    {
        return 1u;
    }

    return 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfPwmSampleZeroCross(uint8_t sampled_level)
{
    uint16_t interval_ticks;
    uint16_t min_interval_ticks;

    if (g_motor.bemf_pwm_sample_valid == 0u)
    {
        g_motor.bemf_pwm_last_level = MOTOR_GetBemfExpectedLevelBeforeZc();
        g_motor.bemf_pwm_sample_valid = 1u;
    }

    if (sampled_level == g_motor.bemf_pwm_last_level)
    {
        return 0u;
    }

    if (sampled_level != MOTOR_GetBemfExpectedLevelAfterZc())
    {
        return 0u;
    }

    interval_ticks = BOARD_GetBemfIntervalTicks();
    min_interval_ticks = (uint16_t)(g_motor.bemf_average_interval_ticks >> 1u);

    if (interval_ticks <= min_interval_ticks)
    {
        return 0u;
    }

    g_motor.bemf_pwm_last_level = sampled_level;
    g_motor.bemf_interval_ticks = interval_ticks;

    return 1u;
}

static void MOTOR_UpdateSinusTiming(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint64_t numerator = (uint64_t)rpm * MOTOR_GetPolePairs() *
                               DRIVER_SIN_TABLE_STEPS * 65536u;
    const uint32_t denominator = 60u * DRIVER_CONTROL_LOOP_HZ;

    if (denominator == 0u)
    {
        g_motor.sinus_step_q16 = 0u;
        return;
    }

    g_motor.sinus_step_q16 = (uint32_t)(numerator / denominator);
}

static uint16_t MOTOR_GetSixStepIntervalTicks(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint32_t pole_pairs = MOTOR_GetPolePairs();
    const uint64_t denominator = (uint64_t)rpm * pole_pairs * MOTOR_SIXSTEP_STEP_COUNT;
    uint64_t ticks;

    if (denominator == 0u)
    {
        return 0u;
    }

    ticks = (((uint64_t)DRIVER_CONTROL_LOOP_HZ * 60u) + (denominator / 2u)) /
            denominator;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFu)
    {
        ticks = 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static uint16_t MOTOR_GetSixStepBemfIntervalTicks(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint32_t pole_pairs = MOTOR_GetPolePairs();
    const uint64_t denominator = (uint64_t)rpm * pole_pairs * MOTOR_SIXSTEP_STEP_COUNT;
    uint64_t ticks;

    if (denominator == 0u)
    {
        return MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    ticks = (((uint64_t)MOTOR_BEMF_TIMER_HZ * 60u) + (denominator / 2u)) /
            denominator;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFu)
    {
        ticks = 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static uint16_t MOTOR_LimitTickCount(uint32_t ticks)
{
    if (ticks == 0u)
    {
        return 1u;
    }

    if (ticks > 0xFFFFu)
    {
        return 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetControlTicksFromBemfInterval(uint16_t bemf_interval_ticks)
{
    uint32_t ticks;

    ticks = (((uint32_t)bemf_interval_ticks * DRIVER_CONTROL_LOOP_HZ) +
             (MOTOR_BEMF_TIMER_HZ / 2u)) / MOTOR_BEMF_TIMER_HZ;

    return MOTOR_LimitTickCount(ticks);
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfFallbackControlTicks(void)
{
    const uint16_t interval_ticks =
        MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);

    g_motor.sixstep_fallback_interval_ticks = interval_ticks;
}

static uint8_t MOTOR_MapFilterLevel(uint16_t interval_ticks)
{
    uint32_t filter_level;

    if (interval_ticks <= MOTOR_BEMF_FILTER_MIN_TICKS)
    {
        return 3u;
    }

    if (interval_ticks >= MOTOR_BEMF_FILTER_MAX_TICKS)
    {
        return 12u;
    }

    filter_level = 3u + ((((uint32_t)interval_ticks - MOTOR_BEMF_FILTER_MIN_TICKS) * 9u) /
                         (MOTOR_BEMF_FILTER_MAX_TICKS - MOTOR_BEMF_FILTER_MIN_TICKS));

    return (uint8_t)filter_level;
}

static void MOTOR_UpdateBemfFilterLevel(void)
{
    uint8_t filter_level;

    if (g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS)
    {
        filter_level = MOTOR_BEMF_ULTRA_FAST_FILTER_LEVEL;
    }
    else if (g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS)
    {
        filter_level = MOTOR_BEMF_FAST_FILTER_LEVEL;
    }
    else if ((g_motor.bemf_zero_cross_count < 100u) &&
        (g_motor.bemf_average_interval_ticks > MOTOR_BEMF_FILTER_MAX_TICKS))
    {
        filter_level = 12u;
    }
    else
    {
        filter_level = MOTOR_MapFilterLevel(g_motor.bemf_average_interval_ticks);
    }

    if (g_motor.bemf_average_interval_ticks < 50u)
    {
        filter_level = 2u;
    }

    BEMF_AM32_SetFilterLevel(filter_level);
}

static uint16_t MOTOR_GetBemfInitialIntervalTicks(uint16_t sector_ticks)
{
    if (sector_ticks == 0u)
    {
        return MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    return MOTOR_LimitTickCount(sector_ticks);
}

static void MOTOR_UpdateBemfMeasuredErpm(void)
{
    const uint16_t interval_ticks = g_motor.bemf_average_interval_ticks;
    uint32_t erpm;

    if (interval_ticks == 0u)
    {
        return;
    }

    erpm = MOTOR_BEMF_ERPM_NUMERATOR / interval_ticks;

    g_motor.measured_erpm = (int32_t)erpm;
}

static MOTOR_FAST_CODE int32_t MOTOR_GetBemfActionAngleOffsetDegX10(void)
{
    return g_driver_config.bemf_action_angle_offset_deg_x10;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfDelayTicks(uint16_t interval_ticks)
{
    int32_t angle_offset_deg_x10 = MOTOR_GetBemfActionAngleOffsetDegX10();
    int32_t delay_ticks;
    int32_t advance_ticks;
    uint32_t abs_angle_deg_x10;

    if (interval_ticks == 0u)
    {
        interval_ticks = g_motor.bemf_average_interval_ticks;
    }

    if (angle_offset_deg_x10 >= 300)
    {
        return 1u;
    }

    if (angle_offset_deg_x10 > 600)
    {
        angle_offset_deg_x10 = 600;
    }
    else if (angle_offset_deg_x10 < -600)
    {
        angle_offset_deg_x10 = -600;
    }

    if (angle_offset_deg_x10 < 0)
    {
        abs_angle_deg_x10 = (uint32_t)(-angle_offset_deg_x10);
    }
    else
    {
        abs_angle_deg_x10 = (uint32_t)angle_offset_deg_x10;
    }

    delay_ticks = (int32_t)(interval_ticks >> 1u);
    advance_ticks = (int32_t)((((uint32_t)interval_ticks * abs_angle_deg_x10 *
                                MOTOR_BEMF_ADVANCE_RECIP_Q16) + 32768u) >> 16u);

    if (angle_offset_deg_x10 < 0)
    {
        advance_ticks = -advance_ticks;
    }

    delay_ticks -= advance_ticks;

    if (delay_ticks < 1)
    {
        delay_ticks = 1;
    }

    if (delay_ticks > 0xFFFF)
    {
        delay_ticks = 0xFFFF;
    }

    return (uint16_t)delay_ticks;
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfWaitTime(void)
{
    g_motor.sixstep_bemf_delay_ticks =
        MOTOR_GetBemfDelayTicks(g_motor.bemf_average_interval_ticks);
}

static MOTOR_FAST_CODE int32_t MOTOR_GetBemfPhaseCorrectionTicks(uint16_t captured_interval_ticks)
{
    const uint16_t average_interval_ticks = g_motor.bemf_average_interval_ticks;
    uint8_t phase_shift = DRIVER_SIXSTEP_BEMF_PHASE_SYNC_SHIFT;
    uint32_t correction_limit_ticks;
    uint32_t correction_ticks;
    uint32_t abs_error_ticks;
    int32_t phase_error_ticks;

    phase_error_ticks = (int32_t)captured_interval_ticks -
                        (int32_t)average_interval_ticks;

    if (phase_error_ticks == 0)
    {
        return 0;
    }

    if (phase_shift > 15u)
    {
        phase_shift = 15u;
    }

    if (phase_error_ticks < 0)
    {
        abs_error_ticks = (uint32_t)(-phase_error_ticks);
    }
    else
    {
        abs_error_ticks = (uint32_t)phase_error_ticks;
    }

    correction_ticks = abs_error_ticks >> phase_shift;

    if (DRIVER_SIXSTEP_BEMF_PHASE_SYNC_LIMIT_DIV != 0u)
    {
        correction_limit_ticks =
            ((uint32_t)average_interval_ticks /
             DRIVER_SIXSTEP_BEMF_PHASE_SYNC_LIMIT_DIV);

        if (correction_limit_ticks == 0u)
        {
            correction_limit_ticks = 1u;
        }

        if (correction_ticks > correction_limit_ticks)
        {
            correction_ticks = correction_limit_ticks;
        }
    }

    if (phase_error_ticks < 0)
    {
        return -(int32_t)correction_ticks;
    }

    return (int32_t)correction_ticks;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfVirtualCommutationDelayTicks(uint16_t captured_interval_ticks)
{
    int32_t delay_ticks = (int32_t)MOTOR_GetBemfDelayTicks(g_motor.bemf_average_interval_ticks);

    delay_ticks += MOTOR_GetBemfPhaseCorrectionTicks(captured_interval_ticks);

    if (delay_ticks < 1)
    {
        return 1u;
    }

    if (delay_ticks > 0xFFFF)
    {
        return 0xFFFFu;
    }

    return (uint16_t)delay_ticks;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfMissingZcTimeoutTicks(void)
{
    uint32_t interval_ticks = g_motor.bemf_average_interval_ticks;
    uint32_t delay_ticks = g_motor.sixstep_bemf_delay_ticks;
    uint32_t timeout_ticks;

    if (interval_ticks == 0u)
    {
        interval_ticks = MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    if (delay_ticks == 0u)
    {
        delay_ticks = interval_ticks >> 1u;
    }

    timeout_ticks = interval_ticks + delay_ticks;

    return MOTOR_LimitTickCount(timeout_ticks);
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfDerivedTiming(void)
{
    uint32_t measured_interval = g_motor.bemf_this_zc_ticks;
    int32_t interval_error;
    uint32_t interval_step;

#if DRIVER_SIXSTEP_BEMF_PAIR_AVERAGE_ENABLE
    if (g_motor.bemf_last_zc_ticks != 0u)
    {
        measured_interval = ((uint32_t)g_motor.bemf_last_zc_ticks +
                             g_motor.bemf_this_zc_ticks) >> 1u;
    }
#endif

    if (measured_interval == 0u)
    {
        measured_interval = g_motor.bemf_average_interval_ticks;
    }

    interval_error = (int32_t)measured_interval -
                     (int32_t)g_motor.bemf_average_interval_ticks;

    if (interval_error >= 0)
    {
        interval_step = (uint32_t)interval_error >> MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT;

        if ((interval_step == 0u) && (interval_error != 0))
        {
            interval_step = 1u;
        }

        g_motor.bemf_average_interval_ticks =
            MOTOR_LimitTickCount((uint32_t)g_motor.bemf_average_interval_ticks +
                                 interval_step);
        return;
    }

    interval_error = -interval_error;
    interval_step = (uint32_t)interval_error >> MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT;

    if (interval_step == 0u)
    {
        interval_step = 1u;
    }

    if (interval_step >= g_motor.bemf_average_interval_ticks)
    {
        g_motor.bemf_average_interval_ticks = 1u;
        return;
    }

    g_motor.bemf_average_interval_ticks =
        (uint16_t)(g_motor.bemf_average_interval_ticks -
                   interval_step);
}

static void MOTOR_ServiceBemfSlowUpdate(void)
{
    if (g_motor.bemf_armed == 0u)
    {
        return;
    }

    if (g_motor.bemf_slow_update_tick < 0xFFu)
    {
        g_motor.bemf_slow_update_tick++;
    }

    if (g_motor.bemf_slow_update_tick < MOTOR_BEMF_SLOW_UPDATE_DIVIDER)
    {
        return;
    }

    g_motor.bemf_slow_update_tick = 0u;
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        MOTOR_UpdateBemfFallbackControlTicks();
        g_motor.sixstep_interval_ticks =
            MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
        MOTOR_UpdateAdaptiveSixStepPwmCarrier();
    }
}

static MOTOR_FAST_CODE void MOTOR_ScheduleBemfCommutation(void)
{
    uint16_t delay_ticks = g_motor.sixstep_bemf_delay_ticks;

    if (delay_ticks <= MOTOR_BEMF_DIRECT_COMMUTATION_TICKS)
    {
        MOTOR_CommitBemfCommutation();
        return;
    }

    if (delay_ticks < 0xFFFFu)
    {
        delay_ticks++;
    }

    g_motor.sixstep_commutation_pending = 1u;
    BOARD_ScheduleBemfCommutation(delay_ticks);
}

#if DRIVER_SIN_TO_6STEP_ENABLE
static uint32_t MOTOR_GetSinToSixStepSettleTicks(void)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_SINUS_RPM);
    const uint64_t numerator = (uint64_t)DRIVER_CONTROL_LOOP_HZ * 60u *
                               DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV;
    uint64_t ticks;

    if (rpm == 0u)
    {
        return 0xFFFFFFFFu;
    }

    ticks = (numerator + (rpm - 1u)) / rpm;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFFFFFu)
    {
        ticks = 0xFFFFFFFFu;
    }

    return (uint32_t)ticks;
}
#endif

static uint32_t MOTOR_GetAlignTicks(void)
{
    return (DRIVER_OPEN_LOOP_ALIGN_MS * DRIVER_CONTROL_LOOP_HZ) / 1000u;
}

static int32_t MOTOR_GetOpenLoopRampedRpm(uint32_t ramp_tick)
{
    const uint32_t target_rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_SINUS_RPM);
    const uint32_t start_rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_START_RPM);
    uint32_t rpm;

    if (target_rpm <= start_rpm)
    {
        return (int32_t)target_rpm;
    }

    rpm = start_rpm + ((ramp_tick * DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC) /
                       DRIVER_CONTROL_LOOP_HZ);

    if (rpm > target_rpm)
    {
        rpm = target_rpm;
    }

    return (int32_t)rpm;
}

static uint8_t MOTOR_GetSixStepFromAngle(uint32_t angle_q16)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    uint32_t sector;

    while (angle_q16 >= full_turn_q16)
    {
        angle_q16 -= full_turn_q16;
    }

    sector = (uint32_t)((((uint64_t)angle_q16 * MOTOR_SIXSTEP_STEP_COUNT) +
                         (full_turn_q16 / 2u)) / full_turn_q16);

    if (sector >= MOTOR_SIXSTEP_STEP_COUNT)
    {
        sector = 0u;
    }

    if (sector < 5u)
    {
        return (uint8_t)(5u - sector);
    }

    return 6u;
}

static void MOTOR_ApplySinusBridge(void)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    const uint32_t phase_shift_120_q16 = full_turn_q16 / 3u;
    const uint32_t phase_shift_240_q16 = (full_turn_q16 * 2u) / 3u;
    const uint16_t index_a = (uint16_t)(g_motor.sinus_angle_q16 >> 16u);
    const uint16_t index_b = (uint16_t)((g_motor.sinus_angle_q16 + phase_shift_120_q16) >> 16u);
    const uint16_t index_c = (uint16_t)((g_motor.sinus_angle_q16 + phase_shift_240_q16) >> 16u);
    const uint16_t sample_a = MOTOR_GetSinusSample(index_a);
    const uint16_t sample_b = MOTOR_GetSinusSample(index_b);
    const uint16_t sample_c = MOTOR_GetSinusSample(index_c);
    const uint16_t duty_a = MOTOR_ScaleSinusDuty(sample_a);
    const uint16_t duty_b = MOTOR_ScaleSinusDuty(sample_b);
    const uint16_t duty_c = MOTOR_ScaleSinusDuty(sample_c);

    BOARD_SetHighPwm(duty_a, duty_b, duty_c);
    BOARD_SetPwmOutputMask(1u, 1u, 1u, 1u, 1u, 1u);
    BOARD_SetPwmBridgeEnabled(1u);
}

static MOTOR_FAST_CODE void MOTOR_ApplySixStepBridge(void)
{
    const uint16_t duty = g_motor.sixstep_pwm_ticks;
    uint16_t bemf_pwm_open_ticks;

    if ((g_motor.sixstep_step == 0u) ||
        (g_motor.sixstep_step > MOTOR_SIXSTEP_STEP_COUNT))
    {
        BOARD_AllPhasesOff();
        return;
    }

    bemf_pwm_open_ticks = MOTOR_GetBemfPwmGatingOpenTicks(duty);

    BOARD_ApplySixStepBridge(g_motor.sixstep_step,
                             duty,
                             bemf_pwm_open_ticks);
}

static void MOTOR_AdvanceSinusAngle(void)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    uint32_t step_q16 = g_motor.sinus_step_q16;

    while (step_q16 >= full_turn_q16)
    {
        step_q16 -= full_turn_q16;
    }

    if (g_motor.direction == MOTOR_DIRECTION_CCW)
    {
        if (g_motor.sinus_angle_q16 >= step_q16)
        {
            g_motor.sinus_angle_q16 -= step_q16;
        }
        else
        {
            g_motor.sinus_angle_q16 = full_turn_q16 -
                                      (step_q16 - g_motor.sinus_angle_q16);
        }

        return;
    }

    g_motor.sinus_angle_q16 += step_q16;

    if (g_motor.sinus_angle_q16 >= full_turn_q16)
    {
        g_motor.sinus_angle_q16 -= full_turn_q16;
    }
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfRisingForStep(void)
{
    const uint8_t step_is_odd = (uint8_t)(g_motor.sixstep_step & 1u);

    if (g_motor.direction == MOTOR_DIRECTION_CW)
    {
        return step_is_odd;
    }

    if (step_is_odd != 0u)
    {
        return 0u;
    }

    return 1u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfInputStepForPwmStep(uint8_t pwm_step)
{
    uint8_t input_step = pwm_step;
    uint8_t shift = (uint8_t)(DRIVER_BEMF_PHASE_MAP_SHIFT % 3u);

    while (shift > 0u)
    {
        input_step++;

        if (input_step > MOTOR_SIXSTEP_STEP_COUNT)
        {
            input_step = 1u;
        }

        shift--;
    }

    return input_step;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfArmed(void)
{
    if (g_motor.bemf_armed != 0u)
    {
        return 1u;
    }

    return 0u;
}

static void MOTOR_ResetBemfValidation(void)
{
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfBlankTicks(void)
{
    uint8_t blank_ticks = DRIVER_SIXSTEP_BEMF_BLANK_TICKS;

    if ((g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS) &&
        (blank_ticks > MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS))
    {
        return MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS;
    }

    if ((g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS) &&
        (blank_ticks > MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS))
    {
        return MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS;
    }

    return blank_ticks;
}

static MOTOR_FAST_CODE void MOTOR_RunVirtualBemfCommutation(void)
{
    if (g_motor.sixstep_missed_zc_count < 0xFFu)
    {
        g_motor.sixstep_missed_zc_count++;
    }

    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_last_zc_ticks = g_motor.bemf_this_zc_ticks;
    g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.sixstep_virtual_zc_pending = 1u;

    BOARD_DisableBemfCommutationTimer();
    BOARD_ResetBemfIntervalTimer();
    MOTOR_AdvanceSixStepStep();
}

static MOTOR_FAST_CODE void MOTOR_StartBemfRecoveryOff(void)
{
    MOTOR_ResetBemfValidation();
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_RECOVERY_OFF;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;

    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_StopBemfPwmGating();
    BOARD_DisableBemfCommutationTimer();
    BOARD_ResetBemfIntervalTimer();
    BOARD_AllPhasesOff();
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfMissingZcClosedLoop(void)
{
    if ((g_motor.bemf_edge_seen != 0u) ||
        (g_motor.sixstep_commutation_pending != 0u))
    {
        return 0u;
    }

    if (g_motor.bemf_interval_ticks <= MOTOR_GetBemfMissingZcTimeoutTicks())
    {
        return 0u;
    }

    if (g_motor.sixstep_missed_zc_count <
        DRIVER_SIXSTEP_BEMF_MISSING_ZC_VIRTUAL_STEPS)
    {
        MOTOR_RunVirtualBemfCommutation();
        return 1u;
    }

    MOTOR_StartBemfRecoveryOff();
    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_RestartSixStepHandoffAfterRecovery(void)
{
    uint16_t interval_ticks = g_motor.sixstep_fallback_interval_ticks;

    if (interval_ticks == 0u)
    {
        interval_ticks = MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
    }

    if (interval_ticks == 0u)
    {
        interval_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
    }

    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_SIN_HANDOFF;
    g_motor.sixstep_interval_ticks = interval_ticks;
    g_motor.sixstep_fallback_interval_ticks = interval_ticks;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_readable = 0u;

    g_motor.sixstep_pwm_ticks =
        MOTOR_GetSixStepPwmTicksFromPermille(MOTOR_GetSixStepStartDutyPermille());
    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_ResetBemfIntervalTimer();
    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfRecoveryOff(void)
{
    uint16_t recovery_ticks = DRIVER_SIXSTEP_BEMF_RECOVERY_OFF_TICKS;

    if (g_motor.sixstep_phase != MOTOR_SIXSTEP_PHASE_RECOVERY_OFF)
    {
        return 0u;
    }

    if (recovery_ticks == 0u)
    {
        recovery_ticks = 1u;
    }

    if (g_motor.sixstep_recovery_off_tick < 0xFFFFu)
    {
        g_motor.sixstep_recovery_off_tick++;
    }

    if (g_motor.sixstep_recovery_off_tick < recovery_ticks)
    {
        return 1u;
    }

    MOTOR_RestartSixStepHandoffAfterRecovery();

    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_ArmBemfForSixStep(void)
{
    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_StopBemfPwmGating();

    if (g_motor.bemf_average_interval_ticks == 0u)
    {
        g_motor.bemf_average_interval_ticks =
            MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(g_motor.ramped_target_rpm));
        MOTOR_UpdateBemfMeasuredErpm();
        MOTOR_UpdateBemfFilterLevel();
    }

    MOTOR_UpdateBemfWaitTime();

    BEMF_AM32_ChangeCompInput(MOTOR_GetBemfInputStepForPwmStep(g_motor.sixstep_step),
                              MOTOR_GetBemfRisingForStep());

    if (MOTOR_isBemfArmed() == 0u)
    {
        g_motor.bemf_armed = 1u;
        g_motor.bemf_edge_seen = 1u;
        g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
        g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
        g_motor.bemf_interval_ticks = g_motor.bemf_average_interval_ticks;
        BOARD_SetBemfIntervalTicks(g_motor.bemf_average_interval_ticks >> 1u);
        MOTOR_ResetBemfValidation();
    }

    if (g_motor.bemf_edge_seen == 0u)
    {
        MOTOR_ResetBemfValidation();
        g_motor.bemf_edge_seen = 1u;
    }

    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_blank_ticks = MOTOR_GetBemfBlankTicks();

    if (g_motor.bemf_blank_ticks == 0u)
    {
        if (MOTOR_StartBemfPwmGating() == 0u)
        {
            BEMF_AM32_EnableCompInterrupts();
        }
    }
}

static MOTOR_FAST_CODE void MOTOR_ServiceBemfBlank(void)
{
    if (g_motor.bemf_blank_ticks == 0u)
    {
        return;
    }

    g_motor.bemf_blank_ticks--;

    if (g_motor.bemf_blank_ticks == 0u)
    {
        if (MOTOR_StartBemfPwmGating() == 0u)
        {
            BEMF_AM32_EnableCompInterrupts();
        }
    }
}

static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStepIndex(void)
{
    if (g_motor.direction == MOTOR_DIRECTION_CW)
    {
        g_motor.sixstep_step++;

        if (g_motor.sixstep_step > MOTOR_SIXSTEP_STEP_COUNT)
        {
            g_motor.sixstep_step = 1u;
        }
    }
    else
    {
        if (g_motor.sixstep_step > 1u)
        {
            g_motor.sixstep_step--;
        }
        else
        {
            g_motor.sixstep_step = MOTOR_SIXSTEP_STEP_COUNT;
        }
    }
}

static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStep(void)
{
    if (g_motor.sixstep_commutation_count < 0xFFFFu)
    {
        g_motor.sixstep_commutation_count++;
    }

    MOTOR_AdvanceSixStepStepIndex();
    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE void MOTOR_CommitBemfCommutation(void)
{
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_tick = 0u;
    MOTOR_AdvanceSixStepStep();
}

static void MOTOR_EnterSixStepFromSinus(void)
{
    MOTOR_RestoreDefaultPwmCarrier();
    MOTOR_UpdateSixStepPwmLimit();

    g_motor.mode = MOTOR_MODE_SIXSTEP;
    g_motor.sixstep_step = MOTOR_GetSixStepFromAngle(g_motor.sinus_angle_q16);
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sixstep_interval_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
    g_motor.sixstep_fallback_interval_ticks = g_motor.sixstep_interval_ticks;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_average_interval_ticks =
        MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(g_motor.ramped_target_rpm));
    g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.hard_current_fault = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_SIN_HANDOFF;

    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_DisableBemfCommutationTimer();

    g_motor.sixstep_pwm_ticks = MOTOR_GetSinToSixStepPwmTicks();

    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static void MOTOR_SixStepTickIrq(void)
{
    if (g_motor.sixstep_interval_ticks == 0u)
    {
        BOARD_AllPhasesOff();
        return;
    }

    g_motor.bemf_interval_ticks = BOARD_GetBemfIntervalTicks();
    MOTOR_ServiceBemfSlowUpdate();

    if (g_motor.sixstep_run_ticks < 0xFFFFFFFFu)
    {
        g_motor.sixstep_run_ticks++;
    }

    if (MOTOR_ServiceBemfRecoveryOff() != 0u)
    {
        return;
    }

    MOTOR_ServiceBemfBlank();

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        if (MOTOR_ServiceBemfMissingZcClosedLoop() != 0u)
        {
            return;
        }

        MOTOR_ServiceSixStepClosedLoopPwmControl();
        return;
    }

    if (g_motor.sixstep_commutation_pending != 0u)
    {
        return;
    }

    g_motor.sixstep_tick++;

    if (g_motor.sixstep_tick >= g_motor.sixstep_interval_ticks)
    {
        g_motor.sixstep_tick = 0u;
        MOTOR_AdvanceSixStepStep();
    }

    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)MOTOR_GetPolePairs();
    g_motor.in_rpm = 1u;
}

static MOTOR_FAST_CODE void MOTOR_EnableBemfClosedLoop(void)
{
    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        return;
    }

    g_motor.sixstep_bemf_closed_loop = 1u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_CLOSED_LOOP;
    g_motor.sixstep_interval_ticks =
        MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
    MOTOR_UpdateAdaptiveSixStepPwmCarrier();
    MOTOR_ResetSixStepClosedLoopControl();
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    MOTOR_ApplySixStepBridge();
}

static MOTOR_FAST_CODE void MOTOR_HandleBemfZeroCross(void)
{
    uint16_t captured_interval_ticks = g_motor.bemf_interval_ticks;
    uint8_t confirm_count = DRIVER_SIN_TO_6STEP_BEMF_CONFIRM_ZC_COUNT;

    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_StopBemfPwmGating();

    if (captured_interval_ticks == 0u)
    {
        captured_interval_ticks = BOARD_GetBemfIntervalTicks();
        g_motor.bemf_interval_ticks = captured_interval_ticks;
    }

    if (g_motor.sixstep_virtual_zc_pending != 0u)
    {
        captured_interval_ticks =
            MOTOR_LimitTickCount((uint32_t)captured_interval_ticks << 1u);
        g_motor.bemf_interval_ticks = captured_interval_ticks;
        g_motor.sixstep_virtual_zc_pending = 0u;
    }

    g_motor.bemf_last_zc_ticks = g_motor.bemf_this_zc_ticks;
    g_motor.bemf_this_zc_ticks = captured_interval_ticks;
    BOARD_ResetBemfIntervalTimer();

    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_poll_count = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;

    if (g_motor.bemf_zero_cross_count < 0xFFu)
    {
        g_motor.bemf_zero_cross_count++;
    }

    MOTOR_UpdateBemfDerivedTiming();

    if (confirm_count == 0u)
    {
        confirm_count = 1u;
    }

    if (g_motor.bemf_zero_cross_count >= confirm_count)
    {
        g_motor.bemf_readable = 1u;

        if (MOTOR_isBemfMonitorOnlyMode() == 0u)
        {
            MOTOR_EnableBemfClosedLoop();
        }
    }

    if (((g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_SIN_HANDOFF) ||
         (g_motor.sixstep_bemf_closed_loop != 0u)) &&
        (MOTOR_isBemfMonitorOnlyMode() == 0u))
    {
        g_motor.sixstep_bemf_delay_ticks =
            MOTOR_GetBemfVirtualCommutationDelayTicks(captured_interval_ticks);
        MOTOR_ScheduleBemfCommutation();
    }
}

static MOTOR_FAST_CODE void MOTOR_StopBemfPwmGating(void)
{
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    BOARD_EnableBemfPwmSampleIrq(0u);
}

static MOTOR_FAST_CODE uint8_t MOTOR_StartBemfPwmGating(void)
{
    if (MOTOR_isBemfPwmGatingEnabled() == 0u)
    {
        return 0u;
    }

    if (MOTOR_PrepareBemfPwmGatingTicks(g_motor.sixstep_pwm_ticks) == 0u)
    {
        return 0u;
    }

    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_last_level = MOTOR_GetBemfExpectedLevelBeforeZc();
    g_motor.bemf_pwm_sample_valid = 1u;
    BEMF_AM32_MaskPhaseInterrupts();
    BOARD_SetBemfPwmSampleTicks(g_motor.bemf_pwm_gating_open_ticks);
    BOARD_EnableBemfPwmSampleIrq(1u);

    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_CloseBemfPwmGatingWindow(void)
{
    BEMF_AM32_MaskPhaseInterrupts();
    g_motor.bemf_pwm_gating_close_pending = 0u;
    BOARD_SetBemfPwmSampleTicks(g_motor.bemf_pwm_gating_open_ticks);
}

static MOTOR_FAST_CODE void MOTOR_ServiceBemfPwmGatingIrq(void)
{
    if (MOTOR_isBemfPwmGatingEnabled() == 0u)
    {
        MOTOR_StopBemfPwmGating();
        return;
    }

    if ((MOTOR_isBemfArmed() == 0u) ||
        (g_motor.bemf_blank_ticks != 0u) ||
        (g_motor.bemf_edge_seen != 0u) ||
        (g_motor.sixstep_commutation_pending != 0u))
    {
        BEMF_AM32_MaskPhaseInterrupts();
        MOTOR_StopBemfPwmGating();
        return;
    }

    if (g_motor.bemf_pwm_gating_close_pending != 0u)
    {
        MOTOR_CloseBemfPwmGatingWindow();
        return;
    }

    if (MOTOR_PrepareBemfPwmGatingTicks(g_motor.sixstep_pwm_ticks) == 0u)
    {
        MOTOR_StopBemfPwmGating();
        BEMF_AM32_MaskPhaseInterrupts();
        BEMF_AM32_EnableCompInterrupts();
        return;
    }

    if (BOARD_GetPwmCounterTicks() >= g_motor.bemf_pwm_gating_close_ticks)
    {
        MOTOR_CloseBemfPwmGatingWindow();
        return;
    }

    BEMF_AM32_MaskPhaseInterrupts();

    if (MOTOR_isBemfPwmSampleZeroCross(BEMF_AM32_GetOutputLevel()) != 0u)
    {
        MOTOR_HandleBemfZeroCross();
        return;
    }

    g_motor.bemf_pwm_gating_close_pending = 1u;
    BOARD_SetBemfPwmSampleTicks(g_motor.bemf_pwm_gating_close_ticks);
}

static uint8_t MOTOR_isSinusReadyForSixStep(void)
{
#if DRIVER_SIN_TO_6STEP_ENABLE
    if (g_motor.in_rpm != 0u)
    {
        if (g_motor.sinus_at_target_ticks < 0xFFFFFFFFu)
        {
            g_motor.sinus_at_target_ticks++;
        }

        if (g_motor.sinus_at_target_ticks >= MOTOR_GetSinToSixStepSettleTicks())
        {
            return 1u;
        }

        return 0u;
    }

    g_motor.sinus_at_target_ticks = 0u;
    return 0u;
#else
    g_motor.sinus_at_target_ticks = 0u;
    return 0u;
#endif
}

void MOTOR_Init(void)
{
    g_motor.mode = MOTOR_MODE_SINUS;
    g_motor.direction = DRIVER_OPEN_LOOP_DIRECTION;
    g_motor.target_rpm = DRIVER_OPEN_LOOP_SINUS_RPM;
    g_motor.ramped_target_rpm = 0;
    g_motor.measured_erpm = 0;
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_step_q16 = 0u;
    g_motor.open_loop_tick = 0u;
    g_motor.sinus_at_target_ticks = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sin_current_target_ma = DRIVER_SIN_CURRENT_TARGET_MA;
    g_motor.sixstep_current_limit_ma = DRIVER_SIXSTEP_CURRENT_LIMIT_MA;
    g_motor.hard_current_limit_ma = DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA;
    g_motor.hard_current_adc_threshold = MOTOR_GetCurrentAdcFromMilliAmps(g_motor.hard_current_limit_ma);
    g_motor.measured_current_ma = 0u;
    g_motor.control_tick_count = 0u;
    g_motor.sin_current_update_count = 0u;
    g_motor.sixstep_speed_update_count = 0u;
    g_motor.sixstep_current_limit_update_count = 0u;
    g_motor.hard_current_fault_count = 0u;
    g_motor.sin_current_error_adc = 0;
    g_motor.sixstep_target_rpm = DRIVER_SIXSTEP_TARGET_RPM;
    g_motor.sixstep_measured_rpm = 0;
    g_motor.sixstep_rpm_error = 0;
    g_motor.sixstep_current_error_ma = 0;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.current_adc_raw = 0u;
    g_motor.current_adc_filtered = 0u;
    g_motor.current_adc_signal = 0u;
    g_motor.sin_current_target_adc = 0u;
    g_motor.sinus_pwm_max_ticks = 0u;
    g_motor.sinus_pwm_ticks = 0u;
    g_motor.sin_current_pi_output_permille = 0u;
    g_motor.sin_current_target_pwm_ticks = 0u;
    g_motor.sixstep_pwm_limit_ticks = 0u;
    g_motor.sixstep_pwm_ticks = 0u;
    g_motor.sixstep_speed_pid_target_permille = 0u;
    g_motor.sixstep_speed_pid_output_permille = 0u;
    g_motor.sixstep_speed_pid_output_q16 = 0u;
    g_motor.sixstep_speed_pid_rise_step_q16 = 0u;
    g_motor.sixstep_speed_pid_fall_step_q16 = 0u;
    g_motor.sixstep_current_limit_reduction_permille = 0u;
    g_motor.sixstep_final_pwm_permille = 0u;
    g_motor.sixstep_interval_ticks = 0u;
    g_motor.sixstep_fallback_interval_ticks = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_average_interval_ticks = 1u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.sinus_table_index = 0u;
    g_motor.sin_current_control_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_current_limit_control_tick = 0u;
    g_motor.in_rpm = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.hard_current_fault = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_step = 0u;
    g_motor.pwm_pulses_per_sector = 0u;

    BEMF_AM32_Init(&g_motor.bemf_interval_ticks,
                   &g_motor.bemf_average_interval_ticks,
                   MOTOR_BemfZeroCrossIrq);
    BEMF_AM32_MaskPhaseInterrupts();
    BEMF_AM32_SetFilterLevel(5u);
    MOTOR_ResetSinusCurrentControl();
    MOTOR_UpdateSixStepPwmLimit();

    MOTOR_UpdateSinusTiming(0);
    MOTOR_ApplySinusBridge();
    MOTOR_UpdateStatusLed();
}

void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction)
{
    (void)rpm;
    (void)direction;

    g_motor.target_rpm = DRIVER_OPEN_LOOP_SINUS_RPM;
    g_motor.ramped_target_rpm = 0;
    g_motor.direction = DRIVER_OPEN_LOOP_DIRECTION;
    g_motor.open_loop_tick = 0u;
    g_motor.sinus_at_target_ticks = 0u;
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_table_index = 0u;
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.pwm_pulses_per_sector = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.sixstep_step = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_speed_update_count = 0u;
    g_motor.sixstep_current_limit_update_count = 0u;
    g_motor.sixstep_target_rpm = DRIVER_SIXSTEP_TARGET_RPM;
    g_motor.sixstep_measured_rpm = 0;
    g_motor.sixstep_rpm_error = 0;
    g_motor.sixstep_current_limit_ma = DRIVER_SIXSTEP_CURRENT_LIMIT_MA;
    g_motor.hard_current_limit_ma = DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA;
    g_motor.hard_current_adc_threshold = MOTOR_GetCurrentAdcFromMilliAmps(g_motor.hard_current_limit_ma);
    g_motor.sixstep_current_error_ma = 0;
    g_motor.sixstep_fallback_interval_ticks = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.sixstep_recovery_off_tick = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_pwm_gating_open_ticks = 0u;
    g_motor.bemf_pwm_gating_close_ticks = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_current_limit_control_tick = 0u;
    g_motor.sixstep_speed_pid_target_permille = 0u;
    g_motor.sixstep_speed_pid_output_permille = 0u;
    g_motor.sixstep_speed_pid_output_q16 = 0u;
    g_motor.sixstep_speed_pid_rise_step_q16 = 0u;
    g_motor.sixstep_speed_pid_fall_step_q16 = 0u;
    g_motor.sixstep_current_limit_reduction_permille = 0u;
    g_motor.sixstep_final_pwm_permille = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.hard_current_fault = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_count = 0u;
    g_motor.sixstep_virtual_zc_pending = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.bemf_pwm_gating_active_window = MOTOR_BEMF_PWM_SAMPLE_OFF_TIME;
    g_motor.bemf_pwm_gating_close_pending = 0u;
    g_motor.bemf_pwm_sample_valid = 0u;
    g_motor.bemf_pwm_last_level = 0u;
    MOTOR_RestoreDefaultPwmCarrier();
    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_StopBemfPwmGating();
    BOARD_DisableBemfCommutationTimer();
    MOTOR_ResetSinusCurrentControl();
    MOTOR_UpdateSinusTiming(0);
}

void MOTOR_ControlTick10kHz(void)
{
    BOARD_ServiceCurrentAdc();
    MOTOR_UpdateCurrentMeasurementFromAdc();
    g_motor.control_tick_count++;

    if (MOTOR_ServiceHardCurrentProtection() != 0u)
    {
        return;
    }

    MOTOR_UpdateStatusLed();

    if (g_motor.mode == MOTOR_MODE_SINUS)
    {
        MOTOR_SinusStepIrq();
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        MOTOR_SixStepTickIrq();
        return;
    }

    BOARD_AllPhasesOff();
}

void MOTOR_SinusStepIrq(void)
{
    const uint32_t align_ticks = MOTOR_GetAlignTicks();
    uint32_t ramp_tick;

    if (g_motor.mode != MOTOR_MODE_SINUS)
    {
        return;
    }

    if (DRIVER_OPEN_LOOP_SINUS_RPM == 0)
    {
        BOARD_AllPhasesOff();
        return;
    }

    if (g_motor.open_loop_tick < align_ticks)
    {
        g_motor.open_loop_tick++;
        g_motor.ramped_target_rpm = 0;
        g_motor.measured_erpm = 0;
        g_motor.sinus_angle_q16 = 0u;
        g_motor.sinus_at_target_ticks = 0u;
        g_motor.sinus_table_index = 0u;
        MOTOR_UpdateSinusTiming(0);
        MOTOR_ServiceSinusCurrentControl();
        MOTOR_ApplySinusBridge();
        return;
    }

    ramp_tick = g_motor.open_loop_tick - align_ticks;
    g_motor.open_loop_tick++;
    g_motor.ramped_target_rpm = MOTOR_GetOpenLoopRampedRpm(ramp_tick);
    MOTOR_UpdateSinusTiming(g_motor.ramped_target_rpm);
    MOTOR_AdvanceSinusAngle();

    g_motor.sinus_table_index = (uint16_t)(g_motor.sinus_angle_q16 >> 16u);
    MOTOR_ServiceSinusCurrentControl();
    MOTOR_ApplySinusBridge();
    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)MOTOR_GetPolePairs();
    g_motor.in_rpm = (g_motor.ramped_target_rpm == DRIVER_OPEN_LOOP_SINUS_RPM) ? 1u : 0u;

    if (MOTOR_isSinusReadyForSixStep())
    {
        MOTOR_EnterSixStepFromSinus();
    }
}

MOTOR_FAST_CODE void MOTOR_BemfCommutationTimerIrq(void)
{
    BOARD_DisableBemfCommutationTimer();

    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        return;
    }

    if (g_motor.sixstep_commutation_pending == 0u)
    {
        return;
    }

    MOTOR_CommitBemfCommutation();
}

MOTOR_FAST_CODE void MOTOR_BemfZeroCrossIrq(void)
{
    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        BEMF_AM32_MaskPhaseInterrupts();
        return;
    }

    MOTOR_HandleBemfZeroCross();
}


MOTOR_FAST_CODE void MOTOR_BemfPwmSampleIrq(void)
{
    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        MOTOR_StopBemfPwmGating();
        return;
    }

    if (MOTOR_isBemfPwmGatingEnabled() == 0u)
    {
        MOTOR_StopBemfPwmGating();
        return;
    }

    MOTOR_ServiceBemfPwmGatingIrq();
}
