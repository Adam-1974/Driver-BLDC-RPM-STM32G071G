#include "motor_control.h"

#include "bemf_am32.h"
#include "board.h"
#include "nvm_config.h"
#include "sinus_table.h"

motor_control_state_t g_motor;

static volatile uint16_t s_bemf_interval_counter;
static volatile uint16_t s_bemf_average_interval = 1000u;

static uint32_t MOTOR_GetAbsRpm(int32_t rpm)
{
    if (rpm < 0)
    {
        return (uint32_t)(-rpm);
    }

    return (uint32_t)rpm;
}

static uint16_t MOTOR_GetSinusSample(uint16_t index)
{
    return g_sinus_table[index % DRIVER_SIN_TABLE_STEPS];
}

static int32_t MOTOR_GetMeasuredRpm(void)
{
    if (g_driver_config.pole_pairs == 0u)
    {
        return 0;
    }

    return g_motor.measured_erpm / (int32_t)g_driver_config.pole_pairs;
}

static uint16_t MOTOR_GetMin3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t min_value = a;

    if (b < min_value)
    {
        min_value = b;
    }

    if (c < min_value)
    {
        min_value = c;
    }

    return min_value;
}

static uint16_t MOTOR_ScaleSinusDuty(uint16_t sample, uint16_t min_sample)
{
    const uint32_t shifted_sample = (uint32_t)(sample - min_sample);

    return (uint16_t)((shifted_sample * g_motor.sinus_pwm_limit_ticks) /
                      DRIVER_SIN_TABLE_MAX_VALUE);
}

static void MOTOR_UpdateSinusPwmLimit(void)
{
    uint32_t duty_permille = DRIVER_STARTUP_SINUS_MAX_DUTY_PERMILLE;

    if (duty_permille > 1000u)
    {
        duty_permille = 1000u;
    }

    g_motor.sinus_pwm_limit_ticks = (uint16_t)(((uint32_t)BOARD_GetPwmPeriodTicks() *
                                                duty_permille) / 1000u);
}

static void MOTOR_UpdateSinusTiming(void)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(g_motor.ramped_target_rpm);
    const uint64_t numerator = (uint64_t)rpm * g_driver_config.pole_pairs *
                               DRIVER_SIN_TABLE_STEPS * 65536u;
    const uint32_t denominator = 60u * DRIVER_CONTROL_LOOP_HZ;

    if (denominator == 0u)
    {
        g_motor.sinus_step_q16 = 0u;
        return;
    }

    g_motor.sinus_step_q16 = (uint32_t)(numerator / denominator);
}

static void MOTOR_ApplySinusBridge(void)
{
    const uint16_t phase_shift_120 = DRIVER_SIN_TABLE_STEPS / 3u;
    const uint16_t phase_shift_240 = (DRIVER_SIN_TABLE_STEPS * 2u) / 3u;
    const uint16_t index_a = g_motor.sinus_table_index;
    const uint16_t index_b = (uint16_t)(index_a + phase_shift_120);
    const uint16_t index_c = (uint16_t)(index_a + phase_shift_240);
    const uint16_t sample_a = MOTOR_GetSinusSample(index_a);
    const uint16_t sample_b = MOTOR_GetSinusSample(index_b);
    const uint16_t sample_c = MOTOR_GetSinusSample(index_c);
    const uint16_t min_sample = MOTOR_GetMin3(sample_a, sample_b, sample_c);
    const uint16_t duty_a = MOTOR_ScaleSinusDuty(sample_a, min_sample);
    const uint16_t duty_b = MOTOR_ScaleSinusDuty(sample_b, min_sample);
    const uint16_t duty_c = MOTOR_ScaleSinusDuty(sample_c, min_sample);
    uint8_t low_a = 0u;
    uint8_t low_b = 0u;
    uint8_t low_c = 0u;

    if ((sample_a <= sample_b) && (sample_a <= sample_c))
    {
        low_a = 1u;
    }
    else if (sample_b <= sample_c)
    {
        low_b = 1u;
    }
    else
    {
        low_c = 1u;
    }

    BOARD_SetLowSideState(0u, 0u, 0u);
    BOARD_SetHighPwm(duty_a, duty_b, duty_c);
    BOARD_SetLowSideState(low_a, low_b, low_c);
}

void MOTOR_Init(void)
{
    g_motor.mode = MOTOR_MODE_SINUS;
    g_motor.direction = MOTOR_DIRECTION_CW;
    g_motor.target_rpm = DRIVER_STARTUP_SINUS_TARGET_RPM;
    g_motor.ramped_target_rpm = DRIVER_STARTUP_SINUS_TARGET_RPM;
    g_motor.measured_erpm = 0;
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_step_q16 = 0u;
    g_motor.sinus_pwm_limit_ticks = 0u;
    g_motor.sinus_table_index = 0u;
    g_motor.in_rpm = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.sixstep_step = 1u;

    BEMF_AM32_Init(&s_bemf_interval_counter, &s_bemf_average_interval, MOTOR_BemfZeroCrossIrq);
    MOTOR_UpdateSinusPwmLimit();
    MOTOR_UpdateSinusTiming();
    MOTOR_ApplySinusBridge();
}

void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction)
{
    g_motor.target_rpm = rpm;
    g_motor.ramped_target_rpm = rpm;
    g_motor.direction = direction;
    MOTOR_UpdateSinusTiming();
}

void MOTOR_ControlTick10kHz(void)
{
    const int32_t measured_rpm = MOTOR_GetMeasuredRpm();
    const int32_t error = g_motor.target_rpm - measured_rpm;

    s_bemf_interval_counter++;

    if (error < 0)
    {
        g_motor.in_rpm = ((uint32_t)(-error) <= g_driver_config.in_rpm_window);
    }
    else
    {
        g_motor.in_rpm = ((uint32_t)error <= g_driver_config.in_rpm_window);
    }

    if (g_motor.mode == MOTOR_MODE_SINUS)
    {
        MOTOR_SinusStepIrq();

        if (((uint32_t)MOTOR_GetAbsRpm(measured_rpm) >= g_driver_config.sin_to_sixstep_rpm) &&
            (g_motor.bemf_readable != 0u))
        {
            g_motor.mode = MOTOR_MODE_SIXSTEP;
        }
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        const uint32_t return_rpm = g_driver_config.sin_to_sixstep_rpm -
                                    g_driver_config.sixstep_to_sin_hysteresis_rpm;

        if ((uint32_t)MOTOR_GetAbsRpm(measured_rpm) < return_rpm)
        {
            g_motor.mode = MOTOR_MODE_SINUS;
        }
    }
}

void MOTOR_SinusStepIrq(void)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;

    if (g_motor.mode != MOTOR_MODE_SINUS)
    {
        return;
    }

    if (g_motor.ramped_target_rpm == 0)
    {
        BOARD_AllPhasesOff();
        return;
    }

    g_motor.sinus_angle_q16 += g_motor.sinus_step_q16;

    while (g_motor.sinus_angle_q16 >= full_turn_q16)
    {
        g_motor.sinus_angle_q16 -= full_turn_q16;
    }

    g_motor.sinus_table_index = (uint16_t)(g_motor.sinus_angle_q16 >> 16u);
    MOTOR_ApplySinusBridge();
    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)g_driver_config.pole_pairs;
}

void MOTOR_BemfZeroCrossIrq(void)
{
    g_motor.bemf_readable = 1u;
    g_motor.sixstep_step++;

    if (g_motor.sixstep_step > 6u)
    {
        g_motor.sixstep_step = 1u;
    }
}
