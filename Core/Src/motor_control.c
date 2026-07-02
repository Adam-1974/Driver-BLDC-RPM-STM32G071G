#include "motor_control.h"

#include "bemf_am32.h"
#include "nvm_config.h"

motor_control_state_t g_motor;

static volatile uint16_t s_bemf_interval_counter;
static volatile uint16_t s_bemf_average_interval = 1000u;

void MOTOR_Init(void)
{
    g_motor.mode = MOTOR_MODE_STOP;
    g_motor.direction = MOTOR_DIRECTION_CW;
    g_motor.target_rpm = 0;
    g_motor.ramped_target_rpm = 0;
    g_motor.measured_erpm = 0;
    g_motor.in_rpm = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.sixstep_step = 1u;

    BEMF_AM32_Init(&s_bemf_interval_counter, &s_bemf_average_interval, MOTOR_BemfZeroCrossIrq);
}

void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction)
{
    g_motor.target_rpm = rpm;
    g_motor.direction = direction;
}

void MOTOR_ControlTick1kHz(void)
{
    const int32_t error = g_motor.target_rpm - g_motor.measured_erpm;

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
        if (((uint32_t)g_motor.measured_erpm >= g_driver_config.sin_to_sixstep_rpm) &&
            (g_motor.bemf_readable != 0u))
        {
            g_motor.mode = MOTOR_MODE_SIXSTEP;
        }
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        const uint32_t return_rpm = g_driver_config.sin_to_sixstep_rpm -
                                    g_driver_config.sixstep_to_sin_hysteresis_rpm;

        if ((uint32_t)g_motor.measured_erpm < return_rpm)
        {
            g_motor.mode = MOTOR_MODE_SINUS;
        }
    }
}

void MOTOR_SinusStepIrq(void)
{
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

