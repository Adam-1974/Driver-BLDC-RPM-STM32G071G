#include "pid.h"

void PID_Init(pid_state_t *pid, const pid_config_t *config)
{
    pid->config = *config;
    PID_Reset(pid);
}

void PID_Reset(pid_state_t *pid)
{
    pid->integrator = 0;
    pid->last_error = 0;
}

int32_t PID_Update(pid_state_t *pid, int32_t setpoint, int32_t feedback)
{
    const int32_t error = setpoint - feedback;
    const int32_t derivative = error - pid->last_error;
    int64_t output;

    pid->integrator += error;

    output = ((int64_t)pid->config.kp_q16 * error);
    output += ((int64_t)pid->config.ki_q16 * pid->integrator);
    output += ((int64_t)pid->config.kd_q16 * derivative);
    output >>= 16;

    if (output > pid->config.out_max)
    {
        output = pid->config.out_max;
    }

    if (output < pid->config.out_min)
    {
        output = pid->config.out_min;
    }

    pid->last_error = error;

    return (int32_t)output;
}

