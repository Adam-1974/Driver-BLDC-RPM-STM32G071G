#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t out_min;
    int32_t out_max;
} pid_config_t;

typedef struct
{
    pid_config_t config;
    int32_t integrator;
    int32_t last_error;
} pid_state_t;

void PID_Init(pid_state_t *pid, const pid_config_t *config);
int32_t PID_Update(pid_state_t *pid, int32_t setpoint, int32_t feedback);
void PID_Reset(pid_state_t *pid);

#endif

