#ifndef NVM_CONFIG_H
#define NVM_CONFIG_H

#include <stdint.h>

#include "app_config.h"
#include "pid.h"

typedef struct
{
    uint32_t init_flash;
    uint16_t struct_version;
    uint16_t pole_pairs;

    uint32_t accel_rpm_per_sec;
    uint32_t decel_rpm_per_sec;
    uint32_t sin_to_sixstep_rpm;
    uint32_t sixstep_to_sin_hysteresis_rpm;
    uint32_t in_rpm_window;

    uint32_t sin_current_ma;
    uint32_t sixstep_max_current_ma;
    uint32_t adc_current_ma_per_count_q16;

    int16_t bemf_action_angle_offset_deg_x10;
    int16_t reserved_alignment;

    pid_config_t pid_sin_current;
    pid_config_t pid_sixstep_speed;
    pid_config_t pid_current_limit;

    uint32_t crc32;
} driver_nvm_config_t;

extern driver_nvm_config_t g_driver_config;

void NVM_LoadOrDefault(void);
void NVM_SetDefaults(driver_nvm_config_t *config);
uint8_t NVM_IsValid(const driver_nvm_config_t *config);

#endif

