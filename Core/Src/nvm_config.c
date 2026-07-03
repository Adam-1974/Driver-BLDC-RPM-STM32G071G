#include "nvm_config.h"

driver_nvm_config_t g_driver_config;

static uint32_t NVM_CalcPlaceholderCrc(const driver_nvm_config_t *config)
{
    const uint32_t *words = (const uint32_t *)config;
    const uint32_t word_count = (sizeof(*config) - sizeof(config->crc32)) / sizeof(uint32_t);
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < word_count; i++)
    {
        crc ^= words[i];
        crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }

    return crc;
}

void NVM_SetDefaults(driver_nvm_config_t *config)
{
    config->init_flash = DRIVER_NVM_FLASH_MARKER;
    config->struct_version = DRIVER_NVM_STRUCT_VERSION;
    config->pole_pairs = 7u;

    config->accel_rpm_per_sec = 1000u;
    config->decel_rpm_per_sec = 1500u;
    config->sin_to_sixstep_rpm = 1200u;
    config->sixstep_to_sin_hysteresis_rpm = 200u;
    config->in_rpm_window = 10u;

    config->sin_current_ma = 1500u;
    config->sixstep_max_current_ma = 8000u;
    config->adc_current_ma_per_count_q16 = 65536u;

    config->bemf_action_angle_offset_deg_x10 = 100;
    config->reserved_alignment = 0;

    config->pid_sin_current = (pid_config_t){ .kp_q16 = 1 << 16, .ki_q16 = 0, .kd_q16 = 0, .out_min = 0, .out_max = 1000 };
    config->pid_sixstep_speed = (pid_config_t){ .kp_q16 = 1 << 16, .ki_q16 = 0, .kd_q16 = 0, .out_min = 0, .out_max = (int32_t)config->sixstep_max_current_ma };
    config->pid_current_limit = (pid_config_t){ .kp_q16 = 1 << 16, .ki_q16 = 0, .kd_q16 = 0, .out_min = 0, .out_max = 1000 };

    config->crc32 = NVM_CalcPlaceholderCrc(config);
}

uint8_t NVM_IsValid(const driver_nvm_config_t *config)
{
    if (config->init_flash == DRIVER_NVM_FLASH_MARKER)
    {
        if (config->struct_version == DRIVER_NVM_STRUCT_VERSION)
        {
            return (config->crc32 == NVM_CalcPlaceholderCrc(config));
        }
    }

    return 0u;
}

void NVM_LoadOrDefault(void)
{
    const driver_nvm_config_t *flash_config = (const driver_nvm_config_t *)0x0801F800u;

    if (NVM_IsValid(flash_config))
    {
        g_driver_config = *flash_config;
        return;
    }

    NVM_SetDefaults(&g_driver_config);
}
