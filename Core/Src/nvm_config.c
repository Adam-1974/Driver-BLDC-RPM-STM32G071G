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

    config->bemf_action_angle_offset_deg_x10 = 100;
    config->reserved_alignment = 0;

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
