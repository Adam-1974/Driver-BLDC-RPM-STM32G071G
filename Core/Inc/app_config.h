#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define DRIVER_SYSCLK_HZ                64000000u
#define DRIVER_PWM_CARRIER_HZ           14000u
#define DRIVER_CONTROL_LOOP_HZ          10000u
#define DRIVER_SIN_TABLE_STEPS          256u
#define DRIVER_SIN_TABLE_MAX_VALUE      1049u
#define DRIVER_STARTUP_SINUS_TARGET_RPM 300
#define DRIVER_STARTUP_SINUS_MAX_DUTY_PERMILLE 250u
#define DRIVER_NVM_FLASH_MARKER         0x424C4443u
#define DRIVER_NVM_STRUCT_VERSION       1u

typedef enum
{
    MOTOR_DIRECTION_CW = 0,
    MOTOR_DIRECTION_CCW = 1
} motor_direction_t;

typedef enum
{
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_SINUS = 1,
    MOTOR_MODE_SIXSTEP = 2
} motor_mode_t;

#endif
