#include "main.h"

#include "board.h"
#include "motor_control.h"
#include "nvm_config.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    BOARD_InitStaticOutputs();
    NVM_LoadOrDefault();
    MOTOR_Init();

    while (1)
    {
    }
}

