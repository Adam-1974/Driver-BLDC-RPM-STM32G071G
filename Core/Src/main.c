#include "main.h"

#include "app_config.h"
#include "board.h"
#include "motor_control.h"
#include "nvm_config.h"

static void MAIN_ErrorHandler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc = { 0 };
    RCC_ClkInitTypeDef rcc_clk = { 0 };

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    rcc_osc.HSIState = RCC_HSI_ON;
    rcc_osc.HSIDiv = RCC_HSI_DIV1;
    rcc_osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    rcc_osc.PLL.PLLState = RCC_PLL_ON;
    rcc_osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    rcc_osc.PLL.PLLM = RCC_PLLM_DIV1;
    rcc_osc.PLL.PLLN = 8u;
    rcc_osc.PLL.PLLP = RCC_PLLP_DIV2;
    rcc_osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    rcc_osc.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK)
    {
        MAIN_ErrorHandler();
    }

    rcc_clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    rcc_clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    rcc_clk.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_2) != HAL_OK)
    {
        MAIN_ErrorHandler();
    }
}

static void MAIN_CheckClockTree(void)
{
    if (HAL_RCC_GetSysClockFreq() != DRIVER_EXPECTED_SYSCLK_HZ)
    {
        MAIN_ErrorHandler();
    }

    if (HAL_RCC_GetHCLKFreq() != DRIVER_EXPECTED_SYSCLK_HZ)
    {
        MAIN_ErrorHandler();
    }

    if (HAL_RCC_GetPCLK1Freq() != DRIVER_EXPECTED_SYSCLK_HZ)
    {
        MAIN_ErrorHandler();
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MAIN_CheckClockTree();

    BOARD_InitStaticOutputs();
    BOARD_InitPwmOutputs();
    BOARD_InitCurrentAdc();
    BOARD_InitBemfComparator();
    BOARD_InitBemfTiming();
    NVM_LoadOrDefault();
    MOTOR_Init();
    BOARD_InitControlTick();

    while (1)
    {
    }
}
