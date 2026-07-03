#include "main.h"

#include "app_config.h"
#include "board.h"
#include "motor_control.h"
#include "nvm_config.h"

#define MAIN_STATUS_LED_UPDATE_MS       20u
#define MAIN_STATUS_LED_BEMF_PERIOD_MS  500u
#define MAIN_STATUS_LED_BEMF_ON_MS      80u
#define MAIN_STATUS_LED_DIM             12u
#define MAIN_STATUS_LED_WHITE           10u
#define MAIN_STATUS_LED_BEMF_DIAG       24u
#define MAIN_STATUS_LED_SIXSTEP_WAIT_RED 10u
#define MAIN_STATUS_LED_SIXSTEP_WAIT_GREEN 5u

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} main_rgb_t;

static void MAIN_ErrorHandler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

static main_rgb_t MAIN_GetStatusLedColor(uint32_t tick_ms)
{
    main_rgb_t color = { 0u, 0u, 0u };

#if DRIVER_BEMF_LED_DIAGNOSTIC_ONLY
    (void)tick_ms;

    if (g_motor.bemf_led_phase == MOTOR_BEMF_LED_PHASE_A)
    {
        color.red = MAIN_STATUS_LED_BEMF_DIAG;
        return color;
    }

    if (g_motor.bemf_led_phase == MOTOR_BEMF_LED_PHASE_B)
    {
        color.green = MAIN_STATUS_LED_BEMF_DIAG;
        return color;
    }

    if (g_motor.bemf_led_phase == MOTOR_BEMF_LED_PHASE_C)
    {
        color.blue = MAIN_STATUS_LED_BEMF_DIAG;
        return color;
    }

    return color;
#else
    (void)tick_ms;

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        if (g_motor.sixstep_bemf_closed_loop != 0u)
        {
            color.green = MAIN_STATUS_LED_BEMF_DIAG;
            return color;
        }

        if ((g_motor.bemf_phase_mask & MOTOR_BEMF_PHASE_A_MASK) != 0u)
        {
            color.red = MAIN_STATUS_LED_BEMF_DIAG;
        }

        if ((g_motor.bemf_phase_mask & MOTOR_BEMF_PHASE_B_MASK) != 0u)
        {
            color.green = MAIN_STATUS_LED_BEMF_DIAG;
        }

        if ((g_motor.bemf_phase_mask & MOTOR_BEMF_PHASE_C_MASK) != 0u)
        {
            color.blue = MAIN_STATUS_LED_BEMF_DIAG;
        }

        if (g_motor.bemf_zero_cross_count != 0u)
        {
            return color;
        }

        if (g_motor.bemf_armed != 0u)
        {
            color.blue = MAIN_STATUS_LED_DIM;
            return color;
        }

        color.red = MAIN_STATUS_LED_SIXSTEP_WAIT_RED;
        color.green = MAIN_STATUS_LED_SIXSTEP_WAIT_GREEN;
        return color;
    }

    if (g_motor.mode == MOTOR_MODE_SINUS)
    {
        color.blue = MAIN_STATUS_LED_DIM;
        return color;
    }

    color.red = MAIN_STATUS_LED_DIM;
    return color;
#endif
}

static void MAIN_ServiceStatusLed(void)
{
    static uint32_t last_update_ms;
    static main_rgb_t previous_color = { 255u, 255u, 255u };
    const uint32_t tick_ms = HAL_GetTick();
    const main_rgb_t color = MAIN_GetStatusLedColor(tick_ms);

    if ((tick_ms - last_update_ms) < MAIN_STATUS_LED_UPDATE_MS)
    {
        return;
    }

    last_update_ms = tick_ms;

    if ((color.red == previous_color.red) &&
        (color.green == previous_color.green) &&
        (color.blue == previous_color.blue))
    {
        return;
    }

    previous_color = color;
    BOARD_SetStatusLedColor(color.red, color.green, color.blue);
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
    BOARD_InitBemfComparator();
    BOARD_InitBemfTiming();
    NVM_LoadOrDefault();
    MOTOR_Init();
    BOARD_InitControlTick();

    while (1)
    {
        MAIN_ServiceStatusLed();
    }
}
