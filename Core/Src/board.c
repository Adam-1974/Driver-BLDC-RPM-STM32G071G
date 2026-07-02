#include "board.h"

void BOARD_InitStaticOutputs(void)
{
    GPIO_InitTypeDef gpio = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio.Pin = BOARD_PIN_LA;
    HAL_GPIO_Init(BOARD_PORT_LA, &gpio);

    gpio.Pin = BOARD_PIN_LB;
    HAL_GPIO_Init(BOARD_PORT_LB, &gpio);

    gpio.Pin = BOARD_PIN_LC;
    HAL_GPIO_Init(BOARD_PORT_LC, &gpio);

    gpio.Pin = BOARD_PIN_WS2812;
    HAL_GPIO_Init(BOARD_PORT_WS2812, &gpio);

    BOARD_AllPhasesOff();
}

void BOARD_AllPhasesOff(void)
{
    HAL_GPIO_WritePin(BOARD_PORT_LA, BOARD_PIN_LA, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_PORT_LB, BOARD_PIN_LB, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_PORT_LC, BOARD_PIN_LC, GPIO_PIN_RESET);
}

