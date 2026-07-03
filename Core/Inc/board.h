#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_comp.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_gpio.h"

#define BOARD_PIN_HA                    GPIO_PIN_10
#define BOARD_PORT_HA                   GPIOA
#define BOARD_PIN_HB                    GPIO_PIN_9
#define BOARD_PORT_HB                   GPIOA
#define BOARD_PIN_HC                    GPIO_PIN_8
#define BOARD_PORT_HC                   GPIOA

#define BOARD_PIN_LA                    GPIO_PIN_1
#define BOARD_PORT_LA                   GPIOB
#define BOARD_PIN_LB                    GPIO_PIN_0
#define BOARD_PORT_LB                   GPIOB
#define BOARD_PIN_LC                    GPIO_PIN_7
#define BOARD_PORT_LC                   GPIOA

#define BOARD_PIN_BEMF_A                GPIO_PIN_7
#define BOARD_PORT_BEMF_A               GPIOB
#define BOARD_PIN_BEMF_B_AM32           GPIO_PIN_3
#define BOARD_PORT_BEMF_B_AM32          GPIOB
#define BOARD_PIN_BEMF_B                GPIO_PIN_1
#define BOARD_PORT_BEMF_B               GPIOD
#define BOARD_PIN_BEMF_C                GPIO_PIN_2
#define BOARD_PORT_BEMF_C               GPIOA
#define BOARD_PIN_BEMF_VGND             GPIO_PIN_3
#define BOARD_PORT_BEMF_VGND            GPIOA

#define BOARD_PIN_CURRENT_FB            GPIO_PIN_5
#define BOARD_PORT_CURRENT_FB           GPIOA
#define BOARD_ADC_CURRENT_CHANNEL       ADC_CHANNEL_5

#define BOARD_PIN_WS2812                GPIO_PIN_8
#define BOARD_PORT_WS2812               GPIOB

#define BOARD_BEMF_COMP                 COMP2
#define BOARD_BEMF_EXTI_LINE            LL_EXTI_LINE_18
#define BOARD_BEMF_COMP_PLUS            LL_COMP_INPUT_PLUS_IO3
#define BOARD_BEMF_COMP_PHASE_A         LL_COMP_INPUT_MINUS_IO2
#define BOARD_BEMF_COMP_PHASE_C         LL_COMP_INPUT_MINUS_IO3
#define BOARD_BEMF_COMP_PHASE_B_AM32    LL_COMP_INPUT_MINUS_IO1

void BOARD_InitStaticOutputs(void);
void BOARD_InitPwmOutputs(void);
void BOARD_InitBemfComparator(void);
void BOARD_InitBemfTiming(void);
void BOARD_InitControlTick(void);
void BOARD_AllPhasesOff(void);
uint16_t BOARD_GetPwmPeriodTicks(void);
uint16_t BOARD_GetBemfIntervalTicks(void);
void BOARD_ResetBemfIntervalTimer(void);
void BOARD_SetBemfIntervalTicks(uint16_t ticks);
void BOARD_ScheduleBemfCommutation(uint16_t delay_ticks);
void BOARD_DisableBemfCommutationTimer(void);
void BOARD_SetBemfPwmSampleTicks(uint16_t ticks);
void BOARD_EnableBemfPwmSampleIrq(uint8_t enabled);
void BOARD_SetPwmBridgeEnabled(uint8_t enabled);
void BOARD_SetHighPwm(uint16_t phase_a_ticks, uint16_t phase_b_ticks, uint16_t phase_c_ticks);
void BOARD_SetPwmOutputMask(uint8_t phase_a_high,
                            uint8_t phase_a_low,
                            uint8_t phase_b_high,
                            uint8_t phase_b_low,
                            uint8_t phase_c_high,
                            uint8_t phase_c_low);
void BOARD_SetLowSideState(uint8_t phase_a_on, uint8_t phase_b_on, uint8_t phase_c_on);
void BOARD_SetStatusLedColor(uint8_t red, uint8_t green, uint8_t blue);

#endif
