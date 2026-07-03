#ifndef BEMF_AM32_H
#define BEMF_AM32_H

#include <stdint.h>

#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_comp.h"
#include "stm32g0xx_ll_exti.h"

typedef void (*bemf_zero_cross_callback_t)(void);

typedef struct
{
    COMP_TypeDef *active_comp;
    uint32_t current_exti_line;
    uint8_t medium_speed_set;
    uint8_t expected_rising;
    uint8_t filter_level;
    volatile uint16_t *interval_counter;
    volatile uint16_t *average_interval;
    bemf_zero_cross_callback_t zero_cross_callback;
} bemf_am32_state_t;

extern bemf_am32_state_t g_bemf_am32;

void BEMF_AM32_Init(volatile uint16_t *interval_counter,
                    volatile uint16_t *average_interval,
                    bemf_zero_cross_callback_t callback);
uint8_t BEMF_AM32_GetOutputLevel(void);
void BEMF_AM32_MaskPhaseInterrupts(void);
void BEMF_AM32_EnableCompInterrupts(void);
void BEMF_AM32_ChangeCompInput(uint8_t step, uint8_t rising);
void BEMF_AM32_SetFilterLevel(uint8_t filter_level);
void BEMF_AM32_IRQHandler(void);

#endif
