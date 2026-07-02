/*
 * BEMF comparator handling for STM32G071.
 *
 * Derived from AM32:
 * https://github.com/am32-firmware/AM32
 * commit 32d7dd0aa6294f64e4355009c3bd4810ab01702f
 * files Mcu/g071/Src/comparator.c and Mcu/g071/Src/stm32g0xx_it.c
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "bemf_am32.h"

#include "board.h"

bemf_am32_state_t g_bemf_am32;

void BEMF_AM32_Init(volatile uint16_t *interval_counter,
                    volatile uint16_t *average_interval,
                    bemf_zero_cross_callback_t callback)
{
    g_bemf_am32.active_comp = BOARD_BEMF_COMP;
    g_bemf_am32.current_exti_line = BOARD_BEMF_EXTI_LINE;
    g_bemf_am32.medium_speed_set = 0u;
    g_bemf_am32.expected_rising = 0u;
    g_bemf_am32.interval_counter = interval_counter;
    g_bemf_am32.average_interval = average_interval;
    g_bemf_am32.zero_cross_callback = callback;
}

uint8_t BEMF_AM32_GetOutputLevel(void)
{
    return (uint8_t)((g_bemf_am32.active_comp->CSR >> 30u) & 1u);
}

void BEMF_AM32_MaskPhaseInterrupts(void)
{
    EXTI->IMR1 &= ~(1u << 18u);
    EXTI->RPR1 = LL_EXTI_LINE_18;
    EXTI->FPR1 = LL_EXTI_LINE_18;
}

void BEMF_AM32_EnableCompInterrupts(void)
{
    EXTI->IMR1 |= g_bemf_am32.current_exti_line;
}

void BEMF_AM32_ChangeCompInput(uint8_t step, uint8_t rising)
{
    const uint16_t average_interval = *g_bemf_am32.average_interval;

    g_bemf_am32.expected_rising = rising;

    if ((average_interval < 400u) && (g_bemf_am32.medium_speed_set != 0u))
    {
        LL_COMP_SetPowerMode(g_bemf_am32.active_comp, LL_COMP_POWERMODE_HIGHSPEED);
        g_bemf_am32.medium_speed_set = 0u;
    }

    if ((average_interval > 600u) && (g_bemf_am32.medium_speed_set == 0u))
    {
        LL_COMP_SetPowerMode(g_bemf_am32.active_comp, LL_COMP_POWERMODE_MEDIUMSPEED);
        g_bemf_am32.medium_speed_set = 1u;
    }

    if ((step == 1u) || (step == 4u))
    {
        LL_COMP_ConfigInputs(g_bemf_am32.active_comp, BOARD_BEMF_COMP_PHASE_C, BOARD_BEMF_COMP_PLUS);
    }

    if ((step == 2u) || (step == 5u))
    {
        LL_COMP_ConfigInputs(g_bemf_am32.active_comp, BOARD_BEMF_COMP_PHASE_A, BOARD_BEMF_COMP_PLUS);
    }

    if ((step == 3u) || (step == 6u))
    {
        LL_COMP_ConfigInputs(g_bemf_am32.active_comp, BOARD_BEMF_COMP_PHASE_B_AM32, BOARD_BEMF_COMP_PLUS);
    }

    if (rising != 0u)
    {
        LL_EXTI_DisableRisingTrig_0_31(LL_EXTI_LINE_18);
        LL_EXTI_EnableFallingTrig_0_31(g_bemf_am32.current_exti_line);
    }
    else
    {
        LL_EXTI_EnableRisingTrig_0_31(g_bemf_am32.current_exti_line);
        LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_18);
    }
}

static void BEMF_AM32_HandleEdge(uint8_t is_rising_flag)
{
    const uint16_t interval = *g_bemf_am32.interval_counter;
    const uint16_t average_interval = *g_bemf_am32.average_interval;

    if (interval > (average_interval >> 1u))
    {
        if (is_rising_flag != 0u)
        {
            LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_18);
        }
        else
        {
            LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_18);
        }

        if (g_bemf_am32.zero_cross_callback != 0)
        {
            g_bemf_am32.zero_cross_callback();
        }

        return;
    }

    if (BEMF_AM32_GetOutputLevel() == g_bemf_am32.expected_rising)
    {
        if (is_rising_flag != 0u)
        {
            LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_18);
        }
        else
        {
            LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_18);
        }
    }
}

void BEMF_AM32_IRQHandler(void)
{
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_18))
    {
        BEMF_AM32_HandleEdge(0u);
        return;
    }

    if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_18))
    {
        BEMF_AM32_HandleEdge(1u);
        return;
    }
}
