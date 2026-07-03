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

#if defined(__GNUC__)
#define BEMF_FAST_CODE                  __attribute__((optimize("O2")))
#else
#define BEMF_FAST_CODE
#endif

bemf_am32_state_t g_bemf_am32;

void BEMF_AM32_Init(volatile uint16_t *interval_counter,
                    volatile uint16_t *average_interval,
                    bemf_zero_cross_callback_t callback)
{
    g_bemf_am32.active_comp = BOARD_BEMF_COMP;
    g_bemf_am32.current_exti_line = BOARD_BEMF_EXTI_LINE;
    g_bemf_am32.medium_speed_set = 0u;
    g_bemf_am32.expected_rising = 0u;
    g_bemf_am32.filter_level = 5u;
    g_bemf_am32.interval_counter = interval_counter;
    g_bemf_am32.average_interval = average_interval;
    g_bemf_am32.zero_cross_callback = callback;
}

static BEMF_FAST_CODE uint8_t BEMF_AM32_ReadOutputLevel(COMP_TypeDef *comp)
{
    return (uint8_t)((comp->CSR >> 30u) & 1u);
}

BEMF_FAST_CODE uint8_t BEMF_AM32_GetOutputLevel(void)
{
    return BEMF_AM32_ReadOutputLevel(g_bemf_am32.active_comp);
}

BEMF_FAST_CODE void BEMF_AM32_MaskPhaseInterrupts(void)
{
    EXTI->IMR1 &= ~(1u << 18u);
    EXTI->RPR1 = LL_EXTI_LINE_18;
    EXTI->FPR1 = LL_EXTI_LINE_18;
}

BEMF_FAST_CODE void BEMF_AM32_EnableCompInterrupts(void)
{
    EXTI->IMR1 |= g_bemf_am32.current_exti_line;
}

BEMF_FAST_CODE void BEMF_AM32_ChangeCompInput(uint8_t step, uint8_t rising)
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

void BEMF_AM32_SetFilterLevel(uint8_t filter_level)
{
    if (filter_level == 0u)
    {
        filter_level = 1u;
    }

    if (filter_level > 16u)
    {
        filter_level = 16u;
    }

    g_bemf_am32.filter_level = filter_level;
}

static BEMF_FAST_CODE void BEMF_AM32_ClearEdgeFlag(uint8_t is_rising_flag)
{
    if (is_rising_flag != 0u)
    {
        LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_18);
        return;
    }

    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_18);
}

static BEMF_FAST_CODE uint8_t BEMF_AM32_HandleEdge(uint8_t is_rising_flag)
{
    COMP_TypeDef * const active_comp = g_bemf_am32.active_comp;
    const uint16_t interval = BOARD_GetBemfIntervalTicks();
    const uint16_t average_interval = *g_bemf_am32.average_interval;
    const uint8_t expected_level = g_bemf_am32.expected_rising;
    const uint8_t filter_level = g_bemf_am32.filter_level;
    uint8_t i;

    *g_bemf_am32.interval_counter = interval;

    if (interval > (average_interval >> 1u))
    {
        BEMF_AM32_ClearEdgeFlag(is_rising_flag);

        for (i = 0u; i < filter_level; i++)
        {
            if (BEMF_AM32_ReadOutputLevel(active_comp) == expected_level)
            {
                return 0u;
            }
        }

        return 1u;
    }

    if (BEMF_AM32_ReadOutputLevel(active_comp) == expected_level)
    {
        BEMF_AM32_ClearEdgeFlag(is_rising_flag);
    }

    return 0u;
}

BEMF_FAST_CODE uint8_t BEMF_AM32_ProcessIrq(void)
{
    if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_18))
    {
        return BEMF_AM32_HandleEdge(0u);
    }

    if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_18))
    {
        return BEMF_AM32_HandleEdge(1u);
    }

    return 0u;
}

BEMF_FAST_CODE void BEMF_AM32_IRQHandler(void)
{
    if (BEMF_AM32_ProcessIrq() != 0u)
    {
        if (g_bemf_am32.zero_cross_callback != 0)
        {
            g_bemf_am32.zero_cross_callback();
        }
    }
}
