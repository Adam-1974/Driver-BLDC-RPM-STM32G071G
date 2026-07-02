#include "stm32g0xx_it.h"

#include "bemf_am32.h"
#include "motor_control.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void ADC1_COMP_IRQHandler(void)
{
    BEMF_AM32_IRQHandler();
}

void TIM6_DAC_LPTIM1_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0u)
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        MOTOR_ControlTick10kHz();
    }
}
