#include "stm32g0xx_it.h"

#include "bemf_am32.h"
#include "motor_control.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"

#if defined(__GNUC__)
#define IT_FAST_CODE                    __attribute__((optimize("O2")))
#else
#define IT_FAST_CODE
#endif

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

IT_FAST_CODE void ADC1_COMP_IRQHandler(void)
{
    if (BEMF_AM32_ProcessIrq() != 0u)
    {
        MOTOR_BemfZeroCrossIrq();
    }
}

void TIM6_DAC_LPTIM1_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0u)
    {
        LL_TIM_ClearFlag_UPDATE(TIM6);
        MOTOR_ControlTick10kHz();
    }
}

IT_FAST_CODE void TIM1_CC_IRQHandler(void)
{
    if (((TIM1->SR & TIM_SR_CC4IF) != 0u) &&
        ((TIM1->DIER & TIM_DIER_CC4IE) != 0u))
    {
        TIM1->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);
        MOTOR_BemfPwmSampleIrq();
    }
}

IT_FAST_CODE void TIM14_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM14) != 0u)
    {
        LL_TIM_ClearFlag_UPDATE(TIM14);
        MOTOR_BemfCommutationTimerIrq();
    }
}
