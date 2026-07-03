#include "board.h"

#include "app_config.h"

#define BOARD_COUNTER_CLOCK_HZ          1000000u

static uint16_t s_pwm_period_ticks;

static uint32_t BOARD_GetTimerClockHz(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE) != RCC_HCLK_DIV1)
    {
        timer_clock_hz *= 2u;
    }

    return timer_clock_hz;
}

static uint16_t BOARD_CalcPeriodTicks(uint32_t timer_clock_hz, uint32_t target_hz)
{
    uint32_t period_ticks;

    if ((timer_clock_hz == 0u) || (target_hz == 0u))
    {
        return 0u;
    }

    period_ticks = (timer_clock_hz + (target_hz / 2u)) / target_hz;

    if (period_ticks == 0u)
    {
        period_ticks = 1u;
    }

    if (period_ticks > 65536u)
    {
        period_ticks = 65536u;
    }

    return (uint16_t)(period_ticks - 1u);
}

static uint16_t BOARD_LimitPwmTicks(uint16_t ticks)
{
    if (ticks > s_pwm_period_ticks)
    {
        return s_pwm_period_ticks;
    }

    return ticks;
}

static void BOARD_WriteLowSide(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    if (on != 0u)
    {
        port->BSRR = pin;
        return;
    }

    port->BSRR = ((uint32_t)pin << 16u);
}

void BOARD_InitStaticOutputs(void)
{
    GPIO_InitTypeDef gpio = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

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

void BOARD_InitPwmOutputs(void)
{
    GPIO_InitTypeDef gpio = { 0 };
    const uint32_t timer_clock_hz = BOARD_GetTimerClockHz();

    s_pwm_period_ticks = BOARD_CalcPeriodTicks(timer_clock_hz, DRIVER_PWM_CARRIER_HZ);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM1;
    gpio.Pin = BOARD_PIN_HA | BOARD_PIN_HB | BOARD_PIN_HC;
    HAL_GPIO_Init(GPIOA, &gpio);

    TIM1->CR1 = 0u;
    TIM1->CR2 = 0u;
    TIM1->SMCR = 0u;
    TIM1->DIER = 0u;
    TIM1->PSC = 0u;
    TIM1->ARR = s_pwm_period_ticks;
    TIM1->CCR1 = 0u;
    TIM1->CCR2 = 0u;
    TIM1->CCR3 = 0u;
    TIM1->CNT = 0u;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE |
                  TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;
    TIM1->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE;
    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void BOARD_InitControlTick(void)
{
    const uint32_t timer_clock_hz = BOARD_GetTimerClockHz();
    const uint16_t prescaler = BOARD_CalcPeriodTicks(timer_clock_hz, BOARD_COUNTER_CLOCK_HZ);
    const uint32_t counter_clock_hz = timer_clock_hz / ((uint32_t)prescaler + 1u);
    const uint16_t period = BOARD_CalcPeriodTicks(counter_clock_hz, DRIVER_CONTROL_LOOP_HZ);

    __HAL_RCC_TIM6_CLK_ENABLE();

    TIM6->CR1 = 0u;
    TIM6->PSC = prescaler;
    TIM6->ARR = period;
    TIM6->CNT = 0u;
    TIM6->SR = 0u;
    TIM6->DIER = TIM_DIER_UIE;
    TIM6->EGR = TIM_EGR_UG;
    TIM6->SR = 0u;

    HAL_NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 1u, 0u);
    HAL_NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);

    TIM6->CR1 = TIM_CR1_CEN;
}

void BOARD_AllPhasesOff(void)
{
    if (__HAL_RCC_TIM1_IS_CLK_ENABLED() != 0u)
    {
        BOARD_SetHighPwm(0u, 0u, 0u);
    }

    BOARD_SetLowSideState(0u, 0u, 0u);
}

uint16_t BOARD_GetPwmPeriodTicks(void)
{
    return s_pwm_period_ticks;
}

void BOARD_SetHighPwm(uint16_t phase_a_ticks, uint16_t phase_b_ticks, uint16_t phase_c_ticks)
{
    TIM1->CCR3 = BOARD_LimitPwmTicks(phase_a_ticks);
    TIM1->CCR2 = BOARD_LimitPwmTicks(phase_b_ticks);
    TIM1->CCR1 = BOARD_LimitPwmTicks(phase_c_ticks);
}

void BOARD_SetLowSideState(uint8_t phase_a_on, uint8_t phase_b_on, uint8_t phase_c_on)
{
    BOARD_WriteLowSide(BOARD_PORT_LA, BOARD_PIN_LA, phase_a_on);
    BOARD_WriteLowSide(BOARD_PORT_LB, BOARD_PIN_LB, phase_b_on);
    BOARD_WriteLowSide(BOARD_PORT_LC, BOARD_PIN_LC, phase_c_on);
}
