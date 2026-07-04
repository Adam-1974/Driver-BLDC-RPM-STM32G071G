#include "board.h"

#include "app_config.h"
#include "stm32g0xx_ll_system.h"

#define BOARD_COUNTER_CLOCK_HZ          1000000u
#define BOARD_BEMF_TIMING_CLOCK_HZ      2000000u
#define BOARD_TIM_CCER_OUTPUT_MASK      (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                                         TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                                         TIM_CCER_CC3E | TIM_CCER_CC3NE)
#define BOARD_TIM_CCMR1_CH1_MASK        (TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE)
#define BOARD_TIM_CCMR1_CH2_MASK        (TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE)
#define BOARD_TIM_CCMR2_CH3_MASK        (TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE)
#define BOARD_TIM_CCMR2_CH4_MASK        (TIM_CCMR2_CC4S | TIM_CCMR2_OC4M | \
                                         TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE)
#define BOARD_TIM_CH1_PWM1              (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE)
#define BOARD_TIM_CH2_PWM1              (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE)
#define BOARD_TIM_CH3_PWM1              (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE)
#define BOARD_TIM_CH1_FORCE_INACTIVE    (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE)
#define BOARD_TIM_CH2_FORCE_INACTIVE    (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE)
#define BOARD_TIM_CH3_FORCE_INACTIVE    (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE)
#define BOARD_PWM_PERIOD_UPDATE_DEADBAND_TICKS 4u

static uint32_t s_pwm_timer_clock_hz;
static uint32_t s_pwm_carrier_hz;
static uint16_t s_pwm_period_ticks;

#if defined(__GNUC__)
#define BOARD_FAST_CODE                 __attribute__((optimize("O2")))
#else
#define BOARD_FAST_CODE
#endif

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

static BOARD_FAST_CODE uint16_t BOARD_LimitPwmTicks(uint16_t ticks)
{
    if (ticks > s_pwm_period_ticks)
    {
        return s_pwm_period_ticks;
    }

    return ticks;
}

static BOARD_FAST_CODE uint8_t BOARD_isPwmPeriodInsideUpdateDeadband(uint16_t period_ticks)
{
    uint16_t period_diff;

    if (s_pwm_period_ticks == 0u)
    {
        return 0u;
    }

    if (period_ticks > s_pwm_period_ticks)
    {
        period_diff = (uint16_t)(period_ticks - s_pwm_period_ticks);
    }
    else
    {
        period_diff = (uint16_t)(s_pwm_period_ticks - period_ticks);
    }

    if (period_diff <= BOARD_PWM_PERIOD_UPDATE_DEADBAND_TICKS)
    {
        return 1u;
    }

    return 0u;
}

static BOARD_FAST_CODE void BOARD_WriteLowSide(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    if (on != 0u)
    {
        port->BSRR = pin;
        return;
    }

    port->BSRR = ((uint32_t)pin << 16u);
}

static BOARD_FAST_CODE void BOARD_SetTimerChannelModes(uint8_t phase_a_pwm,
                                                       uint8_t phase_b_pwm,
                                                       uint8_t phase_c_pwm)
{
    uint32_t ccmr1 = TIM1->CCMR1;
    uint32_t ccmr2 = TIM1->CCMR2;

    ccmr1 &= ~(BOARD_TIM_CCMR1_CH1_MASK | BOARD_TIM_CCMR1_CH2_MASK);
    ccmr2 &= ~BOARD_TIM_CCMR2_CH3_MASK;

    if (phase_c_pwm != 0u)
    {
        ccmr1 |= BOARD_TIM_CH1_PWM1;
    }
    else
    {
        ccmr1 |= BOARD_TIM_CH1_FORCE_INACTIVE;
    }

    if (phase_b_pwm != 0u)
    {
        ccmr1 |= BOARD_TIM_CH2_PWM1;
    }
    else
    {
        ccmr1 |= BOARD_TIM_CH2_FORCE_INACTIVE;
    }

    if (phase_a_pwm != 0u)
    {
        ccmr2 |= BOARD_TIM_CH3_PWM1;
    }
    else
    {
        ccmr2 |= BOARD_TIM_CH3_FORCE_INACTIVE;
    }

    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = ccmr2;
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

    BOARD_AllPhasesOff();
}

void BOARD_InitPwmOutputs(void)
{
    GPIO_InitTypeDef gpio = { 0 };

    s_pwm_timer_clock_hz = BOARD_GetTimerClockHz();
    s_pwm_period_ticks = BOARD_CalcPeriodTicks(s_pwm_timer_clock_hz, DRIVER_PWM_CARRIER_HZ);
    s_pwm_carrier_hz = s_pwm_timer_clock_hz / ((uint32_t)s_pwm_period_ticks + 1u);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    LL_SYSCFG_EnablePinRemap(LL_SYSCFG_PIN_RMP_PA11 | LL_SYSCFG_PIN_RMP_PA12);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM1;
    gpio.Pin = BOARD_PIN_HA | BOARD_PIN_HB | BOARD_PIN_HC | BOARD_PIN_LC;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = BOARD_PIN_LA | BOARD_PIN_LB;
    HAL_GPIO_Init(GPIOB, &gpio);

    TIM1->CR1 = 0u;
    TIM1->CR2 = 0u;
    TIM1->SMCR = 0u;
    TIM1->DIER = 0u;
    TIM1->PSC = 0u;
    TIM1->ARR = s_pwm_period_ticks;
    TIM1->CCR1 = 0u;
    TIM1->CCR2 = 0u;
    TIM1->CCR3 = 0u;
    TIM1->CCR4 = 1u;
    TIM1->CNT = 0u;
    TIM1->CCMR1 = BOARD_TIM_CH1_FORCE_INACTIVE | BOARD_TIM_CH2_FORCE_INACTIVE;
    TIM1->CCMR2 = BOARD_TIM_CH3_FORCE_INACTIVE;
    TIM1->CCMR2 &= ~BOARD_TIM_CCMR2_CH4_MASK;
    TIM1->CCER = 0u;
    TIM1->BDTR = 0u;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void BOARD_InitBemfComparator(void)
{
    GPIO_InitTypeDef gpio = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = BOARD_PIN_BEMF_C | BOARD_PIN_BEMF_VGND;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = BOARD_PIN_BEMF_A | BOARD_PIN_BEMF_B_AM32;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = BOARD_PIN_BEMF_B;
    HAL_GPIO_Init(GPIOD, &gpio);

    if (LL_COMP_IsEnabled(BOARD_BEMF_COMP) != 0u)
    {
        LL_COMP_Disable(BOARD_BEMF_COMP);
    }

    LL_COMP_SetPowerMode(BOARD_BEMF_COMP, LL_COMP_POWERMODE_HIGHSPEED);
    LL_COMP_ConfigInputs(BOARD_BEMF_COMP, BOARD_BEMF_COMP_PHASE_C, BOARD_BEMF_COMP_PLUS);
    LL_COMP_SetInputHysteresis(BOARD_BEMF_COMP, LL_COMP_HYSTERESIS_LOW);
    LL_COMP_SetOutputPolarity(BOARD_BEMF_COMP, LL_COMP_OUTPUTPOL_NONINVERTED);
    LL_COMP_SetOutputBlankingSource(BOARD_BEMF_COMP, LL_COMP_BLANKINGSRC_NONE);

    LL_EXTI_DisableIT_0_31(BOARD_BEMF_EXTI_LINE);
    LL_EXTI_DisableRisingTrig_0_31(BOARD_BEMF_EXTI_LINE);
    LL_EXTI_DisableFallingTrig_0_31(BOARD_BEMF_EXTI_LINE);
    LL_EXTI_ClearRisingFlag_0_31(BOARD_BEMF_EXTI_LINE);
    LL_EXTI_ClearFallingFlag_0_31(BOARD_BEMF_EXTI_LINE);
    LL_EXTI_EnableIT_0_31(BOARD_BEMF_EXTI_LINE);

    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);

    LL_COMP_Enable(BOARD_BEMF_COMP);
}

void BOARD_InitBemfTiming(void)
{
    const uint32_t timer_clock_hz = BOARD_GetTimerClockHz();
    const uint16_t prescaler = BOARD_CalcPeriodTicks(timer_clock_hz, BOARD_BEMF_TIMING_CLOCK_HZ);

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

    TIM2->CR1 = 0u;
    TIM2->PSC = prescaler;
    TIM2->ARR = 0xFFFFu;
    TIM2->CNT = 0u;
    TIM2->SR = 0u;
    TIM2->DIER = 0u;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0u;
    TIM2->CR1 = TIM_CR1_CEN;

    TIM14->CR1 = 0u;
    TIM14->PSC = prescaler;
    TIM14->ARR = 0xFFFFu;
    TIM14->CNT = 0u;
    TIM14->SR = 0u;
    TIM14->DIER = 0u;
    TIM14->EGR = TIM_EGR_UG;
    TIM14->SR = 0u;

    HAL_NVIC_SetPriority(TIM14_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);

    TIM14->CR1 = TIM_CR1_CEN;
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
        BOARD_SetPwmBridgeEnabled(0u);
        BOARD_SetHighPwm(0u, 0u, 0u);
        BOARD_SetPwmOutputMask(0u, 0u, 0u, 0u, 0u, 0u);
    }

    BOARD_SetLowSideState(0u, 0u, 0u);
}

uint16_t BOARD_GetPwmPeriodTicks(void)
{
    return s_pwm_period_ticks;
}

uint32_t BOARD_GetPwmCarrierHz(void)
{
    return s_pwm_carrier_hz;
}

BOARD_FAST_CODE uint16_t BOARD_SetPwmCarrierHz(uint32_t carrier_hz)
{
    uint16_t period_ticks;

    if (carrier_hz == 0u)
    {
        carrier_hz = DRIVER_PWM_CARRIER_HZ;
    }

    if (s_pwm_timer_clock_hz == 0u)
    {
        s_pwm_timer_clock_hz = BOARD_GetTimerClockHz();
    }

    period_ticks = BOARD_CalcPeriodTicks(s_pwm_timer_clock_hz, carrier_hz);

    if (BOARD_isPwmPeriodInsideUpdateDeadband(period_ticks) != 0u)
    {
        return s_pwm_period_ticks;
    }

    s_pwm_period_ticks = period_ticks;
    s_pwm_carrier_hz = s_pwm_timer_clock_hz / ((uint32_t)period_ticks + 1u);

    TIM1->ARR = period_ticks;

    if (TIM1->CNT > period_ticks)
    {
        TIM1->CNT = 0u;
    }

    TIM1->CCR1 = BOARD_LimitPwmTicks((uint16_t)TIM1->CCR1);
    TIM1->CCR2 = BOARD_LimitPwmTicks((uint16_t)TIM1->CCR2);
    TIM1->CCR3 = BOARD_LimitPwmTicks((uint16_t)TIM1->CCR3);
    TIM1->CCR4 = BOARD_LimitPwmTicks((uint16_t)TIM1->CCR4);
    TIM1->EGR = TIM_EGR_UG;

    return s_pwm_period_ticks;
}

BOARD_FAST_CODE uint16_t BOARD_GetBemfIntervalTicks(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0u)
    {
        return 0xFFFFu;
    }

    return (uint16_t)TIM2->CNT;
}

BOARD_FAST_CODE void BOARD_ResetBemfIntervalTimer(void)
{
    TIM2->SR = 0u;
    TIM2->CNT = 0u;
}

BOARD_FAST_CODE void BOARD_SetBemfIntervalTicks(uint16_t ticks)
{
    TIM2->SR = 0u;
    TIM2->CNT = ticks;
}

BOARD_FAST_CODE void BOARD_ScheduleBemfCommutation(uint16_t delay_ticks)
{
    if (delay_ticks == 0u)
    {
        delay_ticks = 1u;
    }

    TIM14->DIER &= ~TIM_DIER_UIE;
    TIM14->CNT = 0u;
    TIM14->ARR = delay_ticks;
    TIM14->SR = 0u;
    TIM14->DIER |= TIM_DIER_UIE;
}

BOARD_FAST_CODE void BOARD_DisableBemfCommutationTimer(void)
{
    TIM14->DIER &= ~TIM_DIER_UIE;
    TIM14->SR = 0u;
}

BOARD_FAST_CODE void BOARD_SetBemfPwmSampleTicks(uint16_t ticks)
{
    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks >= s_pwm_period_ticks)
    {
        ticks = (uint16_t)(s_pwm_period_ticks - 1u);
    }

    TIM1->CCR4 = ticks;
}

BOARD_FAST_CODE void BOARD_EnableBemfPwmSampleIrq(uint8_t enabled)
{
    TIM1->SR &= ~(TIM_SR_CC4IF | TIM_SR_CC4OF);

    if (enabled != 0u)
    {
        TIM1->DIER |= TIM_DIER_CC4IE;
        return;
    }

    TIM1->DIER &= ~TIM_DIER_CC4IE;
}

BOARD_FAST_CODE void BOARD_SetPwmBridgeEnabled(uint8_t enabled)
{
    if (enabled != 0u)
    {
        if ((TIM1->BDTR & TIM_BDTR_MOE) == 0u)
        {
            TIM1->EGR = TIM_EGR_UG;
            TIM1->BDTR |= TIM_BDTR_MOE;
        }

        return;
    }

    TIM1->BDTR &= ~TIM_BDTR_MOE;
}

BOARD_FAST_CODE void BOARD_SetHighPwm(uint16_t phase_a_ticks,
                                      uint16_t phase_b_ticks,
                                      uint16_t phase_c_ticks)
{
    TIM1->CCR3 = BOARD_LimitPwmTicks(phase_a_ticks);
    TIM1->CCR2 = BOARD_LimitPwmTicks(phase_b_ticks);
    TIM1->CCR1 = BOARD_LimitPwmTicks(phase_c_ticks);
}

BOARD_FAST_CODE void BOARD_SetPwmOutputMask(uint8_t phase_a_high,
                                            uint8_t phase_a_low,
                                            uint8_t phase_b_high,
                                            uint8_t phase_b_low,
                                            uint8_t phase_c_high,
                                            uint8_t phase_c_low)
{
    uint32_t ccer = TIM1->CCER;

    ccer &= ~BOARD_TIM_CCER_OUTPUT_MASK;
    TIM1->CCER = ccer;

    BOARD_SetTimerChannelModes(phase_a_high, phase_b_high, phase_c_high);

    ccer |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    if (phase_c_low != 0u)
    {
        ccer |= TIM_CCER_CC1NE;
    }

    if (phase_b_low != 0u)
    {
        ccer |= TIM_CCER_CC2NE;
    }

    if (phase_a_low != 0u)
    {
        ccer |= TIM_CCER_CC3NE;
    }

    TIM1->CCER = ccer;
}

BOARD_FAST_CODE void BOARD_ApplySixStepBridge(uint8_t step,
                                              uint16_t duty_ticks,
                                              uint16_t bemf_sample_ticks)
{
    const uint16_t duty = BOARD_LimitPwmTicks(duty_ticks);
    const uint16_t bemf_sample = BOARD_LimitPwmTicks(bemf_sample_ticks);
    uint32_t ccmr1 = TIM1->CCMR1;
    uint32_t ccmr2 = TIM1->CCMR2;
    uint32_t ccer = TIM1->CCER;
    uint16_t ccr1 = 0u;
    uint16_t ccr2 = 0u;
    uint16_t ccr3 = 0u;

    ccmr1 &= ~(BOARD_TIM_CCMR1_CH1_MASK | BOARD_TIM_CCMR1_CH2_MASK);
    ccmr2 &= ~BOARD_TIM_CCMR2_CH3_MASK;
    ccer &= ~BOARD_TIM_CCER_OUTPUT_MASK;

    switch (step)
    {
    case 1u:
        ccr3 = duty;
        ccmr1 |= BOARD_TIM_CH1_FORCE_INACTIVE | BOARD_TIM_CH2_FORCE_INACTIVE;
        ccmr2 |= BOARD_TIM_CH3_PWM1;
        ccer |= TIM_CCER_CC2NE;
        break;

    case 2u:
        ccr1 = duty;
        ccmr1 |= BOARD_TIM_CH1_PWM1 | BOARD_TIM_CH2_FORCE_INACTIVE;
        ccmr2 |= BOARD_TIM_CH3_FORCE_INACTIVE;
        ccer |= TIM_CCER_CC2NE;
        break;

    case 3u:
        ccr1 = duty;
        ccmr1 |= BOARD_TIM_CH1_PWM1 | BOARD_TIM_CH2_FORCE_INACTIVE;
        ccmr2 |= BOARD_TIM_CH3_FORCE_INACTIVE;
        ccer |= TIM_CCER_CC3NE;
        break;

    case 4u:
        ccr2 = duty;
        ccmr1 |= BOARD_TIM_CH1_FORCE_INACTIVE | BOARD_TIM_CH2_PWM1;
        ccmr2 |= BOARD_TIM_CH3_FORCE_INACTIVE;
        ccer |= TIM_CCER_CC3NE;
        break;

    case 5u:
        ccr2 = duty;
        ccmr1 |= BOARD_TIM_CH1_FORCE_INACTIVE | BOARD_TIM_CH2_PWM1;
        ccmr2 |= BOARD_TIM_CH3_FORCE_INACTIVE;
        ccer |= TIM_CCER_CC1NE;
        break;

    case 6u:
        ccr3 = duty;
        ccmr1 |= BOARD_TIM_CH1_FORCE_INACTIVE | BOARD_TIM_CH2_FORCE_INACTIVE;
        ccmr2 |= BOARD_TIM_CH3_PWM1;
        ccer |= TIM_CCER_CC1NE;
        break;

    default:
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        return;
    }

    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;
    TIM1->CCR4 = bemf_sample;
    TIM1->CCER &= ~BOARD_TIM_CCER_OUTPUT_MASK;
    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = ccmr2;
    TIM1->CCER = ccer | TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;

    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0u)
    {
        TIM1->EGR = TIM_EGR_UG;
        TIM1->BDTR |= TIM_BDTR_MOE;
    }
}

BOARD_FAST_CODE void BOARD_SetLowSideState(uint8_t phase_a_on,
                                           uint8_t phase_b_on,
                                           uint8_t phase_c_on)
{
    BOARD_WriteLowSide(BOARD_PORT_LA, BOARD_PIN_LA, phase_a_on);
    BOARD_WriteLowSide(BOARD_PORT_LB, BOARD_PIN_LB, phase_b_on);
    BOARD_WriteLowSide(BOARD_PORT_LC, BOARD_PIN_LC, phase_c_on);
}
