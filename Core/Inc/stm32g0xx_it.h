#ifndef STM32G0XX_IT_H
#define STM32G0XX_IT_H

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void ADC1_COMP_IRQHandler(void);
void TIM6_DAC_LPTIM1_IRQHandler(void);

#endif

