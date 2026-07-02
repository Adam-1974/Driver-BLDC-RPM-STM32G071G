# Hardware

## GPIO

| Pin | Funkcja | Konfiguracja startowa |
| --- | --- | --- |
| PA2 | BEMF_C | COMP2_INM, analog |
| PA3 | BEMF_Vgnd | COMP2_INP, analog |
| PA5 | current feedback | ADC1_IN5 |
| PA7 | LC | GPIO output |
| PA8 | HC | TIM1_CH1 PWM |
| PA9 | HB | TIM1_CH2 PWM |
| PA10 | HA | TIM1_CH3 PWM |
| PB0 | LB | GPIO output |
| PB3 | BEMF_B | COMP2_INM, analog |
| PB7 | BEMF_A | COMP2_INM, analog |
| PB8 | LED WS2812B-2020 | GPIO output, pozniej TIM/DMA |
| PB15 | LA | GPIO output |

## Timery

- TIM1: PWM wysokich faz, 14 kHz.
- TIM6: bazowy tick sterowania 10 kHz.
- Timer kroku sinus: do przypisania po wygenerowaniu projektu w CubeIDE.
- Timer opoznienia BEMF: do uzycia dla `bemf_action_angle_offset_deg_x10`.

## Komparator BEMF

AM32 dla STM32G071 uzywa COMP2 z wejsciem dodatnim `LL_COMP_INPUT_PLUS_IO3`, czyli PA3, oraz przelaczanymi wejsciami ujemnymi:

- `LL_COMP_INPUT_MINUS_IO2` - PB7,
- `LL_COMP_INPUT_MINUS_IO3` - PA2,
- `LL_COMP_INPUT_MINUS_IO1` - PB3.

Na schemacie BEMF_B jest podlaczony do pinu obudowy nr 23. Dla STM32G071GBU6, czyli wersji UFQFPN28 bez sufiksu `N`, pin 23 to PB3. Wariant z pinem 23 jako PD1 dotyczy wersji `STM32G071GxUxN`.

Wniosek: BEMF_B konfigurujemy jako PB3 / COMP2_INM, zgodnie z AM32.

## ADC

PA5 jest skonfigurowane jako wejscie pomiaru pradu. Przeliczanie ADC -> mA jest parametrem NVM `adc_current_ma_per_count_q16`.
