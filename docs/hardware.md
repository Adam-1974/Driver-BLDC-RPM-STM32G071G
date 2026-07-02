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
| PB7 | BEMF_A | COMP2_INM, analog |
| PB8 | LED WS2812B-2020 | GPIO output, pozniej TIM/DMA |
| PB15 | LA | GPIO output |
| PD1 | BEMF_B | analog, wymaga potwierdzenia komparatora |

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

Nasza lista pinow ma BEMF_B na PD1. W aktualnym naglowku LL dla STM32G071 uzywanym przez AM32 nie ma selektora komparatora dla PD1. To jest punkt do potwierdzenia przed layoutem lub przed pierwszym uruchomieniem:

- przeniesc BEMF_B na PB3, jezeli ma dzialac identycznie jak AM32 na COMP2,
- albo potwierdzic w dokumentacji STM32G071GBU6, ze PD1 ma alternatywne wejscie komparatora i dopisac selektor,
- albo zrobic dla PD1 wariant ADC/analog watchdog, co nie bedzie juz kopia AM32.

## ADC

PA5 jest skonfigurowane jako wejscie pomiaru pradu. Przeliczanie ADC -> mA jest parametrem NVM `adc_current_ma_per_count_q16`.

