# Hardware

## GPIO

| Pin | Funkcja | Konfiguracja startowa |
| --- | --- | --- |
| PA2 | BEMF_C | COMP2_INM, analog |
| PA3 | BEMF_Vgnd | COMP2_INP, analog |
| PA5 | current feedback | ADC1_IN5 |
| PA7 | LC | TIM1_CH1N PWM |
| PA8 | HC | TIM1_CH1 PWM |
| PA9 | HB | TIM1_CH2 PWM przez pad PA11 po `PA11_RMP` |
| PA10 | HA | TIM1_CH3 PWM przez pad PA12 po `PA12_RMP` |
| PB0 | LB | TIM1_CH2N PWM |
| PB1 / pin 15 | LA | TIM1_CH3N PWM |
| PD1 / pin 23 | BEMF_B | wejscie BEMF wg schematu; tor COMP do weryfikacji |
| PB3 | BEMF_B_AM32 | wejscie COMP2_INM uzywane przez kod AM32 dla fazy B |
| PB7 | BEMF_A | COMP2_INM, analog |
| PB8 | LED WS2812B-2020 | GPIO output, pozniej TIM/DMA |

## Timery

- TIM1: trzy pary komplementarne PWM faz `HC/LC`, `HB/LB`, `HA/LA`, 14 kHz.
- Dead-time nie jest generowany w STM32, bo realizuje go driver MOSFET.
- Dla obudow G071 z padami `PA11 [PA9]` i `PA12 [PA10]` program wlacza `LL_SYSCFG_PIN_RMP_PA11` i `LL_SYSCFG_PIN_RMP_PA12`, tak jak AM32.
- Mapa pinow faz pozostaje zgodna ze sprawdzona lista projektu; wariant obudowy decyduje o tym, ktory pad realizuje dana funkcje alternatywna.
- TIM6: bazowy tick sterowania 10 kHz.
- TIM2: licznik interwalu BEMF, 2 MHz, odpowiednik AM32 `INTERVAL_TIMER`.
- TIM14: jednorazowy timer opoznionej komutacji BEMF, 2 MHz, odpowiednik AM32 `COM_TIMER`.
- TIM6 nie wykonuje komutacji closed-loop po BEMF; w tym trybie robi tylko serwis/blanking/timeout, a krok wykonuje przerwanie TIM14.

## Uwaga: LA na fizycznym pinie 15

Na schemacie oraz we wczesnych opisach projektu wyjscie `LA` bylo opisane jako `PB15 / pin 15`. To oznaczenie okazalo sie mylace dla testowanej plytki. Przy konfiguracji `TIM1_CH3N` na `PB15` faza A miala poprawny PWM na gornej galezi `HA`, ale dolna galaz `LA` nie pracowala.

Test rozdzielil problem od komutacji i od konfiguracji pozostalych faz: `HC/LC` oraz `HB/LB` pracowaly poprawnie, a brak sygnalu wystepowal tylko na `LA`. Po skonfigurowaniu fizycznego pinu 15 jako `PB1 / TIM1_CH3N` sygnal komplementarny `LA` pojawil sie poprawnie i wszystkie trzy fazy zaczely pracowac.

Wniosek dla tej wersji plytki: fizyczny pin 15 nalezy obslugiwac w kodzie jako `PB1`, nie jako `PB15`. W pliku `Core/Inc/board.h` makro `BOARD_PIN_LA` ma pozostac ustawione na `GPIO_PIN_1` z portem `GPIOB`. Nie zmieniac tego z powrotem na `PB15` bez ponownego sprawdzenia wariantu obudowy STM32G071GBU6 i pomiaru oscyloskopem na rzeczywistym pinie.

## Komparator BEMF

AM32 dla STM32G071 uzywa COMP2 z wejsciem dodatnim `LL_COMP_INPUT_PLUS_IO3`, czyli PA3, oraz przelaczanymi wejsciami ujemnymi:

- `LL_COMP_INPUT_MINUS_IO2` - PB7,
- `LL_COMP_INPUT_MINUS_IO3` - PA2,
- `LL_COMP_INPUT_MINUS_IO1` - PB3.

Na schemacie BEMF_B jest podlaczony do pinu obudowy nr 23. Uzywamy mapy pinoutu zgodnej ze schematem, czyli wariantu `STM32G071GxUxN`, gdzie pin 23 jest opisany jako PD1.

Wniosek: BEMF_B w mapie projektu jest fizycznym pinem 23 / PD1. To nie jest ten sam tor komparatora co `PB3 / LL_COMP_INPUT_MINUS_IO1` z AM32. Aktualny kod inicjalizuje oba piny jako analogowe, ale walidacja BEMF przez COMP2 dla fazy B bedzie poprawna tylko wtedy, gdy rzeczywisty tor BEMF_B trafia na wejscie `PB3 / COMP2_INM`.

## ADC

PA5 jest skonfigurowane jako wejscie pomiaru pradu. Przeliczanie ADC -> mA jest parametrem NVM `adc_current_ma_per_count_q16`.

## LED statusu

PB8 steruje jedna dioda WS2812B-2020. W aktualnym etapie dioda jest obslugiwana programowo z petli glownej i sluzy tylko do diagnostyki:

- niebieski - praca w trybie SINUS,
- zolty - praca w trybie 6-step bez potwierdzonego BEMF,
- zielony - praca w trybie 6-step z potwierdzonym BEMF,
- czerwony - STOP lub stan inny niz aktywna praca,
- krotki bialy blysk - `BEMF OK`, czyli flaga `bemf_readable` jest ustawiona.

Ramka WS2812 ma 24 bity i na czas jej wysylania przerwania sa maskowane bardzo krotko. To jest rozwiazanie diagnostyczne; jezeli LED bedzie potrzebna przy wysokich obrotach albo z czestymi zmianami, nalezy przeniesc sterowanie WS2812 na timer/DMA.
