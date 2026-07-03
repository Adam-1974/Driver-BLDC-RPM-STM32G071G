# AM32 BEMF notes

Kod BEMF w `Core/Src/bemf_am32.c` jest przeniesiony/adaptowany z AM32:

- repo: https://github.com/am32-firmware/AM32
- commit referencyjny: `32d7dd0aa6294f64e4355009c3bd4810ab01702f`
- pliki:
  - `Mcu/g071/Src/comparator.c`
  - `Mcu/g071/Src/stm32g0xx_it.c`
  - `Inc/targets.h`

Zachowane mechanizmy:

- dynamiczny wybor fazy plywajacej wedlug kroku 1..6,
- wybor zbocza rosnacego/opadajacego wedlug kierunku oczekiwanego BEMF,
- maskowanie EXTI przed zmiana wejsc komparatora,
- filtrowanie zbyt wczesnych zboczy przez porownanie z polowa `average_interval`,
- filtr programowy z AM32: po zboczu kilka kolejnych odczytow komparatora musi potwierdzic prawdziwy zero-cross,
- TIM2 jako `INTERVAL_TIMER` BEMF z zegarem 2 MHz, zgodnie z AM32 dla G071,
- TIM14 jako `COM_TIMER` planujacy opozniona komutacje po zero-cross,
- sekwencja closed-loop: EXTI BEMF -> zapis `thiszctime` -> reset TIM2 -> TIM14 po `waitTime` -> komutacja,
- przelaczenie trybu pracy komparatora high speed / medium speed wedlug sredniego interwalu.

Roznica projektu:

- dodajemy opoznienie/przesuniecie wykonania kroku przez NVM `bemf_action_angle_offset_deg_x10`,
- domyslne wyprzedzenie BEMF na etapie testowym ustawione jest na `0`; proba z 15 stopniami elektrycznymi powodowala natychmiastowa utrate synchronizacji,
- BEMF_B w aktualnym schemacie jest na fizycznym pinie obudowy nr 23. Projekt przyjmuje wariant pinoutu `STM32G071GxUxN`, gdzie pin 23 jest opisany jako PD1.
- Tor AM32 dla fazy B zaklada `PB3 / LL_COMP_INPUT_MINUS_IO1`; aktualny kod inicjalizuje `PB3` jako analogowe wejscie COMP2 oraz `PD1` jako analog wg schematu, ale poprawny BEMF_B przez komparator wymaga fizycznego polaczenia sygnalu z torem `PB3 / COMP2_INM`.
- W aktualnym etapie testujemy 6-step i BEMF oddzielnie od sinusa: start jest open-loop, a po potwierdzeniu BEMF zero-cross planuje kolejna komutacje przez TIM14.
- Przejscie SINUS -> 6-step zostaje celowo odlozone do czasu potwierdzenia stabilnej pracy 6-step z BEMF.
