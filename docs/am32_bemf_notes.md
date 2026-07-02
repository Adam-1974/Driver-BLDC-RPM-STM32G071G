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
- przelaczenie trybu pracy komparatora high speed / medium speed wedlug sredniego interwalu.

Roznica projektu:

- dodajemy opoznienie/przesuniecie wykonania kroku przez NVM `bemf_action_angle_offset_deg_x10`,
- BEMF_B jest na pinie obudowy nr 23, czyli PB3 dla STM32G071GBU6 bez sufiksu `N`; pasuje to do `LL_COMP_INPUT_MINUS_IO1` uzywanego przez AM32.
