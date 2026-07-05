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
- sekwencja closed-loop: detekcja ZC -> zapis `thiszctime` -> reset TIM2 -> TIM14 po `waitTime` -> komutacja,
- przelaczenie trybu pracy komparatora high speed / medium speed wedlug sredniego interwalu.

## Aktualny mechanizm detekcji BEMF

Aktualny kod rozdziela dwa niezalezne pojecia:

- okno komutacji ZC,
- okno probkowania PWM.

Okno komutacji ZC okresla, czy w danym sektorze elektrycznym wolno uznac zmiane
BEMF za prawdziwy zero-cross. To okno wynika z aktualnego kroku 6-step,
kierunku obrotow, sredniego czasu sektora `bemf_average_interval_ticks`,
blankingu po komutacji oraz minimalnego czasu `average_interval / 2`, tak jak w
torze AM32. Okno PWM nie decyduje o fizycznym wystapieniu zbocza. Okno PWM
wyznacza tylko bezpieczne chwile, w ktorych mozna odczytac czysty poziom
komparatora bez szpilek od przelaczania mostka.

W trybie `DRIVER_BEMF_PWM_GATING_MODE != DRIVER_BEMF_PWM_GATING_NONE` zbocze
komparatora EXTI nie jest podstawowym mechanizmem detekcji. Podstawowym
mechanizmem jest porownanie kolejnych probek poziomu komparatora pobranych w
czystych oknach PWM. EXTI pozostaje jako fallback, gdy bramkowanie PWM jest
wylaczone albo gdy okna PWM nie da sie wyliczyc dla aktualnego wypelnienia.

### Uzbrojenie po komutacji

Po wykonaniu kroku 6-step `MOTOR_ArmBemfForSixStep()`:

1. maskuje przerwania komparatora i zatrzymuje bramkowanie PWM,
2. aktualizuje przewidywany czas BEMF i poziom filtracji,
3. wybiera wejscie komparatora dla aktualnej fazy plywajacej,
4. wybiera oczekiwany kierunek przejscia poziomu BEMF dla aktualnego kierunku
   obrotow,
5. zeruje stan `bemf_edge_seen`,
6. uruchamia blanking `bemf_blank_ticks`.

Po zakonczeniu blankingu startuje `MOTOR_StartBemfPwmGating()`. W tym momencie
stan bazowy probkowania PWM nie jest brany z komparatora. Stan bazowy jest
wyznaczony z teorii komutacji dla nowego kroku:

- `MOTOR_GetBemfExpectedLevelBeforeZc()` - poziom oczekiwany przed ZC,
- `MOTOR_GetBemfExpectedLevelAfterZc()` - poziom oczekiwany po ZC.

Do `g_motor.bemf_pwm_last_level` wpisywany jest poziom teoretyczny przed ZC, a
`g_motor.bemf_pwm_sample_valid` ustawiane jest na `1`. Dzieki temu pierwsze
czyste okno PWM moze od razu wykryc ZC, jezeli rzeczywiste przejscie BEMF
nastapilo miedzy komutacja a pierwsza probka PWM.

### Wybor okna PWM

Okno PWM wybierane jest przez `DRIVER_BEMF_PWM_GATING_MODE`:

- `DRIVER_BEMF_PWM_GATING_NONE` - brak bramkowania PWM, dziala ciagly tor
  komparatora/EXTI w oknie komutacji,
- `DRIVER_BEMF_PWM_GATING_OFF_TIME` - probkowanie tylko w stanie PWM low/off,
- `DRIVER_BEMF_PWM_GATING_ON_TIME` - probkowanie tylko w stanie PWM high/on,
- `DRIVER_BEMF_PWM_GATING_AUTO` - automatyczny wybor dluzszego stanu PWM.

W trybie AUTO uzywana jest histereza:

- powyzej `DRIVER_BEMF_PWM_SAMPLE_ON_ENTER_PERMILLE` sterownik przechodzi na
  probkowanie on-time,
- ponizej `DRIVER_BEMF_PWM_SAMPLE_ON_EXIT_PERMILLE` sterownik wraca na
  probkowanie off-time.

Dla on-time okno liczone jest od poczatku okresu PWM:

- otwarcie: `DRIVER_BEMF_PWM_SAMPLE_EDGE_SETTLE_TICKS`,
- zamkniecie: `duty_ticks - DRIVER_BEMF_PWM_SAMPLE_CLOSE_MARGIN_TICKS`.

Dla off-time okno liczone jest po zakonczeniu impulsu PWM:

- otwarcie: `duty_ticks + DRIVER_BEMF_PWM_SAMPLE_EDGE_SETTLE_TICKS`,
- zamkniecie: `period_ticks - DRIVER_BEMF_PWM_SAMPLE_CLOSE_MARGIN_TICKS`.

Jezeli wybrane okno jest za krotkie, AUTO probuje drugiego stanu PWM. Jezeli
zadne okno nie jest poprawne, kod wraca do ciaglego toru komparatora/EXTI.

### Praca TIM1_CH4

TIM1_CH4 nie probuje zlapac zbocza BEMF. CH4 generuje dwa zdarzenia w okresie
PWM:

1. otwarcie czystego okna probkowania,
2. zamkniecie czystego okna probkowania.

Podczas otwarcia okna kod sprawdza, czy BEMF jest uzbrojony, blanking zakonczony
i czy nie czeka juz zaplanowana komutacja. Nastepnie odczytuje poziom
komparatora przez `BEMF_AM32_GetOutputLevel()` i porownuje go z
`g_motor.bemf_pwm_last_level`.

Zero-cross z probkowania PWM jest uznany tylko wtedy, gdy:

- aktualny poziom rozni sie od `g_motor.bemf_pwm_last_level`,
- aktualny poziom jest rowny teoretycznemu poziomowi po ZC,
- od ostatniej komutacji/ZC minelo wiecej niz `bemf_average_interval_ticks / 2`.

Jezeli poziom jest juz po ZC, ale czas jest jeszcze zbyt krotki, probka jest
ignorowana i `g_motor.bemf_pwm_last_level` nie jest zmieniany. To jest celowe:
pozniejsza probka w tym samym sektorze moze wtedy nadal zaliczyc ten sam ZC,
kiedy okno komutacji stanie sie dozwolone.

### Obsluga wykrytego ZC

Po poprawnym wykryciu ZC `MOTOR_HandleBemfZeroCross()`:

1. maskuje tor komparatora i zatrzymuje bramkowanie PWM,
2. pobiera czas z TIM2/BEMF,
3. zapisuje `bemf_last_zc_ticks` i `bemf_this_zc_ticks`,
4. resetuje licznik interwalu BEMF,
5. ustawia `bemf_edge_seen`,
6. resetuje licznik zgubionych ZC,
7. aktualizuje filtrowany interwal, eRPM i filtr AM32,
8. po wymaganej liczbie potwierdzen ustawia `bemf_readable`,
9. planuje kolejna komutacje przez TIM14.

TIM14 nie jest karmiony samym surowym czasem ZC. Po wykryciu ZC surowy pomiar
`bemf_this_zc_ticks` aktualizuje filtr IIR `bemf_average_interval_ticks`, a
opoznienie komutacji dostaje miekka korekte fazy wedlug bledu:

`bemf_this_zc_ticks - bemf_average_interval_ticks`.

Wczesniejsze ZC oznacza, ze wirnik jest przed wirtualnym komutatorem, wiec
korekta skraca oczekiwanie na komutacje. Pozniejsze ZC wydluza oczekiwanie.
Sila korekty jest dzielona przez `2^DRIVER_SIXSTEP_BEMF_PHASE_SYNC_SHIFT`, a
jej maksimum ogranicza `DRIVER_SIXSTEP_BEMF_PHASE_SYNC_LIMIT_DIV`. Komutator
zachowuje wlasny rytm z filtra IIR zamiast przesuwac krok dokladnie z kazdym
pojedynczym, zaszumionym ZC.

### Zgubione ZC

Jezeli ZC nie zostanie wykryte, `bemf_edge_seen` pozostaje `0`. W closed-loop
`MOTOR_ServiceBemfMissingZcClosedLoop()` porownuje czas TIM2 z limitem braku ZC.
Po przekroczeniu limitu sterownik wykonuje wirtualna komutacje przez
`MOTOR_RunVirtualBemfCommutation()`. Wirtualna komutacja:

- przesuwa krok 6-step dalej zgodnie z kierunkiem,
- ustawia `sixstep_virtual_zc_pending`,
- resetuje timer BEMF,
- ponownie uzbraja BEMF dla nowego kroku.

Poniewaz kazde nowe uzbrojenie ustawia `bemf_pwm_last_level` z teorii nowego
kroku, zgubione ZC w jednym sektorze nie przenosi starego stanu probkowania PWM
do nastepnego sektora. To ogranicza blad do jednego zgubionego sektora zamiast
zatruwac kolejne detekcje.

Jezeli zgubionych ZC jest wiecej niz
`DRIVER_SIXSTEP_BEMF_MISSING_ZC_VIRTUAL_STEPS`, sterownik przechodzi do fazy
recovery off (`MOTOR_SIXSTEP_PHASE_RECOVERY_OFF`), wylacza mostek na
`DRIVER_SIXSTEP_BEMF_RECOVERY_OFF_TICKS`, a potem probuje ponownie zlapac BEMF.

### Zmienne diagnostyczne

Do obserwacji w debuggerze przydatne sa:

- `g_motor.bemf_pwm_gating_active_window` - `0` oznacza off-time, `1` oznacza
  on-time,
- `g_motor.bemf_pwm_gating_open_ticks` - tick TIM1 otwarcia okna PWM,
- `g_motor.bemf_pwm_gating_close_ticks` - tick TIM1 zamkniecia okna PWM,
- `g_motor.bemf_pwm_gating_close_pending` - trwa otwarte okno i czekamy na
  zamkniecie,
- `g_motor.bemf_pwm_sample_valid` - stan bazowy probkowania PWM jest wazny,
- `g_motor.bemf_pwm_last_level` - ostatni/bazowy poziom dla porownania probek,
- `g_motor.bemf_edge_seen` - w tym sektorze wykryto ZC,
- `g_motor.sixstep_virtual_zc_pending` - ostatni krok byl komutacja wirtualna.

Roznica projektu:

- dodajemy regulowane opoznienie/przesuniecie wykonania kroku przez NVM `bemf_action_angle_offset_deg_x10`,
- podstawowa detekcja BEMF przy aktywnym `DRIVER_BEMF_PWM_GATING_MODE` probkuje poziom komparatora w czystym oknie PWM; tor komparatora/EXTI zostaje jako fallback dla trybu bez bramkowania PWM,
- BEMF_B w aktualnym schemacie jest na fizycznym pinie obudowy nr 23. Projekt przyjmuje wariant pinoutu `STM32G071GxUxN`, gdzie pin 23 jest opisany jako PD1.
- Tor AM32 dla fazy B zaklada `PB3 / LL_COMP_INPUT_MINUS_IO1`; aktualny kod inicjalizuje `PB3` jako analogowe wejscie COMP2 oraz `PD1` jako analog wg schematu, ale poprawny BEMF_B przez komparator wymaga fizycznego polaczenia sygnalu z torem `PB3 / COMP2_INM`.
- 6-step jest aktualnie uruchamiany jako handoff z sinusa; pierwsze sektory przejmuja timing i PWM z sinusa, a po potwierdzeniu ZC sterownik przechodzi do closed-loop BEMF.
