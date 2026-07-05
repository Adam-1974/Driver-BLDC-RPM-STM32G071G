# Zalozenia projektu

## Procesor

STM32G071GBU6, rdzen Cortex-M0+, taktowanie startowe 64 MHz z HSI + PLL.

## Zegary

- `SystemClock_Config()` ustawia 64 MHz z HSI 16 MHz przez PLL.
- Po konfiguracji program sprawdza, czy SYSCLK/HCLK/PCLK1 maja 64 MHz.
- TIM1 PWM i TIM6 tick sterowania sa przeliczane z aktualnego zegara RCC odczytanego przez HAL, a nie ze stalej kompilacji.
- Celem jest unikniecie rozjazdu miedzy konfiguracja CubeIDE i reczna warstwa `board.c`.

## Tryby pracy

1. SINUS
   - Uzywana jest statyczna tabela sinus.
   - Timer generuje przerwania kolejnych krokow katowych.
   - Czas kroku wynika z RPM, liczby par biegunow i liczby probek tabeli.
   - PID pradu wylicza wspolczynnik wypelnienia PWM.
   - eRPM w tym trybie wynika z timera kroku.

2. 6-step
   - Faza plywajaca jest mierzona komparatorem wzgledem `BEMF_Vgnd`.
   - 6-step jest uruchamiany po przejsciu z sinusa.
   - Pierwsze sektory po przejsciu pracuja jako handoff open-loop z czasem sektora przejetym z sinusa.
   - Po potwierdzeniu BEMF sterownik przechodzi do closed-loop 6-step.
   - ZC BEMF wyznacza opoznienie do nastepnego kroku 6-step.
   - W closed-loop 6-step wypelnienie PWM ustawia PID RPM, a ogranicznik pradu zmniejsza PWM po przekroczeniu limitu [mA].
- Niezaleznie od regulatorow dziala twardy fault pradowy liczony z pomiaru ADC po filtrze IIR. To kompromis dla obecnej PCB, gdzie surowy tor ADC lapie zaklocenia komutacji.

## Przejscie trybow

- SINUS -> 6-step: po osiagnieciu `DRIVER_OPEN_LOOP_SINUS_RPM` sinus pracuje jeszcze przez `DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV` obrotow mechanicznych.
- Startowy sektor 6-step jest wyliczany z aktualnego kata elektrycznego sinusa.
- Handoff 6-step bierze czas sektora z ostatniej predkosci sinusa.
- Handoff 6-step bierze PWM z sinusa przeskalowany przez `DRIVER_SIN_TO_6STEP_PWM_SCALE_PERMILLE`.
- Po `DRIVER_SIN_TO_6STEP_BEMF_CONFIRM_ZC_COUNT` poprawnych ZC sterownik przechodzi do closed-loop 6-step.
- Docelowo SINUS -> 6-step: RPM powyzej progu NVM i czytelny BEMF.
- 6-step -> SINUS: RPM ponizej progu przejscia minus histereza NVM.

## PWM

- Startowo stala czestotliwosc nosna: 14 kHz.
- Docelowo w 6-step modulowana jest wysoka faza, dolna aktywna faza jest zapinana do GND, a faza niepracujaca ma gorna i dolna galaz off dla pomiaru BEMF.
- Limit wypelnienia w 6-step jest ustawiany przez `DRIVER_SIXSTEP_MAX_DUTY_PERMILLE`.
- W closed-loop 6-step wypelnienie PWM wynika z regulatora RPM i ogranicznika pradu.
- W 6-step nosna PWM moze byc dynamicznie dobrana przez `DRIVER_PWM_DYNAMIC_6STEP_ENABLE`, `DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR` i limit `DRIVER_PWM_6STEP_MAX_CARRIER_HZ`.
- W aktualnym etapie SINUS open-loop uzywane sa trzy pary komplementarne TIM1: `HC/LC`, `HB/LB`, `HA/LA`.
- TIM1 pracuje z preload dla CCR1/CCR2/CCR3, zeby nowe wypelnienia trzech faz byly przenoszone przez zdarzenie update PWM.
- Dead-time nie jest dodawany w STM32, bo realizuje go zewnetrzny driver MOSFET.
- Program wlacza remap `PA11/PA12`, zeby piny opisane jako `PA9/PA10` pracowaly na odpowiednich padach obudowy.
- Program konfiguruje `LA / TIM1_CH3N` jako `PB1`, bo test sprzetowy potwierdzil, ze fizyczny pin 15 na tej plytce nie pracuje poprawnie przy konfiguracji jako `PB15`.

## Etap 10 - pierwszy rozruch SINUS open-loop

- Sterownik po `MOTOR_Init()` startuje w trybie `MOTOR_MODE_SINUS`.
- Predkosc rozruchowa jest stala w programie: `DRIVER_OPEN_LOOP_SINUS_RPM`.
- Maksymalne wypelnienie PWM jest stale w programie: `DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE`.
- Kierunek jest na razie staly w programie: `DRIVER_OPEN_LOOP_DIRECTION`.
- `MOTOR_DIRECTION_CW` ma oznaczac fizyczny obrot CW zarowno w SINUS, jak i w 6-step. `MOTOR_DIRECTION_CCW` ma oznaczac fizyczny obrot CCW w obu trybach.
- Do testu przeciwnego kierunku nalezy zmienic `DRIVER_OPEN_LOOP_DIRECTION` z `MOTOR_DIRECTION_CW` na `MOTOR_DIRECTION_CCW` i ponownie wgrac firmware.
- Rozruch zaczyna sie od fazy align: pole stoi przez `DRIVER_OPEN_LOOP_ALIGN_MS`, zeby ustawic wirnik w znanym polozeniu.
- Po align predkosc rosnie od `DRIVER_OPEN_LOOP_START_RPM` do `DRIVER_OPEN_LOOP_SINUS_RPM` z rampa `DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC`.
- TIM6 daje tick 10 kHz, a pozycja elektryczna jest prowadzona akumulatorem Q16.
- Przesuniecia faz B i C sa liczone w akumulatorze Q16 jako 120/240 stopni elektrycznych wzgledem fazy A.
- W trybie SINUS wszystkie trzy polmostki pracuja komplementarnie: `HA/LA`, `HB/LB`, `HC/LC`.
- Kazda probka sinus jest mnozona przez limit `DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE`, czyli przebieg jest skalowany do maksymalnego wypelnienia, a nie ucinany od gory.
- Ten etap wykorzystuje trzy przesuniete fazowo kanaly PWM dla startu sinusoidalnego, bez fazy `LOW` i bez fazy `FLOAT`.
- Test sprzetowy potwierdzil dzialanie trzech par PWM po korekcie mapowania `LA`: fizyczny pin 15 pracuje jako `PB1 / TIM1_CH3N`.
- Po osiagnieciu zadanej predkosci i czasie stabilizacji sterownik przechodzi do 6-step handoff.
- PID pradu sinusa ustawia wypelnienie PWM od zera przy kazdym uruchomieniu sinusa.

## Etap 11 - 6-step handoff i closed-loop BEMF

- Warunek przejscia: `g_motor.ramped_target_rpm == DRIVER_OPEN_LOOP_SINUS_RPM` oraz uplyw `DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV` obrotow mechanicznych.
- Startowy sektor 6-step jest wyliczany z aktualnego kata elektrycznego sinusa przez dopasowanie najblizszego wektora 6-step.
- Kierunek `MOTOR_DIRECTION_CW` przechodzi po sektorach rosnaco 1 -> 6. `MOTOR_DIRECTION_CCW` przechodzi po sektorach malejaco 6 -> 1. Ta sama zmienna `DRIVER_OPEN_LOOP_DIRECTION` steruje kierunkiem w SINUS i w 6-step.
- Sekwencja sektorow 6-step:
  - 1: A PWM, B LOW, C FLOAT,
  - 2: C PWM, B LOW, A FLOAT,
  - 3: C PWM, A LOW, B FLOAT,
  - 4: B PWM, A LOW, C FLOAT,
  - 5: B PWM, C LOW, A FLOAT,
  - 6: A PWM, C LOW, B FLOAT.
- Wypelnienie startowe 6-step bierze aktualne PWM z sinusa przeskalowane przez `DRIVER_SIN_TO_6STEP_PWM_SCALE_PERMILLE`.
- Po kazdej komutacji BEMF ma blanking `DRIVER_SIXSTEP_BEMF_BLANK_TICKS`.
- `bemf_readable` ustawia sie po `DRIVER_SIN_TO_6STEP_BEMF_CONFIRM_ZC_COUNT` poprawnych ZC.
- Po potwierdzeniu BEMF sterownik wlacza closed-loop 6-step, resetuje PID RPM i ogranicznik pradu, a PWM startuje od `DRIVER_SIXSTEP_RPM_PID_START_DUTY_PERMILLE`.
- PID RPM reguluje wypelnienie PWM do `DRIVER_SIXSTEP_TARGET_RPM`.
- Dynamika wyjscia PID RPM ma osobne limity: wzrost przez `DRIVER_SIXSTEP_RPM_PID_MAX_RISE_PER_SEC_PERMILLE`, a spadek przez `DRIVER_SIXSTEP_RPM_PID_MAX_FALL_PER_SEC_PERMILLE`.
- Limity sa podane w promilach na sekunde i przeliczane na krok aktualizacji PID wedlug `DRIVER_SIXSTEP_RPM_PID_UPDATE_DIVIDER_TICKS`.
- Limiter dynamiki dziala za PID RPM: `sixstep_speed_pid_target_permille` pokazuje wartosc zadana przez PID, a `sixstep_speed_pid_output_permille` pokazuje wartosc po ograniczeniu rise/fall.
- Wewnatrz limiter pracuje w Q16 (`sixstep_speed_pid_output_q16`), wiec male wartosci promile/s nie gina przez zaokraglanie do calych promili na jedna aktualizacje.
- `sixstep_final_pwm_permille` pokazuje PWM po odjeciu ogranicznika pradu; jesli rozni sie od `sixstep_speed_pid_output_permille`, reakcje silnika maskuje ogranicznik pradowy.
- Ogranicznik pradu zmniejsza wypelnienie PWM, gdy `measured_current_ma` przekroczy `DRIVER_SIXSTEP_CURRENT_LIMIT_MA`.
- Twardy limit `DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA` nie jest regulatorem; szczegoly toru zabezpieczenia opisuje sekcja "Twarde zabezpieczenie pradowe po IIR ADC".
- W closed-loop ZC synchronizuje filtr okresu i miekko koryguje faze; fizyczna komutacja idzie z wirtualnego komutatora planowanego przez TIM14.
- Jesli w closed-loop zabraknie ZC, sterownik wykonuje do `DRIVER_SIXSTEP_BEMF_MISSING_ZC_VIRTUAL_STEPS` komutacji wirtualnych na ostatnim dobrym interwale.
- Po przekroczeniu limitu brakow ZC sterownik przechodzi do `MOTOR_SIXSTEP_PHASE_RECOVERY_OFF` na `DRIVER_SIXSTEP_BEMF_RECOVERY_OFF_TICKS`, a potem probuje ponownie zlapac BEMF.

## Twarde zabezpieczenie pradowe po IIR ADC

- Wejscie pradu to `PA4 / ADC1_IN4`, zgodnie z fizycznym pinem 10 ukladu.
- ADC1 pracuje w trybie ciaglym z DMA. `BOARD_ServiceCurrentAdc()` aktualizuje filtr IIR zdefiniowany przez `DRIVER_CURRENT_ADC_IIR_SHIFT`.
- Aktualna PCB ma zaklocony surowy tor pomiaru pradu, dlatego twardy fault nie uzywa juz `ADC1 AWD1` na raw ADC.
- Decyzja hard fault jest wykonywana w petli 10 kHz po aktualizacji `measured_current_ma`, czyli na wartosci po filtrze IIR.
- Prog fault ustawia `DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA`. Wartosc `0` wylacza ten fault programowy.
- Po przekroczeniu progu wywolywane jest `MOTOR_EnterHardCurrentFault()`: kasuje oczekujaca komutacje, maskuje BEMF, zatrzymuje timer komutacji TIM14, zeruje PWM i wykonuje `BOARD_AllPhasesOff()`.
- Fault jest zatrzaskiwany. Petla 10 kHz podtrzymuje stan OFF i czerwony status LED.
- `g_motor.hard_current_adc_threshold` trzyma przeliczony prog ADC dla diagnostyki, a `g_motor.current_adc_raw`, `g_motor.current_adc_filtered` i `g_motor.measured_current_ma` pokazuja tor pomiaru.
- Miekki ogranicznik `DRIVER_SIXSTEP_CURRENT_LIMIT_MA` nadal jest osobnym regulatorem PWM w 6-step C-L. Nie zastapuje twardego faultu po IIR.

## Etap 12 - probkowanie BEMF w oknie PWM

- Detekcja BEMF ma dwa rozdzielone okna:
  - okno komutacji ZC, czyli kiedy w sektorze wolno uznac ZC,
  - okno probkowania PWM, czyli kiedy poziom komparatora jest czytany bez szpilek od mostka.
- Okno PWM nie lapie zbocza EXTI. Okno PWM probkuje poziom komparatora.
- Stan bazowy `g_motor.bemf_pwm_last_level` jest ustawiany z teorii aktualnego kroku, a nie z odczytu komparatora.
- Pierwsza czysta probka PWM moze od razu wykryc, ze poziom komparatora jest juz po ZC.
- ZC z probkowania PWM jest uznane tylko wtedy, gdy:
  - probka rozni sie od `g_motor.bemf_pwm_last_level`,
  - probka jest zgodna z teoretycznym poziomem po ZC,
  - minelo wiecej niz `bemf_average_interval_ticks / 2`.
- Jezeli probka jest po ZC, ale czas jest zbyt krotki, stan bazowy nie jest zmieniany. Nastepna probka moze nadal zaliczyc ten sam ZC.
- Tryb wyboru okna PWM ustawia `DRIVER_BEMF_PWM_GATING_MODE`: `NONE`, `OFF_TIME`, `ON_TIME` albo `AUTO`.
- W trybie AUTO dziala histereza `DRIVER_BEMF_PWM_SAMPLE_ON_ENTER_PERMILLE` / `DRIVER_BEMF_PWM_SAMPLE_ON_EXIT_PERMILLE`.
- Marginesy okna PWM ustawiaja `DRIVER_BEMF_PWM_SAMPLE_EDGE_SETTLE_TICKS` i `DRIVER_BEMF_PWM_SAMPLE_CLOSE_MARGIN_TICKS`.
- Szczegolowy opis mechanizmu jest w `docs/am32_bemf_notes.md`.

## NVM

Wszystkie parametry trzymane sa w jednej globalnej strukturze. Kopia tej struktury ma byc zapisana we flash procesora. Jezeli znacznik `init_flash` nie jest ustawiony albo CRC/wersja nie pasuja, ladowane sa wartosci domyslne.

Dodana zmienna:

- `bemf_action_angle_offset_deg_x10` - wyprzedzenie katowe pomiedzy wykrytym zero-cross BEMF a wykonaniem kroku, w dziesiatych czesciach stopnia elektrycznego.

## LED

PB8 / WS2812B jest uzywana jako status poza szybkim torem komutacji: zielony oznacza SINUS albo 6-step C-L, czerwony oznacza twardy fault pradowy. W przerwaniach BEMF/TIM14 nie ma diagnostycznego migania fazami.
