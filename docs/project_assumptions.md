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
   - Aktualny etap testowy startuje bezposrednio w 6-step open-loop, a po potwierdzeniu BEMF przechodzi na komutacje wyzwalana BEMF.
   - Zbocze BEMF po locku wyznacza opoznienie do nastepnego kroku 6-step.
   - PID predkosci wylicza prad zadany, ograniczony limitem NVM.
   - Prad jest dalej pilnowany przez ograniczenie wypelnienia PWM.

## Przejscie trybow

- Aktualny etap testowy ma wlaczony `DRIVER_TEST_BLIND_6STEP_ONLY`, wiec sinus i przejscie SINUS -> 6-step nie sa uzywane w biezacym tescie.
- Najpierw 6-step pracuje open-loop ze stalym RPM i PWM, potem po poprawnych zboczach BEMF sterownik przechodzi na komutacje z BEMF.
- Docelowo SINUS -> 6-step: RPM powyzej progu NVM i czytelny BEMF.
- 6-step -> SINUS: RPM ponizej progu przejscia minus histereza NVM.

## PWM

- Startowo stala czestotliwosc nosna: 14 kHz.
- Docelowo w 6-step modulowana jest wysoka faza, dolna aktywna faza jest zapinana do GND, a faza niepracujaca ma gorna i dolna galaz off dla pomiaru BEMF.
- Limit testowy wypelnienia w 6-step jest osobny: `DRIVER_OPEN_LOOP_6STEP_MAX_DUTY_PERMILLE`.
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
- Po osiagnieciu zadanej predkosci sterownik przechodzi do testowego 6-step open-loop.
- PID pradu jest odlozony na kolejny krok.

## Etap 11 - 6-step open-loop z przejsciem na BEMF

- Aktualny build moze byc przelaczony w tryb pasywnej diagnostyki BEMF przez `DRIVER_BEMF_LED_DIAGNOSTIC_ONLY`.
- W tym trybie mostek jest wylaczony, silnik nie startuje, a program probkuje komparatorem fazy A/B/C podczas recznego krecenia silnikiem.
- WS2812 pokazuje ostatnia wykryta zmiane BEMF: faza A = czerwony, faza B = zielony, faza C = niebieski.
- Aktualnie wlaczony jest tryb diagnostyczny `DRIVER_TEST_BLIND_6STEP_ONLY`.
- W tym trybie sterownik startuje od razu w 6-step open-loop, bez sinusa i bez przejscia SINUS -> 6-step.
- Parametry testu sa stale w programie:
  - `DRIVER_BLIND_6STEP_RPM` - predkosc komutacji open-loop,
  - `DRIVER_BLIND_6STEP_DUTY_PERMILLE` - stale wypelnienie PWM; wartosc `100` oznacza 10%,
  - `DRIVER_BLIND_6STEP_START_STEP` - sektor startowy 6-step.
- BEMF nie jest uzbrajany od razu. Sterownik najpierw odczekuje `DRIVER_SIXSTEP_BEMF_ARM_DELAY_MS`, `DRIVER_SIXSTEP_BEMF_ARM_AFTER_MECH_REV` i `DRIVER_SIXSTEP_BEMF_ARM_AFTER_STEPS`.
- Po uzbrojeniu BEMF kolejne poprawne zbocza sa zliczane. `bemf_readable` ustawia sie dopiero po `DRIVER_SIXSTEP_BEMF_OK_COUNT` zboczach i po zobaczeniu wszystkich trzech faz plywajacych.
- Po ustawieniu `bemf_readable` sterownik przestaje komutowac z samego timera open-loop. Zbocze BEMF zapisuje interwal TIM2, zeruje TIM2 i ustawia TIM14 na `waitTime`.
- `waitTime` jest liczony jak w AM32: okolo polowy interwalu sektora minus `bemf_action_angle_offset_deg_x10`.
- TIM2 i TIM14 dla BEMF pracuja z zegarem 2 MHz, czyli jedna jednostka czasu ma 0,5 us jak w AM32 dla STM32G071.
- Po przerwaniu TIM14 wykonywany jest kolejny krok 6-step, a sredni interwal BEMF jest aktualizowany z `lastzctime/thiszctime` dla nastepnego sektora.
- Jesli po locku nie pojawi sie kolejne zbocze BEMF w oczekiwanym czasie, sterownik kasuje lock i wraca do komutacji open-loop.
- Dla tego testu wyjscia high sa zawsze sterowane przez TIM1: tylko aktywna faza ma tryb PWM, a pozostale kanaly high sa ustawiane w tryb forced inactive. To eliminuje przypadek, w ktorym nieaktywne wejscia high drivera moglyby plywac albo zachowywac poprzedni PWM.

## Etap 12 - przejscie SINUS -> 6-step open-loop

- Ten etap zostaje odlozony do czasu potwierdzenia, ze 6-step i BEMF dzialaja poprawnie oddzielnie.
- Warunek przejscia: `g_motor.ramped_target_rpm == DRIVER_OPEN_LOOP_SINUS_RPM`.
- Po osiagnieciu zadanej predkosci sinus musi jeszcze pracowac przez `DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV` obrotow mechanicznych. To zabezpiecza przypadek testowy, gdy `DRIVER_OPEN_LOOP_START_RPM == DRIVER_OPEN_LOOP_SINUS_RPM` i rampa formalnie konczy sie natychmiast po align.
- Startowy sektor 6-step jest wyliczany z aktualnego kata elektrycznego sinusa przez dopasowanie najblizszego wektora 6-step. Dla kata sinusa `0 deg` najblizszy jest sektor 5, nie sektor 1.
- Kierunek `MOTOR_DIRECTION_CW` przechodzi po sektorach rosnaco 1 -> 6. `MOTOR_DIRECTION_CCW` przechodzi po sektorach malejaco 6 -> 1. Ta sama zmienna `DRIVER_OPEN_LOOP_DIRECTION` steruje kierunkiem w SINUS i w 6-step.
- Sekwencja sektorow jest zgodna z AM32 `comStep()`:
  - 1: A PWM, B LOW, C FLOAT,
  - 2: C PWM, B LOW, A FLOAT,
  - 3: C PWM, A LOW, B FLOAT,
  - 4: B PWM, A LOW, C FLOAT,
  - 5: B PWM, C LOW, A FLOAT,
  - 6: A PWM, C LOW, B FLOAT.
- Wypelnienie startowe 6-step bierze limit sinusa i ogranicza go przez `DRIVER_OPEN_LOOP_6STEP_MAX_DUTY_PERMILLE`.
- Czas sektora jest liczony z ostatniej predkosci sinusa, liczby par biegunow i 6 sektorow na obrot elektryczny.
- Po wejsciu w 6-step BEMF nie jest uzbrajany od razu. Sterownik czeka jednoczesnie na:
  - minimalny czas pracy 6-step `DRIVER_SIXSTEP_BEMF_ARM_DELAY_MS`,
  - minimalna liczbe obrotow mechanicznych `DRIVER_SIXSTEP_BEMF_ARM_AFTER_MECH_REV`,
  - minimalna liczbe sektorow `DRIVER_SIXSTEP_BEMF_ARM_AFTER_STEPS`.
- Dopiero po spelnieniu tych warunkow wlaczane sa przerwania komparatora BEMF. Ma to odfiltrowac impulsy od samego przelaczenia mostka i zbyt wczesne potwierdzenie BEMF.
- Po kazdej komutacji BEMF jest maskowany przez `DRIVER_SIXSTEP_BEMF_BLANK_TICKS`.
- `bemf_readable` ustawia sie po `DRIVER_SIXSTEP_BEMF_OK_COUNT` poprawnych zboczach w oknie czasowym sektora i dopiero gdy zbocza wystapily na wszystkich trzech fazach plywajacych.
- W biezacym tescie standalone BEMF po potwierdzeniu wykonuje juz komutacje. W scenariuszu SINUS -> 6-step ta sama mechanika bedzie podlaczona pozniej.

## NVM

Wszystkie parametry trzymane sa w jednej globalnej strukturze. Kopia tej struktury ma byc zapisana we flash procesora. Jezeli znacznik `init_flash` nie jest ustawiony albo CRC/wersja nie pasuja, ladowane sa wartosci domyslne.

Dodana zmienna:

- `bemf_action_angle_offset_deg_x10` - wyprzedzenie katowe pomiedzy wykrytym zero-cross BEMF a wykonaniem kroku, w dziesiatych czesciach stopnia elektrycznego. Na etapie testowym default wraca do `0`, bo wymuszone 15 stopni powodowalo natychmiastowa utrate synchronizacji.

## Diagnostyka LED

PB8 / WS2812B pokazuje aktualny stan regulatora:

- niebieski - `MOTOR_MODE_SINUS`,
- zolty - `MOTOR_MODE_SIXSTEP` bez potwierdzonego BEMF,
- zielony - `MOTOR_MODE_SIXSTEP` z `BEMF OK`,
- czerwony - `MOTOR_MODE_STOP` lub stan bez aktywnej pracy,
- bialy blysk - krotki znacznik aktywnego `BEMF OK`.

LED nie jest elementem regulacji. Aktualizacja odbywa sie z petli glownej, a stan jest wyliczany na podstawie `g_motor.mode` oraz `g_motor.bemf_readable`.
