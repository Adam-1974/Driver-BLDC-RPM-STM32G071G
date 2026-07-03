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
   - Zbocze BEMF uruchamia przerwanie i przejscie do kolejnego kroku.
   - PID predkosci wylicza prad zadany, ograniczony limitem NVM.
   - Prad jest dalej pilnowany przez ograniczenie wypelnienia PWM.

## Przejscie trybow

- SINUS -> 6-step: RPM powyzej progu NVM i czytelny BEMF.
- 6-step -> SINUS: RPM ponizej progu przejscia minus histereza NVM.

## PWM

- Startowo stala czestotliwosc nosna: 14 kHz.
- Docelowo w 6-step modulowana jest wysoka faza, dolna aktywna faza jest zapinana do GND, a faza niepracujaca ma gorna i dolna galaz off dla pomiaru BEMF.
- W aktualnym etapie SINUS open-loop uzywane sa trzy kanaly PWM wysokich faz, zgodnie z podejsciem AM32 `allpwm`.
- TIM1 pracuje z preload dla CCR1/CCR2/CCR3, zeby nowe wypelnienia trzech faz byly przenoszone przez zdarzenie update PWM.

## Etap 10 - pierwszy rozruch SINUS open-loop

- Sterownik po `MOTOR_Init()` startuje w trybie `MOTOR_MODE_SINUS`.
- Predkosc rozruchowa jest stala w programie: `DRIVER_OPEN_LOOP_SINUS_RPM`.
- Maksymalne wypelnienie PWM jest stale w programie: `DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE`.
- Kierunek jest na razie staly: `MOTOR_DIRECTION_CW`.
- Rozruch zaczyna sie od fazy align: pole stoi przez `DRIVER_OPEN_LOOP_ALIGN_MS`, zeby ustawic wirnik w znanym polozeniu.
- Po align predkosc rosnie od `DRIVER_OPEN_LOOP_START_RPM` do `DRIVER_OPEN_LOOP_SINUS_RPM` z rampa `DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC`.
- TIM6 daje tick 10 kHz, a pozycja elektryczna jest prowadzona akumulatorem Q16.
- Przesuniecia faz B i C sa liczone w akumulatorze Q16 jako 120/240 stopni elektrycznych wzgledem fazy A.
- W trybie SINUS wszystkie trzy gorne fazy `HA/HB/HC` dostaja PWM z tabeli sinus, a `LA/LB/LC` pozostaja off.
- Kazda probka sinus jest mnozona przez limit `DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE`, czyli przebieg jest skalowany do maksymalnego wypelnienia, a nie ucinany od gory.
- Ten etap jest zgodny z podejsciem AM32 `stepper_sine/allpwm`: trzy przesuniete fazowo kanaly PWM dla startu sinusoidalnego.
- W tym etapie nie ma przejscia do 6-step, BEMF nie steruje komutacja, PID pradu jest odlozony na kolejny krok.

## NVM

Wszystkie parametry trzymane sa w jednej globalnej strukturze. Kopia tej struktury ma byc zapisana we flash procesora. Jezeli znacznik `init_flash` nie jest ustawiony albo CRC/wersja nie pasuja, ladowane sa wartosci domyslne.

Dodana zmienna:

- `bemf_action_angle_offset_deg_x10` - przesuniecie katowe pomiedzy wykrytym sygnalem BEMF a wykonaniem dzialania/kroku, w dziesiatych czesciach stopnia elektrycznego.
