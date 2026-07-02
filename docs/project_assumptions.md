# Zalozenia projektu

## Procesor

STM32G071GBU6, rdzen Cortex-M0+, taktowanie startowe 64 MHz z HSI + PLL.

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
- Modulowana jest zawsze wysoka faza.
- Dolna faza aktywna jest zapinana do GND.
- Faza niepracujaca ma gorna i dolna galaz w stanie off, zeby mozna bylo mierzyc BEMF.

## NVM

Wszystkie parametry trzymane sa w jednej globalnej strukturze. Kopia tej struktury ma byc zapisana we flash procesora. Jezeli znacznik `init_flash` nie jest ustawiony albo CRC/wersja nie pasuja, ladowane sa wartosci domyslne.

Dodana zmienna:

- `bemf_action_angle_offset_deg_x10` - przesuniecie katowe pomiedzy wykrytym sygnalem BEMF a wykonaniem dzialania/kroku, w dziesiatych czesciach stopnia elektrycznego.

