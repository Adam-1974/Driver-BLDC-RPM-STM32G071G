#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

typedef enum
{
    MOTOR_DIRECTION_CW = 0,
    MOTOR_DIRECTION_CCW = 1
} motor_direction_t;

typedef enum
{
    MOTOR_MODE_STOP = 0,
    MOTOR_MODE_SINUS = 1,
    MOTOR_MODE_SIXSTEP = 2
} motor_mode_t;

/* Parametry ogolne */
#define DRIVER_OPEN_LOOP_DIRECTION      MOTOR_DIRECTION_CW /* Domyslny kierunek obrotow dla startu i testow. */
#define DRIVER_CONTROL_LOOP_HZ          10000u             /* Czestotliwosc glownej petli sterowania [Hz]. */
#define DRIVER_PWM_CARRIER_HZ           14000u             /* Podstawowa czestotliwosc nosna PWM [Hz]. */
#define DRIVER_CURRENT_ADC_FULL_SCALE_MA 20000u            /* Prad odpowiadajacy napieciu pelnej skali czujnika [mA]. */
#define DRIVER_CURRENT_ADC_FULL_SCALE_MV 3000u             /* Napiecie ADC odpowiadajace pradowi pelnej skali [mV]. */
#define DRIVER_CURRENT_ADC_REFERENCE_MV 3300u              /* Napiecie odniesienia ADC uzywane do przeliczenia [mV]. */
#define DRIVER_CURRENT_ADC_MAX_VALUE    4095u              /* Maksymalny wynik ADC 12-bit. */
#define DRIVER_CURRENT_ADC_ZERO_OFFSET  0u                 /* Reczna korekta zera pomiaru pradu [ADC]. */
#define DRIVER_CURRENT_ADC_IIR_SHIFT    6u                 /* Filtr IIR pomiaru pradu: 4 oznacza stala okolo 1/16. */
#define DRIVER_CURRENT_ADC_MA_PER_ADC_Q16 \
    ((uint32_t)(((((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MA * \
                   DRIVER_CURRENT_ADC_REFERENCE_MV) << 16u) + \
                 (((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MV * \
                   DRIVER_CURRENT_ADC_MAX_VALUE) >> 1u)) / \
                ((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MV * \
                 DRIVER_CURRENT_ADC_MAX_VALUE)))            /* Przelicznik ADC -> mA w formacie Q16. */
#define DRIVER_EXPECTED_SYSCLK_HZ       64000000u          /* Oczekiwany zegar systemowy procesora [Hz]. */
#define DRIVER_BEMF_LED_DIAGNOSTIC_ONLY 0u                 /* Wlacza tryb diagnostyki BEMF na LED zamiast pracy silnika. */
#define DRIVER_NVM_FLASH_MARKER         0x424C4443u        /* Znacznik poprawnie zainicjalizowanej struktury NVM. */
#define DRIVER_NVM_STRUCT_VERSION       6u                 /* Wersja struktury NVM uzywana do zgodnosci danych. */

/* Parametry sinus */
#define DRIVER_OPEN_LOOP_SINUS_RPM      1000               /* Docelowe mechaniczne RPM w trybie sinus. */
#define DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE 150u             /* Maksymalne wypelnienie PWM sinusa [promile]. */
#define DRIVER_SIN_CURRENT_TARGET_MA    3000u              /* Zadany prad pracy sinusa [mA]. */
#define DRIVER_SIN_CURRENT_PID_KP_ADC_Q16 537            /* Wzmocnienie P regulatora pradu sinusa dla bledu ADC w formacie Q16. */
#define DRIVER_SIN_CURRENT_PID_KI_ADC_Q16 53             /* Wzmocnienie I regulatora pradu sinusa dla bledu ADC w formacie Q16. */
#define DRIVER_SIN_CURRENT_PID_OUT_MIN_PERMILLE 0          /* Minimalne wyjscie PI pradu sinusa [promile]. */
#define DRIVER_SIN_CURRENT_PID_OUT_MAX_PERMILLE 1500       /* Maksymalne wyjscie PI pradu sinusa [promile]. */
#define DRIVER_SIN_CURRENT_CONTROL_DIVIDER_TICKS 1u        /* Co ile tickow aktualizowac regulator pradu sinusa. */
#define DRIVER_SIN_CURRENT_MAX_PWM_STEP_TICKS 16u          /* Maksymalna zmiana wypelnienia sinusa na jedna aktualizacje PI. */
#define DRIVER_OPEN_LOOP_START_RPM      100                /* Poczatkowe mechaniczne RPM rampy sinusa. */
#define DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC 200u             /* Przyrost zadanych RPM sinusa w czasie rampy [rpm/s]. */
#define DRIVER_OPEN_LOOP_ALIGN_MS       800u               /* Czas ustawienia wirnika przed rampa sinusa [ms]. */
#define DRIVER_SIN_TABLE_STEPS          256u               /* Liczba probek w tablicy sinusa na pelny okres elektryczny. */
#define DRIVER_SIN_TABLE_MAX_VALUE      1049u              /* Maksymalna wartosc probki w tablicy sinusa. */
#define DRIVER_SIN_TO_6STEP_ENABLE      0u                 /* Wlacza automatyczne przejscie z sinusa do 6-step. */
#define DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV 1u             /* Liczba obrotow mechanicznych na ustabilizowanie sinusa przed 6-step. */
#define DRIVER_OPEN_LOOP_6STEP_MAX_DUTY_PERMILLE 70u       /* Limit wypelnienia PWM po przejsciu z sinusa do 6-step [promile]. */

/* Parametry 6-step */
#define DRIVER_TEST_BLIND_6STEP_ONLY    0u                 /* Wlacza test startu bezposrednio w 6-step zamiast startu z sinusa. */
#define DRIVER_BLIND_6STEP_RPM          3000               /* Docelowe mechaniczne RPM w tescie 6-step open-loop. */
#define DRIVER_BLIND_6STEP_CLOSED_LOOP_DUTY_PERMILLE 700u  /* Wypelnienie PWM po zlapaniu closed-loop BEMF [promile]. */
#define DRIVER_BLIND_6STEP_BEMF_ADVANCE_DEG_X10 288u       /* Wyprzedzenie komutacji wzgledem ZC BEMF [0.1 stopnia]. */
#define DRIVER_BLIND_6STEP_DUTY_PERMILLE 300u              /* Wypelnienie PWM w rampie 6-step open-loop [promile]. */
#define DRIVER_BLIND_6STEP_START_RPM    300                /* Poczatkowe mechaniczne RPM rampy 6-step open-loop. */
#define DRIVER_BLIND_6STEP_RAMP_RPM_PER_SEC 5000u          /* Przyrost zadanych RPM w rampie 6-step open-loop [rpm/s]. */
#define DRIVER_BLIND_6STEP_KICK_DUTY_PERMILLE 350u         /* Wypelnienie PWM kopniecia przy starcie z postoju [promile]. */
#define DRIVER_BLIND_6STEP_ALIGN_MS     80u                /* Czas ustawienia wirnika przed kopnieciem 6-step [ms]. */
#define DRIVER_BLIND_6STEP_ALIGN_DUTY_PERMILLE 120u        /* Wypelnienie PWM podczas ustawiania wirnika 6-step [promile]. */
#define DRIVER_BLIND_6STEP_ZERO_BEMF_TIMEOUT_MS 80u        /* Maksymalny czas czekania na pierwszy BEMF po kopnieciu [ms]. */
#define DRIVER_BLIND_6STEP_HANDOFF_DUTY_PERMILLE 350u      /* Wypelnienie PWM przy przekazywaniu z open-loop do BEMF [promile]. */
#define DRIVER_BLIND_6STEP_CLOSED_LOOP_HANDOFF_STEPS 6u    /* Liczba krokow ochronnych po wejsciu w closed-loop. */
#define DRIVER_BLIND_6STEP_START_STEP   1u                 /* Pierwszy krok komutacji uzywany przy starcie 6-step. */
#define DRIVER_BLIND_6STEP_COAST_HANDOFF_MIN_RPM 1000      /* Minimalne mechaniczne RPM do nasluchu BEMF przed closed-loop. */
#define DRIVER_PWM_DYNAMIC_6STEP_ENABLE 1u                 /* Wlacza dynamiczne dopasowanie nosnej PWM w 6-step. */
#define DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR 12u      /* Docelowa liczba impulsow PWM na sektor komutacji. */
#define DRIVER_PWM_6STEP_MIN_CARRIER_HZ DRIVER_PWM_CARRIER_HZ /* Minimalna nosna PWM w 6-step [Hz]. */
#define DRIVER_PWM_6STEP_MAX_CARRIER_HZ 128000u            /* Maksymalna nosna PWM w 6-step [Hz]. */
#define DRIVER_SIXSTEP_SPEED_CONTROL_DIVIDER_TICKS 10u     /* Dzielnik aktualizacji regulatora predkosci 6-step. */
#define DRIVER_SIXSTEP_SPEED_KP_ERPM_PER_PWM_TICK 96u      /* Ile eRPM bledu odpowiada jednemu krokowi PWM. */
#define DRIVER_SIXSTEP_SPEED_MAX_STEP_TICKS 8u             /* Maksymalna jednorazowa zmiana PWM regulatora 6-step [ticki]. */
#define DRIVER_SIXSTEP_CLOSED_LOOP_MIN_DUTY_PERMILLE 0u    /* Minimalne wypelnienie PWM w closed-loop 6-step [promile]. */
#define DRIVER_SIXSTEP_BEMF_ARM_DELAY_MS 1200u             /* Opoznienie uzbrojenia BEMF od startu 6-step [ms]. */
#define DRIVER_SIXSTEP_BEMF_ARM_AFTER_STEPS 12u            /* Minimalna liczba krokow przed uzbrojeniem BEMF. */
#define DRIVER_SIXSTEP_BEMF_ARM_AFTER_MECH_REV 2u          /* Minimalna liczba obrotow mechanicznych przed uzbrojeniem BEMF. */
#define DRIVER_SIXSTEP_BEMF_OK_COUNT     6u                /* Liczba poprawnych ZC potrzebna do uznania BEMF za czytelny. */
#define DRIVER_SIXSTEP_BEMF_LOCK_EXTRA_COUNT 0u            /* Dodatkowe ZC wymagane przed wejsciem w closed-loop. */
#define DRIVER_SIXSTEP_BEMF_BLANK_TICKS  2u                /* Czas blankingu komparatora po komutacji [ticki sterowania]. */
#define DRIVER_SIXSTEP_BEMF_POLL_COUNT   2u                /* Liczba probek potwierdzajacych stan komparatora. */
#define DRIVER_SIXSTEP_BEMF_POLLING_CHANGEOVER_TICKS 2000u /* Prog czasu sektora dla zmiany sposobu probkowania BEMF. */
#define DRIVER_SIXSTEP_BEMF_MISSED_ZC_BLIND_STEPS 1u       /* Liczba slepych krokow po zgubionym ZC przed relockiem. */
#define DRIVER_SIXSTEP_BEMF_MISSED_ZC_NEXT_TIMEOUT_PERCENT 125u /* Limit czekania na nastepny ZC po slepym kroku [%]. */
#define DRIVER_SIXSTEP_BEMF_RELOCK_LISTEN_SECTORS 2u       /* Ile sektorow sluchac BEMF podczas relocku. */
#define DRIVER_SIXSTEP_BEMF_RELOCK_MIN_TIMEOUT_TICKS 120u  /* Minimalny timeout nasluchu relocku [ticki BEMF]. */
#define DRIVER_SIXSTEP_BEMF_RELOCK_MAX_TIMEOUT_TICKS 4000u /* Maksymalny timeout nasluchu relocku [ticki BEMF]. */
#define DRIVER_SIXSTEP_BEMF_COAST_WAIT_SYNC_COUNT 2u       /* Liczba ZC do synchronizacji po okresie coast. */
#define DRIVER_SIXSTEP_BEMF_MONITOR_ONLY 0u                /* Wlacza sam monitoring BEMF bez przejmowania komutacji. */
#define DRIVER_BEMF_PHASE_MAP_SHIFT      0u                /* Przesuniecie mapowania faz BEMF wzgledem krokow PWM. */

#endif
