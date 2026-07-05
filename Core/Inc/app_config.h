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
#define DRIVER_STATUS_LED_ENABLE        1u                 /* Wlacza diode statusu PB8 / WS2812B. */
#define DRIVER_PWM_CARRIER_HZ           14000u             /* Podstawowa czestotliwosc nosna PWM [Hz]. */
#define DRIVER_CURRENT_ADC_FULL_SCALE_MA 20000u            /* Prad odpowiadajacy napieciu pelnej skali czujnika [mA]. */
#define DRIVER_CURRENT_ADC_FULL_SCALE_MV 3000u             /* Napiecie ADC odpowiadajace pradowi pelnej skali [mV]. */
#define DRIVER_CURRENT_ADC_REFERENCE_MV 3300u              /* Napiecie odniesienia ADC uzywane do przeliczenia [mV]. */
#define DRIVER_CURRENT_ADC_MAX_VALUE    4095u              /* Maksymalny wynik ADC 12-bit. */
#define DRIVER_CURRENT_ADC_ZERO_OFFSET  0u                 /* Reczna korekta zera pomiaru pradu [ADC]. */
#define DRIVER_CURRENT_ADC_IIR_SHIFT    6u                 /* Filtr IIR pomiaru pradu: 4 oznacza stala okolo 1/16. */
#define DRIVER_CURRENT_ADC_STARTUP_SETTLE_MS 100u         /* Czas inicjalnego nasycenia filtra IIR ADC przy mostku OFF [ms]. */
#define DRIVER_CURRENT_ADC_MA_PER_ADC_Q16 \
    ((uint32_t)(((((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MA * \
                   DRIVER_CURRENT_ADC_REFERENCE_MV) << 16u) + \
                 (((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MV * \
                   DRIVER_CURRENT_ADC_MAX_VALUE) >> 1u)) / \
                ((uint64_t)DRIVER_CURRENT_ADC_FULL_SCALE_MV * \
                 DRIVER_CURRENT_ADC_MAX_VALUE)))            /* Przelicznik ADC -> mA w formacie Q16. */
#define DRIVER_EXPECTED_SYSCLK_HZ       64000000u          /* Oczekiwany zegar systemowy procesora [Hz]. */
#define DRIVER_NVM_FLASH_MARKER         0x424C4443u        /* Znacznik poprawnie zainicjalizowanej struktury NVM. */
#define DRIVER_NVM_STRUCT_VERSION       6u                 /* Wersja struktury NVM uzywana do zgodnosci danych. */

/* Parametry sinus */
#define DRIVER_OPEN_LOOP_SINUS_RPM      500               /* Docelowe mechaniczne RPM w trybie sinus. */
#define DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE 150u             /* Maksymalne wypelnienie PWM sinusa [promile]. */
#define DRIVER_SIN_CURRENT_TARGET_MA    3000u              /* Zadany prad pracy sinusa [mA]. */
#define DRIVER_SIN_CURRENT_PID_KP_ADC_Q16 537            /* Wzmocnienie P regulatora pradu sinusa dla bledu ADC w formacie Q16. */
#define DRIVER_SIN_CURRENT_PID_KI_ADC_Q16 53             /* Wzmocnienie I regulatora pradu sinusa dla bledu ADC w formacie Q16. */
#define DRIVER_SIN_CURRENT_PID_OUT_MIN_PERMILLE 0          /* Minimalne wyjscie PI pradu sinusa [promile]. */
#define DRIVER_SIN_CURRENT_PID_OUT_MAX_PERMILLE 1500       /* Maksymalne wyjscie PI pradu sinusa [promile]. */
#define DRIVER_SIN_CURRENT_CONTROL_DIVIDER_TICKS 1u        /* Co ile tickow aktualizowac regulator pradu sinusa. */
#define DRIVER_SIN_CURRENT_MAX_PWM_STEP_TICKS 16u          /* Maksymalna zmiana wypelnienia sinusa na jedna aktualizacje PI. */
#define DRIVER_OPEN_LOOP_START_RPM      100                /* Poczatkowe mechaniczne RPM rampy sinusa. */
#define DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC 3000u             /* Przyrost zadanych RPM sinusa w czasie rampy [rpm/s]. */
#define DRIVER_OPEN_LOOP_ALIGN_MS       200u               /* Czas ustawienia wirnika przed rampa sinusa [ms]. */
#define DRIVER_SIN_TABLE_STEPS          256u               /* Liczba probek w tablicy sinusa na pelny okres elektryczny. */
#define DRIVER_SIN_TABLE_MAX_VALUE      1049u              /* Maksymalna wartosc probki w tablicy sinusa. */
#define DRIVER_SIN_TO_6STEP_ENABLE      1u                 /* Wlacza automatyczne przejscie z sinusa do 6-step. */
#define DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV 1u             /* Liczba obrotow mechanicznych na ustabilizowanie sinusa przed 6-step. */
#define DRIVER_SIN_TO_6STEP_PWM_SCALE_PERMILLE 10u       /* Skala PWM przejmowanego z sinusa do 6-step [promile]. */
#define DRIVER_SIN_TO_6STEP_BEMF_CONFIRM_ZC_COUNT 3u       /* Liczba kolejnych ZC do potwierdzenia synchronizacji po przejsciu. */

/* Parametry 6-step */
#define DRIVER_SIXSTEP_TARGET_RPM      2500u			   /* Zadane mechaniczne RPM dla regulatora 6-step C-L. */
#define DRIVER_SIXSTEP_RPM_PID_START_DUTY_PERMILLE 30u     /* Startowe wypelnienie PWM po wejsciu w regulator RPM 6-step [promile]. */
#define DRIVER_SIXSTEP_RPM_PID_KP_Q16  10000               /* Wzmocnienie P regulatora RPM 6-step w formacie Q16. */
#define DRIVER_SIXSTEP_RPM_PID_KI_Q16  100                  /* Wzmocnienie I regulatora RPM 6-step w formacie Q16. */
#define DRIVER_SIXSTEP_RPM_PID_KD_Q16  0                  /* Wzmocnienie D regulatora RPM 6-step w formacie Q16. */
#define DRIVER_SIXSTEP_RPM_PID_UPDATE_DIVIDER_TICKS 10u    /* Co ile tickow 10kHz aktualizowac PID RPM 6-step. */
#define DRIVER_SIXSTEP_RPM_PID_MAX_RISE_PER_SEC_PERMILLE 5000u /* Maksymalny wzrost PWM PID RPM 6-step [promile/s]. */
#define DRIVER_SIXSTEP_RPM_PID_MAX_FALL_PER_SEC_PERMILLE 10000u /* Maksymalny spadek PWM PID RPM 6-step [promile/s]. */
#define DRIVER_SIXSTEP_CURRENT_LIMIT_MA 1000u              /* Maksymalny prad 6-step C-L [mA]. */
#define DRIVER_SIXSTEP_HARD_CURRENT_LIMIT_MA 20000u        /* Prog twardego faultu pradu liczony z pomiaru ADC po IIR [mA]. */
#define DRIVER_SIXSTEP_CURRENT_LIMIT_PID_KP_Q16 655        /* Wzmocnienie P ogranicznika pradu 6-step w formacie Q16. */
#define DRIVER_SIXSTEP_CURRENT_LIMIT_PID_KI_Q16 7          /* Wzmocnienie I ogranicznika pradu 6-step w formacie Q16. */
#define DRIVER_SIXSTEP_CURRENT_LIMIT_UPDATE_DIVIDER_TICKS 1u /* Co ile tickow 10kHz aktualizowac ogranicznik pradu 6-step. */
#define DRIVER_SIXSTEP_CURRENT_LIMIT_RELEASE_STEP_PERMILLE 1u /* Szybkosc odpuszczania ograniczenia pradu [promile/aktualizacje]. */
#define DRIVER_SIXSTEP_MAX_DUTY_PERMILLE 1000u             /* Maksymalne wypelnienie PWM dostepne w 6-step [promile]. */
#define DRIVER_PWM_DYNAMIC_6STEP_ENABLE 1u                 /* Wlacza dynamiczne dopasowanie nosnej PWM w 6-step. */
#define DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR 128u      /* Docelowa liczba impulsow PWM na sektor komutacji. */
#define DRIVER_PWM_6STEP_MIN_CARRIER_HZ DRIVER_PWM_CARRIER_HZ /* Minimalna nosna PWM w 6-step [Hz]. */
#define DRIVER_PWM_6STEP_MAX_CARRIER_HZ 128000u            /* Maksymalna nosna PWM w 6-step [Hz]. */
#define DRIVER_SIXSTEP_BEMF_BLANK_TICKS  2u                /* Czas blankingu komparatora po komutacji [ticki sterowania]. */
#define DRIVER_SIXSTEP_BEMF_POLL_COUNT   2u                /* Liczba probek potwierdzajacych stan komparatora. */
#define DRIVER_SIXSTEP_BEMF_INTERVAL_IIR_SHIFT 6u          /* Szybkosc filtra okresu ZC: 1=1/2, 2=1/4, 3=1/8 zmiany. */
#define DRIVER_SIXSTEP_BEMF_PHASE_SYNC_SHIFT 2u            /* Miekka korekta fazy komutatora z bledu ZC: 2 oznacza 1/4 bledu. */
#define DRIVER_SIXSTEP_BEMF_PHASE_SYNC_LIMIT_DIV 8u         /* Maksymalna korekta fazy na ZC: sektor / ta wartosc. */
#define DRIVER_SIXSTEP_BEMF_PAIR_AVERAGE_ENABLE 0u         /* Wlacza dodatkowe usrednianie dwoch ostatnich okresow ZC. */
#define DRIVER_BEMF_PWM_GATING_NONE      0u                /* Bez bramkowania PWM: komparator dziala wedlug okna komutacji. */
#define DRIVER_BEMF_PWM_GATING_OFF_TIME  1u                /* Sluchaj BEMF tylko w czystym stanie PWM low/off. */
#define DRIVER_BEMF_PWM_GATING_ON_TIME   2u                /* Sluchaj BEMF tylko w czystym stanie PWM high/on. */
#define DRIVER_BEMF_PWM_GATING_AUTO      3u                /* Automatycznie wybierz dluzszy stan PWM z histereza. */
#define DRIVER_BEMF_PWM_GATING_MODE DRIVER_BEMF_PWM_GATING_AUTO /* Tryb bramkowania komparatora wzgledem PWM. */
#define DRIVER_BEMF_PWM_SAMPLE_ON_ENTER_PERMILLE 550u      /* Prog wejscia AUTO w probkowanie on-time [promile PWM]. */
#define DRIVER_BEMF_PWM_SAMPLE_ON_EXIT_PERMILLE 450u       /* Prog powrotu AUTO do probkowania off-time [promile PWM]. */
#define DRIVER_BEMF_PWM_SAMPLE_EDGE_SETTLE_TICKS 32u       /* Opoznienie po zboczu PWM przed otwarciem komparatora [ticki TIM1]. */
#define DRIVER_BEMF_PWM_SAMPLE_CLOSE_MARGIN_TICKS 16u      /* Margines zamkniecia komparatora przed kolejnym zboczem [ticki TIM1]. */
#define DRIVER_SIXSTEP_BEMF_MISSING_ZC_VIRTUAL_STEPS 2u    /* Liczba kolejnych brakow ZC obslugiwanych wirtualna komutacja. */
#define DRIVER_SIXSTEP_BEMF_RECOVERY_OFF_TICKS 20u         /* Czas PWM off przed restartem 6-step po utracie BEMF [ticki 10kHz]. */
#define DRIVER_SIXSTEP_BEMF_MONITOR_ONLY 0u                /* Wlacza sam monitoring BEMF bez przejmowania komutacji. */
#define DRIVER_BEMF_PHASE_MAP_SHIFT      0u                /* Przesuniecie mapowania faz BEMF wzgledem krokow PWM. */

#endif
