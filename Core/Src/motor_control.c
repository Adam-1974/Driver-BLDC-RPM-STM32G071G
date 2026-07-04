#include "motor_control.h"

#include "bemf_am32.h"
#include "board.h"
#include "nvm_config.h"
#include "sinus_table.h"

#define MOTOR_SIXSTEP_STEP_COUNT        6u
#define MOTOR_BEMF_TIMER_HZ             2000000u
#define MOTOR_BEMF_START_INTERVAL_TICKS 10000u
#define MOTOR_BEMF_START_COUNTER_TICKS  5000u
#define MOTOR_BEMF_FILTER_MIN_TICKS     100u
#define MOTOR_BEMF_FILTER_MAX_TICKS     500u
#define MOTOR_BEMF_PWM_OFF_SAMPLE_PERCENT 80u
#define MOTOR_BEMF_SLOW_UPDATE_DIVIDER  16u
#define MOTOR_BEMF_ADVANCE_RECIP_Q16    109u
#define MOTOR_BEMF_DIRECT_COMMUTATION_TICKS 2u
#define MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS 800u
#define MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS 1u
#define MOTOR_BEMF_FAST_FILTER_LEVEL    4u
#define MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS 320u
#define MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS 0u
#define MOTOR_BEMF_ULTRA_FAST_FILTER_LEVEL 2u
#define MOTOR_BEMF_ERPM_NUMERATOR       ((MOTOR_BEMF_TIMER_HZ * 60u) / MOTOR_SIXSTEP_STEP_COUNT)
#define MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT 3u
#define MOTOR_BEMF_FALLBACK_DECAY_NUMERATOR 33u
#define MOTOR_BEMF_FALLBACK_DECAY_DENOMINATOR 32u
#define MOTOR_BEMF_RELOCK_SCAN_STEP_LIMIT 12u
#define MOTOR_PWM_PULSES_MAX_DISPLAY    255u
#define MOTOR_BEMF_ALL_PHASES_MASK      (MOTOR_BEMF_PHASE_A_MASK | \
                                         MOTOR_BEMF_PHASE_B_MASK | \
                                         MOTOR_BEMF_PHASE_C_MASK)

#if defined(__GNUC__)
#define MOTOR_FAST_CODE                 __attribute__((optimize("O2")))
#else
#define MOTOR_FAST_CODE
#endif

volatile motor_control_state_t g_motor;

static MOTOR_FAST_CODE void MOTOR_HandleBemfZeroCross(void);
static MOTOR_FAST_CODE void MOTOR_EnableBemfCompIrqFromPwmSample(void);
static MOTOR_FAST_CODE void MOTOR_CommitBemfCommutation(void);
static MOTOR_FAST_CODE void MOTOR_ApplySixStepBridge(void);
static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStepIndex(void);
static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStep(void);
static MOTOR_FAST_CODE void MOTOR_ArmBemfForSixStep(void);
static MOTOR_FAST_CODE void MOTOR_StartBemfRelockCoast(void);
static MOTOR_FAST_CODE void MOTOR_StartBemfHandoffCoast(void);
static uint32_t MOTOR_GetSixStepBemfArmDelayTicks(void);
static MOTOR_FAST_CODE void MOTOR_ServiceBlindSixStepOpenLoopSpeedRamp(void);
static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBlindSixStepZeroStart(void);

static uint32_t MOTOR_GetAbsRpm(int32_t rpm)
{
    if (rpm < 0)
    {
        return (uint32_t)(-rpm);
    }

    return (uint32_t)rpm;
}

static uint16_t MOTOR_GetPolePairs(void)
{
    if (g_driver_config.pole_pairs > 0u)
    {
        return g_driver_config.pole_pairs;
    }

    return 1u;
}

static uint16_t MOTOR_GetSinusSample(uint16_t index)
{
    return g_sinus_table[index % DRIVER_SIN_TABLE_STEPS];
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBlindSixStepTestMode(void)
{
#if DRIVER_TEST_BLIND_6STEP_ONLY
    return 1u;
#else
    return 0u;
#endif
}

static uint8_t MOTOR_isBemfLedDiagnosticMode(void)
{
#if DRIVER_BEMF_LED_DIAGNOSTIC_ONLY
    return 1u;
#else
    return 0u;
#endif
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfMonitorOnlyMode(void)
{
#if DRIVER_SIXSTEP_BEMF_MONITOR_ONLY
    return 1u;
#else
    return 0u;
#endif
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfPwmGatedMode(void)
{
    if (g_motor.bemf_average_interval_ticks > DRIVER_SIXSTEP_BEMF_POLLING_CHANGEOVER_TICKS)
    {
        return 1u;
    }

    return 0u;
}

static uint16_t MOTOR_GetPwmTicksFromPermille(uint32_t duty_permille)
{
    if (duty_permille > 1000u)
    {
        duty_permille = 1000u;
    }

    return (uint16_t)(((uint32_t)BOARD_GetPwmPeriodTicks() * duty_permille) / 1000u);
}

static uint16_t MOTOR_ScaleSinusDuty(uint16_t sample)
{
    return (uint16_t)(((uint32_t)sample * g_motor.sinus_pwm_limit_ticks) /
                      DRIVER_SIN_TABLE_MAX_VALUE);
}

static void MOTOR_UpdateSinusPwmLimit(void)
{
    g_motor.sinus_pwm_limit_ticks =
        MOTOR_GetPwmTicksFromPermille(DRIVER_OPEN_LOOP_MAX_DUTY_PERMILLE);
}

static void MOTOR_UpdateSixStepPwmLimit(void)
{
    g_motor.sixstep_pwm_limit_ticks =
        MOTOR_GetPwmTicksFromPermille(DRIVER_OPEN_LOOP_6STEP_MAX_DUTY_PERMILLE);
}

static uint32_t MOTOR_GetBlindSixStepOpenLoopDutyPermille(void)
{
    if (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_ALIGN)
    {
        return DRIVER_BLIND_6STEP_ALIGN_DUTY_PERMILLE;
    }

    if (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT)
    {
        return DRIVER_BLIND_6STEP_KICK_DUTY_PERMILLE;
    }

    const uint32_t start_duty = DRIVER_BLIND_6STEP_DUTY_PERMILLE;
    const uint32_t handoff_duty = DRIVER_BLIND_6STEP_HANDOFF_DUTY_PERMILLE;
    const uint32_t ramp_ticks = MOTOR_GetSixStepBemfArmDelayTicks();
    const uint32_t run_ticks = g_motor.sixstep_run_ticks;
    uint32_t duty_delta;
    uint32_t ramped_delta;

    if ((ramp_ticks <= 1u) || (run_ticks >= ramp_ticks))
    {
        return handoff_duty;
    }

    if (handoff_duty >= start_duty)
    {
        duty_delta = handoff_duty - start_duty;
        ramped_delta = (uint32_t)((((uint64_t)duty_delta * run_ticks) +
                                   (ramp_ticks >> 1u)) / ramp_ticks);
        return start_duty + ramped_delta;
    }

    duty_delta = start_duty - handoff_duty;
    ramped_delta = (uint32_t)((((uint64_t)duty_delta * run_ticks) +
                               (ramp_ticks >> 1u)) / ramp_ticks);

    return start_duty - ramped_delta;
}

static uint8_t MOTOR_UpdateBlindSixStepPwm(void)
{
    const uint16_t next_pwm_ticks =
        MOTOR_GetPwmTicksFromPermille(MOTOR_GetBlindSixStepOpenLoopDutyPermille());
    const uint8_t changed =
        ((g_motor.sixstep_pwm_limit_ticks != next_pwm_ticks) ||
         (g_motor.sixstep_pwm_ticks != next_pwm_ticks)) ? 1u : 0u;

    g_motor.sixstep_pwm_limit_ticks = next_pwm_ticks;
    g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;

    return changed;
}

static uint32_t MOTOR_GetBlindSixStepAlignTicks(void)
{
    const uint64_t ticks = ((uint64_t)DRIVER_BLIND_6STEP_ALIGN_MS *
                            DRIVER_CONTROL_LOOP_HZ) / 1000u;

    if (ticks == 0u)
    {
        return 1u;
    }

    if (ticks > 0xFFFFFFFFu)
    {
        return 0xFFFFFFFFu;
    }

    return (uint32_t)ticks;
}

static uint32_t MOTOR_GetBlindSixStepZeroBemfTimeoutTicks(void)
{
    const uint64_t ticks = ((uint64_t)DRIVER_BLIND_6STEP_ZERO_BEMF_TIMEOUT_MS *
                            DRIVER_CONTROL_LOOP_HZ) / 1000u;

    if (ticks == 0u)
    {
        return 1u;
    }

    if (ticks > 0xFFFFFFFFu)
    {
        return 0xFFFFFFFFu;
    }

    return (uint32_t)ticks;
}

static void MOTOR_UpdateBlindSixStepClosedLoopPwm(void)
{
    g_motor.sixstep_pwm_ticks =
        MOTOR_GetPwmTicksFromPermille(DRIVER_BLIND_6STEP_CLOSED_LOOP_DUTY_PERMILLE);
}

static uint16_t MOTOR_RescalePwmTicks(uint16_t ticks, uint16_t old_period, uint16_t new_period)
{
    uint32_t scaled_ticks;

    if (old_period == 0u)
    {
        return ticks;
    }

    scaled_ticks = (((uint32_t)ticks * new_period) + (old_period >> 1u)) / old_period;

    if (scaled_ticks > new_period)
    {
        scaled_ticks = new_period;
    }

    return (uint16_t)scaled_ticks;
}

static uint32_t MOTOR_ClampPwmCarrierHz(uint32_t carrier_hz)
{
    if (carrier_hz < DRIVER_PWM_6STEP_MIN_CARRIER_HZ)
    {
        return DRIVER_PWM_6STEP_MIN_CARRIER_HZ;
    }

    if (carrier_hz > DRIVER_PWM_6STEP_MAX_CARRIER_HZ)
    {
        return DRIVER_PWM_6STEP_MAX_CARRIER_HZ;
    }

    return carrier_hz;
}

static void MOTOR_UpdatePwmCarrierDebug(uint16_t sector_ticks)
{
    uint32_t pulses;

    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();

    if (sector_ticks == 0u)
    {
        g_motor.pwm_pulses_per_sector = 0u;
        return;
    }

    pulses = (((uint32_t)g_motor.pwm_carrier_hz * sector_ticks) +
              (MOTOR_BEMF_TIMER_HZ / 2u)) / MOTOR_BEMF_TIMER_HZ;

    if (pulses > MOTOR_PWM_PULSES_MAX_DISPLAY)
    {
        pulses = MOTOR_PWM_PULSES_MAX_DISPLAY;
    }

    g_motor.pwm_pulses_per_sector = (uint8_t)pulses;
}

static void MOTOR_RecalculateSixStepPwmAfterCarrierChange(uint16_t old_period, uint16_t new_period)
{
    if (MOTOR_isBlindSixStepTestMode())
    {
        if ((g_motor.sixstep_bemf_closed_loop != 0u) &&
            (g_motor.sixstep_closed_loop_handoff_steps == 0u))
        {
            MOTOR_UpdateBlindSixStepClosedLoopPwm();
            return;
        }

        (void)MOTOR_UpdateBlindSixStepPwm();
        return;
    }

    g_motor.sixstep_pwm_ticks =
        MOTOR_RescalePwmTicks(g_motor.sixstep_pwm_ticks, old_period, new_period);
    MOTOR_UpdateSixStepPwmLimit();

    if (g_motor.sixstep_pwm_ticks > g_motor.sixstep_pwm_limit_ticks)
    {
        g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
    }
}

static void MOTOR_SetPwmCarrierHz(uint32_t carrier_hz)
{
    const uint16_t old_period = BOARD_GetPwmPeriodTicks();
    uint16_t new_period;

    carrier_hz = MOTOR_ClampPwmCarrierHz(carrier_hz);
    new_period = BOARD_SetPwmCarrierHz(carrier_hz);
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();

    if (new_period == old_period)
    {
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        MOTOR_RecalculateSixStepPwmAfterCarrierChange(old_period, new_period);

        if ((g_motor.sixstep_step != 0u) &&
            (g_motor.sixstep_step <= MOTOR_SIXSTEP_STEP_COUNT))
        {
            MOTOR_ApplySixStepBridge();
        }

        return;
    }

    MOTOR_UpdateSinusPwmLimit();
    MOTOR_UpdateSixStepPwmLimit();
}

static MOTOR_FAST_CODE void MOTOR_UpdateAdaptiveSixStepPwmCarrier(void)
{
#if DRIVER_PWM_DYNAMIC_6STEP_ENABLE
    const uint16_t sector_ticks = g_motor.bemf_average_interval_ticks;
    const uint32_t target_pulses =
        (DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR == 0u) ?
        1u : DRIVER_PWM_6STEP_TARGET_PULSES_PER_SECTOR;
    uint32_t carrier_hz;

    if (sector_ticks == 0u)
    {
        return;
    }

    carrier_hz = (((uint32_t)MOTOR_BEMF_TIMER_HZ * target_pulses) +
                  (sector_ticks >> 1u)) / sector_ticks;

    MOTOR_SetPwmCarrierHz(carrier_hz);
    MOTOR_UpdatePwmCarrierDebug(sector_ticks);
#else
    MOTOR_UpdatePwmCarrierDebug(g_motor.bemf_average_interval_ticks);
#endif
}

static void MOTOR_RestoreDefaultPwmCarrier(void)
{
    BOARD_SetPwmCarrierHz(DRIVER_PWM_CARRIER_HZ);
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.pwm_pulses_per_sector = 0u;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfPwmSampleTicks(uint16_t duty_ticks)
{
    const uint16_t period_ticks = BOARD_GetPwmPeriodTicks();
    uint32_t off_ticks;
    uint32_t sample_ticks;

    if (period_ticks <= 2u)
    {
        return 1u;
    }

    if (duty_ticks >= (uint16_t)(period_ticks - 1u))
    {
        return (uint16_t)(period_ticks - 2u);
    }

    off_ticks = (uint32_t)period_ticks - duty_ticks;
    sample_ticks = (uint32_t)duty_ticks +
                   ((off_ticks * MOTOR_BEMF_PWM_OFF_SAMPLE_PERCENT) / 100u);

    if (sample_ticks >= (uint32_t)(period_ticks - 1u))
    {
        sample_ticks = (uint32_t)(period_ticks - 2u);
    }

    if (sample_ticks == 0u)
    {
        sample_ticks = 1u;
    }

    return (uint16_t)sample_ticks;
}

static MOTOR_FAST_CODE void MOTOR_ServiceBlindSixStepClosedLoopHandoff(void)
{
    if (MOTOR_isBlindSixStepTestMode() == 0u)
    {
        return;
    }

    if (g_motor.sixstep_closed_loop_handoff_steps == 0u)
    {
        return;
    }

    g_motor.sixstep_closed_loop_handoff_steps--;

    if (g_motor.sixstep_closed_loop_handoff_steps == 0u)
    {
        MOTOR_UpdateBlindSixStepClosedLoopPwm();
    }
}

static MOTOR_FAST_CODE void MOTOR_ServiceBlindSixStepOpenLoopDutyRamp(void)
{
    if (MOTOR_isBlindSixStepTestMode() == 0u)
    {
        return;
    }

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        return;
    }

    if ((g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_ALIGN) ||
        (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT))
    {
        return;
    }

    if (MOTOR_UpdateBlindSixStepPwm() == 0u)
    {
        return;
    }

    MOTOR_ApplySixStepBridge();
}

static uint16_t MOTOR_GetSixStepClosedLoopMinPwmTicks(void)
{
    return MOTOR_GetPwmTicksFromPermille(DRIVER_SIXSTEP_CLOSED_LOOP_MIN_DUTY_PERMILLE);
}

static void MOTOR_UpdateSinusTiming(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint64_t numerator = (uint64_t)rpm * MOTOR_GetPolePairs() *
                               DRIVER_SIN_TABLE_STEPS * 65536u;
    const uint32_t denominator = 60u * DRIVER_CONTROL_LOOP_HZ;

    if (denominator == 0u)
    {
        g_motor.sinus_step_q16 = 0u;
        return;
    }

    g_motor.sinus_step_q16 = (uint32_t)(numerator / denominator);
}

static uint16_t MOTOR_GetSixStepIntervalTicks(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint32_t pole_pairs = MOTOR_GetPolePairs();
    const uint64_t denominator = (uint64_t)rpm * pole_pairs * MOTOR_SIXSTEP_STEP_COUNT;
    uint64_t ticks;

    if (denominator == 0u)
    {
        return 0u;
    }

    ticks = (((uint64_t)DRIVER_CONTROL_LOOP_HZ * 60u) + (denominator / 2u)) /
            denominator;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFu)
    {
        ticks = 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static int32_t MOTOR_GetBlindSixStepRampedRpm(void)
{
    const uint32_t start_rpm = MOTOR_GetAbsRpm(DRIVER_BLIND_6STEP_START_RPM);
    const uint32_t target_rpm = MOTOR_GetAbsRpm(DRIVER_BLIND_6STEP_RPM);
    const uint64_t ramped_delta =
        ((uint64_t)DRIVER_BLIND_6STEP_RAMP_RPM_PER_SEC * g_motor.sixstep_run_ticks) /
        DRIVER_CONTROL_LOOP_HZ;
    uint32_t rpm;

    if (target_rpm <= start_rpm)
    {
        return (int32_t)target_rpm;
    }

    rpm = start_rpm + (uint32_t)ramped_delta;

    if (rpm > target_rpm)
    {
        rpm = target_rpm;
    }

    return (int32_t)rpm;
}

static MOTOR_FAST_CODE void MOTOR_ServiceBlindSixStepOpenLoopSpeedRamp(void)
{
    int32_t next_rpm;

    if (MOTOR_isBlindSixStepTestMode() == 0u)
    {
        return;
    }

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        return;
    }

    if (g_motor.sixstep_phase != MOTOR_SIXSTEP_PHASE_OPEN_LOOP)
    {
        return;
    }

    next_rpm = MOTOR_GetBlindSixStepRampedRpm();

    if (next_rpm == g_motor.ramped_target_rpm)
    {
        return;
    }

    g_motor.ramped_target_rpm = next_rpm;
    g_motor.sixstep_interval_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
}

static uint16_t MOTOR_GetSixStepBemfIntervalTicks(int32_t rpm_command)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(rpm_command);
    const uint32_t pole_pairs = MOTOR_GetPolePairs();
    const uint64_t denominator = (uint64_t)rpm * pole_pairs * MOTOR_SIXSTEP_STEP_COUNT;
    uint64_t ticks;

    if (denominator == 0u)
    {
        return MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    ticks = (((uint64_t)MOTOR_BEMF_TIMER_HZ * 60u) + (denominator / 2u)) /
            denominator;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFu)
    {
        ticks = 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static uint16_t MOTOR_LimitTickCount(uint32_t ticks)
{
    if (ticks == 0u)
    {
        return 1u;
    }

    if (ticks > 0xFFFFu)
    {
        return 0xFFFFu;
    }

    return (uint16_t)ticks;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetControlTicksFromBemfInterval(uint16_t bemf_interval_ticks)
{
    uint32_t ticks;

    ticks = (((uint32_t)bemf_interval_ticks * DRIVER_CONTROL_LOOP_HZ) +
             (MOTOR_BEMF_TIMER_HZ / 2u)) / MOTOR_BEMF_TIMER_HZ;

    return MOTOR_LimitTickCount(ticks);
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfFallbackControlTicks(void)
{
    if (g_motor.sixstep_fallback_interval_ticks != 0u)
    {
        return g_motor.sixstep_fallback_interval_ticks;
    }

    if (g_motor.bemf_average_interval_ticks != 0u)
    {
        return MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
    }

    return MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfFallbackControlTicks(void)
{
    const uint16_t interval_ticks =
        MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);

    g_motor.sixstep_fallback_interval_ticks = interval_ticks;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfRelockTimeoutTicks(void)
{
    const uint32_t sectors =
        (DRIVER_SIXSTEP_BEMF_RELOCK_LISTEN_SECTORS == 0u) ?
        1u : DRIVER_SIXSTEP_BEMF_RELOCK_LISTEN_SECTORS;
    uint32_t min_ticks = DRIVER_SIXSTEP_BEMF_RELOCK_MIN_TIMEOUT_TICKS;
    uint32_t max_ticks = DRIVER_SIXSTEP_BEMF_RELOCK_MAX_TIMEOUT_TICKS;
    uint32_t timeout_ticks = g_motor.bemf_average_interval_ticks;

    if (timeout_ticks == 0u)
    {
        timeout_ticks = MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    timeout_ticks *= sectors;

    if (max_ticks < min_ticks)
    {
        max_ticks = min_ticks;
    }

    if (timeout_ticks < min_ticks)
    {
        timeout_ticks = min_ticks;
    }

    if (timeout_ticks > max_ticks)
    {
        timeout_ticks = max_ticks;
    }

    return MOTOR_LimitTickCount(timeout_ticks);
}

static MOTOR_FAST_CODE void MOTOR_DecayBemfFallbackOpenLoopSpeed(void)
{
    uint32_t interval_ticks = g_motor.sixstep_interval_ticks;
    const uint16_t target_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);

    if ((interval_ticks == 0u) || (target_ticks == 0u))
    {
        return;
    }

    if (interval_ticks >= target_ticks)
    {
        return;
    }

    interval_ticks = (interval_ticks * MOTOR_BEMF_FALLBACK_DECAY_NUMERATOR) /
                     MOTOR_BEMF_FALLBACK_DECAY_DENOMINATOR;

    if (interval_ticks <= g_motor.sixstep_interval_ticks)
    {
        interval_ticks = (uint32_t)g_motor.sixstep_interval_ticks + 1u;
    }

    if (interval_ticks > target_ticks)
    {
        interval_ticks = target_ticks;
    }

    g_motor.sixstep_interval_ticks = (uint16_t)interval_ticks;
    g_motor.sixstep_fallback_interval_ticks = g_motor.sixstep_interval_ticks;
}

static uint8_t MOTOR_MapFilterLevel(uint16_t interval_ticks)
{
    uint32_t filter_level;

    if (interval_ticks <= MOTOR_BEMF_FILTER_MIN_TICKS)
    {
        return 3u;
    }

    if (interval_ticks >= MOTOR_BEMF_FILTER_MAX_TICKS)
    {
        return 12u;
    }

    filter_level = 3u + ((((uint32_t)interval_ticks - MOTOR_BEMF_FILTER_MIN_TICKS) * 9u) /
                         (MOTOR_BEMF_FILTER_MAX_TICKS - MOTOR_BEMF_FILTER_MIN_TICKS));

    return (uint8_t)filter_level;
}

static void MOTOR_UpdateBemfFilterLevel(void)
{
    uint8_t filter_level;

    if (g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS)
    {
        filter_level = MOTOR_BEMF_ULTRA_FAST_FILTER_LEVEL;
    }
    else if (g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS)
    {
        filter_level = MOTOR_BEMF_FAST_FILTER_LEVEL;
    }
    else if ((g_motor.bemf_zero_cross_count < 100u) &&
        (g_motor.bemf_average_interval_ticks > MOTOR_BEMF_FILTER_MAX_TICKS))
    {
        filter_level = 12u;
    }
    else
    {
        filter_level = MOTOR_MapFilterLevel(g_motor.bemf_average_interval_ticks);
    }

    if (g_motor.bemf_average_interval_ticks < 50u)
    {
        filter_level = 2u;
    }

    BEMF_AM32_SetFilterLevel(filter_level);
}

static uint16_t MOTOR_GetBemfInitialIntervalTicks(uint16_t sector_ticks)
{
    if (sector_ticks == 0u)
    {
        return MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    return MOTOR_LimitTickCount(sector_ticks);
}

static void MOTOR_UpdateBemfMeasuredErpm(void)
{
    const uint16_t interval_ticks = g_motor.bemf_average_interval_ticks;
    uint32_t erpm;

    if (interval_ticks == 0u)
    {
        return;
    }

    erpm = MOTOR_BEMF_ERPM_NUMERATOR / interval_ticks;

    g_motor.measured_erpm = (int32_t)erpm;
}

static MOTOR_FAST_CODE int32_t MOTOR_GetBemfActionAngleOffsetDegX10(void)
{
    if (MOTOR_isBlindSixStepTestMode())
    {
        return DRIVER_BLIND_6STEP_BEMF_ADVANCE_DEG_X10;
    }

    return g_driver_config.bemf_action_angle_offset_deg_x10;
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfDelayTicks(uint16_t interval_ticks)
{
    int32_t angle_offset_deg_x10 = MOTOR_GetBemfActionAngleOffsetDegX10();
    int32_t delay_ticks;
    int32_t advance_ticks;
    uint32_t abs_angle_deg_x10;

    if (interval_ticks == 0u)
    {
        interval_ticks = g_motor.bemf_average_interval_ticks;
    }

    if (angle_offset_deg_x10 >= 300)
    {
        return 1u;
    }

    if (angle_offset_deg_x10 > 600)
    {
        angle_offset_deg_x10 = 600;
    }
    else if (angle_offset_deg_x10 < -600)
    {
        angle_offset_deg_x10 = -600;
    }

    if (angle_offset_deg_x10 < 0)
    {
        abs_angle_deg_x10 = (uint32_t)(-angle_offset_deg_x10);
    }
    else
    {
        abs_angle_deg_x10 = (uint32_t)angle_offset_deg_x10;
    }

    delay_ticks = (int32_t)(interval_ticks >> 1u);
    advance_ticks = (int32_t)((((uint32_t)interval_ticks * abs_angle_deg_x10 *
                                MOTOR_BEMF_ADVANCE_RECIP_Q16) + 32768u) >> 16u);

    if (angle_offset_deg_x10 < 0)
    {
        advance_ticks = -advance_ticks;
    }

    delay_ticks -= advance_ticks;

    if (delay_ticks < 1)
    {
        delay_ticks = 1;
    }

    if (delay_ticks > 0xFFFF)
    {
        delay_ticks = 0xFFFF;
    }

    return (uint16_t)delay_ticks;
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfWaitTime(void)
{
    g_motor.sixstep_bemf_delay_ticks =
        MOTOR_GetBemfDelayTicks(g_motor.bemf_average_interval_ticks);
}

static MOTOR_FAST_CODE uint16_t MOTOR_GetBemfMissingZcTimeoutTicks(void)
{
    uint32_t interval_ticks = g_motor.bemf_average_interval_ticks;
    uint32_t delay_ticks = g_motor.sixstep_bemf_delay_ticks;
    uint32_t timeout_ticks;

    if (interval_ticks == 0u)
    {
        interval_ticks = MOTOR_BEMF_START_INTERVAL_TICKS;
    }

    if (delay_ticks == 0u)
    {
        delay_ticks = interval_ticks >> 1u;
    }

    if (g_motor.sixstep_missed_zc_blind_steps == 0u)
    {
        timeout_ticks = interval_ticks + delay_ticks;
        return MOTOR_LimitTickCount(timeout_ticks);
    }

    if (interval_ticks > delay_ticks)
    {
        timeout_ticks = interval_ticks - delay_ticks;
    }
    else
    {
        timeout_ticks = 1u;
    }

    timeout_ticks = (timeout_ticks *
                     DRIVER_SIXSTEP_BEMF_MISSED_ZC_NEXT_TIMEOUT_PERCENT) / 100u;

    return MOTOR_LimitTickCount(timeout_ticks);
}

static MOTOR_FAST_CODE void MOTOR_UpdateBemfDerivedTiming(void)
{
    uint32_t measured_interval = g_motor.bemf_this_zc_ticks;
    int32_t interval_error;
    uint32_t interval_step;

    if (g_motor.bemf_last_zc_ticks != 0u)
    {
        measured_interval = ((uint32_t)g_motor.bemf_last_zc_ticks +
                             g_motor.bemf_this_zc_ticks) >> 1u;
    }

    if (measured_interval == 0u)
    {
        measured_interval = g_motor.bemf_average_interval_ticks;
    }

    interval_error = (int32_t)measured_interval -
                     (int32_t)g_motor.bemf_average_interval_ticks;

    if (interval_error >= 0)
    {
        interval_step = (uint32_t)interval_error >> MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT;

        if ((interval_step == 0u) && (interval_error != 0))
        {
            interval_step = 1u;
        }

        g_motor.bemf_average_interval_ticks =
            MOTOR_LimitTickCount((uint32_t)g_motor.bemf_average_interval_ticks +
                                 interval_step);
        return;
    }

    interval_error = -interval_error;
    interval_step = (uint32_t)interval_error >> MOTOR_BEMF_INTERVAL_AVERAGE_SHIFT;

    if (interval_step == 0u)
    {
        interval_step = 1u;
    }

    if (interval_step >= g_motor.bemf_average_interval_ticks)
    {
        g_motor.bemf_average_interval_ticks = 1u;
        return;
    }

    g_motor.bemf_average_interval_ticks =
        (uint16_t)(g_motor.bemf_average_interval_ticks -
                   interval_step);
}

static void MOTOR_ServiceBemfSlowUpdate(void)
{
    if (g_motor.bemf_armed == 0u)
    {
        return;
    }

    if (g_motor.bemf_slow_update_tick < 0xFFu)
    {
        g_motor.bemf_slow_update_tick++;
    }

    if (g_motor.bemf_slow_update_tick < MOTOR_BEMF_SLOW_UPDATE_DIVIDER)
    {
        return;
    }

    g_motor.bemf_slow_update_tick = 0u;
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        MOTOR_UpdateBemfFallbackControlTicks();
        g_motor.sixstep_interval_ticks =
            MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
        MOTOR_UpdateAdaptiveSixStepPwmCarrier();
    }
}

static MOTOR_FAST_CODE void MOTOR_ScheduleBemfCommutation(void)
{
    uint16_t delay_ticks = g_motor.sixstep_bemf_delay_ticks;

    if (delay_ticks <= MOTOR_BEMF_DIRECT_COMMUTATION_TICKS)
    {
        MOTOR_CommitBemfCommutation();
        return;
    }

    if (delay_ticks < 0xFFFFu)
    {
        delay_ticks++;
    }

    g_motor.sixstep_commutation_pending = 1u;
    BOARD_ScheduleBemfCommutation(delay_ticks);
}

#if DRIVER_SIN_TO_6STEP_ENABLE
static uint32_t MOTOR_GetSinToSixStepSettleTicks(void)
{
    const uint32_t rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_SINUS_RPM);
    const uint64_t numerator = (uint64_t)DRIVER_CONTROL_LOOP_HZ * 60u *
                               DRIVER_SIN_TO_6STEP_SETTLE_MECH_REV;
    uint64_t ticks;

    if (rpm == 0u)
    {
        return 0xFFFFFFFFu;
    }

    ticks = (numerator + (rpm - 1u)) / rpm;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > 0xFFFFFFFFu)
    {
        ticks = 0xFFFFFFFFu;
    }

    return (uint32_t)ticks;
}
#endif

static uint32_t MOTOR_GetSixStepBemfArmDelayTicks(void)
{
    const uint64_t ticks = ((uint64_t)DRIVER_SIXSTEP_BEMF_ARM_DELAY_MS *
                            DRIVER_CONTROL_LOOP_HZ) / 1000u;

    if (ticks == 0u)
    {
        return 1u;
    }

    if (ticks > 0xFFFFFFFFu)
    {
        return 0xFFFFFFFFu;
    }

    return (uint32_t)ticks;
}

static uint32_t MOTOR_GetSixStepBemfArmMechRevSteps(void)
{
    const uint64_t steps = (uint64_t)DRIVER_SIXSTEP_BEMF_ARM_AFTER_MECH_REV *
                           MOTOR_GetPolePairs() * MOTOR_SIXSTEP_STEP_COUNT;

    if (steps == 0u)
    {
        return 0u;
    }

    if (steps > 0xFFFFFFFFu)
    {
        return 0xFFFFFFFFu;
    }

    return (uint32_t)steps;
}

static uint32_t MOTOR_GetAlignTicks(void)
{
    return (DRIVER_OPEN_LOOP_ALIGN_MS * DRIVER_CONTROL_LOOP_HZ) / 1000u;
}

static int32_t MOTOR_GetOpenLoopRampedRpm(uint32_t ramp_tick)
{
    const uint32_t target_rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_SINUS_RPM);
    const uint32_t start_rpm = MOTOR_GetAbsRpm(DRIVER_OPEN_LOOP_START_RPM);
    uint32_t rpm;

    if (target_rpm <= start_rpm)
    {
        return (int32_t)target_rpm;
    }

    rpm = start_rpm + ((ramp_tick * DRIVER_OPEN_LOOP_RAMP_RPM_PER_SEC) /
                       DRIVER_CONTROL_LOOP_HZ);

    if (rpm > target_rpm)
    {
        rpm = target_rpm;
    }

    return (int32_t)rpm;
}

static uint8_t MOTOR_GetSixStepFromAngle(uint32_t angle_q16)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    uint32_t sector;

    while (angle_q16 >= full_turn_q16)
    {
        angle_q16 -= full_turn_q16;
    }

    sector = (uint32_t)((((uint64_t)angle_q16 * MOTOR_SIXSTEP_STEP_COUNT) +
                         (full_turn_q16 / 2u)) / full_turn_q16);

    if (sector >= MOTOR_SIXSTEP_STEP_COUNT)
    {
        sector = 0u;
    }

    if (sector < 5u)
    {
        return (uint8_t)(5u - sector);
    }

    return 6u;
}

static uint8_t MOTOR_GetBlindSixStepStartStep(void)
{
    if ((DRIVER_BLIND_6STEP_START_STEP >= 1u) &&
        (DRIVER_BLIND_6STEP_START_STEP <= MOTOR_SIXSTEP_STEP_COUNT))
    {
        return DRIVER_BLIND_6STEP_START_STEP;
    }

    return 1u;
}

static uint8_t MOTOR_GetBlindSixStepAlignStep(void)
{
    const uint8_t kick_step = MOTOR_GetBlindSixStepStartStep();

    if (g_motor.direction == MOTOR_DIRECTION_CW)
    {
        if (kick_step > 1u)
        {
            return (uint8_t)(kick_step - 1u);
        }

        return MOTOR_SIXSTEP_STEP_COUNT;
    }

    if (kick_step < MOTOR_SIXSTEP_STEP_COUNT)
    {
        return (uint8_t)(kick_step + 1u);
    }

    return 1u;
}

static void MOTOR_ApplySinusBridge(void)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    const uint32_t phase_shift_120_q16 = full_turn_q16 / 3u;
    const uint32_t phase_shift_240_q16 = (full_turn_q16 * 2u) / 3u;
    const uint16_t index_a = (uint16_t)(g_motor.sinus_angle_q16 >> 16u);
    const uint16_t index_b = (uint16_t)((g_motor.sinus_angle_q16 + phase_shift_120_q16) >> 16u);
    const uint16_t index_c = (uint16_t)((g_motor.sinus_angle_q16 + phase_shift_240_q16) >> 16u);
    const uint16_t sample_a = MOTOR_GetSinusSample(index_a);
    const uint16_t sample_b = MOTOR_GetSinusSample(index_b);
    const uint16_t sample_c = MOTOR_GetSinusSample(index_c);
    const uint16_t duty_a = MOTOR_ScaleSinusDuty(sample_a);
    const uint16_t duty_b = MOTOR_ScaleSinusDuty(sample_b);
    const uint16_t duty_c = MOTOR_ScaleSinusDuty(sample_c);

    BOARD_SetHighPwm(duty_a, duty_b, duty_c);
    BOARD_SetPwmOutputMask(1u, 1u, 1u, 1u, 1u, 1u);
    BOARD_SetPwmBridgeEnabled(1u);
}

static MOTOR_FAST_CODE void MOTOR_ApplySixStepBridge(void)
{
    const uint16_t duty = g_motor.sixstep_pwm_ticks;

    if ((g_motor.sixstep_step == 0u) ||
        (g_motor.sixstep_step > MOTOR_SIXSTEP_STEP_COUNT))
    {
        BOARD_AllPhasesOff();
        return;
    }

    BOARD_ApplySixStepBridge(g_motor.sixstep_step,
                             duty,
                             MOTOR_GetBemfPwmSampleTicks(duty));
}

static int32_t MOTOR_GetSixStepTargetErpm(void)
{
    const uint64_t erpm = (uint64_t)MOTOR_GetAbsRpm(g_motor.ramped_target_rpm) *
                          MOTOR_GetPolePairs();

    if (erpm > 0x7FFFFFFFu)
    {
        return 0x7FFFFFFF;
    }

    return (int32_t)erpm;
}

static int32_t MOTOR_GetSixStepInRpmWindowErpm(void)
{
    const uint64_t window_erpm = (uint64_t)g_driver_config.in_rpm_window *
                                 MOTOR_GetPolePairs();

    if (window_erpm > 0x7FFFFFFFu)
    {
        return 0x7FFFFFFF;
    }

    return (int32_t)window_erpm;
}

static int32_t MOTOR_GetSixStepDutyDeltaTicks(int32_t error_erpm)
{
    const int32_t erpm_per_pwm_tick =
        (DRIVER_SIXSTEP_SPEED_KP_ERPM_PER_PWM_TICK == 0u) ?
        1 : (int32_t)DRIVER_SIXSTEP_SPEED_KP_ERPM_PER_PWM_TICK;
    const int32_t max_step =
        (DRIVER_SIXSTEP_SPEED_MAX_STEP_TICKS == 0u) ?
        1 : (int32_t)DRIVER_SIXSTEP_SPEED_MAX_STEP_TICKS;
    int64_t abs_error = error_erpm;
    int32_t delta;

    if (error_erpm == 0)
    {
        return 0;
    }

    if (abs_error < 0)
    {
        abs_error = -abs_error;
    }

    delta = (int32_t)(abs_error / erpm_per_pwm_tick);

    if (delta == 0)
    {
        delta = 1;
    }

    if (delta > max_step)
    {
        delta = max_step;
    }

    if (error_erpm < 0)
    {
        delta = -delta;
    }

    return delta;
}

static void MOTOR_UpdateSixStepClosedLoopDuty(void)
{
    const uint16_t divider =
        (DRIVER_SIXSTEP_SPEED_CONTROL_DIVIDER_TICKS == 0u) ?
        1u : DRIVER_SIXSTEP_SPEED_CONTROL_DIVIDER_TICKS;
    const int32_t target_erpm = MOTOR_GetSixStepTargetErpm();
    const int32_t measured_erpm = g_motor.measured_erpm;
    const int32_t error_erpm = target_erpm - measured_erpm;
    const int32_t rpm_window_erpm = MOTOR_GetSixStepInRpmWindowErpm();
    int32_t next_pwm_ticks;
    int32_t min_pwm_ticks;
    int32_t max_pwm_ticks;

    if (MOTOR_isBlindSixStepTestMode())
    {
        g_motor.in_rpm = 1u;
        return;
    }

    if (g_motor.sixstep_speed_control_tick < 0xFFFFu)
    {
        g_motor.sixstep_speed_control_tick++;
    }

    if (g_motor.sixstep_speed_control_tick < divider)
    {
        return;
    }

    g_motor.sixstep_speed_control_tick = 0u;

    if ((error_erpm >= -rpm_window_erpm) && (error_erpm <= rpm_window_erpm))
    {
        g_motor.in_rpm = 1u;
        return;
    }

    g_motor.in_rpm = 0u;
    min_pwm_ticks = MOTOR_GetSixStepClosedLoopMinPwmTicks();
    max_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;

    if (min_pwm_ticks > max_pwm_ticks)
    {
        min_pwm_ticks = max_pwm_ticks;
    }

    next_pwm_ticks = (int32_t)g_motor.sixstep_pwm_ticks +
                     MOTOR_GetSixStepDutyDeltaTicks(error_erpm);

    if (next_pwm_ticks > max_pwm_ticks)
    {
        next_pwm_ticks = max_pwm_ticks;
    }

    if (next_pwm_ticks < min_pwm_ticks)
    {
        next_pwm_ticks = min_pwm_ticks;
    }

    if (g_motor.sixstep_pwm_ticks == (uint16_t)next_pwm_ticks)
    {
        return;
    }

    g_motor.sixstep_pwm_ticks = (uint16_t)next_pwm_ticks;
    MOTOR_ApplySixStepBridge();
}

static void MOTOR_AdvanceSinusAngle(void)
{
    const uint32_t full_turn_q16 = DRIVER_SIN_TABLE_STEPS * 65536u;
    uint32_t step_q16 = g_motor.sinus_step_q16;

    while (step_q16 >= full_turn_q16)
    {
        step_q16 -= full_turn_q16;
    }

    if (g_motor.direction == MOTOR_DIRECTION_CCW)
    {
        if (g_motor.sinus_angle_q16 >= step_q16)
        {
            g_motor.sinus_angle_q16 -= step_q16;
        }
        else
        {
            g_motor.sinus_angle_q16 = full_turn_q16 -
                                      (step_q16 - g_motor.sinus_angle_q16);
        }

        return;
    }

    g_motor.sinus_angle_q16 += step_q16;

    if (g_motor.sinus_angle_q16 >= full_turn_q16)
    {
        g_motor.sinus_angle_q16 -= full_turn_q16;
    }
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfRisingForStep(void)
{
    const uint8_t step_is_odd = (uint8_t)(g_motor.sixstep_step & 1u);

    if (g_motor.direction == MOTOR_DIRECTION_CW)
    {
        return step_is_odd;
    }

    if (step_is_odd != 0u)
    {
        return 0u;
    }

    return 1u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfInputStepForPwmStep(uint8_t pwm_step)
{
    uint8_t input_step = pwm_step;
    uint8_t shift = (uint8_t)(DRIVER_BEMF_PHASE_MAP_SHIFT % 3u);

    while (shift > 0u)
    {
        input_step++;

        if (input_step > MOTOR_SIXSTEP_STEP_COUNT)
        {
            input_step = 1u;
        }

        shift--;
    }

    return input_step;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfPhaseMaskForInputStep(uint8_t input_step)
{
    if ((input_step == 1u) || (input_step == 4u))
    {
        return MOTOR_BEMF_PHASE_C_MASK;
    }

    if ((input_step == 2u) || (input_step == 5u))
    {
        return MOTOR_BEMF_PHASE_A_MASK;
    }

    return MOTOR_BEMF_PHASE_B_MASK;
}

static void MOTOR_SelectBemfDiagnosticPhase(uint8_t phase)
{
    BEMF_AM32_MaskPhaseInterrupts();

    if (phase == MOTOR_BEMF_LED_PHASE_A)
    {
        BEMF_AM32_ChangeCompInput(2u, 0u);
        return;
    }

    if (phase == MOTOR_BEMF_LED_PHASE_B)
    {
        BEMF_AM32_ChangeCompInput(3u, 0u);
        return;
    }

    BEMF_AM32_ChangeCompInput(1u, 0u);
}

static uint8_t MOTOR_GetNextBemfDiagnosticPhase(uint8_t phase)
{
    if (phase == MOTOR_BEMF_LED_PHASE_A)
    {
        return MOTOR_BEMF_LED_PHASE_B;
    }

    if (phase == MOTOR_BEMF_LED_PHASE_B)
    {
        return MOTOR_BEMF_LED_PHASE_C;
    }

    return MOTOR_BEMF_LED_PHASE_A;
}

static void MOTOR_RecordBemfDiagnosticLevel(uint8_t phase, uint8_t level)
{
    volatile uint8_t *stored_level;

    if (phase == MOTOR_BEMF_LED_PHASE_A)
    {
        stored_level = &g_motor.bemf_diag_level_a;
    }
    else if (phase == MOTOR_BEMF_LED_PHASE_B)
    {
        stored_level = &g_motor.bemf_diag_level_b;
    }
    else
    {
        stored_level = &g_motor.bemf_diag_level_c;
    }

    if (g_motor.bemf_diag_initialized == 0u)
    {
        *stored_level = level;
        g_motor.bemf_diag_initialized = 1u;
        return;
    }

    if (*stored_level == level)
    {
        return;
    }

    *stored_level = level;
    g_motor.bemf_led_phase = phase;
    g_motor.bemf_readable = 1u;
    g_motor.bemf_diag_phase = MOTOR_GetNextBemfDiagnosticPhase(phase);
    g_motor.bemf_diag_initialized = 0u;
    MOTOR_SelectBemfDiagnosticPhase(g_motor.bemf_diag_phase);
}

static void MOTOR_BemfLedDiagnosticTick(void)
{
    const uint8_t sampled_phase = g_motor.bemf_diag_phase;
    const uint8_t level = BEMF_AM32_GetOutputLevel();

    MOTOR_RecordBemfDiagnosticLevel(sampled_phase, level);

    BOARD_AllPhasesOff();
}

static void MOTOR_StartBemfLedDiagnostic(void)
{
    g_motor.mode = MOTOR_MODE_STOP;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_led_phase = MOTOR_BEMF_LED_PHASE_NONE;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.bemf_led_hold_ticks = 0u;
    g_motor.bemf_diag_phase = MOTOR_BEMF_LED_PHASE_A;
    g_motor.bemf_diag_initialized = 0u;
    g_motor.bemf_diag_level_a = 0u;
    g_motor.bemf_diag_level_b = 0u;
    g_motor.bemf_diag_level_c = 0u;

    BOARD_DisableBemfCommutationTimer();
    BOARD_AllPhasesOff();
    BEMF_AM32_MaskPhaseInterrupts();
    BEMF_AM32_SetFilterLevel(1u);
    MOTOR_SelectBemfDiagnosticPhase(g_motor.bemf_diag_phase);
}

static uint8_t MOTOR_isBemfArmAllowed(void)
{
    if (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT)
    {
        return 1u;
    }

    if (g_motor.sixstep_run_ticks < MOTOR_GetSixStepBemfArmDelayTicks())
    {
        return 0u;
    }

    if (g_motor.sixstep_commutation_count < DRIVER_SIXSTEP_BEMF_ARM_AFTER_STEPS)
    {
        return 0u;
    }

    if (g_motor.sixstep_commutation_count < MOTOR_GetSixStepBemfArmMechRevSteps())
    {
        return 0u;
    }

    return 1u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfArmed(void)
{
    if (g_motor.bemf_armed != 0u)
    {
        return 1u;
    }

    return 0u;
}

static void MOTOR_ResetBemfValidation(void)
{
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_GetBemfBlankTicks(void)
{
    uint8_t blank_ticks = DRIVER_SIXSTEP_BEMF_BLANK_TICKS;

    if ((g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_ULTRA_FAST_INTERVAL_TICKS) &&
        (blank_ticks > MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS))
    {
        return MOTOR_BEMF_ULTRA_FAST_BLANK_CONTROL_TICKS;
    }

    if ((g_motor.bemf_average_interval_ticks <= MOTOR_BEMF_FAST_BLANK_INTERVAL_TICKS) &&
        (blank_ticks > MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS))
    {
        return MOTOR_BEMF_FAST_BLANK_CONTROL_TICKS;
    }

    return blank_ticks;
}

static void MOTOR_ResetBemfLock(void)
{
    MOTOR_ResetBemfValidation();
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_interval_ticks = MOTOR_GetBemfFallbackControlTicks();
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.bemf_armed = 0u;

    g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;

    g_motor.sixstep_bemf_delay_ticks = 0u;

    if (g_motor.sixstep_fallback_interval_ticks != 0u)
    {
        MOTOR_UpdateAdaptiveSixStepPwmCarrier();
    }
    else
    {
        MOTOR_RestoreDefaultPwmCarrier();
    }

    BOARD_EnableBemfPwmSampleIrq(0u);
    BOARD_DisableBemfCommutationTimer();
}

static MOTOR_FAST_CODE void MOTOR_ArmBemfCoastWaitStep(void)
{
    BOARD_AllPhasesOff();
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_blank_ticks = 0u;
    MOTOR_ArmBemfForSixStep();

    if (MOTOR_isBemfArmed() != 0u)
    {
        g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT;
        g_motor.bemf_interval_ticks = 0u;
        BOARD_ResetBemfIntervalTimer();
    }
}

static MOTOR_FAST_CODE void MOTOR_FallbackBemfCoastWaitToOpenLoop(void)
{
    g_motor.sixstep_handoff_coast_done = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    MOTOR_ResetBemfLock();
    g_motor.sixstep_handoff_coast_done = 0u;
    g_motor.bemf_edge_seen = 1u;
    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE void MOTOR_ScanNextBemfCoastWaitStep(void)
{
    if (g_motor.sixstep_relock_scan_steps >= MOTOR_BEMF_RELOCK_SCAN_STEP_LIMIT)
    {
        MOTOR_FallbackBemfCoastWaitToOpenLoop();
        return;
    }

    g_motor.sixstep_relock_scan_steps++;
    MOTOR_AdvanceSixStepStepIndex();
    MOTOR_ResetBemfValidation();
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT;
    MOTOR_ArmBemfCoastWaitStep();
}

static MOTOR_FAST_CODE void MOTOR_StartBemfRelockCoast(void)
{
    MOTOR_ResetBemfValidation();
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT;
    g_motor.sixstep_interval_ticks = MOTOR_GetBemfFallbackControlTicks();
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;

    BEMF_AM32_MaskPhaseInterrupts();
    BOARD_EnableBemfPwmSampleIrq(0u);
    BOARD_DisableBemfCommutationTimer();
    MOTOR_ArmBemfCoastWaitStep();
}

static MOTOR_FAST_CODE void MOTOR_StartBemfHandoffCoast(void)
{
    g_motor.sixstep_handoff_coast_done = 1u;
    MOTOR_StartBemfRelockCoast();
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfCoastWait(void)
{
    if (g_motor.sixstep_phase != MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT)
    {
        return 0u;
    }

    if (g_motor.bemf_interval_ticks > MOTOR_GetBemfRelockTimeoutTicks())
    {
        MOTOR_ScanNextBemfCoastWaitStep();
    }

    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_StartBemfMissedZcBlindStep(void)
{
    if (g_motor.sixstep_missed_zc_blind_steps < 0xFFu)
    {
        g_motor.sixstep_missed_zc_blind_steps++;
    }

    g_motor.sixstep_commutation_pending = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;

    BOARD_DisableBemfCommutationTimer();
    BOARD_ResetBemfIntervalTimer();
    MOTOR_AdvanceSixStepStep();
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfMissingZcClosedLoop(void)
{
    if ((g_motor.bemf_edge_seen != 0u) ||
        (g_motor.sixstep_commutation_pending != 0u))
    {
        return 0u;
    }

    if (g_motor.bemf_interval_ticks <= MOTOR_GetBemfMissingZcTimeoutTicks())
    {
        return 0u;
    }

    if (g_motor.sixstep_missed_zc_blind_steps <
        DRIVER_SIXSTEP_BEMF_MISSED_ZC_BLIND_STEPS)
    {
        MOTOR_StartBemfMissedZcBlindStep();
        return 1u;
    }

    g_motor.sixstep_missed_zc_blind_steps = 0u;
    MOTOR_StartBemfRelockCoast();

    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_ArmBemfForSixStep(void)
{
    const uint8_t zero_start_wait =
        (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT) ? 1u : 0u;

    BEMF_AM32_MaskPhaseInterrupts();
    BOARD_EnableBemfPwmSampleIrq(0u);

    if (g_motor.bemf_average_interval_ticks == 0u)
    {
        g_motor.bemf_average_interval_ticks =
            MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(g_motor.ramped_target_rpm));
        MOTOR_UpdateBemfMeasuredErpm();
        MOTOR_UpdateBemfFilterLevel();
    }

    MOTOR_UpdateBemfWaitTime();

    BEMF_AM32_ChangeCompInput(MOTOR_GetBemfInputStepForPwmStep(g_motor.sixstep_step),
                              MOTOR_GetBemfRisingForStep());

    if (MOTOR_isBemfArmAllowed() == 0u)
    {
        g_motor.bemf_armed = 0u;
        g_motor.bemf_edge_seen = 1u;
        g_motor.bemf_blank_ticks = 0u;
        MOTOR_ResetBemfLock();
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        g_motor.bemf_armed = 1u;

        if (zero_start_wait == 0u)
        {
            g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_BEMF_ACQUIRE;
        }

        g_motor.bemf_edge_seen = 1u;
        g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
        g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
        g_motor.bemf_interval_ticks = g_motor.bemf_average_interval_ticks;
        BOARD_SetBemfIntervalTicks(g_motor.bemf_average_interval_ticks >> 1u);
        MOTOR_ResetBemfValidation();

    }

    if (g_motor.bemf_edge_seen == 0u)
    {
        MOTOR_StartBemfRelockCoast();
        return;
    }

    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_blank_ticks = MOTOR_GetBemfBlankTicks();

    if ((g_motor.bemf_blank_ticks == 0u) &&
        (MOTOR_isBemfPwmGatedMode() != 0u))
    {
        BOARD_EnableBemfPwmSampleIrq(1u);
    }
    else if (g_motor.bemf_blank_ticks == 0u)
    {
        BEMF_AM32_EnableCompInterrupts();
    }
}

static MOTOR_FAST_CODE void MOTOR_ServiceBemfBlank(void)
{
    if (g_motor.bemf_blank_ticks == 0u)
    {
        return;
    }

    g_motor.bemf_blank_ticks--;

    if (g_motor.bemf_blank_ticks == 0u)
    {
        if (MOTOR_isBemfPwmGatedMode() != 0u)
        {
            BOARD_EnableBemfPwmSampleIrq(1u);
        }
        else
        {
            BEMF_AM32_EnableCompInterrupts();
        }
    }
}

static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStepIndex(void)
{
    if (g_motor.direction == MOTOR_DIRECTION_CW)
    {
        g_motor.sixstep_step++;

        if (g_motor.sixstep_step > MOTOR_SIXSTEP_STEP_COUNT)
        {
            g_motor.sixstep_step = 1u;
        }
    }
    else
    {
        if (g_motor.sixstep_step > 1u)
        {
            g_motor.sixstep_step--;
        }
        else
        {
            g_motor.sixstep_step = MOTOR_SIXSTEP_STEP_COUNT;
        }
    }
}

static MOTOR_FAST_CODE void MOTOR_AdvanceSixStepStep(void)
{
    if (g_motor.sixstep_commutation_count < 0xFFFFu)
    {
        g_motor.sixstep_commutation_count++;
    }

    MOTOR_AdvanceSixStepStepIndex();
    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE void MOTOR_CommitBemfCommutation(void)
{
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_tick = 0u;
    MOTOR_AdvanceSixStepStep();
    MOTOR_ServiceBlindSixStepClosedLoopHandoff();
}

static void MOTOR_EnterSixStepFromSinus(void)
{
    MOTOR_RestoreDefaultPwmCarrier();
    MOTOR_UpdateSixStepPwmLimit();

    g_motor.mode = MOTOR_MODE_SIXSTEP;
    g_motor.sixstep_step = MOTOR_GetSixStepFromAngle(g_motor.sinus_angle_q16);
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sixstep_interval_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
    g_motor.sixstep_fallback_interval_ticks = g_motor.sixstep_interval_ticks;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_average_interval_ticks =
        MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(g_motor.ramped_target_rpm));
    g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_handoff_coast_done = 0u;

    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_DisableBemfCommutationTimer();

    g_motor.sixstep_pwm_ticks = g_motor.sinus_pwm_limit_ticks;

    if (g_motor.sixstep_pwm_ticks > g_motor.sixstep_pwm_limit_ticks)
    {
        g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
    }

    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE void MOTOR_StartBlindSixStepOpenLoopRamp(void)
{
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.ramped_target_rpm = DRIVER_BLIND_6STEP_START_RPM;
    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)MOTOR_GetPolePairs();
    g_motor.sixstep_interval_ticks = MOTOR_GetSixStepIntervalTicks(g_motor.ramped_target_rpm);
    g_motor.sixstep_fallback_interval_ticks = g_motor.sixstep_interval_ticks;
    g_motor.bemf_average_interval_ticks =
        MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(g_motor.ramped_target_rpm));
    g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;

    (void)MOTOR_UpdateBlindSixStepPwm();
    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_SetBemfIntervalTicks(MOTOR_BEMF_START_COUNTER_TICKS);
    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_ApplySixStepBridge();
}

static MOTOR_FAST_CODE void MOTOR_StartBlindSixStepZeroBemfWait(void)
{
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_step = MOTOR_GetBlindSixStepStartStep();
    g_motor.ramped_target_rpm = 0;
    g_motor.measured_erpm = 0;
    g_motor.bemf_average_interval_ticks = 1u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;

    (void)MOTOR_UpdateBlindSixStepPwm();
    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_ResetBemfIntervalTimer();
    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_ApplySixStepBridge();
    MOTOR_ArmBemfForSixStep();
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBlindSixStepZeroStart(void)
{
    if (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_ALIGN)
    {
        if (g_motor.sixstep_run_ticks < 0xFFFFFFFFu)
        {
            g_motor.sixstep_run_ticks++;
        }

        if (g_motor.sixstep_run_ticks >= MOTOR_GetBlindSixStepAlignTicks())
        {
            MOTOR_StartBlindSixStepZeroBemfWait();
        }

        return 1u;
    }

    if (g_motor.sixstep_phase != MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT)
    {
        return 0u;
    }

    if (g_motor.sixstep_run_ticks < 0xFFFFFFFFu)
    {
        g_motor.sixstep_run_ticks++;
    }

    if (g_motor.sixstep_run_ticks >= MOTOR_GetBlindSixStepZeroBemfTimeoutTicks())
    {
        MOTOR_StartBlindSixStepOpenLoopRamp();
    }

    return 1u;
}

static void MOTOR_StartBlindSixStepTest(void)
{
    MOTOR_RestoreDefaultPwmCarrier();
    g_motor.mode = MOTOR_MODE_SIXSTEP;
    g_motor.direction = DRIVER_OPEN_LOOP_DIRECTION;
    g_motor.target_rpm = DRIVER_BLIND_6STEP_RPM;
    g_motor.ramped_target_rpm = 0;
    g_motor.measured_erpm = 0;
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_step_q16 = 0u;
    g_motor.open_loop_tick = 0u;
    g_motor.sinus_at_target_ticks = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sixstep_step = MOTOR_GetBlindSixStepAlignStep();
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.sixstep_interval_ticks = MOTOR_GetSixStepIntervalTicks(DRIVER_BLIND_6STEP_START_RPM);
    g_motor.sixstep_fallback_interval_ticks = g_motor.sixstep_interval_ticks;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_average_interval_ticks =
        MOTOR_GetBemfInitialIntervalTicks(MOTOR_GetSixStepBemfIntervalTicks(DRIVER_BLIND_6STEP_START_RPM));
    g_motor.bemf_last_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_this_zc_ticks = g_motor.bemf_average_interval_ticks;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_ZERO_ALIGN;
    g_motor.sixstep_handoff_coast_done = 0u;
    g_motor.in_rpm = 1u;

    (void)MOTOR_UpdateBlindSixStepPwm();
    MOTOR_UpdateBemfWaitTime();
    MOTOR_UpdateBemfMeasuredErpm();
    MOTOR_UpdateBemfFilterLevel();
    BOARD_DisableBemfCommutationTimer();
    BOARD_SetBemfIntervalTicks(MOTOR_BEMF_START_COUNTER_TICKS);
    BEMF_AM32_MaskPhaseInterrupts();
    MOTOR_ApplySixStepBridge();
}

static void MOTOR_SixStepTickIrq(void)
{
    if (MOTOR_ServiceBlindSixStepZeroStart() != 0u)
    {
        return;
    }

    if (g_motor.sixstep_interval_ticks == 0u)
    {
        BOARD_AllPhasesOff();
        return;
    }

    g_motor.bemf_interval_ticks = BOARD_GetBemfIntervalTicks();
    MOTOR_ServiceBemfSlowUpdate();

    if (g_motor.sixstep_run_ticks < 0xFFFFFFFFu)
    {
        g_motor.sixstep_run_ticks++;
    }

    MOTOR_ServiceBemfBlank();

    if (MOTOR_ServiceBemfCoastWait() != 0u)
    {
        return;
    }

    MOTOR_ServiceBlindSixStepOpenLoopSpeedRamp();
    MOTOR_ServiceBlindSixStepOpenLoopDutyRamp();

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        if (MOTOR_ServiceBemfMissingZcClosedLoop() != 0u)
        {
            return;
        }

        MOTOR_UpdateSixStepClosedLoopDuty();
        return;
    }

    g_motor.sixstep_tick++;

    if (g_motor.sixstep_tick >= g_motor.sixstep_interval_ticks)
    {
        g_motor.sixstep_tick = 0u;
        MOTOR_AdvanceSixStepStep();
        MOTOR_DecayBemfFallbackOpenLoopSpeed();
    }

    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)MOTOR_GetPolePairs();
    g_motor.in_rpm = 1u;
}

static uint8_t MOTOR_isBemfPhaseSetComplete(void)
{
    if ((g_motor.bemf_phase_mask & MOTOR_BEMF_ALL_PHASES_MASK) == MOTOR_BEMF_ALL_PHASES_MASK)
    {
        return 1u;
    }

    return 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfReadableCandidate(void)
{
    if ((g_motor.bemf_zero_cross_count >= DRIVER_SIXSTEP_BEMF_OK_COUNT) &&
        (MOTOR_isBemfPhaseSetComplete() != 0u))
    {
        return 1u;
    }

    return 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBemfLockReady(void)
{
    const uint16_t lock_count = DRIVER_SIXSTEP_BEMF_OK_COUNT +
                                DRIVER_SIXSTEP_BEMF_LOCK_EXTRA_COUNT;

    if ((g_motor.bemf_zero_cross_count >= lock_count) &&
        (MOTOR_isBemfPhaseSetComplete() != 0u))
    {
        return 1u;
    }

    return 0u;
}

static MOTOR_FAST_CODE uint8_t MOTOR_isBlindSixStepCoastHandoffSpeed(void)
{
    if (MOTOR_GetAbsRpm(g_motor.ramped_target_rpm) >=
        (uint32_t)DRIVER_BLIND_6STEP_COAST_HANDOFF_MIN_RPM)
    {
        return 1u;
    }

    return 0u;
}

static MOTOR_FAST_CODE void MOTOR_EnableBemfClosedLoop(void)
{
    const uint8_t bridge_is_coasting =
        (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT) ? 1u : 0u;

    if (g_motor.sixstep_bemf_closed_loop != 0u)
    {
        return;
    }

    g_motor.sixstep_bemf_closed_loop = 1u;
    g_motor.sixstep_handoff_coast_done = 1u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_CLOSED_LOOP;
    g_motor.sixstep_interval_ticks =
        MOTOR_GetControlTicksFromBemfInterval(g_motor.bemf_average_interval_ticks);
    MOTOR_UpdateAdaptiveSixStepPwmCarrier();
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;

    if (MOTOR_isBlindSixStepTestMode())
    {
        g_motor.sixstep_closed_loop_handoff_steps =
            DRIVER_BLIND_6STEP_CLOSED_LOOP_HANDOFF_STEPS;

        if (g_motor.sixstep_closed_loop_handoff_steps == 0u)
        {
            MOTOR_UpdateBlindSixStepClosedLoopPwm();
        }
        else
        {
            g_motor.sixstep_pwm_ticks = g_motor.sixstep_pwm_limit_ticks;
        }

        if (bridge_is_coasting == 0u)
        {
            MOTOR_ApplySixStepBridge();
        }
    }
    else
    {
        g_motor.sixstep_closed_loop_handoff_steps = 0u;
    }
}

static MOTOR_FAST_CODE uint8_t MOTOR_ServiceBemfCoastWaitSync(void)
{
    uint8_t sync_count = DRIVER_SIXSTEP_BEMF_COAST_WAIT_SYNC_COUNT;

    if (sync_count == 0u)
    {
        sync_count = 1u;
    }

    if (g_motor.bemf_zero_cross_count >= sync_count)
    {
        return 0u;
    }

    MOTOR_AdvanceSixStepStepIndex();
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_edge_seen = 1u;
    MOTOR_ArmBemfForSixStep();

    return 1u;
}

static MOTOR_FAST_CODE void MOTOR_HandleBemfZeroCross(void)
{
    const uint8_t detected_phase_mask =
        MOTOR_GetBemfPhaseMaskForInputStep(MOTOR_GetBemfInputStepForPwmStep(g_motor.sixstep_step));
    const uint8_t zero_start_active =
        (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_ZERO_BEMF_WAIT) ? 1u : 0u;
    const uint8_t coast_wait_active =
        (g_motor.sixstep_phase == MOTOR_SIXSTEP_PHASE_BEMF_COAST_WAIT) ? 1u : 0u;
    uint16_t captured_interval_ticks = g_motor.bemf_interval_ticks;

    BEMF_AM32_MaskPhaseInterrupts();
    BOARD_EnableBemfPwmSampleIrq(0u);

    if (captured_interval_ticks == 0u)
    {
        captured_interval_ticks = BOARD_GetBemfIntervalTicks();
        g_motor.bemf_interval_ticks = captured_interval_ticks;
    }

    if (zero_start_active != 0u)
    {
        captured_interval_ticks =
            MOTOR_LimitTickCount((uint32_t)captured_interval_ticks << 1u);
        g_motor.bemf_interval_ticks = captured_interval_ticks;
    }
    else if (g_motor.sixstep_missed_zc_blind_steps != 0u)
    {
        captured_interval_ticks =
            MOTOR_LimitTickCount((uint32_t)captured_interval_ticks +
                                 g_motor.sixstep_bemf_delay_ticks);
        g_motor.bemf_interval_ticks = captured_interval_ticks;
        g_motor.sixstep_missed_zc_blind_steps = 0u;
    }

    g_motor.bemf_last_zc_ticks = g_motor.bemf_this_zc_ticks;
    g_motor.bemf_this_zc_ticks = captured_interval_ticks;
    BOARD_ResetBemfIntervalTimer();

    g_motor.bemf_edge_seen = 1u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_phase_mask |= detected_phase_mask;
    g_motor.bemf_led_phase_mask |= detected_phase_mask;

    if (g_motor.bemf_zero_cross_count < 0xFFu)
    {
        g_motor.bemf_zero_cross_count++;
    }

    if (zero_start_active != 0u)
    {
        g_motor.bemf_average_interval_ticks = captured_interval_ticks;
        MOTOR_UpdateBemfMeasuredErpm();
        MOTOR_UpdateBemfFilterLevel();
        MOTOR_UpdateBemfFallbackControlTicks();
        g_motor.bemf_readable = 1u;
        g_motor.sixstep_handoff_coast_done = 1u;
        MOTOR_EnableBemfClosedLoop();
    }
    else if ((coast_wait_active != 0u) &&
        (g_motor.sixstep_handoff_coast_done != 0u) &&
        (MOTOR_isBemfMonitorOnlyMode() == 0u))
    {
        if (MOTOR_ServiceBemfCoastWaitSync() != 0u)
        {
            return;
        }

        MOTOR_UpdateBemfDerivedTiming();
        g_motor.bemf_readable = 1u;
        MOTOR_EnableBemfClosedLoop();
    }
    else
    {
        MOTOR_UpdateBemfDerivedTiming();

        if (MOTOR_isBemfReadableCandidate() != 0u)
        {
            g_motor.bemf_readable = 1u;

            if ((MOTOR_isBemfMonitorOnlyMode() == 0u) &&
                (MOTOR_isBemfLockReady() != 0u))
            {
                if (g_motor.sixstep_handoff_coast_done == 0u)
                {
                    if (MOTOR_isBlindSixStepCoastHandoffSpeed() != 0u)
                    {
                        MOTOR_StartBemfHandoffCoast();
                        return;
                    }

                    g_motor.sixstep_handoff_coast_done = 1u;
                }

                MOTOR_EnableBemfClosedLoop();
            }
        }
    }

    if ((g_motor.sixstep_bemf_closed_loop != 0u) &&
        (MOTOR_isBemfMonitorOnlyMode() == 0u))
    {
        g_motor.sixstep_bemf_delay_ticks =
            MOTOR_GetBemfDelayTicks(g_motor.bemf_this_zc_ticks);
        MOTOR_ScheduleBemfCommutation();
    }
}

static MOTOR_FAST_CODE void MOTOR_EnableBemfCompIrqFromPwmSample(void)
{
    if (MOTOR_isBemfPwmGatedMode() == 0u)
    {
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        return;
    }

    if (g_motor.bemf_blank_ticks != 0u)
    {
        return;
    }

    if ((g_motor.bemf_edge_seen != 0u) ||
        (g_motor.sixstep_commutation_pending != 0u))
    {
        BOARD_EnableBemfPwmSampleIrq(0u);
        return;
    }

    BOARD_EnableBemfPwmSampleIrq(0u);
    BEMF_AM32_MaskPhaseInterrupts();
    BEMF_AM32_EnableCompInterrupts();
}

static uint8_t MOTOR_isSinusReadyForSixStep(void)
{
#if DRIVER_SIN_TO_6STEP_ENABLE
    if (g_motor.in_rpm != 0u)
    {
        if (g_motor.sinus_at_target_ticks < 0xFFFFFFFFu)
        {
            g_motor.sinus_at_target_ticks++;
        }

        if (g_motor.sinus_at_target_ticks >= MOTOR_GetSinToSixStepSettleTicks())
        {
            return 1u;
        }

        return 0u;
    }

    g_motor.sinus_at_target_ticks = 0u;
    return 0u;
#else
    g_motor.sinus_at_target_ticks = 0u;
    return 0u;
#endif
}

void MOTOR_Init(void)
{
    g_motor.mode = MOTOR_MODE_SINUS;
    g_motor.direction = DRIVER_OPEN_LOOP_DIRECTION;
    g_motor.target_rpm = DRIVER_OPEN_LOOP_SINUS_RPM;
    g_motor.ramped_target_rpm = 0;
    g_motor.measured_erpm = 0;
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_step_q16 = 0u;
    g_motor.open_loop_tick = 0u;
    g_motor.sinus_at_target_ticks = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sinus_pwm_limit_ticks = 0u;
    g_motor.sixstep_pwm_limit_ticks = 0u;
    g_motor.sixstep_pwm_ticks = 0u;
    g_motor.sixstep_interval_ticks = 0u;
    g_motor.sixstep_fallback_interval_ticks = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_average_interval_ticks = 1u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_led_hold_ticks = 0u;
    g_motor.sinus_table_index = 0u;
    g_motor.in_rpm = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_blank_ticks = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_step = 0u;
    g_motor.sixstep_handoff_coast_done = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.pwm_pulses_per_sector = 0u;
    g_motor.bemf_led_phase = MOTOR_BEMF_LED_PHASE_NONE;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.bemf_diag_phase = MOTOR_BEMF_LED_PHASE_A;
    g_motor.bemf_diag_initialized = 0u;
    g_motor.bemf_diag_level_a = 0u;
    g_motor.bemf_diag_level_b = 0u;
    g_motor.bemf_diag_level_c = 0u;

    BEMF_AM32_Init(&g_motor.bemf_interval_ticks,
                   &g_motor.bemf_average_interval_ticks,
                   MOTOR_BemfZeroCrossIrq);
    BEMF_AM32_MaskPhaseInterrupts();
    BEMF_AM32_SetFilterLevel(5u);
    MOTOR_UpdateSinusPwmLimit();
    MOTOR_UpdateSixStepPwmLimit();

    if (MOTOR_isBemfLedDiagnosticMode())
    {
        MOTOR_StartBemfLedDiagnostic();
        return;
    }

    if (MOTOR_isBlindSixStepTestMode())
    {
        MOTOR_StartBlindSixStepTest();
        return;
    }

    MOTOR_UpdateSinusTiming(0);
    MOTOR_ApplySinusBridge();
}

void MOTOR_SetTargetRpm(int32_t rpm, motor_direction_t direction)
{
    (void)rpm;
    (void)direction;

    g_motor.target_rpm = DRIVER_OPEN_LOOP_SINUS_RPM;
    g_motor.ramped_target_rpm = 0;
    g_motor.direction = DRIVER_OPEN_LOOP_DIRECTION;
    g_motor.open_loop_tick = 0u;
    g_motor.sinus_at_target_ticks = 0u;
    g_motor.sinus_angle_q16 = 0u;
    g_motor.sinus_table_index = 0u;
    g_motor.pwm_carrier_hz = BOARD_GetPwmCarrierHz();
    g_motor.pwm_pulses_per_sector = 0u;
    g_motor.bemf_readable = 0u;
    g_motor.bemf_zero_cross_count = 0u;
    g_motor.bemf_poll_count = 0u;
    g_motor.bemf_slow_update_tick = 0u;
    g_motor.bemf_edge_seen = 0u;
    g_motor.bemf_armed = 0u;
    g_motor.bemf_phase_mask = 0u;
    g_motor.sixstep_step = 0u;
    g_motor.sixstep_commutation_count = 0u;
    g_motor.sixstep_run_ticks = 0u;
    g_motor.sixstep_fallback_interval_ticks = 0u;
    g_motor.sixstep_relock_scan_steps = 0u;
    g_motor.sixstep_tick = 0u;
    g_motor.sixstep_speed_control_tick = 0u;
    g_motor.sixstep_bemf_delay_ticks = 0u;
    g_motor.bemf_interval_ticks = 0u;
    g_motor.bemf_last_zc_ticks = 0u;
    g_motor.bemf_this_zc_ticks = 0u;
    g_motor.bemf_led_hold_ticks = 0u;
    g_motor.sixstep_bemf_closed_loop = 0u;
    g_motor.sixstep_commutation_pending = 0u;
    g_motor.sixstep_closed_loop_handoff_steps = 0u;
    g_motor.sixstep_phase = MOTOR_SIXSTEP_PHASE_OPEN_LOOP;
    g_motor.sixstep_handoff_coast_done = 0u;
    g_motor.sixstep_missed_zc_blind_steps = 0u;
    g_motor.bemf_led_phase = MOTOR_BEMF_LED_PHASE_NONE;
    g_motor.bemf_led_phase_mask = 0u;
    g_motor.bemf_diag_initialized = 0u;
    MOTOR_RestoreDefaultPwmCarrier();
    BEMF_AM32_MaskPhaseInterrupts();
    BOARD_EnableBemfPwmSampleIrq(0u);
    BOARD_DisableBemfCommutationTimer();
    MOTOR_UpdateSinusTiming(0);
}

void MOTOR_ControlTick10kHz(void)
{
    if (MOTOR_isBemfLedDiagnosticMode())
    {
        MOTOR_BemfLedDiagnosticTick();
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SINUS)
    {
        MOTOR_SinusStepIrq();
        return;
    }

    if (g_motor.mode == MOTOR_MODE_SIXSTEP)
    {
        MOTOR_SixStepTickIrq();
        return;
    }

    BOARD_AllPhasesOff();
}

void MOTOR_SinusStepIrq(void)
{
    const uint32_t align_ticks = MOTOR_GetAlignTicks();
    uint32_t ramp_tick;

    if (g_motor.mode != MOTOR_MODE_SINUS)
    {
        return;
    }

    if (DRIVER_OPEN_LOOP_SINUS_RPM == 0)
    {
        BOARD_AllPhasesOff();
        return;
    }

    if (g_motor.open_loop_tick < align_ticks)
    {
        g_motor.open_loop_tick++;
        g_motor.ramped_target_rpm = 0;
        g_motor.measured_erpm = 0;
        g_motor.sinus_angle_q16 = 0u;
        g_motor.sinus_at_target_ticks = 0u;
        g_motor.sinus_table_index = 0u;
        MOTOR_UpdateSinusTiming(0);
        MOTOR_ApplySinusBridge();
        return;
    }

    ramp_tick = g_motor.open_loop_tick - align_ticks;
    g_motor.open_loop_tick++;
    g_motor.ramped_target_rpm = MOTOR_GetOpenLoopRampedRpm(ramp_tick);
    MOTOR_UpdateSinusTiming(g_motor.ramped_target_rpm);
    MOTOR_AdvanceSinusAngle();

    g_motor.sinus_table_index = (uint16_t)(g_motor.sinus_angle_q16 >> 16u);
    MOTOR_ApplySinusBridge();
    g_motor.measured_erpm = g_motor.ramped_target_rpm * (int32_t)MOTOR_GetPolePairs();
    g_motor.in_rpm = (g_motor.ramped_target_rpm == DRIVER_OPEN_LOOP_SINUS_RPM) ? 1u : 0u;

    if (MOTOR_isSinusReadyForSixStep())
    {
        MOTOR_EnterSixStepFromSinus();
    }
}

MOTOR_FAST_CODE void MOTOR_BemfCommutationTimerIrq(void)
{
    BOARD_DisableBemfCommutationTimer();

    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        return;
    }

    if (g_motor.sixstep_commutation_pending == 0u)
    {
        return;
    }

    MOTOR_CommitBemfCommutation();
}

MOTOR_FAST_CODE void MOTOR_BemfZeroCrossIrq(void)
{
    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        return;
    }

    if (MOTOR_isBemfArmed() == 0u)
    {
        BEMF_AM32_MaskPhaseInterrupts();
        return;
    }

    MOTOR_HandleBemfZeroCross();
}

MOTOR_FAST_CODE void MOTOR_BemfPwmSampleIrq(void)
{
    if (g_motor.mode != MOTOR_MODE_SIXSTEP)
    {
        BOARD_EnableBemfPwmSampleIrq(0u);
        return;
    }

    if (MOTOR_isBemfPwmGatedMode() == 0u)
    {
        BOARD_EnableBemfPwmSampleIrq(0u);
        return;
    }

    MOTOR_EnableBemfCompIrqFromPwmSample();
}
