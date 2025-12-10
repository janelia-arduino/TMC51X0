// ----------------------------------------------------------------------------
// DriverParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_PARAMETERS_HPP
#define TMC51X0_DRIVER_PARAMETERS_HPP

#include "Registers.hpp"

class TMC51X0;

namespace tmc51x0
{
enum MotorDirection
{
  ForwardDirection = 0,
  ReverseDirection = 1,
};

enum StandstillMode
{
  NormalMode           = 0,
  FreewheelingMode     = 1,
  PassiveBrakingLsMode = 2,
  PassiveBrakingHsMode = 3,
};

enum ChopperMode
{
  SpreadCycleMode = 0,
  ClassicMode     = 1,
};

enum ComparatorBlankTime
{
  ClockCycles16 = 0,
  ClockCycles24 = 1,
  ClockCycles36 = 2,
  ClockCycles54 = 3,
};

struct DriverParameters
{
  // global_current_scaler only available on TMC5160
  uint8_t             global_current_scaler;
  uint8_t             run_current;
  uint8_t             hold_current;
  uint8_t             hold_delay;
  uint8_t             pwm_offset;
  uint8_t             pwm_gradient;
  bool                automatic_current_control_enabled;
  MotorDirection      motor_direction;
  StandstillMode      standstill_mode;
  ChopperMode         chopper_mode;
  uint32_t            stealth_chop_threshold;
  bool                stealth_chop_enabled;
  uint32_t            cool_step_threshold;
  uint8_t             cool_step_min;
  uint8_t             cool_step_max;
  bool                cool_step_enabled;
  uint32_t            high_velocity_threshold;
  bool                high_velocity_fullstep_enabled;
  bool                high_velocity_chopper_switch_enabled;
  int8_t              stall_guard_threshold;
  bool                stall_guard_filter_enabled;
  bool                short_to_ground_protection_enabled;
  uint8_t             enabled_toff;
  ComparatorBlankTime comparator_blank_time;
  uint16_t            dc_time;
  uint8_t             dc_stall_guard_threshold;

  // Base constructor – all defaults kept identical to your original struct
  constexpr DriverParameters(
    uint8_t             global_current_scaler                = 100,
    uint8_t             run_current                          = 50,
    uint8_t             hold_current                         = 20,
    uint8_t             hold_delay                           = 5,
    uint8_t             pwm_offset                           = 25,
    uint8_t             pwm_gradient                         = 5,
    bool                automatic_current_control_enabled    = false,
    MotorDirection      motor_direction                      = ForwardDirection,
    StandstillMode      standstill_mode                      = NormalMode,
    ChopperMode         chopper_mode                         = SpreadCycleMode,
    uint32_t            stealth_chop_threshold               = 100,
    bool                stealth_chop_enabled                 = true,
    uint32_t            cool_step_threshold                  = 150,
    uint8_t             cool_step_min                        = 1,
    uint8_t             cool_step_max                        = 0,
    bool                cool_step_enabled                    = false,
    uint32_t            high_velocity_threshold              = 200,
    bool                high_velocity_fullstep_enabled       = false,
    bool                high_velocity_chopper_switch_enabled = false,
    int8_t              stall_guard_threshold                = 0,
    bool                stall_guard_filter_enabled           = false,
    bool                short_to_ground_protection_enabled   = true,
    uint8_t             enabled_toff                         = 3,
    ComparatorBlankTime comparator_blank_time                = ClockCycles36,
    uint16_t            dc_time                              = 0,
    uint8_t             dc_stall_guard_threshold             = 0)
  : global_current_scaler(global_current_scaler),
    run_current(run_current),
    hold_current(hold_current),
    hold_delay(hold_delay),
    pwm_offset(pwm_offset),
    pwm_gradient(pwm_gradient),
    automatic_current_control_enabled(automatic_current_control_enabled),
    motor_direction(motor_direction),
    standstill_mode(standstill_mode),
    chopper_mode(chopper_mode),
    stealth_chop_threshold(stealth_chop_threshold),
    stealth_chop_enabled(stealth_chop_enabled),
    cool_step_threshold(cool_step_threshold),
    cool_step_min(cool_step_min),
    cool_step_max(cool_step_max),
    cool_step_enabled(cool_step_enabled),
    high_velocity_threshold(high_velocity_threshold),
    high_velocity_fullstep_enabled(high_velocity_fullstep_enabled),
    high_velocity_chopper_switch_enabled(high_velocity_chopper_switch_enabled),
    stall_guard_threshold(stall_guard_threshold),
    stall_guard_filter_enabled(stall_guard_filter_enabled),
    short_to_ground_protection_enabled(short_to_ground_protection_enabled),
    enabled_toff(enabled_toff),
    comparator_blank_time(comparator_blank_time),
    dc_time(dc_time),
    dc_stall_guard_threshold(dc_stall_guard_threshold)
  {}

  // --- "Named parameter" style helpers: each returns a new instance ---

  constexpr DriverParameters withGlobalCurrentScaler(uint8_t value) const
  {
    return DriverParameters(
      value,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withRunCurrent(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      value,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withHoldCurrent(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      value,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withHoldDelay(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      value,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withPwmOffset(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      value,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withPwmGradient(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      value,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withAutomaticCurrentControlEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      value,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withMotorDirection(MotorDirection value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      value,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withStandstillMode(StandstillMode value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      value,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withChopperMode(ChopperMode value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      value,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withStealthChopThreshold(uint32_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      value,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withStealthChopEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      value,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withCoolStepThreshold(uint32_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      value,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withCoolStepMin(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      value,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withCoolStepMax(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      value,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withCoolStepEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      value,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withHighVelocityThreshold(uint32_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      value,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withHighVelocityFullstepEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      value,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withHighVelocityChopperSwitchEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      value,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withStallGuardThreshold(int8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      value,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withStallGuardFilterEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      value,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withShortToGroundProtectionEnabled(bool value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      value,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withEnabledToff(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      value,
      comparator_blank_time,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withComparatorBlankTime(ComparatorBlankTime value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      value,
      dc_time,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withDcTime(uint16_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      value,
      dc_stall_guard_threshold);
  }

  constexpr DriverParameters withDcStallGuardThreshold(uint8_t value) const
  {
    return DriverParameters(
      global_current_scaler,
      run_current,
      hold_current,
      hold_delay,
      pwm_offset,
      pwm_gradient,
      automatic_current_control_enabled,
      motor_direction,
      standstill_mode,
      chopper_mode,
      stealth_chop_threshold,
      stealth_chop_enabled,
      cool_step_threshold,
      cool_step_min,
      cool_step_max,
      cool_step_enabled,
      high_velocity_threshold,
      high_velocity_fullstep_enabled,
      high_velocity_chopper_switch_enabled,
      stall_guard_threshold,
      stall_guard_filter_enabled,
      short_to_ground_protection_enabled,
      enabled_toff,
      comparator_blank_time,
      dc_time,
      value);
  }
};

} // namespace tmc51x0

#endif // TMC51X0_DRIVER_PARAMETERS_HPP
