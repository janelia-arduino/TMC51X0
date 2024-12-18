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
  FORWARD = 0,
  REVERSE = 1
};
enum StandstillMode
{
  NORMAL=0,
  FREEWHEELING=1,
  PASSIVE_BRAKING_LS=2,
  PASSIVE_BRAKING_HS=3
};
enum ChopperMode
{
  SPREAD_CYCLE = 0,
  CLASSIC = 1
};
enum ComparatorBlankTime
{
  CLOCK_CYCLES_16 = 0,
  CLOCK_CYCLES_24 = 1,
  CLOCK_CYCLES_36 = 2,
  CLOCK_CYCLES_54 = 3
};
struct DriverParameters
{
  uint8_t global_current_scaler = GLOBAL_CURRENT_SCALER_DEFAULT;
  uint8_t run_current = CURRENT_DEFAULT;
  uint8_t hold_current = CURRENT_DEFAULT;
  uint8_t hold_delay = HOLD_DELAY_DEFAULT;
  uint8_t pwm_offset = PWM_DEFAULT;
  uint8_t pwm_gradient = PWM_DEFAULT;
  bool automatic_current_control_enabled = AUTOMATIC_CURRENT_CONTROL_ENABLED_DEFAULT;
  MotorDirection motor_direction = MOTOR_DIRECTION_DEFAULT;
  StandstillMode standstill_mode = STANDSTILL_MODE_DEFAULT;
  ChopperMode chopper_mode = CHOPPER_MODE_DEFAULT;
  uint32_t stealth_chop_threshold = TSTEP_THRESHOLD_DEFAULT;
  bool stealth_chop_enabled = STEALTH_CHOP_ENABLED_DEFAULT;
  uint32_t cool_step_threshold = TSTEP_THRESHOLD_DEFAULT;
  uint8_t cool_step_min = COOL_STEP_MIN_DEFAULT;
  uint8_t cool_step_max = COOL_STEP_MAX_DEFAULT;
  bool cool_step_enabled = COOL_STEP_ENABLED_DEFAULT;
  uint32_t high_velocity_threshold = TSTEP_THRESHOLD_DEFAULT;
  bool high_velocity_fullstep_enabled = HIGH_VELOCITY_FULLSTEP_ENABLED_DEFAULT;
  bool high_velocity_chopper_switch_enabled = HIGH_VELOCITY_CHOPPER_SWITCH_ENABLED_DEFAULT;
  int8_t stall_guard_threshold = STALL_GUARD_THRESHOLD_DEFAULT;
  bool stall_guard_filter_enabled = STALL_GUARD_FILTER_ENABLED_DEFAULT;
  bool short_to_ground_protection_enabled = SHORT_TO_GROUND_PROTECTION_ENABLED_DEFAULT;
  uint8_t enabled_toff = ENABLED_TOFF_DEFAULT;
  ComparatorBlankTime comparator_blank_time = COMPARATOR_BLANK_TIME_DEFAULT;
  uint16_t dc_time = DC_TIME_DEFAULT;
  uint8_t dc_stall_guard_threshold = DC_STALL_GUARD_THRESHOLD_DEFAULT;

  DriverParameters(uint8_t global_current_scaler_ = GLOBAL_CURRENT_SCALER_DEFAULT,
    uint8_t run_current_ = CURRENT_DEFAULT,
    uint8_t hold_current_ = CURRENT_DEFAULT,
    uint8_t hold_delay_ = HOLD_DELAY_DEFAULT,
    uint8_t pwm_offset_ = PWM_DEFAULT,
    uint8_t pwm_gradient_ = PWM_DEFAULT,
    bool automatic_current_control_enabled_ = AUTOMATIC_CURRENT_CONTROL_ENABLED_DEFAULT,
    MotorDirection motor_direction_ = MOTOR_DIRECTION_DEFAULT,
    StandstillMode standstill_mode_ = STANDSTILL_MODE_DEFAULT,
    ChopperMode chopper_mode_ = CHOPPER_MODE_DEFAULT,
    uint32_t stealth_chop_threshold_ = TSTEP_THRESHOLD_DEFAULT,
    bool stealth_chop_enabled_ = STEALTH_CHOP_ENABLED_DEFAULT,
    uint32_t cool_step_threshold_ = TSTEP_THRESHOLD_DEFAULT,
    uint8_t cool_step_min_ = COOL_STEP_MIN_DEFAULT,
    uint8_t cool_step_max_ = COOL_STEP_MAX_DEFAULT,
    bool cool_step_enabled_ = COOL_STEP_ENABLED_DEFAULT,
    uint32_t high_velocity_threshold_ = TSTEP_THRESHOLD_DEFAULT,
    bool high_velocity_fullstep_enabled_ = HIGH_VELOCITY_FULLSTEP_ENABLED_DEFAULT,
    bool high_velocity_chopper_switch_enabled_ = HIGH_VELOCITY_CHOPPER_SWITCH_ENABLED_DEFAULT,
    int8_t stall_guard_threshold_ = STALL_GUARD_THRESHOLD_DEFAULT,
    bool stall_guard_filter_enabled_ = STALL_GUARD_FILTER_ENABLED_DEFAULT,
    bool short_to_ground_protection_enabled_ = SHORT_TO_GROUND_PROTECTION_ENABLED_DEFAULT,
    uint8_t enabled_toff_ = ENABLED_TOFF_DEFAULT,
    ComparatorBlankTime comparator_blank_time_ = COMPARATOR_BLANK_TIME_DEFAULT,
    uint16_t dc_time_ = DC_TIME_DEFAULT,
    uint8_t dc_stall_guard_threshold_ = DC_STALL_GUARD_THRESHOLD_DEFAULT)
  {
    global_current_scaler = global_current_scaler_;
    run_current = run_current_;
    hold_current = hold_current_;
    hold_delay = hold_delay_;
    pwm_offset = pwm_offset_;
    pwm_gradient = pwm_gradient_;
    automatic_current_control_enabled = automatic_current_control_enabled_;
    motor_direction = motor_direction_;
    standstill_mode = standstill_mode_;
    chopper_mode = chopper_mode_;
    stealth_chop_threshold = stealth_chop_threshold_;
    stealth_chop_enabled = stealth_chop_enabled_;
    cool_step_threshold = cool_step_threshold_;
    cool_step_min = cool_step_min_;
    cool_step_max = cool_step_max_;
    cool_step_enabled = cool_step_enabled_;
    high_velocity_threshold = high_velocity_threshold_;
    high_velocity_fullstep_enabled = high_velocity_fullstep_enabled_;
    high_velocity_chopper_switch_enabled = high_velocity_chopper_switch_enabled_;
    stall_guard_threshold = stall_guard_threshold_;
    stall_guard_filter_enabled = stall_guard_filter_enabled_;
    short_to_ground_protection_enabled = short_to_ground_protection_enabled_;
    enabled_toff = enabled_toff_;
    comparator_blank_time = comparator_blank_time_;
    dc_time = dc_time_;
    dc_stall_guard_threshold = dc_stall_guard_threshold_;
  };

  bool operator==(const DriverParameters & rhs) const
  {
    if ((this->global_current_scaler == rhs.global_current_scaler) &&
      (this->run_current == rhs.run_current) &&
      (this->hold_current == rhs.hold_current) &&
      (this->hold_delay == rhs.hold_delay) &&
      (this->pwm_offset == rhs.pwm_offset) &&
      (this->pwm_gradient == rhs.pwm_gradient) &&
      (this->automatic_current_control_enabled == rhs.automatic_current_control_enabled) &&
      (this->motor_direction == rhs.motor_direction) &&
      (this->standstill_mode == rhs.standstill_mode) &&
      (this->chopper_mode == rhs.chopper_mode) &&
      (this->stealth_chop_threshold == rhs.stealth_chop_threshold) &&
      (this->stealth_chop_enabled == rhs.stealth_chop_enabled) &&
      (this->cool_step_threshold == rhs.cool_step_threshold) &&
      (this->cool_step_min == rhs.cool_step_min) &&
      (this->cool_step_max == rhs.cool_step_max) &&
      (this->cool_step_enabled == rhs.cool_step_enabled) &&
      (this->high_velocity_threshold == rhs.high_velocity_threshold) &&
      (this->high_velocity_fullstep_enabled == rhs.high_velocity_fullstep_enabled) &&
      (this->high_velocity_chopper_switch_enabled == rhs.high_velocity_chopper_switch_enabled) &&
      (this->stall_guard_threshold == rhs.stall_guard_threshold) &&
      (this->stall_guard_filter_enabled == rhs.stall_guard_filter_enabled) &&
      (this->short_to_ground_protection_enabled == rhs.short_to_ground_protection_enabled) &&
      (this->enabled_toff == rhs.enabled_toff) &&
      (this->comparator_blank_time == rhs.comparator_blank_time) &&
      (this->dc_time == rhs.dc_time) &&
      (this->dc_stall_guard_threshold == rhs.dc_stall_guard_threshold))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const DriverParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint8_t GLOBAL_CURRENT_SCALER_DEFAULT = 0;
  const static uint8_t CURRENT_DEFAULT = 0;
  const static uint8_t HOLD_DELAY_DEFAULT = 0;
  const static uint32_t PWM_DEFAULT = 0;
  const static bool AUTOMATIC_CURRENT_CONTROL_ENABLED_DEFAULT = false;
  const static MotorDirection MOTOR_DIRECTION_DEFAULT = FORWARD;
  const static StandstillMode STANDSTILL_MODE_DEFAULT = NORMAL;
  const static ChopperMode CHOPPER_MODE_DEFAULT = SPREAD_CYCLE;
  const static uint32_t TSTEP_THRESHOLD_DEFAULT = 0;
  const static bool STEALTH_CHOP_ENABLED_DEFAULT = true;
  const static uint8_t COOL_STEP_MIN_DEFAULT = 1;
  const static uint8_t COOL_STEP_MAX_DEFAULT = 0;
  const static bool COOL_STEP_ENABLED_DEFAULT = true;
  const static bool HIGH_VELOCITY_FULLSTEP_ENABLED_DEFAULT = true;
  const static bool HIGH_VELOCITY_CHOPPER_SWITCH_ENABLED_DEFAULT = true;
  const static int8_t STALL_GUARD_THRESHOLD_DEFAULT = 0;
  const static bool STALL_GUARD_FILTER_ENABLED_DEFAULT = true;
  const static bool SHORT_TO_GROUND_PROTECTION_ENABLED_DEFAULT = true;
  const static uint8_t ENABLED_TOFF_DEFAULT = 3;
  const static ComparatorBlankTime COMPARATOR_BLANK_TIME_DEFAULT = CLOCK_CYCLES_36;
  const static uint16_t DC_TIME_DEFAULT = 0;
  const static uint8_t DC_STALL_GUARD_THRESHOLD_DEFAULT = 0;

  friend class Driver;
};
}
#endif
