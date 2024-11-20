// ----------------------------------------------------------------------------
// Driver.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_HPP
#define TMC51X0_DRIVER_HPP
#include <Arduino.h>

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
enum MotorDirection
{
  FORWARD = 0,
  REVERSE = 1,
};
enum StandstillMode
{
  NORMAL=0,
  FREEWHEELING=1,
  PASSIVE_BRAKING_LS=2,
  PASSIVE_BRAKING_HS=3,
};
enum ChopperMode
{
  SPREAD_CYCLE = 0,
  CLASSIC = 1,
};
struct DriverParameters
{
  uint8_t global_current_scalar = GLOBAL_CURRENT_SCALAR_DEFAULT;
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

  DriverParameters(uint8_t global_current_scalar_ = GLOBAL_CURRENT_SCALAR_DEFAULT,
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
    bool short_to_ground_protection_enabled_ = SHORT_TO_GROUND_PROTECTION_ENABLED_DEFAULT)
  {
    global_current_scalar = global_current_scalar_;
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
  };

  bool operator==(const DriverParameters & rhs) const
  {
    if ((this->global_current_scalar == rhs.global_current_scalar) &&
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
      (this->short_to_ground_protection_enabled == rhs.short_to_ground_protection_enabled))
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
  const static uint8_t GLOBAL_CURRENT_SCALAR_DEFAULT = 129;
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

  friend class Driver;
};

class Driver
{
public:
  Driver();

  void setup(DriverParameters driver_parameters);

  // driver must be enabled before use it is disabled by default
  void setEnableHardwarePin(size_t hardware_enable_pin);
  void enable();
  void disable();

  // 0: full scale
  // 1..31: not allowed for operation
  // 32..255: 32/256..255/256 of maximum current
  // >128 recommended for best results
  // reset default: 129
  void writeGlobalCurrentScaler(uint8_t scaler);

  // 0..31: 1/32..32/32
  // 16..31: for best microstep performance
  // reset default: 0
  void writeRunCurrent(uint8_t run_current);
  // 0..31: 1/32..32/32
  // reset default: 0
  void writeHoldCurrent(uint8_t hold_current);
  // 0: instant power down
  // 1..15: delay per current reduction step in multiple of 2^18 clocks
  void writeHoldDelay(uint8_t hold_delay);

  // range 0..255
  void writePwmOffset(uint8_t pwm_amplitude);
  // range 0..255
  void writePwmGradient(uint8_t pwm_amplitude);

  // autograd = automatic gradient adaptation
  // pwm_reg range 1..15 -> slowest regulation..fastest regulation
  void enableAutomaticCurrentControl(bool autograd=true,
    uint8_t pwm_reg=4);
  void disableAutomaticCurrentControl();

  void writeMotorDirection(MotorDirection motor_direction);

  // only available with StealthChop enabled
  void writeStandstillMode(StandstillMode mode);

  void writeChopperMode(ChopperMode chopper_mode);

  void writeStealthChopThreshold(uint32_t tstep);
  void enableStealthChop();
  void disableStealthChop();

  void writeCoolStepThreshold(uint32_t tstep);
  // min: 1..15
  // max: 0..15
  void enableCoolStep(uint8_t min=1,
    uint8_t max=0);
  void disableCoolStep();

  void writeHighVelocityThreshold(uint32_t tstep);

  void enableHighVelocityFullstep();
  void disableHighVelocityFullstep();
  void enableHighVelocityChopperSwitch();
  void disableHighVelocityChopperSwitch();

  // 0 indifferent value
  // 1..63 less sensitivity
  // -1..-64 higher sensitivity
  void writeStallGuardThreshold(int8_t threshold);
  void enableStallGuardFilter();
  void disableStallGuardFilter();
  bool stalled();
  uint16_t readStallGuardResult();

  uint8_t readActualCurrentScaling();

  void enableShortToGroundProtection();
  void disableShortToGroundProtection();

private:
  Registers * registers_ptr_;
  int16_t hardware_enable_pin_;
  uint8_t toff_;

  const static uint8_t STALL_GUARD_FILTER_ENABLE = 1;
  const static uint8_t STALL_GUARD_FILTER_DISABLE = 0;

  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;

  const static uint8_t SEMIN_OFF = 0;

  void initialize(Registers & registers);

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();

  friend class ::TMC51X0;
};
}
#endif
