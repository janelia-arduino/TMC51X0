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

  DriverParameters(uint8_t global_current_scalar_ = GLOBAL_CURRENT_SCALAR_DEFAULT,
    uint8_t run_current_ = CURRENT_DEFAULT,
    uint8_t hold_current_ = CURRENT_DEFAULT,
    uint8_t hold_delay_ = HOLD_DELAY_DEFAULT,
    uint8_t pwm_offset_ = PWM_DEFAULT,
    uint8_t pwm_gradient_ = PWM_DEFAULT,
    bool automatic_current_control_enabled_ = AUTOMATIC_CURRENT_CONTROL_ENABLED_DEFAULT,
    MotorDirection motor_direction_ = MOTOR_DIRECTION_DEFAULT,
    StandstillMode standstill_mode_ = STANDSTILL_MODE_DEFAULT,
    ChopperMode chopper_mode_ = CHOPPER_MODE_DEFAULT)
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
      (this->chopper_mode == rhs.chopper_mode))
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
  void writeHighVelocityThreshold(uint32_t tstep);

  void enableHighVelocityFullstep();
  void disableHighVelocityFullstep();
  void enableHighVelocityChopperSwitch();
  void disableHighVelocityChopperSwitch();

  // minimum: 1..15
  // maximum: 0..15
  void enableCoolStep(uint8_t minimum=1,
    uint8_t maximum=0);
  void disableCoolStep();

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

  const static uint32_t TSTEP_THRESHOLD_DEFAULT = 0;

  const static uint32_t STALL_GUARD_THRESHOLD_DEFAULT = 0;
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
