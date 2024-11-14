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
#include "Converter.hpp"


class TMC51X0;

namespace tmc51x0
{
class Driver
{
public:
  Driver();

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

  void enableStealthChop();
  void disableStealthChop();

  // range 0..255
  void writePwmOffset(uint8_t pwm_amplitude);
  // range 0..255
  void writePwmGradient(uint8_t pwm_amplitude);

  // autograd = automatic gradient adaptation
  // pwm_reg range 1..15 -> slowest regulation..fastest regulation
  void enableAutomaticCurrentControl(bool autograd=true,
    uint8_t pwm_reg=4);
  void disableAutomaticCurrentControl();

  enum StandstillMode
  {
    NORMAL=0,
    FREEWHEELING=1,
    PASSIVE_BRAKING_LS=2,
    PASSIVE_BRAKING_HS=3,
  };
  // only available with StealthChop enabled
  void writeStandstillMode(StandstillMode mode);

  enum MotorDirection
  {
    FORWARD = 0,
    REVERSE = 1,
  };
  void writeMotorDirection(MotorDirection motor_direction);

  enum ChopperMode
  {
    SPREAD_CYCLE = 0,
    CLASSIC = 1,
  };
  void writeChopperMode(ChopperMode chopper_mode);

  void writeStealthChopThreshold(uint32_t tstep);
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
  Converter * converter_ptr_;
  int16_t hardware_enable_pin_;
  uint8_t toff_;

  const static uint32_t GLOBAL_SCALER_DEFAULT = 129;
  const static uint32_t CURRENT_SETTING_DEFAULT = 0;

  const static uint32_t PWM_SETTING_DEFAULT = 0;
  const static uint32_t TSTEP_THRESHOLD_DEFAULT = 0;

  const static StandstillMode STANDSTILL_MODE_DEFAULT = NORMAL;
  const static MotorDirection MOTOR_DIRECTION_DEFAULT = FORWARD;
  const static ChopperMode CHOPPER_MODE_DEFAULT = SPREAD_CYCLE;

  const static uint32_t STALL_GUARD_THRESHOLD_DEFAULT = 0;
  const static uint8_t STALL_GUARD_FILTER_ENABLE = 1;
  const static uint8_t STALL_GUARD_FILTER_DISABLE = 0;

  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;

  const static uint8_t SEMIN_OFF = 0;

  void initialize(Registers & registers,
    Converter & converter);

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();

  void minimizeMotorCurrent();

  friend class ::TMC51X0;
};
}
#endif
