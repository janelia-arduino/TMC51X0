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
#include "TMC51X0/DriverParameters.hpp"


class TMC51X0;

namespace tmc51x0
{
class Driver
{
public:
  Driver();

  void setup();
  void setup(DriverParameters parameters);

  // driver must be enabled before use it is disabled by default
  void setEnableHardwarePin(size_t hardware_enable_pin);
  void enable();
  void disable();

  // 0: full scale
  // 1..31: not allowed for operation
  // 32..255: 32/256..255/256 of maximum current
  // >128 recommended for best results
  // reset default: 0
  // only available on the TMC5160
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
  // stall guard result
  // 0 highest load
  // low value high load
  // high value less load
  uint16_t readStallGuardResult();

  uint8_t readActualCurrentScaling();

  void enableShortToGroundProtection();
  void disableShortToGroundProtection();

private:
  Registers * registers_ptr_;
  DriverParameters setup_driver_parameters_;
  DriverParameters cached_driver_settings_;

  size_t hardware_enable_pin_;
  uint8_t toff_;

  const static uint8_t STALL_GUARD_FILTER_ENABLE = 1;
  const static uint8_t STALL_GUARD_FILTER_DISABLE = 0;

  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;

  const static uint8_t SEMIN_OFF = 0;

  void initialize(Registers & registers);
  void reinitialize();
  void writeDriverParameters(DriverParameters parameters);
  void cacheDriverSettings();
  void restoreDriverSettings();

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();

  friend class ::TMC51X0;
};
}
#endif
