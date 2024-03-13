// ----------------------------------------------------------------------------
// Driver.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_HPP
#define TMC51X0_DRIVER_HPP
#include <Arduino.h>

#include "TMC51X0/Constants.hpp"
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
  void setHardwareEnablePin(size_t hardware_enable_pin);
  void enable();
  void disable();

  // 0: full scale
  // 1..31: not allowed for operation
  // 32..255: 32/256..255/256 of maximum current
  // >128 recommended for best results
  // reset default: 129
  void setGlobalCurrentScaler(uint8_t scaler);

  // 0..31: 1/32..32/32
  // 16..31: for best microstep performance
  // reset default: 0
  void setRunCurrent(uint8_t run_current);
  // 0..31: 1/32..32/32
  // reset default: 0
  void setHoldCurrent(uint8_t hold_current);
  // 0: instant power down
  // 1..15: delay per current reduction step in multiple of 2^18 clocks
  void setHoldDelay(uint8_t hold_delay);

  void enableAutomaticCurrentScaling();
  void disableAutomaticCurrentScaling();
  void enableAutomaticGradientAdaptation();
  void disableAutomaticGradientAdaptation();

  // range 0..255
  void setPwmOffset(uint8_t pwm_amplitude);
  // range 0..255
  void setPwmGradient(uint8_t pwm_amplitude);

  void enableStealthChop();
  void disableStealthChop();

  // 0 indifferent value
  // 1..63 less sensitivity
  // -1..-64 higher sensitivity
  void setStallGuardThreshold(int8_t threshold);

  // minimum: 1..15
  // maximum: 0..15
  void enableCoolStep(uint8_t minimum=1,
    uint8_t maximum=0);
  void disableCoolStep();

private:
  Registers * registers_ptr_;
  Converter * converter_ptr_;
  int16_t hardware_enable_pin_;
  uint8_t toff_;

  const static uint32_t GLOBAL_SCALER_DEFAULT = 129;
  const static uint32_t CURRENT_SETTING_DEFAULT = 0;

  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;

  const static uint8_t SEMIN_OFF = 0;

  void setup(Registers & registers,
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
