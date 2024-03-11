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

  // range 0-100
  // 100: full scale
  // 0..11: not allowed for operation
  // values >50 recommended for best results
  // 12 = reset default
  void setGlobalCurrentScaler(uint8_t percent);

  // range 0-100
  // 0 = reset default
  void setRunCurrent(uint8_t percent);
  // range 0-100
  // 0 = reset default
  void setHoldCurrent(uint8_t percent);
  // range 0-100
  void setHoldDelay(uint8_t percent);
  // range 0-100
  void setAllCurrentValues(uint8_t run_current_percent,
    uint8_t hold_current_percent,
    uint8_t hold_delay_percent);

  void enableAutomaticCurrentControl();
  void disableAutomaticCurrentControl();
private:
  Registers * registers_ptr_;
  int16_t hardware_enable_pin_;
  uint8_t toff_;

  const static uint8_t DISABLE_TOFF = 0b0;
  const static uint8_t TOFF_ENABLE_DEFAULT = 3;
  // const static uint8_t HSTART_DEFAULT = 0b101;
  // const static uint8_t HEND_DEFAULT = 0b10;
  // const static uint8_t TBL_DEFAULT = 0b10;
  // const static uint8_t TPFD_DEFAULT = 0b100;
  // const static uint8_t INTERPOLATION_DEFAULT = 0b1;

  // const static uint8_t MRES_256 = 0b0000;
  // const static uint8_t MRES_128 = 0b0001;
  // const static uint8_t MRES_064 = 0b0010;
  // const static uint8_t MRES_032 = 0b0011;
  // const static uint8_t MRES_016 = 0b0100;
  // const static uint8_t MRES_008 = 0b0101;
  // const static uint8_t MRES_004 = 0b0110;
  // const static uint8_t MRES_002 = 0b0111;
  // const static uint8_t MRES_001 = 0b1000;

  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;
  const static uint32_t GLOBAL_SCALER_MIN = 0;
  const static uint32_t GLOBAL_SCALER_THRESHOLD = 32;
  const static uint32_t GLOBAL_SCALER_MAX = 256;
  const static uint8_t CURRENT_SETTING_MIN = 0;
  const static uint8_t CURRENT_SETTING_MAX = 31;
  const static uint8_t HOLD_DELAY_MIN = 0;
  const static uint8_t HOLD_DELAY_MAX = 15;
  const static uint8_t IHOLD_DEFAULT = 16;
  const static uint8_t IRUN_DEFAULT = 31;
  const static uint8_t IHOLDDELAY_DEFAULT = 1;

  void setup(Registers & registers);

  void hardwareEnable();
  void hardwareDisable();
  void softwareEnable();
  void softwareDisable();

  uint8_t percentToGlobalCurrentScaler(uint8_t percent);
  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);
  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

  void minimizeMotorCurrent();

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);

  friend class ::TMC51X0;
};
}
#endif
