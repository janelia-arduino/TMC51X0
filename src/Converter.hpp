// ----------------------------------------------------------------------------
// Converter.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONVERTER_HPP
#define TMC51X0_CONVERTER_HPP
#include <Arduino.h>


namespace tmc51x0
{
class Converter
{
public:
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

  const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
  const static uint8_t SEIMIN_LOWER_SETTING = 0;
  const static uint8_t SEIMIN_UPPER_SETTING = 1;
  const static uint8_t SEMIN_MIN = 1;
  const static uint8_t SEMIN_MAX = 15;
  const static uint8_t SEMAX_MIN = 0;
  const static uint8_t SEMAX_MAX = 15;

  void setup(uint8_t clock_frequency_mhz);

  uint8_t percentToGlobalCurrentScaler(uint8_t percent);
  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);
  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

private:
  uint8_t clock_frequency_mhz_;

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
};
}
#endif
