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
  Converter();

  void setClockFrequencyMHz(uint8_t clock_frequency_mhz);
  void setMicrostepsPerRealUnit(uint32_t microsteps_per_real_unit);

  int32_t positionChipToReal(int32_t position_chip);
  int32_t positionRealToChip(int32_t position_real);

  uint32_t velocityChipToReal(uint32_t velocity_chip);
  uint32_t velocityRealToChip(uint32_t velocity_real);

  uint32_t tstepToVelocityReal(uint32_t tstep);
  uint32_t velocityRealToTstep(uint32_t velocity_real);

  uint32_t accelerationChipToReal(uint32_t acceleration_chip);
  uint32_t accelerationRealToChip(uint32_t acceleration_chip);

  uint8_t percentToGlobalCurrentScaler(uint8_t percent);

  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);

  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

  uint8_t percentToPwmSetting(uint8_t percent);

private:
  uint8_t clock_frequency_mhz_;
  uint32_t microsteps_per_real_unit_;
  const static uint8_t CLOCK_FREQUENCY_MHZ_DEFAULT = 12;
  const static uint32_t VELOCITY_SCALER = 16777216;
  const static uint32_t ACCELERATION_SCALER = 2199;
  const static uint32_t MICROSTEPS_PER_REAL_UNIT_DEFAULT = 1;
  const static uint32_t DIVISOR_DEFAULT = 1;

  const static uint8_t PERCENT_MIN = 0;
  const static uint8_t PERCENT_MAX = 100;
  const static uint32_t GLOBAL_SCALER_MIN = 0;
  const static uint32_t GLOBAL_SCALER_THRESHOLD = 32;
  const static uint32_t GLOBAL_SCALER_MAX = 256;
  const static uint8_t CURRENT_SETTING_MIN = 0;
  const static uint8_t CURRENT_SETTING_MAX = 31;
  const static uint8_t HOLD_DELAY_MIN = 0;
  const static uint8_t HOLD_DELAY_MAX = 15;
  const static uint8_t PWM_SETTING_MIN = 0;
  const static uint8_t PWM_SETTING_MAX = 255;

  const static uint8_t SEMIN_MIN = 1;
  const static uint8_t SEMIN_MAX = 15;
  const static uint8_t SEMAX_MIN = 0;
  const static uint8_t SEMAX_MAX = 15;

  uint32_t velocityChipToHz(uint32_t velocity_chip);
  uint32_t velocityHzToChip(uint32_t velocity_hz);
  uint32_t velocityRealToHz(uint32_t velocity_real);
  uint32_t velocityHzToReal(uint32_t velocity_hz);

  uint32_t tstepToVelocityHz(uint32_t tstep);
  uint32_t velocityHzToTstep(uint32_t velocity_hz);

  uint32_t accelerationChipToHzPerS(uint32_t acceleration_chip);
  uint32_t accelerationHzPerSToChip(uint32_t acceleration_hz_per_s);

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
};
}
#endif
