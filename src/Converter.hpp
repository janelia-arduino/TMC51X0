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

  struct Settings
  {
    uint8_t clock_frequency_mhz = CLOCK_FREQUENCY_MHZ_DEFAULT;
    int32_t microsteps_per_real_position_unit = MICROSTEPS_PER_REAL_POSITION_UNIT_DEFAULT;
    int32_t hz_per_real_velocity_unit = HZ_PER_REAL_VELOCITY_UNIT_DEFAULT;
  };
  void setup(Settings settings);

  int32_t positionChipToReal(int32_t position_chip);
  int32_t positionRealToChip(int32_t position_real);

  int32_t velocityChipToReal(int32_t velocity_chip);
  int32_t velocityRealToChip(int32_t velocity_real);

  int32_t tstepToVelocityReal(int32_t tstep);
  int32_t velocityRealToTstep(int32_t velocity_real);

  int32_t accelerationChipToReal(int32_t acceleration_chip);
  int32_t accelerationRealToChip(int32_t acceleration_chip);

  uint8_t percentToGlobalCurrentScaler(uint8_t percent);

  uint8_t percentToCurrentSetting(uint8_t percent);
  uint8_t currentSettingToPercent(uint8_t current_setting);

  uint8_t percentToHoldDelaySetting(uint8_t percent);
  uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

  uint8_t percentToPwmSetting(uint8_t percent);

  uint16_t millisecondsToTzerowait(uint16_t milliseconds);

private:
  uint8_t clock_frequency_mhz_;
  uint8_t clock_duration_ns_;
  int32_t microsteps_per_real_position_unit_;
  int32_t hz_per_real_velocity_unit_;
  const static uint8_t CLOCK_FREQUENCY_MHZ_DEFAULT = 12;
  const static int32_t MICROSTEPS_PER_REAL_POSITION_UNIT_DEFAULT = 1;
  const static int32_t HZ_PER_REAL_VELOCITY_UNIT_DEFAULT = 1;

  const static uint16_t CLOCK_FREQUENCY_TO_DURATION_SCALER = 1000;
  const static uint32_t MILLISECONDS_PER_SECOND = 1000000;
  const static uint32_t TZEROWAIT_SCALER = 512;
  const static int64_t VELOCITY_SCALER = 16777216;
  const static int64_t ACCELERATION_SCALER = 2199;
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

  void setClockFrequencyMHz(uint8_t clock_frequency_mhz);
  void setMicrostepsPerRealPositionUnit(int32_t microsteps_per_real_position_unit);
  void setHzPerRealVelocityUnit(int32_t hz_per_real_velocity_unit);

  int32_t velocityChipToHz(int32_t velocity_chip);
  int32_t velocityHzToChip(int32_t velocity_hz);
  int32_t velocityRealToHz(int32_t velocity_real);
  int32_t velocityHzToReal(int32_t velocity_hz);

  int32_t tstepToVelocityHz(int32_t tstep);
  int32_t velocityHzToTstep(int32_t velocity_hz);

  int32_t accelerationChipToHzPerS(int32_t acceleration_chip);
  int32_t accelerationHzPerSToChip(int32_t acceleration_hz_per_s);

  uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
};
}
#endif
