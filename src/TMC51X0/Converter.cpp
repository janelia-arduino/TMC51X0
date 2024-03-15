// ----------------------------------------------------------------------------
// Converter.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Converter.hpp"


using namespace tmc51x0;


Converter::Converter()
{
  clock_frequency_mhz_ = CLOCK_FREQUENCY_MHZ_DEFAULT;
  microsteps_to_real_units_count_ = MICROSTEPS_TO_REAL_UNITS_COUNT_DEFAULT;
}

void Converter::setClockFrequencyMHz(uint8_t clock_frequency_mhz)
{
  clock_frequency_mhz_ = clock_frequency_mhz;
}

void Converter::setMicrostepsToRealUnitsCount(uint32_t microsteps_to_real_units_count)
{
  if (microsteps_to_real_units_count == 0)
  {
    microsteps_to_real_units_count = MICROSTEPS_TO_REAL_UNITS_COUNT_DEFAULT;
  }
  microsteps_to_real_units_count_ = microsteps_to_real_units_count;
}

int32_t Converter::positionChipToReal(int32_t position_chip)
{
  int32_t position_real;
  position_real = position_chip / microsteps_to_real_units_count_;
  return position_real;
}

int32_t Converter::positionRealToChip(int32_t position_real)
{
  int32_t position_chip;
  position_chip = position_real * microsteps_to_real_units_count_;
  return position_chip;
}

uint32_t Converter::velocityChipToReal(uint32_t velocity_chip)
{
  uint64_t velocity_real;
  velocity_real = velocityChipToHz(velocity_chip);
  velocity_real = positionChipToReal(velocity_real);
  return velocity_real;
}

uint32_t Converter::velocityRealToChip(uint32_t velocity_real)
{
  uint64_t velocity_chip;
  velocity_chip = positionRealToChip(velocity_real);
  velocity_chip = velocityHzToChip(velocity_chip);
  return velocity_chip;
}

uint32_t Converter::tstepToVelocityReal(uint32_t tstep)
{
  uint64_t velocity_real;
  velocity_real = velocityHzToReal(tstepToVelocityHz(tstep));
  return velocity_real;
}

uint32_t Converter::velocityRealToTstep(uint32_t velocity_real)
{
  uint64_t tstep;
  tstep = velocityHzToTstep(velocityRealToHz(velocity_real));
  return tstep;
}

uint32_t Converter::accelerationChipToReal(uint32_t acceleration_chip)
{
  uint64_t acceleration_real;
  acceleration_real = accelerationChipToHzPerS(acceleration_chip);
  acceleration_real = positionChipToReal(acceleration_real);
  return acceleration_real;
}

uint32_t Converter::accelerationRealToChip(uint32_t acceleration_real)
{
  uint64_t acceleration_chip;
  acceleration_chip = positionRealToChip(acceleration_real);
  acceleration_chip = accelerationHzPerSToChip(acceleration_chip);
  return acceleration_chip;
}

uint8_t Converter::percentToGlobalCurrentScaler(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint16_t scaler = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    GLOBAL_SCALER_MIN,
    GLOBAL_SCALER_MAX);
  if (scaler < GLOBAL_SCALER_THRESHOLD)
  {
    scaler = GLOBAL_SCALER_THRESHOLD;
  }
  else if (scaler >= GLOBAL_SCALER_MAX)
  {
    scaler = 0;
  }
  return scaler;
}

uint8_t Converter::percentToCurrentSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

uint8_t Converter::currentSettingToPercent(uint8_t current_setting)
{
  uint8_t percent = map(current_setting,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

uint8_t Converter::percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay_setting;
}

uint8_t Converter::holdDelaySettingToPercent(uint8_t hold_delay_setting)
{
  uint8_t percent = map(hold_delay_setting,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

uint8_t Converter::percentToPwmSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t pwm_setting = map(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    PWM_SETTING_MIN,
    PWM_SETTING_MAX);
  return pwm_setting;
}

// private

uint32_t Converter::velocityChipToHz(uint32_t velocity_chip)
{
  uint64_t velocity_hz;
  velocity_hz = (uint64_t)velocity_chip * ((uint64_t)clock_frequency_mhz_ * 1000000);
  velocity_hz = velocity_hz / (uint64_t)VELOCITY_SCALER;
  return velocity_hz;
}

uint32_t Converter::velocityHzToChip(uint32_t velocity_hz)
{
  uint64_t velocity_chip;
  velocity_chip = (uint64_t)velocity_hz * (uint64_t)VELOCITY_SCALER;
  velocity_chip = velocity_chip / ((uint64_t)clock_frequency_mhz_ * 1000000);
  return velocity_chip;
}

uint32_t Converter::velocityRealToHz(uint32_t velocity_real)
{
  uint64_t velocity_hz;
  velocity_hz = positionRealToChip(velocity_real);
  return velocity_hz;
}

uint32_t Converter::velocityHzToReal(uint32_t velocity_hz)
{
  uint64_t velocity_real;
  velocity_real = positionChipToReal(velocity_hz);
  return velocity_real;
}

uint32_t Converter::tstepToVelocityHz(uint32_t tstep)
{
  if (tstep == 0)
  {
    tstep = DIVISOR_DEFAULT;
  }
  uint64_t velocity_hz;
  velocity_hz = ((uint64_t)clock_frequency_mhz_ * 1000000) / tstep;
  return velocity_hz;
}

uint32_t Converter::velocityHzToTstep(uint32_t velocity_hz)
{
  if (velocity_hz == 0)
  {
    velocity_hz = DIVISOR_DEFAULT;
  }
  uint64_t tstep;
  tstep = ((uint64_t)clock_frequency_mhz_ * 1000000) / velocity_hz;
  return tstep;
}

uint32_t Converter::accelerationChipToHzPerS(uint32_t acceleration_chip)
{
  uint64_t acceleration_hz_per_s;
  acceleration_hz_per_s = (uint64_t)acceleration_chip * ((uint64_t)clock_frequency_mhz_ * (uint64_t)clock_frequency_mhz_ * 1000);
  acceleration_hz_per_s = acceleration_hz_per_s / (uint64_t)ACCELERATION_SCALER;
  return acceleration_hz_per_s;
}

uint32_t Converter::accelerationHzPerSToChip(uint32_t acceleration_hz_per_s)
{
  uint64_t acceleration_chip;
  acceleration_chip = (uint64_t)acceleration_hz_per_s * (uint64_t)ACCELERATION_SCALER;
  acceleration_chip = acceleration_chip / ((uint64_t)clock_frequency_mhz_ * (uint64_t)clock_frequency_mhz_ * 1000);
  return acceleration_chip;
}

uint32_t Converter::constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}
