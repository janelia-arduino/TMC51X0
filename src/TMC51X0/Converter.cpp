// ----------------------------------------------------------------------------
// Converter.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Converter.hpp"


using namespace tmc51x0;


void Converter::setup(ConverterParameters converter_parameters)
{
  converter_parameters_ = converter_parameters;
}

DriverParameters Converter::driverParametersRealToChip(DriverParameters dp_real)
{
  DriverParameters dp_chip = dp_real;
  dp_chip.global_current_scaler = percentToGlobalCurrentScaler(dp_real.global_current_scaler);
  dp_chip.run_current = percentToCurrentSetting(dp_real.run_current);
  dp_chip.hold_current = percentToCurrentSetting(dp_real.hold_current);
  dp_chip.hold_delay = percentToHoldDelaySetting(dp_real.hold_delay);
  dp_chip.pwm_offset = percentToPwmSetting(dp_real.pwm_offset);
  dp_chip.pwm_gradient = percentToPwmSetting(dp_real.pwm_gradient);
  dp_chip.stealth_chop_threshold = velocityRealToTstep(dp_real.stealth_chop_threshold);
  dp_chip.cool_step_threshold = velocityRealToTstep(dp_real.cool_step_threshold);
  dp_chip.high_velocity_threshold = velocityRealToTstep(dp_real.high_velocity_threshold);

  return dp_chip;
}

ControllerParameters Converter::controllerParametersRealToChip(ControllerParameters cp_real)
{
  ControllerParameters cp_chip = cp_real;
  cp_chip.max_velocity = velocityRealToChip(cp_real.max_velocity);
  cp_chip.max_acceleration = accelerationRealToChip(cp_real.max_acceleration);
  cp_chip.start_velocity = velocityRealToChip(cp_real.start_velocity);
  cp_chip.stop_velocity = velocityRealToChip(cp_real.stop_velocity);
  cp_chip.first_velocity = velocityRealToChip(cp_real.first_velocity);
  cp_chip.first_acceleration = accelerationRealToChip(cp_real.first_acceleration);
  cp_chip.max_deceleration = accelerationRealToChip(cp_real.max_deceleration);
  cp_chip.first_deceleration = accelerationRealToChip(cp_real.first_deceleration);
  cp_chip.zero_wait_duration = millisecondsToTzerowait(cp_real.zero_wait_duration);

  return cp_chip;
}

HomeParameters Converter::homeParametersRealToChip(HomeParameters hp_real)
{
  HomeParameters hp_chip = hp_real;
  hp_chip.target_position = positionRealToChip(hp_real.target_position);
  hp_chip.velocity = velocityRealToChip(hp_real.velocity);
  hp_chip.acceleration = accelerationRealToChip(hp_real.acceleration);
  hp_chip.run_current = percentToCurrentSetting(hp_real.run_current);
  hp_chip.pwm_offset = percentToPwmSetting(hp_real.pwm_offset);

  return hp_chip;
}

int32_t Converter::positionChipToReal(int32_t position_chip)
{
  return position_chip / (int32_t)converter_parameters_.microsteps_per_real_position_unit;
}

int32_t Converter::positionRealToChip(int32_t position_real)
{
  return position_real * (int32_t)converter_parameters_.microsteps_per_real_position_unit;
}

int32_t Converter::velocityChipToReal(int32_t velocity_chip)
{
  return velocityHzToReal(velocityChipToHz(velocity_chip));
}

int32_t Converter::velocityRealToChip(int32_t velocity_real)
{
  return velocityHzToChip(velocityRealToHz(velocity_real));
}

int32_t Converter::tstepToVelocityReal(int32_t tstep)
{
  return velocityHzToReal(tstepToVelocityHz(tstep));
}

int32_t Converter::velocityRealToTstep(int32_t velocity_real)
{
  return velocityHzToTstep(velocityRealToHz(velocity_real));
}

int32_t Converter::accelerationChipToReal(int32_t acceleration_chip)
{
  return accelerationHzPerSToReal(accelerationChipToHzPerS(acceleration_chip));
}

int32_t Converter::accelerationRealToChip(int32_t acceleration_real)
{
  return accelerationHzPerSToChip(accelerationRealToHzPerS(acceleration_real));
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

uint16_t Converter::millisecondsToTzerowait(uint16_t milliseconds)
{
  return ((uint32_t)milliseconds * MILLISECONDS_PER_SECOND) / (TZEROWAIT_SCALER * converter_parameters_.clock_duration_ns);
}

// private

int32_t Converter::velocityChipToHz(int32_t velocity_chip)
{
  int64_t velocity_hz;
  velocity_hz = (int64_t)velocity_chip * ((int64_t)converter_parameters_.clock_frequency_mhz * 1000000);
  velocity_hz = velocity_hz / VELOCITY_SCALER;
  return velocity_hz;
}

int32_t Converter::velocityHzToChip(int32_t velocity_hz)
{
  int64_t velocity_chip;
  velocity_chip = (int64_t)velocity_hz * VELOCITY_SCALER;
  velocity_chip = velocity_chip / ((int64_t)converter_parameters_.clock_frequency_mhz * 1000000);
  return velocity_chip;
}

int32_t Converter::velocityRealToHz(int32_t velocity_real)
{
  int32_t velocity_hz;
  velocity_hz = positionRealToChip(velocity_real);
  velocity_hz = velocity_hz / (int32_t)converter_parameters_.seconds_per_real_velocity_unit;
  return velocity_hz;
}

int32_t Converter::velocityHzToReal(int32_t velocity_hz)
{
  int32_t velocity_real;
  velocity_real = velocity_hz * (int32_t)converter_parameters_.seconds_per_real_velocity_unit;
  velocity_real = positionChipToReal(velocity_real);
  return velocity_real;
}

int32_t Converter::tstepToVelocityHz(int32_t tstep)
{
  if (tstep == 0)
  {
    tstep = DIVISOR_DEFAULT;
  }
  int64_t velocity_hz;
  velocity_hz = ((int64_t)converter_parameters_.clock_frequency_mhz * 1000000) / tstep;
  return velocity_hz;
}

int32_t Converter::velocityHzToTstep(int32_t velocity_hz)
{
  if (velocity_hz == 0)
  {
    velocity_hz = DIVISOR_DEFAULT;
  }
  int64_t tstep;
  tstep = ((int64_t)converter_parameters_.clock_frequency_mhz * 1000000) / velocity_hz;
  return tstep;
}

int32_t Converter::accelerationChipToHzPerS(int32_t acceleration_chip)
{
  int64_t acceleration_hz_per_s;
  acceleration_hz_per_s = (int64_t)acceleration_chip * ((int64_t)converter_parameters_.clock_frequency_mhz * (int64_t)converter_parameters_.clock_frequency_mhz * 1000);
  acceleration_hz_per_s = acceleration_hz_per_s / ACCELERATION_SCALER;
  return acceleration_hz_per_s;
}

int32_t Converter::accelerationHzPerSToChip(int32_t acceleration_hz_per_s)
{
  int64_t acceleration_chip;
  acceleration_chip = (int64_t)acceleration_hz_per_s * ACCELERATION_SCALER;
  acceleration_chip = acceleration_chip / ((int64_t)converter_parameters_.clock_frequency_mhz * (int64_t)converter_parameters_.clock_frequency_mhz * 1000);
  return acceleration_chip;
}

int32_t Converter::accelerationRealToHzPerS(int32_t acceleration_real)
{
  int32_t acceleration_hz_per_s;
  acceleration_hz_per_s = positionRealToChip(acceleration_real);
  acceleration_hz_per_s = acceleration_hz_per_s / (int32_t)converter_parameters_.seconds_per_real_velocity_unit;
  return acceleration_hz_per_s;
}

int32_t Converter::accelerationHzPerSToReal(int32_t acceleration_hz_per_s)
{
  int64_t acceleration_real;
  acceleration_real = acceleration_hz_per_s * (int32_t)converter_parameters_.seconds_per_real_velocity_unit;
  acceleration_real = positionChipToReal(acceleration_real);
  return acceleration_real;
}

uint32_t Converter::constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}
