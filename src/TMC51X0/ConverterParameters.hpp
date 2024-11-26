// ----------------------------------------------------------------------------
// ConverterParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONVERTER_PARAMETERS_HPP
#define TMC51X0_CONVERTER_PARAMETERS_HPP


namespace tmc51x0
{
struct ConverterParameters
{
  uint8_t clock_frequency_mhz = CLOCK_FREQUENCY_MHZ_DEFAULT;
  uint32_t microsteps_per_real_position_unit = MICROSTEPS_PER_REAL_POSITION_UNIT_DEFAULT;
  uint32_t seconds_per_real_velocity_unit = SECONDS_PER_REAL_VELOCITY_UNIT_DEFAULT;
  uint8_t clock_duration_ns;

  ConverterParameters(uint8_t clock_frequency_mhz_ = CLOCK_FREQUENCY_MHZ_DEFAULT,
    uint32_t microsteps_per_real_position_unit_ = MICROSTEPS_PER_REAL_POSITION_UNIT_DEFAULT,
    uint32_t seconds_per_real_velocity_unit_ = SECONDS_PER_REAL_VELOCITY_UNIT_DEFAULT)
  {
    if (clock_frequency_mhz_ != 0)
    {
      clock_frequency_mhz = clock_frequency_mhz_;
    }
    if (microsteps_per_real_position_unit_ != 0)
    {
      microsteps_per_real_position_unit = microsteps_per_real_position_unit_;
    }
    if (seconds_per_real_velocity_unit_ != 0)
    {
      seconds_per_real_velocity_unit = seconds_per_real_velocity_unit_;
    }
    clock_duration_ns = CLOCK_FREQUENCY_TO_DURATION_SCALER / (uint16_t)clock_frequency_mhz;
  };

  bool operator==(const ConverterParameters & rhs) const
  {
    if ((this->clock_frequency_mhz == rhs.clock_frequency_mhz) &&
      (this->microsteps_per_real_position_unit == rhs.microsteps_per_real_position_unit) &&
      (this->seconds_per_real_velocity_unit == rhs.seconds_per_real_velocity_unit))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const ConverterParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint8_t CLOCK_FREQUENCY_MHZ_DEFAULT = 12;
  const static int32_t MICROSTEPS_PER_REAL_POSITION_UNIT_DEFAULT = 1;
  const static int32_t SECONDS_PER_REAL_VELOCITY_UNIT_DEFAULT = 1;
  const static uint16_t CLOCK_FREQUENCY_TO_DURATION_SCALER = 1000;
};
}
#endif
