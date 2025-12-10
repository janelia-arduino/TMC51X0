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
  uint8_t  clock_frequency_mhz;
  uint32_t microsteps_per_real_position_unit;
  uint32_t seconds_per_real_velocity_unit;

  // Base constructor with original defaults:
  //   clock_frequency_mhz              = 12
  //   microsteps_per_real_position_unit = 1
  //   seconds_per_real_velocity_unit    = 1
  constexpr ConverterParameters(
    uint8_t  clock_frequency_mhz              = 12,
    uint32_t microsteps_per_real_position_unit = 1,
    uint32_t seconds_per_real_velocity_unit    = 1)
  : clock_frequency_mhz(clock_frequency_mhz),
    microsteps_per_real_position_unit(microsteps_per_real_position_unit),
    seconds_per_real_velocity_unit(seconds_per_real_velocity_unit)
  {}

  // "Named parameter" style helpers

  constexpr ConverterParameters withClockFrequencyMHz(uint8_t value) const
  {
    return ConverterParameters(
      value,
      microsteps_per_real_position_unit,
      seconds_per_real_velocity_unit);
  }

  constexpr ConverterParameters withMicrostepsPerRealPositionUnit(uint32_t value) const
  {
    return ConverterParameters(
      clock_frequency_mhz,
      value,
      seconds_per_real_velocity_unit);
  }

  constexpr ConverterParameters withSecondsPerRealVelocityUnit(uint32_t value) const
  {
    return ConverterParameters(
      clock_frequency_mhz,
      microsteps_per_real_position_unit,
      value);
  }
};
}
#endif
