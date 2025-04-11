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
  uint8_t clock_frequency_mhz = 12;
  uint32_t microsteps_per_real_position_unit = 1;
  uint32_t seconds_per_real_velocity_unit = 1;
};
}
#endif
