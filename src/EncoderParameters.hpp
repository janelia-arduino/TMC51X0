// ----------------------------------------------------------------------------
// EncoderParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_ENCODER_PARAMETERS_HPP
#define TMC51X0_ENCODER_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
enum FractionalMode
{
  BINARY = 0,
  DECIMAL = 1,
};
struct EncoderParameters
{
  FractionalMode fractional_mode = BINARY;
  int32_t microsteps_per_pulse_integer = 1;
  int32_t microsteps_per_pulse_fractional = 0;
};
}
#endif
