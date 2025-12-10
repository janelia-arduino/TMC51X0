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
  BinaryMode = 0,
  DecimalMode = 1,
};

struct EncoderParameters
{
  FractionalMode fractional_mode;
  int32_t        microsteps_per_pulse_integer;
  int32_t        microsteps_per_pulse_fractional;

  // Base constructor with original defaults:
  //   fractional_mode                 = BinaryMode
  //   microsteps_per_pulse_integer    = 1
  //   microsteps_per_pulse_fractional = 0
  constexpr EncoderParameters(
    FractionalMode fractional_mode                 = BinaryMode,
    int32_t        microsteps_per_pulse_integer    = 1,
    int32_t        microsteps_per_pulse_fractional = 0)
  : fractional_mode(fractional_mode),
    microsteps_per_pulse_integer(microsteps_per_pulse_integer),
    microsteps_per_pulse_fractional(microsteps_per_pulse_fractional)
  {}

  // "Named parameter" style helpers

  constexpr EncoderParameters withFractionalMode(FractionalMode mode) const
  {
    return EncoderParameters(
      mode,
      microsteps_per_pulse_integer,
      microsteps_per_pulse_fractional);
  }

  constexpr EncoderParameters withMicrostepsPerPulseInteger(int32_t value) const
  {
    return EncoderParameters(
      fractional_mode,
      value,
      microsteps_per_pulse_fractional);
  }

  constexpr EncoderParameters withMicrostepsPerPulseFractional(int32_t value) const
  {
    return EncoderParameters(
      fractional_mode,
      microsteps_per_pulse_integer,
      value);
  }
};
}
#endif
