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
  FractionalMode fractional_mode = FRACTIONAL_MODE_DEFAULT;
  int32_t microsteps_per_pulse_integer = MICROSTEPS_PER_PULSE_INTEGER_DEFAULT;
  int32_t microsteps_per_pulse_fractional = MICROSTEPS_PER_PULSE_FRACTIONAL_DEFAULT;

  EncoderParameters(FractionalMode fractional_mode_ = FRACTIONAL_MODE_DEFAULT,
    int32_t microsteps_per_pulse_integer_ = MICROSTEPS_PER_PULSE_INTEGER_DEFAULT,
    int32_t microsteps_per_pulse_fractional_ = MICROSTEPS_PER_PULSE_FRACTIONAL_DEFAULT)
  {
    fractional_mode = fractional_mode_;
    microsteps_per_pulse_integer = microsteps_per_pulse_integer_;
    microsteps_per_pulse_fractional = microsteps_per_pulse_fractional_;
  };

  bool operator==(const EncoderParameters & rhs) const
  {
    if ((this->fractional_mode == rhs.fractional_mode) &&
      (this->microsteps_per_pulse_integer == rhs.microsteps_per_pulse_integer) &&
      (this->microsteps_per_pulse_fractional == rhs.microsteps_per_pulse_fractional))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const EncoderParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static FractionalMode FRACTIONAL_MODE_DEFAULT = BINARY;
  const static int32_t MICROSTEPS_PER_PULSE_INTEGER_DEFAULT = 1;
  const static int32_t MICROSTEPS_PER_PULSE_FRACTIONAL_DEFAULT = 0;

  friend class Encoder;
};
}
#endif
