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

namespace tmc51x0 {
enum FractionalMode {
  BinaryMode = 0,
  DecimalMode = 1,
};

// ENC_CONST's denominator, set by ENCMODE.enc_sel_decimal.
constexpr int32_t microstepsPerPulseDenominator(FractionalMode mode) {
  return (mode == DecimalMode) ? 10000 : 65536;
}

// Floor division, not C++'s truncating division, because that is what
// ENC_CONST's format means for a negative value: the fractional part is always
// ADDED to a signed integer part, so -12.8 is -13 + 0.2 and never -12 + 0.8.
constexpr int32_t microstepsPerPulseFloorDiv(int32_t numerator,
                                             int32_t denominator) {
  return (numerator / denominator) - (((numerator % denominator) != 0 &&
                                       ((numerator < 0) != (denominator < 0)))
                                          ? 1
                                          : 0);
}

struct EncoderParameters {
  FractionalMode fractional_mode;
  int32_t microsteps_per_pulse_integer;
  int32_t microsteps_per_pulse_fractional;

  constexpr EncoderParameters(FractionalMode fractional_mode = BinaryMode,
                              int32_t microsteps_per_pulse_integer = 1,
                              int32_t microsteps_per_pulse_fractional = 0)
      : fractional_mode(fractional_mode),
        microsteps_per_pulse_integer(microsteps_per_pulse_integer),
        microsteps_per_pulse_fractional(microsteps_per_pulse_fractional) {}

  // "Named parameter" style helpers

  // Set the whole scaling from ONE signed value, in the units of the fractional
  // mode being selected: DecimalMode -> value/10000, BinaryMode -> value/65536.
  // So -12.8 microsteps per pulse is `withMicrostepsPerPulse(DecimalMode,
  // -128000)`.
  //
  // Prefer this over setting the integer and fractional parts separately
  // whenever the value is negative. ENC_CONST is one signed fixed-point number
  // and its fractional part is always ADDED, so a reversed encoder at -12.8 is
  // integer -13 with fractional 2000 -- while the obvious pairing of integer
  // -12 with fractional 8000 is -11.2. That is a 12.5% scale error which reads
  // as a working encoder with a slightly wrong resolution, and nothing reports
  // it. This helper does the split with floor division so the caller never has
  // to know.
  //
  // The mode is a parameter rather than being read from `fractional_mode`,
  // because the split depends on it: taking it here means the call cannot
  // depend on the order of the builder chain.
  constexpr EncoderParameters withMicrostepsPerPulse(FractionalMode mode,
                                                     int32_t scaled) const {
    return EncoderParameters(
        mode,
        microstepsPerPulseFloorDiv(scaled, microstepsPerPulseDenominator(mode)),
        scaled - microstepsPerPulseFloorDiv(
                     scaled, microstepsPerPulseDenominator(mode)) *
                     microstepsPerPulseDenominator(mode));
  }

  constexpr EncoderParameters withFractionalMode(FractionalMode mode) const {
    return EncoderParameters(mode, microsteps_per_pulse_integer,
                             microsteps_per_pulse_fractional);
  }

  constexpr EncoderParameters
  withMicrostepsPerPulseInteger(int32_t value) const {
    return EncoderParameters(fractional_mode, value,
                             microsteps_per_pulse_fractional);
  }

  constexpr EncoderParameters
  withMicrostepsPerPulseFractional(int32_t value) const {
    return EncoderParameters(fractional_mode, microsteps_per_pulse_integer,
                             value);
  }
};
} // namespace tmc51x0
#endif
