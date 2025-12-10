// ----------------------------------------------------------------------------
// StallParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_STALL_PARAMETERS_HPP
#define TMC51X0_STALL_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
struct StallParameters
{
  int8_t   stall_guard_threshold;
  uint32_t cool_step_threshold;

  constexpr StallParameters(int8_t   stall_guard_threshold = 0,
                            uint32_t cool_step_threshold   = 0)
  : stall_guard_threshold(stall_guard_threshold),
    cool_step_threshold(cool_step_threshold)
  {}

  // "Named parameter" style helpers

  constexpr StallParameters withStallGuardThreshold(int8_t value) const
  {
    return StallParameters(
      value,
      cool_step_threshold);
  }

  constexpr StallParameters withCoolStepThreshold(uint32_t value) const
  {
    return StallParameters(
      stall_guard_threshold,
      value);
  }
};
} // namespace tmc51x0

#endif // TMC51X0_STALL_PARAMETERS_HPP
