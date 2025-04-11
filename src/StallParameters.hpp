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
  int8_t stall_guard_threshold = 0;
  uint32_t cool_step_threshold = 0;
};
}
#endif
