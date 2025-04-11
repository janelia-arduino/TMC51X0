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
enum StallMode
{
  COOL_STEP,
  DC_STEP
};
struct StallParameters
{
  StallMode stall_mode = COOL_STEP;
  int8_t stall_guard_threshold = 0;
  uint32_t cool_step_threshold = 0;
  uint32_t min_dc_step_velocity = 0;
  uint8_t dc_stall_guard_threshold = 0;
};
}
#endif
