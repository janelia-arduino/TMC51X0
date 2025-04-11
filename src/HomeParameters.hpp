// ----------------------------------------------------------------------------
// HomeParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HOME_PARAMETERS_HPP
#define TMC51X0_HOME_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
struct HomeParameters
{
  uint8_t run_current = 0;
  uint8_t hold_current = 0;
  int32_t target_position = 0;
  uint32_t velocity = 0;
  uint32_t acceleration = 0;
  uint32_t zero_wait_duration = 100;
};
}
#endif
