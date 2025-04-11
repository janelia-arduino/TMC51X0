// ----------------------------------------------------------------------------
// SwitchParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_SWITCH_PARAMETERS_HPP
#define TMC51X0_SWITCH_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
struct SwitchParameters
{
  bool left_stop_enabled = false;
  bool right_stop_enabled = false;
  bool invert_left_polarity = false;
  bool invert_right_polarity = false;
  bool swap_left_right = false;
  bool latch_left_active = false;
  bool latch_left_inactive = false;
  bool latch_right_active = false;
  bool latch_right_inactive = false;
  bool latch_encoder_enabled = false;
};
}
#endif
