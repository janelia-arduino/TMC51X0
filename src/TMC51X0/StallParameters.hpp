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
  COOL_STEP_THRESHOLD,
  DC_STEP
};
struct StallParameters
{
  bool stall_mode = STALL_MODE_DEFAULT;

  StallParameters(bool stall_mode_ = STALL_MODE_DEFAULT)
  {
    stall_mode = stall_mode_;
  };

  bool operator==(const StallParameters & rhs) const
  {
    if ((this->stall_mode == rhs.stall_mode))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const StallParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static bool STALL_MODE_DEFAULT = false;
};
}
#endif
