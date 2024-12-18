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
  StallMode stall_mode = STALL_MODE_DEFAULT;
  int8_t stall_guard_threshold = STALL_GUARD_THRESHOLD_DEFAULT;
  uint32_t cool_step_threshold = COOL_STEP_THRESHOLD_DEFAULT;
  uint32_t min_dc_step_velocity = MIN_DC_STEP_VELOCITY_DEFAULT;
  uint8_t dc_stall_guard_threshold = DC_STALL_GUARD_THRESHOLD_DEFAULT;

  StallParameters(StallMode stall_mode_ = STALL_MODE_DEFAULT,
    int8_t stall_guard_threshold_ = STALL_GUARD_THRESHOLD_DEFAULT,
    uint32_t cool_step_threshold_ = COOL_STEP_THRESHOLD_DEFAULT,
    uint32_t min_dc_step_velocity_ = MIN_DC_STEP_VELOCITY_DEFAULT,
    uint8_t dc_stall_guard_threshold_ = DC_STALL_GUARD_THRESHOLD_DEFAULT)
  {
    stall_mode = stall_mode_;
    stall_guard_threshold = stall_guard_threshold_;
    cool_step_threshold = cool_step_threshold_;
    min_dc_step_velocity = min_dc_step_velocity_;
    dc_stall_guard_threshold = dc_stall_guard_threshold_;
  };

  bool operator==(const StallParameters & rhs) const
  {
    if ((this->stall_mode == rhs.stall_mode) &&
      (this->stall_guard_threshold == rhs.stall_guard_threshold) &&
      (this->cool_step_threshold == rhs.cool_step_threshold) &&
      (this->min_dc_step_velocity == rhs.min_dc_step_velocity) &&
      (this->dc_stall_guard_threshold == rhs.dc_stall_guard_threshold))
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
  const static StallMode STALL_MODE_DEFAULT = COOL_STEP;
  const static int8_t STALL_GUARD_THRESHOLD_DEFAULT = 0;
  const static uint32_t COOL_STEP_THRESHOLD_DEFAULT = 0;
  const static uint32_t MIN_DC_STEP_VELOCITY_DEFAULT = 0;
  const static uint8_t DC_STALL_GUARD_THRESHOLD_DEFAULT = 0;
};
}
#endif
