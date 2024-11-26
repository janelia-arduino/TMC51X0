// ----------------------------------------------------------------------------
// ControllerParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONTROLLER_PARAMETERS_HPP
#define TMC51X0_CONTROLLER_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
enum RampMode
{
  POSITION = 0,
  VELOCITY_POSITIVE = 1,
  VELOCITY_NEGATIVE = 2,
  HOLD = 3,
};
enum StopMode
{
  HARD = 0,
  SOFT = 1,
};
struct ControllerParameters
{
  RampMode ramp_mode = RAMP_MODE_DEFAULT;
  StopMode stop_mode = STOP_MODE_DEFAULT;
  uint32_t max_velocity = MAX_VELOCITY_DEFAULT;
  uint32_t max_acceleration = MAX_ACCELERATION_DEFAULT;

  ControllerParameters(RampMode ramp_mode_ = RAMP_MODE_DEFAULT,
    StopMode stop_mode_ = STOP_MODE_DEFAULT,
    uint32_t max_velocity_ = MAX_VELOCITY_DEFAULT,
    uint32_t max_acceleration_ = MAX_ACCELERATION_DEFAULT)
  {
    ramp_mode = ramp_mode_;
    stop_mode = stop_mode_;
    max_velocity = max_velocity_;
    max_acceleration = max_acceleration_;
  };

  bool operator==(const ControllerParameters & rhs) const
  {
    if ((this->ramp_mode == rhs.ramp_mode) &&
      (this->stop_mode == rhs.stop_mode) &&
      (this->max_velocity == rhs.max_velocity) &&
      (this->max_acceleration == rhs.max_acceleration))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const ControllerParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static RampMode RAMP_MODE_DEFAULT = VELOCITY_POSITIVE;
  const static StopMode STOP_MODE_DEFAULT = HARD;
  const static uint32_t MAX_VELOCITY_DEFAULT = 0;
  const static uint32_t MAX_ACCELERATION_DEFAULT = 10000;
};
}
#endif
