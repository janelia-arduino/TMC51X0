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
  RampMode ramp_mode = VELOCITY_POSITIVE;
  StopMode stop_mode = HARD;
  uint32_t max_velocity = 10;
  uint32_t max_acceleration = 10;
  uint32_t start_velocity = 1;
  uint32_t stop_velocity = 10;
  uint32_t first_velocity = 0;
  uint32_t first_acceleration = 0;
  uint32_t max_deceleration = 0;
  uint32_t first_deceleration = 10;
  uint32_t zero_wait_duration = 0;
  bool stall_stop_enabled = false;
  uint32_t min_dc_step_velocity = 0;
};
}
#endif
