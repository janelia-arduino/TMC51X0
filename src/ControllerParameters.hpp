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
  PositionMode = 0,
  VelocityPositiveMode = 1,
  VelocityNegativeMode = 2,
  HoldMode = 3,
};
enum StopMode
{
  HardMode = 0,
  SoftMode = 1,
};
struct ControllerParameters
{
  RampMode ramp_mode = VelocityPositiveMode;
  StopMode stop_mode = HardMode;
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
