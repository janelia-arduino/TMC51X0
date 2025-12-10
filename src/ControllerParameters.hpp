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
  PositionMode         = 0,
  VelocityPositiveMode = 1,
  VelocityNegativeMode = 2,
  HoldMode             = 3,
};

enum StopMode
{
  HardMode = 0,
  SoftMode = 1,
};

struct ControllerParameters
{
  RampMode  ramp_mode;
  StopMode  stop_mode;
  uint32_t  max_velocity;
  uint32_t  max_acceleration;
  uint32_t  start_velocity;
  uint32_t  stop_velocity;
  uint32_t  first_velocity;
  uint32_t  first_acceleration;
  uint32_t  max_deceleration;
  uint32_t  first_deceleration;
  uint32_t  zero_wait_duration;
  bool      stall_stop_enabled;
  uint32_t  min_dc_step_velocity;

  constexpr ControllerParameters(
    RampMode  ramp_mode            = VelocityPositiveMode,
    StopMode  stop_mode            = HardMode,
    uint32_t  max_velocity         = 10,
    uint32_t  max_acceleration     = 10,
    uint32_t  start_velocity       = 1,
    uint32_t  stop_velocity        = 10,
    uint32_t  first_velocity       = 0,
    uint32_t  first_acceleration   = 0,
    uint32_t  max_deceleration     = 0,
    uint32_t  first_deceleration   = 10,
    uint32_t  zero_wait_duration   = 0,
    bool      stall_stop_enabled   = false,
    uint32_t  min_dc_step_velocity = 0)
  : ramp_mode(ramp_mode),
    stop_mode(stop_mode),
    max_velocity(max_velocity),
    max_acceleration(max_acceleration),
    start_velocity(start_velocity),
    stop_velocity(stop_velocity),
    first_velocity(first_velocity),
    first_acceleration(first_acceleration),
    max_deceleration(max_deceleration),
    first_deceleration(first_deceleration),
    zero_wait_duration(zero_wait_duration),
    stall_stop_enabled(stall_stop_enabled),
    min_dc_step_velocity(min_dc_step_velocity)
  {}

  // "Named parameter" style helpers.

  constexpr ControllerParameters withRampMode(RampMode value) const
  {
    return ControllerParameters(
      value,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withStopMode(StopMode value) const
  {
    return ControllerParameters(
      ramp_mode,
      value,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withMaxVelocity(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      value,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withMaxAcceleration(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      value,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withStartVelocity(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      value,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withStopVelocity(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      value,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withFirstVelocity(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      value,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withFirstAcceleration(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      value,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withMaxDeceleration(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      value,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withFirstDeceleration(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      value,
      zero_wait_duration,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withZeroWaitDuration(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      value,
      stall_stop_enabled,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withStallStopEnabled(bool value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      value,
      min_dc_step_velocity);
  }

  constexpr ControllerParameters withMinDcStepVelocity(uint32_t value) const
  {
    return ControllerParameters(
      ramp_mode,
      stop_mode,
      max_velocity,
      max_acceleration,
      start_velocity,
      stop_velocity,
      first_velocity,
      first_acceleration,
      max_deceleration,
      first_deceleration,
      zero_wait_duration,
      stall_stop_enabled,
      value);
  }
};
}

#endif
