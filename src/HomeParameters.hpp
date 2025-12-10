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
  uint8_t  run_current;
  uint8_t  hold_current;
  int32_t  target_position;
  uint32_t velocity;
  uint32_t acceleration;
  uint32_t zero_wait_duration;

  constexpr HomeParameters(
    uint8_t  run_current        = 0,
    uint8_t  hold_current       = 0,
    int32_t  target_position    = 0,
    uint32_t velocity           = 0,
    uint32_t acceleration       = 0,
    uint32_t zero_wait_duration = 100)
  : run_current(run_current),
    hold_current(hold_current),
    target_position(target_position),
    velocity(velocity),
    acceleration(acceleration),
    zero_wait_duration(zero_wait_duration)
  {}

  // "Named parameter" style helpers

  constexpr HomeParameters withRunCurrent(uint8_t value) const
  {
    return HomeParameters(
      value,
      hold_current,
      target_position,
      velocity,
      acceleration,
      zero_wait_duration);
  }

  constexpr HomeParameters withHoldCurrent(uint8_t value) const
  {
    return HomeParameters(
      run_current,
      value,
      target_position,
      velocity,
      acceleration,
      zero_wait_duration);
  }

  constexpr HomeParameters withTargetPosition(int32_t value) const
  {
    return HomeParameters(
      run_current,
      hold_current,
      value,
      velocity,
      acceleration,
      zero_wait_duration);
  }

  constexpr HomeParameters withVelocity(uint32_t value) const
  {
    return HomeParameters(
      run_current,
      hold_current,
      target_position,
      value,
      acceleration,
      zero_wait_duration);
  }

  constexpr HomeParameters withAcceleration(uint32_t value) const
  {
    return HomeParameters(
      run_current,
      hold_current,
      target_position,
      velocity,
      value,
      zero_wait_duration);
  }

  constexpr HomeParameters withZeroWaitDuration(uint32_t value) const
  {
    return HomeParameters(
      run_current,
      hold_current,
      target_position,
      velocity,
      acceleration,
      value);
  }
};
}
#endif
