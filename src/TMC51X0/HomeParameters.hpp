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
  uint8_t run_current = CURRENT_DEFAULT;
  uint8_t hold_current = CURRENT_DEFAULT;
  int32_t target_position = TARGET_POSITION_DEFAULT;
  uint32_t velocity = VELOCITY_DEFAULT;
  uint32_t acceleration = ACCELERATION_DEFAULT;
  uint32_t zero_wait_duration = ZERO_WAIT_DURATION_DEFAULT;

  HomeParameters(uint8_t run_current_ = CURRENT_DEFAULT,
    uint8_t hold_current_ = CURRENT_DEFAULT,
    int32_t target_position_ = TARGET_POSITION_DEFAULT,
    uint32_t velocity_ = VELOCITY_DEFAULT,
    uint32_t acceleration_ = ACCELERATION_DEFAULT,
    uint32_t zero_wait_duration_ = ZERO_WAIT_DURATION_DEFAULT)
  {
    run_current = run_current_;
    hold_current = hold_current_;
    target_position = target_position_;
    velocity = velocity_;
    acceleration = acceleration_;
    zero_wait_duration = zero_wait_duration_;
  };

  bool operator==(const HomeParameters & rhs) const
  {
    if ((this->run_current == rhs.run_current) &&
      (this->hold_current == rhs.hold_current) &&
      (this->target_position == rhs.target_position) &&
      (this->velocity == rhs.velocity) &&
      (this->acceleration == rhs.acceleration) &&
      (this->zero_wait_duration == rhs.zero_wait_duration))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const HomeParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint8_t CURRENT_DEFAULT = 0;
  const static int32_t TARGET_POSITION_DEFAULT = 0;
  const static uint32_t VELOCITY_DEFAULT = 0;
  const static uint32_t ACCELERATION_DEFAULT = 0;
  const static uint32_t ZERO_WAIT_DURATION_DEFAULT = 3000; // ~100 milliseconds
};
}
#endif
