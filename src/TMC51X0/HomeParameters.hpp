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
  int32_t target_position = TARGET_POSITION_DEFAULT;
  uint32_t velocity = VELOCITY_DEFAULT;
  uint32_t acceleration = ACCELERATION_DEFAULT;
  uint8_t run_current = RUN_CURRENT_DEFAULT;
  uint8_t pwm_offset = PWM_OFFSET_DEFAULT;

  HomeParameters(int32_t target_position_ = TARGET_POSITION_DEFAULT,
    uint32_t velocity_ = VELOCITY_DEFAULT,
    uint32_t acceleration_ = ACCELERATION_DEFAULT,
    uint8_t run_current_ = RUN_CURRENT_DEFAULT,
    uint8_t pwm_offset_ = PWM_OFFSET_DEFAULT)
  {
    target_position = target_position_;
    velocity = velocity_;
    acceleration = acceleration_;
    run_current = run_current_;
    pwm_offset = pwm_offset_;
  };

  bool operator==(const HomeParameters & rhs) const
  {
    if ((this->target_position == rhs.target_position) &&
      (this->velocity == rhs.velocity) &&
      (this->acceleration == rhs.acceleration) &&
      (this->run_current == rhs.run_current) &&
      (this->pwm_offset == rhs.pwm_offset))
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
  const static int32_t TARGET_POSITION_DEFAULT = 0;
  const static uint32_t VELOCITY_DEFAULT = 0;
  const static uint32_t ACCELERATION_DEFAULT = 0;
  const static uint8_t RUN_CURRENT_DEFAULT = 0;
  const static uint8_t PWM_OFFSET_DEFAULT = 0;
};
}
#endif
