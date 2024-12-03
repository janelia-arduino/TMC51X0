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
  uint32_t start_velocity = START_VELOCITY_DEFAULT;
  uint32_t stop_velocity = STOP_VELOCITY_DEFAULT;
  uint32_t first_velocity = FIRST_VELOCITY_DEFAULT;
  uint32_t first_acceleration = FIRST_ACCELERATION_DEFAULT;
  uint32_t max_deceleration = MAX_DECELERATION_DEFAULT;
  uint32_t first_deceleration = FIRST_DECELERATION_DEFAULT;
  uint16_t zero_wait_duration = ZERO_WAIT_DURATION_DEFAULT;

  ControllerParameters(RampMode ramp_mode_ = RAMP_MODE_DEFAULT,
    StopMode stop_mode_ = STOP_MODE_DEFAULT,
    uint32_t max_velocity_ = MAX_VELOCITY_DEFAULT,
    uint32_t max_acceleration_ = MAX_ACCELERATION_DEFAULT,
    uint32_t start_velocity_ = START_VELOCITY_DEFAULT,
    uint32_t stop_velocity_ = STOP_VELOCITY_DEFAULT,
    uint32_t first_velocity_ = FIRST_VELOCITY_DEFAULT,
    uint32_t first_acceleration_ = FIRST_ACCELERATION_DEFAULT,
    uint32_t max_deceleration_ = MAX_DECELERATION_DEFAULT,
    uint32_t first_deceleration_ = FIRST_DECELERATION_DEFAULT,
    uint16_t zero_wait_duration_ = ZERO_WAIT_DURATION_DEFAULT)
  {
    ramp_mode = ramp_mode_;
    stop_mode = stop_mode_;
    max_velocity = max_velocity_;
    max_acceleration = max_acceleration_;
    start_velocity = start_velocity_;
    stop_velocity = stop_velocity_;
    first_velocity = first_velocity_;
    first_acceleration = first_acceleration_;
    max_deceleration = max_deceleration_;
    first_deceleration = first_deceleration_;
    zero_wait_duration = zero_wait_duration_;
  };

  bool operator==(const ControllerParameters & rhs) const
  {
    if ((this->ramp_mode == rhs.ramp_mode) &&
      (this->stop_mode == rhs.stop_mode) &&
      (this->max_velocity == rhs.max_velocity) &&
      (this->max_acceleration == rhs.max_acceleration) &&
      (this->start_velocity == rhs.start_velocity) &&
      (this->stop_velocity == rhs.stop_velocity) &&
      (this->first_velocity == rhs.first_velocity) &&
      (this->first_acceleration == rhs.first_acceleration) &&
      (this->max_deceleration == rhs.max_deceleration) &&
      (this->first_deceleration == rhs.first_deceleration) &&
      (this->zero_wait_duration == rhs.zero_wait_duration))
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
  const static uint32_t START_VELOCITY_DEFAULT = 0;
  const static uint32_t STOP_VELOCITY_DEFAULT = 10;
  const static uint32_t FIRST_VELOCITY_DEFAULT = 0;
  const static uint32_t FIRST_ACCELERATION_DEFAULT = 0;
  const static uint32_t MAX_DECELERATION_DEFAULT = 0;
  const static uint32_t FIRST_DECELERATION_DEFAULT = 10000;
  const static uint16_t ZERO_WAIT_DURATION_DEFAULT = 0;
};
struct SwitchParameters
{
  bool enable_left_stop = ENABLE_LEFT_STOP_DEFAULT;
  bool enable_right_stop = ENABLE_RIGHT_STOP_DEFAULT;
  bool invert_left_polarity = INVERT_LEFT_POLARITY_DEFAULT;
  bool invert_right_polarity = INVERT_RIGHT_POLARITY_DEFAULT;
  bool swap_left_right = SWAP_LEFT_RIGHT_DEFAULT;
  bool latch_left_active = LATCH_LEFT_ACTIVE_DEFAULT;
  bool latch_left_inactive = LATCH_LEFT_INACTIVE_DEFAULT;
  bool latch_right_active = LATCH_RIGHT_ACTIVE_DEFAULT;
  bool latch_right_inactive = LATCH_RIGHT_INACTIVE_DEFAULT;
  bool enable_latch_encoder = ENABLE_LATCH_ENCODER_DEFAULT;

  SwitchParameters(bool enable_left_stop_ = ENABLE_LEFT_STOP_DEFAULT,
    bool enable_right_stop_ = ENABLE_RIGHT_STOP_DEFAULT,
    bool invert_left_polarity_ = INVERT_LEFT_POLARITY_DEFAULT,
    bool invert_right_polarity_ = INVERT_RIGHT_POLARITY_DEFAULT,
    bool swap_left_right_ = SWAP_LEFT_RIGHT_DEFAULT,
    bool latch_left_active_ = LATCH_LEFT_ACTIVE_DEFAULT,
    bool latch_left_inactive_ = LATCH_LEFT_INACTIVE_DEFAULT,
    bool latch_right_active_ = LATCH_RIGHT_ACTIVE_DEFAULT,
    bool latch_right_inactive_ = LATCH_RIGHT_INACTIVE_DEFAULT,
    bool enable_latch_encoder_ = ENABLE_LATCH_ENCODER_DEFAULT)
  {
    enable_left_stop = enable_left_stop_;
    enable_right_stop = enable_right_stop_;
    invert_left_polarity = invert_left_polarity_;
    invert_right_polarity = invert_right_polarity_;
    swap_left_right = swap_left_right_;
    latch_left_active = latch_left_active_;
    latch_left_inactive = latch_left_inactive_;
    latch_right_active = latch_right_active_;
    latch_right_inactive = latch_right_inactive_;
    enable_latch_encoder = enable_latch_encoder_;
  };

  bool operator==(const SwitchParameters & rhs) const
  {
    if ((this->enable_left_stop == rhs.enable_left_stop) &&
      (this->enable_right_stop == rhs.enable_right_stop) &&
      (this->invert_left_polarity == rhs.invert_left_polarity) &&
      (this->invert_right_polarity == rhs.invert_right_polarity) &&
      (this->swap_left_right == rhs.swap_left_right) &&
      (this->latch_left_active == rhs.latch_left_active) &&
      (this->latch_left_inactive == rhs.latch_left_inactive) &&
      (this->latch_right_active == rhs.latch_right_active) &&
      (this->latch_right_inactive == rhs.latch_right_inactive) &&
      (this->enable_latch_encoder == rhs.enable_latch_encoder))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const SwitchParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static bool ENABLE_LEFT_STOP_DEFAULT = false;
  const static bool ENABLE_RIGHT_STOP_DEFAULT = false;
  const static bool INVERT_LEFT_POLARITY_DEFAULT = false;
  const static bool INVERT_RIGHT_POLARITY_DEFAULT = false;
  const static bool SWAP_LEFT_RIGHT_DEFAULT = false;
  const static bool LATCH_LEFT_ACTIVE_DEFAULT = false;
  const static bool LATCH_LEFT_INACTIVE_DEFAULT = false;
  const static bool LATCH_RIGHT_ACTIVE_DEFAULT = false;
  const static bool LATCH_RIGHT_INACTIVE_DEFAULT = false;
  const static bool ENABLE_LATCH_ENCODER_DEFAULT = false;
};
}
#endif
