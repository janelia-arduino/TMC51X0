// ----------------------------------------------------------------------------
// SwitchParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_SWITCH_PARAMETERS_HPP
#define TMC51X0_SWITCH_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
struct SwitchParameters
{
  bool left_stop_enabled = LEFT_STOP_ENABLED_DEFAULT;
  bool right_stop_enabled = RIGHT_STOP_ENABLED_DEFAULT;
  bool invert_left_polarity = INVERT_LEFT_POLARITY_DEFAULT;
  bool invert_right_polarity = INVERT_RIGHT_POLARITY_DEFAULT;
  bool swap_left_right = SWAP_LEFT_RIGHT_DEFAULT;
  bool latch_left_active = LATCH_LEFT_ACTIVE_DEFAULT;
  bool latch_left_inactive = LATCH_LEFT_INACTIVE_DEFAULT;
  bool latch_right_active = LATCH_RIGHT_ACTIVE_DEFAULT;
  bool latch_right_inactive = LATCH_RIGHT_INACTIVE_DEFAULT;
  bool latch_encoder_enabled = LATCH_ENCODER_ENABLED_DEFAULT;

  SwitchParameters(bool left_stop_enabled_ = LEFT_STOP_ENABLED_DEFAULT,
    bool right_stop_enabled_ = RIGHT_STOP_ENABLED_DEFAULT,
    bool invert_left_polarity_ = INVERT_LEFT_POLARITY_DEFAULT,
    bool invert_right_polarity_ = INVERT_RIGHT_POLARITY_DEFAULT,
    bool swap_left_right_ = SWAP_LEFT_RIGHT_DEFAULT,
    bool latch_left_active_ = LATCH_LEFT_ACTIVE_DEFAULT,
    bool latch_left_inactive_ = LATCH_LEFT_INACTIVE_DEFAULT,
    bool latch_right_active_ = LATCH_RIGHT_ACTIVE_DEFAULT,
    bool latch_right_inactive_ = LATCH_RIGHT_INACTIVE_DEFAULT,
    bool latch_encoder_enabled_ = LATCH_ENCODER_ENABLED_DEFAULT)
  {
    left_stop_enabled = left_stop_enabled_;
    right_stop_enabled = right_stop_enabled_;
    invert_left_polarity = invert_left_polarity_;
    invert_right_polarity = invert_right_polarity_;
    swap_left_right = swap_left_right_;
    latch_left_active = latch_left_active_;
    latch_left_inactive = latch_left_inactive_;
    latch_right_active = latch_right_active_;
    latch_right_inactive = latch_right_inactive_;
    latch_encoder_enabled = latch_encoder_enabled_;
  };

  bool operator==(const SwitchParameters & rhs) const
  {
    if ((this->left_stop_enabled == rhs.left_stop_enabled) &&
      (this->right_stop_enabled == rhs.right_stop_enabled) &&
      (this->invert_left_polarity == rhs.invert_left_polarity) &&
      (this->invert_right_polarity == rhs.invert_right_polarity) &&
      (this->swap_left_right == rhs.swap_left_right) &&
      (this->latch_left_active == rhs.latch_left_active) &&
      (this->latch_left_inactive == rhs.latch_left_inactive) &&
      (this->latch_right_active == rhs.latch_right_active) &&
      (this->latch_right_inactive == rhs.latch_right_inactive) &&
      (this->latch_encoder_enabled == rhs.latch_encoder_enabled))
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
  const static bool LEFT_STOP_ENABLED_DEFAULT = false;
  const static bool RIGHT_STOP_ENABLED_DEFAULT = false;
  const static bool INVERT_LEFT_POLARITY_DEFAULT = false;
  const static bool INVERT_RIGHT_POLARITY_DEFAULT = false;
  const static bool SWAP_LEFT_RIGHT_DEFAULT = false;
  const static bool LATCH_LEFT_ACTIVE_DEFAULT = false;
  const static bool LATCH_LEFT_INACTIVE_DEFAULT = false;
  const static bool LATCH_RIGHT_ACTIVE_DEFAULT = false;
  const static bool LATCH_RIGHT_INACTIVE_DEFAULT = false;
  const static bool LATCH_ENCODER_ENABLED_DEFAULT = false;
};
}
#endif
