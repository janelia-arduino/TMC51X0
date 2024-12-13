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
