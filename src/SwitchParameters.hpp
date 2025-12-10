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
  bool left_stop_enabled;
  bool right_stop_enabled;
  bool invert_left_polarity;
  bool invert_right_polarity;
  bool swap_left_right;
  bool latch_left_active;
  bool latch_left_inactive;
  bool latch_right_active;
  bool latch_right_inactive;
  bool latch_encoder_enabled;

  constexpr SwitchParameters(
    bool left_stop_enabled      = false,
    bool right_stop_enabled     = false,
    bool invert_left_polarity   = false,
    bool invert_right_polarity  = false,
    bool swap_left_right        = false,
    bool latch_left_active      = false,
    bool latch_left_inactive    = false,
    bool latch_right_active     = false,
    bool latch_right_inactive   = false,
    bool latch_encoder_enabled  = false)
  : left_stop_enabled(left_stop_enabled),
    right_stop_enabled(right_stop_enabled),
    invert_left_polarity(invert_left_polarity),
    invert_right_polarity(invert_right_polarity),
    swap_left_right(swap_left_right),
    latch_left_active(latch_left_active),
    latch_left_inactive(latch_left_inactive),
    latch_right_active(latch_right_active),
    latch_right_inactive(latch_right_inactive),
    latch_encoder_enabled(latch_encoder_enabled)
  {}

  // "Named parameter" style helpers

  constexpr SwitchParameters withLeftStopEnabled(bool value) const
  {
    return SwitchParameters(
      value,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withRightStopEnabled(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      value,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withInvertLeftPolarity(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      value,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withInvertRightPolarity(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      value,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withSwapLeftRight(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      value,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withLatchLeftActive(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      value,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withLatchLeftInactive(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      value,
      latch_right_active,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withLatchRightActive(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      value,
      latch_right_inactive,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withLatchRightInactive(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      value,
      latch_encoder_enabled);
  }

  constexpr SwitchParameters withLatchEncoderEnabled(bool value) const
  {
    return SwitchParameters(
      left_stop_enabled,
      right_stop_enabled,
      invert_left_polarity,
      invert_right_polarity,
      swap_left_right,
      latch_left_active,
      latch_left_inactive,
      latch_right_active,
      latch_right_inactive,
      value);
  }
};
} // namespace tmc51x0

#endif // TMC51X0_SWITCH_PARAMETERS_HPP
