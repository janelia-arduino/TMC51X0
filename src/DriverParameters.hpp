// ----------------------------------------------------------------------------
// DriverParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_DRIVER_PARAMETERS_HPP
#define TMC51X0_DRIVER_PARAMETERS_HPP

#include "Registers.hpp"


class TMC51X0;

namespace tmc51x0
{
enum MotorDirection
{
  ForwardDirection = 0,
  ReverseDirection = 1
};
enum StandstillMode
{
  NormalMode=0,
  FreewheelingMode=1,
  PassiveBrakingLsMode=2,
  PassiveBrakingHsMode=3
};
enum ChopperMode
{
  SpreadCycleMode = 0,
  ClassicMode = 1
};
enum ComparatorBlankTime
{
  ClockCycles16 = 0,
  ClockCycles24 = 1,
  ClockCycles36 = 2,
  ClockCycles54 = 3
};

struct DriverParameters
{
  // global_current_scaler only available on TMC5160
  uint8_t global_current_scaler = 100;
  uint8_t run_current = 50;
  uint8_t hold_current = 20;
  uint8_t hold_delay = 5;
  uint8_t pwm_offset = 25;
  uint8_t pwm_gradient = 5;
  bool automatic_current_control_enabled = false;
  MotorDirection motor_direction = ForwardDirection;
  StandstillMode standstill_mode = NormalMode;
  ChopperMode chopper_mode = SpreadCycleMode;
  uint32_t stealth_chop_threshold = 100;
  bool stealth_chop_enabled = true;
  uint32_t cool_step_threshold = 150;
  uint8_t cool_step_min = 1;
  uint8_t cool_step_max = 0;
  bool cool_step_enabled = false;
  uint32_t high_velocity_threshold = 200;
  bool high_velocity_fullstep_enabled = false;
  bool high_velocity_chopper_switch_enabled = false;
  int8_t stall_guard_threshold = 0;
  bool stall_guard_filter_enabled = false;
  bool short_to_ground_protection_enabled = true;
  uint8_t enabled_toff = 3;
  ComparatorBlankTime comparator_blank_time = ClockCycles36;
  uint16_t dc_time = 0;
  uint8_t dc_stall_guard_threshold = 0;
};
}
#endif
