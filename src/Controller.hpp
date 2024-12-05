// ----------------------------------------------------------------------------
// Controller.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_CONTROLLER_HPP
#define TMC51X0_CONTROLLER_HPP

#include "Registers.hpp"
#include "TMC51X0/ControllerParameters.hpp"


class TMC51X0;

namespace tmc51x0
{
class Controller
{
public:
  Controller();

  void setup();
  void setup(ControllerParameters controller_parameters);

  // reset default: VELOCITY_POSITIVE
  void writeRampMode(RampMode ramp_mode);

  // reset default: HARD
  void writeStopMode(StopMode stop_mode);

  uint32_t readTstep();

  // -2^31..(2^31)-1 microsteps
  int32_t readActualPosition();
  // -2^31..(2^31)-1 microsteps
  void writeActualPosition(int32_t position);
  // -(2^23)-1..(2^23)-1 microsteps/t
  int32_t readActualVelocity();
  void zeroActualPosition();

  bool velocityReached();
  bool positionReached();

  void beginRampToZeroVelocity();
  void endRampToZeroVelocity();
  bool zeroVelocity();

  // 0..(2^23)-512 microsteps/t
  // reset default: 0
  void writeMaxVelocity(uint32_t velocity);
  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 10000
  void writeMaxAcceleration(uint32_t acceleration);

  // 0..(2^18)-1 microsteps/t
  // reset default: 0
  void writeStartVelocity(uint32_t velocity);
  // 0..(2^18)-1 microsteps/t
  // reset default: 10
  // stop velocity should be identical or greater than start velocity
  // do not set to zero in positioning mode!
  void writeStopVelocity(uint32_t velocity);

  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 0
  void writeFirstAcceleration(uint32_t acceleration);
  // 0..(2^20)-1 microsteps/t
  // reset default: 0
  void writeFirstVelocity(uint32_t velocity);

  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 0
  void writeMaxDeceleration(uint32_t deceleration);
  // 0..(2^16)-1 microsteps/ta^2
  // reset default: 10000
  // do not set to zero in positioning mode!
  void writeFirstDeceleration(uint32_t deceleration);

  // 0..(2^16)-1 *512 tclk
  // ~0..2s
  // reset default: 0
  void writeTzerowait(uint32_t tzerowait);

  // -2^31..(2^31)-1 microsteps
  int32_t readTargetPosition();
  // -2^31..(2^31)-1 microsteps
  void writeTargetPosition(int32_t position);
  void zeroTargetPosition();

  // -2^31..(2^31)-1 microsteps
  void writeComparePosition(int32_t position);

  // enabling stall stop sets stop mode to hard
  void enableStallStop();
  // set stop mode after disabling stall stop if desired
  void disableStallStop();

  void setupSwitches();
  void setupSwitches(SwitchParameters switch_parameters);
  bool leftSwitchActive();
  bool rightSwitchActive();
  bool leftLatchActive();
  bool rightLatchActive();
  bool leftStopEvent();
  bool rightStopEvent();

  // void beginHomeToSwitch(HomeParameters home_parameters);
  // void endHomeToSwitch();
  // bool homedToSwitch();

private:
  Registers * registers_ptr_;
  ControllerParameters setup_controller_parameters_;
  ControllerParameters cached_controller_settings_;
  SwitchParameters setup_switch_parameters_;

  const static int32_t MAX_POSITIVE_VELOCITY = 8388607;
  const static int32_t VELOCITY_SIGN_CONVERSION = 16777216;

  void initialize(Registers & registers);
  void writeControllerParameters(ControllerParameters controller_parameters);
  void cacheControllerSettings();
  void restoreControllerSettings();
  void writeSwitchParameters(SwitchParameters switch_parameters);

  friend class ::TMC51X0;
};
}
#endif
