// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"

using namespace tmc51x0;

TMC51X0::TMC51X0 ()
{
  enable_power_pin_ = NO_PIN;
  pin_value_when_enabled_ = HIGH;
}

void
TMC51X0::setupSpi (SpiParameters spi_parameters,
                   Registers::DeviceModel expected_device_model)
{
  interface_spi_.setup (spi_parameters);
  registers.initialize (interface_spi_);
  if (expected_device_model != Registers::DeviceModel::Unknown)
    {
      registers.setDeviceModel (expected_device_model);
    }
  else
    {
      (void)communicating ();
    }
  registers.assumeDeviceReset ();
  initialize ();
  (void)finishSetupOrRecovery_ ();
}

void
TMC51X0::setupUart (UartParameters uart_parameters,
                    Registers::DeviceModel expected_device_model)
{
  interface_uart_.setup (uart_parameters);
  registers.initialize (interface_uart_);
  if (expected_device_model != Registers::DeviceModel::Unknown)
    {
      registers.setDeviceModel (expected_device_model);
    }
  else
    {
      (void)communicating ();
    }
  registers.assumeDeviceReset ();
  initialize ();
  (void)finishSetupOrRecovery_ ();
}

uint8_t
TMC51X0::readVersion ()
{
  Registers::Ioin ioin;
  ioin.raw = registers.read (Registers::IoinAddress);
  (void)updateDeviceModelFromVersion_ (ioin.version ());
  return ioin.version ();
}

bool
TMC51X0::communicating ()
{
  uint8_t version = readVersion ();
  return ((version == Registers::VERSION_TMC5130) || (version == Registers::VERSION_TMC5160));
}

void
TMC51X0::setEnablePowerPin (size_t enable_power_pin)
{
  enable_power_pin_ = enable_power_pin;
  pinMode (enable_power_pin_, OUTPUT);
}

void
TMC51X0::setEnablePowerPolarity (uint8_t pin_value_when_enabled)
{
  pin_value_when_enabled_ = pin_value_when_enabled;
}

void
TMC51X0::enablePower ()
{
  if (enable_power_pin_ != NO_PIN)
    {
      digitalWrite (enable_power_pin_, pin_value_when_enabled_);
    }
}

void
TMC51X0::disablePower ()
{
  if (enable_power_pin_ != NO_PIN)
    {
      if (pin_value_when_enabled_ == LOW)
        {
          digitalWrite (enable_power_pin_, HIGH);
        }
      else
        {
          digitalWrite (enable_power_pin_, LOW);
        }
    }
}

void
TMC51X0::notePossibleMirrorDrift ()
{
  registers.notePossibleDrift ();
}

bool
TMC51X0::mirrorResyncRequired () const
{
  return registers.resyncRequired ();
}

tmc51x0::Registers::DeviceModel
TMC51X0::deviceModel () const
{
  return registers.deviceModel ();
}

bool
TMC51X0::resyncReadableConfiguration ()
{
  return registers.resyncReadableConfiguration ();
}

void
TMC51X0::reinitialize ()
{
  (void)recoverFromDeviceReset ();
}

bool
TMC51X0::recoverFromDeviceReset ()
{
  if (!registers.deviceModelKnown ())
    {
      (void)communicating ();
    }
  registers.assumeDeviceReset ();
  driver.reinitialize ();
  controller.reinitialize ();
  encoder.reinitialize ();
  return finishSetupOrRecovery_ ();
}

bool
TMC51X0::recoverIfNeeded ()
{
  if (!mirrorResyncRequired ())
    {
      return true;
    }
  return recoverFromDeviceReset ();
}

void
TMC51X0::beginHomeToSwitch (tmc51x0::HomeParameters home_parameters,
                            tmc51x0::SwitchParameters switch_parameters)
{
  driver.cacheDriverSettings ();
  controller.cacheControllerSettings ();
  controller.cacheSwitchSettings ();

  DriverParameters driver_parameters;
  driver.setup (driver_parameters);
  ControllerParameters controller_parameters;
  controller.setup (controller_parameters);

  controller.setupSwitches (switch_parameters);

  driver.writeRunCurrent (home_parameters.run_current);
  driver.writeHoldCurrent (home_parameters.hold_current);
  driver.writeHoldDelay (0);
  driver.writeStandstillMode (NormalMode);

  controller.writeRampMode (HoldMode);

  driver.writeChopperMode (SpreadCycleMode);
  driver.disableStealthChop ();
  driver.disableCoolStep ();
  driver.disableHighVelocityFullstep ();

  controller.writeStopMode (HardMode);
  controller.zeroActualPosition ();
  controller.writeTargetPosition (home_parameters.target_position);
  controller.writeMaxVelocity (home_parameters.velocity);
  controller.writeMaxAcceleration (home_parameters.acceleration);
  controller.writeZeroWaitDuration (home_parameters.zero_wait_duration);

  controller.writeRampMode (PositionMode);
}

void
TMC51X0::beginHomeToStall (tmc51x0::HomeParameters home_parameters,
                           tmc51x0::StallParameters stall_parameters)
{
  driver.cacheDriverSettings ();
  controller.cacheControllerSettings ();
  controller.cacheSwitchSettings ();

  DriverParameters driver_parameters;
  driver.setup (driver_parameters);
  ControllerParameters controller_parameters;
  controller.setup (controller_parameters);

  controller.writeStopMode (HardMode);
  controller.enableStallStop ();
  driver.disableStallGuardFilter ();
  driver.writeStallGuardThreshold (stall_parameters.stall_guard_threshold);

  driver.writeRunCurrent (home_parameters.run_current);
  driver.writeHoldCurrent (home_parameters.hold_current);
  driver.writeHoldDelay (0);
  driver.writeStandstillMode (NormalMode);

  controller.writeRampMode (HoldMode);

  driver.disableStealthChop ();
  driver.disableCoolStep ();
  driver.writeCoolStepThreshold (stall_parameters.cool_step_threshold);

  driver.writeChopperMode (SpreadCycleMode);

  controller.zeroActualPosition ();
  controller.writeTargetPosition (home_parameters.target_position);
  controller.writeMaxVelocity (home_parameters.velocity);
  controller.writeMaxAcceleration (home_parameters.acceleration);
  controller.writeZeroWaitDuration (home_parameters.zero_wait_duration);

  controller.writeRampMode (PositionMode);
}

void
TMC51X0::endHome ()
{
  controller.writeRampMode (HoldMode);

  controller.zeroActualPosition ();
  controller.zeroTargetPosition ();

  driver.restoreDriverSettings ();
  controller.restoreControllerSettings ();
  controller.restoreSwitchSettings ();

  // clear ramp_stat flags
  registers.read (Registers::RampStatAddress);

  controller.writeRampMode (PositionMode);
}

bool
TMC51X0::homed ()
{
  // reading ramp_stat clears flags and may cause motion after stall stop
  // better to read actual velocity instead
  int32_t actual_velocity = controller.readActualVelocity ();
  bool still = (actual_velocity == 0);
  if (still)
    {
      controller.writeRampMode (HoldMode);
    }
  return still;
}

// private
void
TMC51X0::initialize ()
{
  driver.initialize (registers);
  controller.initialize (registers);
  encoder.initialize (registers);
  printer.initialize (registers);
}

bool
TMC51X0::updateDeviceModelFromVersion_ (uint8_t version)
{
  if (version == Registers::VERSION_TMC5130)
    {
      registers.setDeviceModel (Registers::DeviceModel::TMC5130A);
      return true;
    }
  if (version == Registers::VERSION_TMC5160)
    {
      registers.setDeviceModel (Registers::DeviceModel::TMC5160A);
      return true;
    }
  return false;
}

bool
TMC51X0::finishSetupOrRecovery_ ()
{
  registers.notePossibleDrift ();
  (void)registers.readAndClearGstat ();

  const Registers::DeviceModel model_before = registers.deviceModel ();
  const bool version_ok = communicating ();
  const bool model_changed = version_ok && (registers.deviceModel () != model_before);
  if (model_changed)
    {
      registers.assumeDeviceReset ();
      driver.reinitialize ();
      controller.reinitialize ();
      encoder.reinitialize ();
      registers.notePossibleDrift ();
      (void)registers.readAndClearGstat ();
    }

  const bool readable_ok = version_ok ? registers.resyncReadableConfiguration () : false;
  if (version_ok && readable_ok)
    {
      registers.clearResyncRequired ();
    }
  return version_ok && readable_ok;
}
