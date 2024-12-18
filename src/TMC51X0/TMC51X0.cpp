// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"


using namespace tmc51x0;

TMC51X0::TMC51X0()
{
  enable_power_pin_ = NO_PIN;
  pin_value_when_enabled_ = HIGH;
}

void TMC51X0::setupSpi(SpiParameters spi_parameters)
{
  interface_spi_.setup(spi_parameters);
  registers.initialize(interface_spi_);
  initialize();
}

void TMC51X0::setupUart(UartParameters uart_parameters)
{
  interface_uart_.setup(uart_parameters);
  registers.initialize(interface_uart_);
  initialize();
}

uint8_t TMC51X0::readVersion()
{
  Registers::Ioin ioin;
  ioin.bytes = registers.read(Registers::IOIN);
  return ioin.version;
}

bool TMC51X0::communicating()
{
  uint8_t version = readVersion();
  return ((version == Registers::VERSION_TMC5130) ||
    (version == Registers::VERSION_TMC5160));
}

void TMC51X0::setEnablePowerPin(size_t enable_power_pin)
{
  enable_power_pin_ = enable_power_pin;
  pinMode(enable_power_pin_, OUTPUT);
}

void TMC51X0::setEnablePowerPolarity(uint8_t pin_value_when_enabled)
{
  pin_value_when_enabled_ = pin_value_when_enabled;
}

void TMC51X0::enablePower()
{
  if (enable_power_pin_ != NO_PIN)
  {
    digitalWrite(enable_power_pin_, pin_value_when_enabled_);
  }
}

void TMC51X0::disablePower()
{
  if (enable_power_pin_ != NO_PIN)
  {
    if (pin_value_when_enabled_ == LOW)
    {
      digitalWrite(enable_power_pin_, HIGH);
    }
    else
    {
      digitalWrite(enable_power_pin_, LOW);
    }
  }
}

void TMC51X0::reinitialize()
{
  driver.reinitialize();
  controller.reinitialize();
  encoder.reinitialize();
}

void TMC51X0::beginHomeToSwitch(tmc51x0::HomeParameters home_parameters,
  tmc51x0::SwitchParameters switch_parameters)
{
  driver.cacheDriverSettings();
  controller.cacheControllerSettings();
  controller.cacheSwitchSettings();

  controller.setupSwitches(switch_parameters);

  driver.writeRunCurrent(home_parameters.run_current);
  driver.writeHoldCurrent(home_parameters.hold_current);
  driver.writeHoldDelay(0);
  driver.writeStandstillMode(NORMAL);

  controller.writeRampMode(HOLD);

  driver.writeChopperMode(SPREAD_CYCLE);
  driver.disableStealthChop();
  driver.disableCoolStep();
  driver.disableHighVelocityFullstep();

  controller.writeStopMode(HARD);
  controller.zeroActualPosition();
  controller.writeTargetPosition(home_parameters.target_position);
  controller.writeMaxVelocity(home_parameters.velocity);
  controller.writeStartVelocity(home_parameters.velocity);
  controller.writeStopVelocity(home_parameters.velocity);
  controller.writeFirstVelocity(0);
  controller.writeMaxAcceleration(home_parameters.acceleration);
  controller.writeZeroWaitDuration(home_parameters.zero_wait_duration);

  controller.writeRampMode(POSITION);
}

void TMC51X0::beginHomeToStall(tmc51x0::HomeParameters home_parameters,
  tmc51x0::StallParameters stall_parameters)
{
  driver.cacheDriverSettings();
  controller.cacheControllerSettings();
  controller.cacheSwitchSettings();

  controller.writeStopMode(HARD);
  controller.enableStallStop();
  driver.disableStallGuardFilter();
  driver.writeStallGuardThreshold(stall_parameters.stall_guard_threshold);

  driver.writeRunCurrent(home_parameters.run_current);
  driver.writeHoldCurrent(home_parameters.hold_current);
  driver.writeHoldDelay(0);
  driver.writeStandstillMode(NORMAL);

  controller.writeRampMode(HOLD);

  driver.disableStealthChop();
  driver.disableCoolStep();
  int32_t tstep = converter.velocityChipToTstep(home_parameters.velocity);

  switch (stall_parameters.stall_mode)
  {
    case COOL_STEP_THRESHOLD:
    {
      driver.writeChopperMode(SPREAD_CYCLE);
      driver.writeCoolStepThreshold(stall_parameters.cool_step_threshold);
      break;
    }
    case DC_STEP:
    {
      driver.writeChopperMode(CLASSIC);
      driver.writeHighVelocityThreshold(tstep + 100);
      driver.enableHighVelocityFullstep();
      driver.enableHighVelocityChopperSwitch();
      driver.writeEnabledToff(8);
      driver.writeComparatorBlankTime(CLOCK_CYCLES_36);
      driver.writeDcTime(CLOCK_CYCLES_36);
      controller.writeMinDcStepVelocity(home_parameters.velocity - 100);
      break;
    }
  }
  controller.zeroActualPosition();
  controller.writeTargetPosition(home_parameters.target_position);
  controller.writeMaxVelocity(home_parameters.velocity);
  controller.writeStartVelocity(home_parameters.velocity);
  controller.writeStopVelocity(home_parameters.velocity);
  controller.writeFirstVelocity(0);
  controller.writeMaxAcceleration(home_parameters.acceleration);
  controller.writeZeroWaitDuration(home_parameters.zero_wait_duration);

  controller.writeRampMode(POSITION);
}

void TMC51X0::endHome()
{
  controller.writeRampMode(HOLD);

  controller.zeroActualPosition();
  controller.zeroTargetPosition();

  driver.restoreDriverSettings();
  controller.restoreControllerSettings();
  controller.restoreSwitchSettings();

  // clear ramp_stat flags
  registers.read(Registers::RAMP_STAT);

  controller.writeRampMode(POSITION);
}

bool TMC51X0::homed()
{
  // reading ramp_stat clears flags and may cause motion after stall stop
  // better to read actual velocity instead
  int32_t actual_velocity = controller.readActualVelocity();
  bool stalled = (actual_velocity == 0);
  if (stalled)
  {
    controller.writeRampMode(HOLD);
  }
  return stalled;
}

// private
void TMC51X0::initialize()
{
  driver.initialize(registers);
  controller.initialize(registers);
  encoder.initialize(registers);
  printer.initialize(registers);
}
