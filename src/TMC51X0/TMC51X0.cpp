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

// optional
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

void TMC51X0::beginHome(tmc51x0::HomeParameters home_parameters)
{
  driver.cacheDriverSettings();
  controller.cacheControllerSettings();

  driver.writeStandstillMode(PASSIVE_BRAKING_LS);

  driver.disable();

  driver.disableAutomaticCurrentControl();
  driver.writeStealthChopThreshold(home_parameters.velocity + 1);
  driver.enableStealthChop();
  driver.disableCoolStep();
  driver.disableHighVelocityFullstep();
  driver.writePwmOffset(home_parameters.pwm_offset);
  driver.writePwmGradient(0);

  controller.writeRampMode(POSITION);
  controller.writeStopMode(HARD);
  controller.zeroActualPosition();
  controller.writeTargetPosition(home_parameters.target_position);
  controller.writeMaxVelocity(home_parameters.velocity);
  controller.writeStartVelocity(home_parameters.velocity);
  controller.writeStopVelocity(home_parameters.velocity);
  controller.writeFirstVelocity(0);
  controller.writeMaxAcceleration(home_parameters.acceleration);

  driver.enable();
}

void TMC51X0::endHome()
{
  controller.zeroActualPosition();
  controller.zeroTargetPosition();

  driver.restoreDriverSettings();
  controller.restoreControllerSettings();
}

bool TMC51X0::homed()
{
  return controller.zeroVelocity();
}

// private
void TMC51X0::initialize()
{
  driver.initialize(registers);
  controller.initialize(registers);
  encoder.initialize(registers);
  printer.initialize(registers);
}
