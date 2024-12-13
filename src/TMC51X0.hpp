// ----------------------------------------------------------------------------
// TMC51X0.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HPP
#define TMC51X0_HPP
#include <Arduino.h>

#include "./TMC51X0/SpiInterface.hpp"
#include "./TMC51X0/UartInterface.hpp"
#include "./TMC51X0/HomeParameters.hpp"
#include "./TMC51X0/SwitchParameters.hpp"
#include "./TMC51X0/StallParameters.hpp"

#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"


struct TMC51X0
{
  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

  TMC51X0();

  void setupSpi(tmc51x0::SpiParameters spi_parameters);
  void setupUart(tmc51x0::UartParameters uart_parameters);

  uint8_t readVersion();
  bool communicating();

  void setEnablePowerPin(size_t enable_power_pin);
  void setEnablePowerPolarity(uint8_t pin_value_when_enabled);
  void enablePower();
  void disablePower();

  // call reinitialize after cycling power to the chip
  void reinitialize();

  void beginHomeToSwitch(tmc51x0::HomeParameters home_parameters,
    tmc51x0::SwitchParameters switch_parameters);
  void beginHomeToStall(tmc51x0::HomeParameters home_parameters,
    tmc51x0::StallParameters stall_parameters);
  void endHome();
  bool homed();

private:
  tmc51x0::SpiInterface interface_spi_;
  tmc51x0::UartInterface interface_uart_;
  size_t enable_power_pin_;
  uint8_t pin_value_when_enabled_;
  void initialize();
};

#endif
