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

#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"


struct TMC51X0
{
  void setupSpi(tmc51x0::SpiParameters spi_parameters);
  void setupUart(tmc51x0::UartParameters uart_parameters);

  uint8_t readVersion();

  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

private:
  tmc51x0::SpiInterface interface_spi_;
  tmc51x0::UartInterface interface_uart_;
  void initialize();
};

#endif
