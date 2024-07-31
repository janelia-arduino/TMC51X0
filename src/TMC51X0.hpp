// ----------------------------------------------------------------------------
// TMC51X0.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HPP
#define TMC51X0_HPP
#include <Arduino.h>
#include <SPI.h>

#include "SPIInterface.hpp"
#include "UARTInterface.hpp"
#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"


struct TMC51X0
{
  void setupSPI(tmc51x0::SPIParameters spi_parameters);
  void setupUART(tmc51x0::UARTParameters uart_parameters);

  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

private:
  tmc51x0::SPIInterface interface_spi_;
  tmc51x0::UARTInterface interface_uart_;
  void initialize();
};

#endif
