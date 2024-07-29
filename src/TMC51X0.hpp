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

#include "InterfaceSPI.hpp"
#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"


struct TMC51X0
{
  void setupSPI(SPIClass & spi,
    size_t chip_select_pin);

  void setupUART(HardwareSerial & uart,
    size_t enable_tx_pin,
    size_t enable_rx_pin,
    size_t enable_tx_polarity=HIGH,
    size_t enable_rx_polarity=LOW);

  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

private:
  tmc51x0::InterfaceSPI interface_spi_;
  void initialize();
};

#endif
