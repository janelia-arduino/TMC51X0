// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"


using namespace tmc51x0;

void TMC51X0::setupSPI(SPIClass & spi,
  size_t chip_select_pin)
{
  interface_spi_.setup(spi, chip_select_pin);
  registers.initialize(interface_spi_);
  initialize();
}

void TMC51X0::setupUART(HardwareSerial & uart,
  size_t enable_tx_pin,
  size_t enable_rx_pin,
  size_t enable_tx_polarity,
  size_t enable_rx_polarity)
{
  initialize();
}

// private
void TMC51X0::initialize()
{
  driver.initialize(registers, converter);
  controller.initialize(registers, converter);
  encoder.initialize(registers, converter);
  printer.initialize(registers);
}
