// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.hpp"


using namespace tmc51x0;

void TMC51X0::setup(SPIClass & spi,
  size_t chip_select_pin,
  uint8_t clock_frequency_mhz)
{
  registers.setup(spi, chip_select_pin);
  converter_.setup(clock_frequency_mhz);
  driver.setup(registers, converter_);
  controller.setup(registers, converter_);
  encoder.setup(registers, converter_);
}

Registers::Gstat TMC51X0::readAndClearGlobalStatus()
{
  Registers::Gstat gstat_read, gstat_write;
  gstat_read.bytes = registers.read(tmc51x0::Registers::GSTAT);
  gstat_write.reset = 1;
  gstat_write.drv_err = 1;
  gstat_write.uv_cp = 1;
  registers.write(tmc51x0::Registers::GSTAT, gstat_write.bytes);
  return gstat_read;
}

