// ----------------------------------------------------------------------------
// SpiParameters.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_SPI_PARAMETERS_HPP
#define TMC51X0_SPI_PARAMETERS_HPP
#include <SPI.h>


namespace tmc51x0
{
struct SpiParameters
{
  SPIClass * spi_ptr;
  uint32_t clock_rate = 1000000;
  size_t chip_select_pin;

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_MBED)
  const static BitOrder bit_order = MSBFIRST;
#else
  const static uint8_t bit_order = MSBFIRST;
#endif
  const static uint8_t data_mode = SPI_MODE3;
};
}
#endif
