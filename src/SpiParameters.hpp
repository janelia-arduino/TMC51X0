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
  uint32_t clock_rate;
  size_t chip_select_pin;

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_MBED)
  const static BitOrder bit_order = MSBFIRST;
#else
  const static uint8_t bit_order = MSBFIRST;
#endif
  const static uint8_t data_mode = SPI_MODE3;

  constexpr SpiParameters(SPIClass *s = nullptr,
                          uint32_t hz = 1000000,
                          size_t cs = 0)
  : spi_ptr(s), clock_rate(hz), chip_select_pin(cs) {}

  constexpr SpiParameters withSpi(SPIClass *s) const
  {
    return SpiParameters{s, clock_rate, chip_select_pin};
  }

  constexpr SpiParameters withClockRate(uint32_t hz) const
  {
    return SpiParameters{spi_ptr, hz, chip_select_pin};
  }

  constexpr SpiParameters withChipSelectPin(size_t cs) const
  {
    return SpiParameters{spi_ptr, clock_rate, cs};
  }
};
}
#endif
