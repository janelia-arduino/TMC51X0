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
  uint32_t   clock_rate;
  size_t     chip_select_pin;

  constexpr SpiParameters(SPIClass *spi_ptr          = nullptr,
                          uint32_t   clock_rate      = 1000000,
                          size_t     chip_select_pin = 0)
  : spi_ptr(spi_ptr),
    clock_rate(clock_rate),
    chip_select_pin(chip_select_pin)
  {}

  // "Named parameter" style helpers

  constexpr SpiParameters withSpi(SPIClass *spi) const
  {
    return SpiParameters(
                         spi,
                         clock_rate,
                         chip_select_pin);
  }

  constexpr SpiParameters withClockRate(uint32_t hz) const
  {
    return SpiParameters(
                         spi_ptr,
                         hz,
                         chip_select_pin);
  }

  constexpr SpiParameters withChipSelectPin(size_t cs) const
  {
    return SpiParameters(
                         spi_ptr,
                         clock_rate,
                         cs);
  }
};
} // namespace tmc51x0

#endif // TMC51X0_SPI_PARAMETERS_HPP
