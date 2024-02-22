// ----------------------------------------------------------------------------
// Constants.hpp
//
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#ifndef TMC51X0_CONSTANTS_HPP
#define TMC51X0_CONSTANTS_HPP
#include <Arduino.h>
#include <SPI.h>


namespace tmc51x0
{
namespace constants
{
// SPI Settings
constexpr uint32_t spi_clock = 2500000;
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040)
  constexpr BitOrder spi_bit_order = MSBFIRST;
#else
  constexpr uint8_t spi_bit_order = MSBFIRST;
#endif
constexpr uint8_t spi_data_mode = SPI_MODE3;
}
}
#endif
