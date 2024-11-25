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

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS)
  const static BitOrder bit_order = MSBFIRST;
#else
  const static uint8_t bit_order = MSBFIRST;
#endif
  const static uint8_t data_mode = SPI_MODE3;

  SpiParameters(SPIClass & spi_,
    uint32_t clock_rate_,
    size_t chip_select_pin_=PIN_DEFAULT)
  {
    spi_ptr = &spi_;
    clock_rate = clock_rate_;
    chip_select_pin = chip_select_pin_;
  };

  SpiParameters()
  {
    spi_ptr = nullptr;
    clock_rate = CLOCK_RATE_DEFAULT;
    chip_select_pin = PIN_DEFAULT;
  };

  bool operator==(const SpiParameters & rhs) const
  {
    if ((this->spi_ptr == rhs.spi_ptr) &&
      (this->clock_rate == rhs.clock_rate) &&
      (this->chip_select_pin == rhs.chip_select_pin))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const SpiParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint32_t CLOCK_RATE_DEFAULT = 1000000;
  const static size_t PIN_DEFAULT = 255;
};
}
#endif
