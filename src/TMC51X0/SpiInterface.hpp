// ----------------------------------------------------------------------------
// SpiInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_INTERFACE_SPI_HPP
#define TMC51X0_INTERFACE_SPI_HPP
#include <Arduino.h>
#include <SPI.h>

#include "Interface.hpp"

#if defined(ARDUINO_ARCH_STM32)
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             ((uint32_t)digitalPinToPinName(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, pin)          digitalReadFast((PinName)pin)
#define DIRECT_WRITE_LOW(base, pin)     digitalWriteFast((PinName)pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWriteFast((PinName)pin, HIGH)
#define DIRECT_MODE_INPUT(base, pin)    pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0))
#define DIRECT_MODE_OUTPUT(base, pin)   pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0))

#define pin_size_t IO_REG_TYPE

#elif defined(ARDUINO_ARCH_ESP32)
#define pin_size_t uint16_t
#endif

namespace tmc51x0
{
struct SpiParameters
{
  SPIClass * spi_ptr;
  uint32_t clock_rate;
  pin_size_t chip_select_pin;

#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS) || (ARDUINO_ARCH_STM32)
  const static BitOrder bit_order = MSBFIRST;
#else
  const static uint8_t bit_order = MSBFIRST;
#endif
  const static uint8_t data_mode = SPI_MODE3;

  SpiParameters(SPIClass & spi_,
    uint32_t clock_rate_,
    pin_size_t chip_select_pin_)
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
  const static pin_size_t PIN_DEFAULT = 255;
};

class SpiInterface : public Interface
{
public:
  void setup(SpiParameters spi_parameters);

  void writeRegister(uint8_t register_address, uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  SpiParameters spi_parameters_;
  SPISettings spi_settings_;

  struct SpiStatus
  {
    uint8_t reset_flag : 1;
    uint8_t driver_error : 1;
    uint8_t sg2 : 1;
    uint8_t standstill : 1;
    uint8_t velocity_reached : 1;
    uint8_t position_reached : 1;
    uint8_t status_stop_l : 1;
    uint8_t status_stop_r : 1;
  };
  SpiStatus spi_status_;

  const static uint8_t DATAGRAM_SIZE = 5;

  // Copi Datagrams
  union CopiDatagram
  {
    struct
    {
      uint64_t data : 32;
      uint64_t register_address : 7;
      uint64_t rw : 1;
      uint64_t reserved : 24;
    };
    uint64_t bytes;
  };
  const static uint8_t RW_READ = 0;
  const static uint8_t RW_WRITE = 1;

  // Cipo Datagrams
  union CipoDatagram
  {
    struct
    {
      uint64_t data : 32;
      SpiStatus spi_status;
      uint64_t reserved : 24;
    };
    uint64_t bytes;
  };

  uint8_t spi_buffer_[DATAGRAM_SIZE];

  CipoDatagram writeRead(CopiDatagram copi_datagram);

  void enableChipSelect();
  void disableChipSelect();
  void beginTransaction();
  void endTransaction();
};
}
#endif
