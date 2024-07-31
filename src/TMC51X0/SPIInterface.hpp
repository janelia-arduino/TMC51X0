// ----------------------------------------------------------------------------
// SPIInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_INTERFACE_SPI_HPP
#define TMC51X0_INTERFACE_SPI_HPP
#include <Arduino.h>
#include <SPI.h>

#include "Interface.hpp"
#include "Constants.hpp"


namespace tmc51x0
{
struct SPIParameters
{
  SPIClass * spi_ptr;
  size_t chip_select_pin;

  SPIParameters(SPIClass & spi_,
    size_t chip_select_pin_)
  {
    spi_ptr = &spi_;
    chip_select_pin = chip_select_pin_;
  };

  SPIParameters()
  {
    spi_ptr = SPI_PTR_DEFAULT;
    chip_select_pin = PIN_DEFAULT;
  };

  bool operator==(const SPIParameters & rhs) const
  {
    if ((this->spi_ptr == rhs.spi_ptr) &&
      (this->chip_select_pin == rhs.chip_select_pin))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const SPIParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static SPIClass * SPI_PTR_DEFAULT = nullptr;
  const static size_t PIN_DEFAULT = 255;

};

class SPIInterface : public Interface
{
public:
  SPIInterface();

  void setup(SPIParameters spi_parameters);

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  SPIParameters spi_parameters_;
  const SPISettings spi_settings_;

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

  const static uint8_t SPI_DATAGRAM_SIZE = 5;

  // PICO Datagrams
  union PicoDatagram
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
  const static uint8_t SPI_RW_READ = 0;
  const static uint8_t SPI_RW_WRITE = 1;

  // MISO Datagrams
  union MisoDatagram
  {
    struct
    {
      uint64_t data : 32;
      SpiStatus spi_status;
      uint64_t reserved : 24;
    };
    uint64_t bytes;
  };

  uint8_t spi_buffer_[SPI_DATAGRAM_SIZE];

  MisoDatagram writeRead(PicoDatagram pico_datagram);

  void enableChipSelect();
  void disableChipSelect();
  void beginTransaction();
  void endTransaction();
};
}
#endif
