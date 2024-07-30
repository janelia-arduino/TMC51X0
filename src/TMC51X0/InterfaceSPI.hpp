// ----------------------------------------------------------------------------
// InterfaceSPI.hpp
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
class SPIParameters
{
public:
  SPIParameters(SPIClass & spi,
    size_t chip_select_pin)
  {
    spi_ptr_ = &spi;
    chip_select_pin_ = chip_select_pin;
  };

  bool operator==(const SPIParameters & rhs) const
  {
    if ((this->spi_ptr_ == rhs.spi_ptr_) &&
        (this->chip_select_pin_ == rhs.chip_select_pin_))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const SPIParameters & rhs) const
  {
    return !(*this == rhs);
  }

  SPIClass * getSPIPointer() const
  {
    return spi_ptr_;
  }
  uint8_t getChipSelectPin() const
  {
    return chip_select_pin_;
  }
private:
  SPIClass * spi_ptr_;
  uint8_t chip_select_pin_;
};

class InterfaceSPI : public Interface
{
public:
  InterfaceSPI();

  void setup(SPIParameters parameters);

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  // SPI
  SPIClass * spi_ptr_;
  int16_t chip_select_pin_;

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

  // MOSI Datagrams
  union MosiDatagram
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

  MisoDatagram writeRead(MosiDatagram mosi_datagram);

  void enableChipSelect();
  void disableChipSelect();
  void beginTransaction();
  void endTransaction();
};
}
#endif
