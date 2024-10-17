// ----------------------------------------------------------------------------
// SpiInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "SpiInterface.hpp"


using namespace tmc51x0;

void SpiInterface::setup(tmc51x0::SpiParameters spi_parameters)
{
  interface_mode = Interface::SPI;
  spi_parameters_ = spi_parameters;
  spi_settings_ = SPISettings(spi_parameters.clock_rate,
    spi_parameters.bit_order,
    spi_parameters.data_mode);

  pinMode(spi_parameters_.chip_select_pin, OUTPUT);
  disableChipSelect();
}

void SpiInterface::writeRegister(uint8_t register_address,
  uint32_t data)
{
  CopiDatagram copi_datagram;
  copi_datagram.register_address = register_address;
  copi_datagram.rw = RW_WRITE;
  copi_datagram.data = data;
  writeRead(copi_datagram);
}

uint32_t SpiInterface::readRegister(uint8_t register_address)
{
  CopiDatagram copi_datagram;
  copi_datagram.register_address = register_address;
  copi_datagram.rw = RW_READ;
  copi_datagram.data = 0;
  CipoDatagram cipo_datagram = writeRead(copi_datagram);
  // cipo data is returned on second read
  cipo_datagram = writeRead(copi_datagram);
  return cipo_datagram.data;
}

// private

SpiInterface::CipoDatagram SpiInterface::writeRead(CopiDatagram copi_datagram)
{
  uint8_t write_byte, read_byte;
  CipoDatagram cipo_datagram;
  cipo_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    write_byte = (copi_datagram.bytes >> (8*i)) & 0xff;
    read_byte = spi_parameters_.spi_ptr->transfer(write_byte);
    cipo_datagram.bytes |= ((uint32_t)read_byte) << (8*i);
  }
  endTransaction();
  noInterrupts();
  spi_status_ = cipo_datagram.spi_status;
  interrupts();
  return cipo_datagram;
}

void SpiInterface::enableChipSelect()
{
  digitalWrite(spi_parameters_.chip_select_pin, LOW);
}

void SpiInterface::disableChipSelect()
{
  digitalWrite(spi_parameters_.chip_select_pin, HIGH);
}

void SpiInterface::beginTransaction()
{
  spi_parameters_.spi_ptr->beginTransaction(spi_settings_);
  enableChipSelect();
}

void SpiInterface::endTransaction()
{
  disableChipSelect();
  spi_parameters_.spi_ptr->endTransaction();
}
