// ----------------------------------------------------------------------------
// InterfaceSPI.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "InterfaceSPI.hpp"


using namespace tmc51x0;

InterfaceSPI::InterfaceSPI() :
spi_settings_(SPISettings(constants::spi_clock, constants::spi_bit_order, constants::spi_data_mode))
{
}

void InterfaceSPI::setup(tmc51x0::SPIParameters spi_parameters)
{
  spi_parameters_ = spi_parameters;

  pinMode(spi_parameters_.chip_select_pin,OUTPUT);
  disableChipSelect();

  spi_parameters_.spi_ptr->begin();
}

void InterfaceSPI::writeRegister(uint8_t register_address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.register_address = register_address;
  mosi_datagram.rw = SPI_RW_WRITE;
  mosi_datagram.data = data;
  writeRead(mosi_datagram);
}

uint32_t InterfaceSPI::readRegister(uint8_t register_address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.register_address = register_address;
  mosi_datagram.rw = SPI_RW_READ;
  mosi_datagram.data = 0;
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  // miso data is returned on second read
  miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.data;
}

// private

InterfaceSPI::MisoDatagram InterfaceSPI::writeRead(MosiDatagram mosi_datagram)
{
  uint8_t byte_write, byte_read;
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(SPI_DATAGRAM_SIZE - 1); i>=0; --i)
  {
    byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    byte_read = spi_parameters_.spi_ptr->transfer(byte_write);
    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }
  endTransaction();
  noInterrupts();
  spi_status_ = miso_datagram.spi_status;
  interrupts();
  return miso_datagram;
}

void InterfaceSPI::enableChipSelect()
{
  digitalWrite(spi_parameters_.chip_select_pin, LOW);
}

void InterfaceSPI::disableChipSelect()
{
  digitalWrite(spi_parameters_.chip_select_pin, HIGH);
}

void InterfaceSPI::beginTransaction()
{
  spi_parameters_.spi_ptr->beginTransaction(spi_settings_);
  enableChipSelect();
}

void InterfaceSPI::endTransaction()
{
  disableChipSelect();
  spi_parameters_.spi_ptr->endTransaction();
}
