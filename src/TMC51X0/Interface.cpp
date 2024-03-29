// ----------------------------------------------------------------------------
// Interface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "Interface.hpp"


using namespace tmc51x0;

Interface::Interface() :
spi_settings_(SPISettings(constants::spi_clock, constants::spi_bit_order, constants::spi_data_mode))
{
  chip_select_pin_ = -1;
}

void Interface::setup(SPIClass & spi,
  size_t chip_select_pin)
{
  spi_ptr_ = &spi;
  chip_select_pin_ = chip_select_pin;

  pinMode(chip_select_pin_,OUTPUT);
  disableChipSelect();

  spi_ptr_->begin();
}

void Interface::writeRegister(uint8_t register_address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.register_address = register_address;
  mosi_datagram.rw = SPI_RW_WRITE;
  mosi_datagram.data = data;
  writeRead(mosi_datagram);
}

uint32_t Interface::readRegister(uint8_t register_address)
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

Interface::MisoDatagram Interface::writeRead(MosiDatagram mosi_datagram)
{
  uint8_t byte_write, byte_read;
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(SPI_DATAGRAM_SIZE - 1); i>=0; --i)
  {
    byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    byte_read = spi_ptr_->transfer(byte_write);
    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }
  endTransaction();
  noInterrupts();
  spi_status_ = miso_datagram.spi_status;
  interrupts();
  return miso_datagram;
}

void Interface::enableChipSelect()
{
  digitalWrite(chip_select_pin_, LOW);
}

void Interface::disableChipSelect()
{
  digitalWrite(chip_select_pin_, HIGH);
}

void Interface::beginTransaction()
{
  spi_ptr_->beginTransaction(spi_settings_);
  enableChipSelect();
}

void Interface::endTransaction()
{
  disableChipSelect();
  spi_ptr_->endTransaction();
}
