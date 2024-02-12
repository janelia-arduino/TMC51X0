// ----------------------------------------------------------------------------
// TMC51X0.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC51X0.h"


TMC51X0::TMC51X0()
{
  hardware_enable_pin_ = -1;
}

void TMC51X0::setup(size_t chip_select_pin)
{
  chip_select_pin_ = chip_select_pin;

  pinMode(chip_select_pin_,OUTPUT);
  disableChipSelect();

  spiBegin();
}

void TMC51X0::setHardwareEnablePin(size_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  hardwareDisable();
}

void TMC51X0::enable()
{
  hardwareEnable();
  // chopper_config_.toff = toff_;
  // writeStoredChopperConfig();
}

void TMC51X0::disable()
{
  hardwareDisable();
  // chopper_config_.toff = TOFF_DISABLE;
  // writeStoredChopperConfig();
}

uint8_t TMC51X0::getVersion()
{
  Input input;
  input.bytes = readRegister(REGISTER_ADDRESS_IOIN);

  return input.version;
}

// private

void TMC51X0::hardwareEnable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
}

void TMC51X0::hardwareDisable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
}

void TMC51X0::writeRegister(uint8_t register_address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.register_address = register_address;
  mosi_datagram.rw = SPI_RW_WRITE;
  mosi_datagram.data = data;
  writeRead(mosi_datagram);
}

uint32_t TMC51X0::readRegister(uint8_t register_address)
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

TMC51X0::MisoDatagram TMC51X0::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.bytes = 0x0;
  beginTransaction();
  for (int i=(SPI_DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.bytes >> (8*i)) & 0xff;
    uint8_t byte_read = spiTransfer(byte_write);
    miso_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
  }
  endTransaction();
  noInterrupts();
  spi_status_ = miso_datagram.spi_status;
  interrupts();
  return miso_datagram;
}

void TMC51X0::enableChipSelect()
{
  digitalWrite(chip_select_pin_, LOW);
}

void TMC51X0::disableChipSelect()
{
  digitalWrite(chip_select_pin_, HIGH);
}

void TMC51X0::beginTransaction()
{
  spiBeginTransaction(SPISettings(SPI_CLOCK, SPI_BIT_ORDER, SPI_MODE));
  enableChipSelect();
}

void TMC51X0::endTransaction()
{
  disableChipSelect();
  spiEndTransaction();
}

// protected
void TMC51X0::spiBegin()
{
  SPI.begin();
}

void TMC51X0::spiBeginTransaction(SPISettings settings)
{
  SPI.beginTransaction(settings);
}

void TMC51X0::spiEndTransaction()
{
  SPI.endTransaction();
}

uint8_t TMC51X0::spiTransfer(uint8_t byte)
{
  return SPI.transfer(byte);
}

void TMC51X0::spiTransfer(void *buffer, size_t count)
{
  return SPI.transfer(buffer, count);
}
