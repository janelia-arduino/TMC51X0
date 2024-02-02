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
  digitalWrite(chip_select_pin_,HIGH);

  spiBegin();
}

void TMC51X0::setHardwareEnablePin(uint8_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  digitalWrite(hardware_enable_pin_, HIGH);
}

void TMC51X0::enable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
  // chopper_config_.toff = toff_;
  // writeStoredChopperConfig();
}

void TMC51X0::disable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
  // chopper_config_.toff = TOFF_DISABLE;
  // writeStoredChopperConfig();
}

uint32_t TMC51X0::getVersion()
{
  return readRegister(SMDA_COMMON, ADDRESS_TYPE_VERSION_429);
}

// private
uint32_t TMC51X0::readRegister(uint8_t smda,
  uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.rrs = RRS_REGISTER;
  mosi_datagram.address = address;
  mosi_datagram.smda = smda;
  mosi_datagram.rw = RW_READ;
  mosi_datagram.data = 0;
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.data;
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
  enableChipSelect();
  delayMicroseconds(1);
  spiBeginTransaction(SPISettings(SPI_CLOCK, SPI_BIT_ORDER, SPI_MODE));
}

void TMC51X0::endTransaction()
{
  spiEndTransaction();
  delayMicroseconds(1);
  disableChipSelect();
}

// protected
void TMC51X0::spiBegin()
{
  SPI.begin();
}

uint8_t TMC51X0::spiTransfer(uint8_t byte)
{
  return SPI.transfer(byte);
}

void TMC51X0::spiBeginTransaction(SPISettings settings)
{
  SPI.beginTransaction(settings);
}

void TMC51X0::spiEndTransaction()
{
  SPI.endTransaction();
}
