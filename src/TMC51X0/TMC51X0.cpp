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

uint8_t TMC51X0::getVersion()
{
  Input input;
  input.bytes = read(ADDRESS_IOIN);

  return input.version;
}

// private
write(uint8_t register_address,
  uint32_t data)
{
  WriteReadReplyDatagram write_datagram;
  write_datagram.bytes = 0;
  write_datagram.sync = SYNC;
  write_datagram.serial_address = serial_address_;
  write_datagram.register_address = register_address;
  write_datagram.rw = RW_WRITE;
  write_datagram.data = reverseData(data);
  write_datagram.crc = calculateCrc(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

  sendDatagramUnidirectional(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

uint32_t TMC2209::read(uint8_t register_address)
{
  ReadRequestDatagram read_request_datagram;
  read_request_datagram.bytes = 0;
  read_request_datagram.sync = SYNC;
  read_request_datagram.serial_address = serial_address_;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw = RW_READ;
  read_request_datagram.crc = calculateCrc(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

  sendDatagramBidirectional(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

  uint32_t reply_delay = 0;
  while ((serialAvailable() < WRITE_READ_REPLY_DATAGRAM_SIZE) and
    (reply_delay < REPLY_DELAY_MAX_MICROSECONDS))
  {
    delayMicroseconds(REPLY_DELAY_INC_MICROSECONDS);
    reply_delay += REPLY_DELAY_INC_MICROSECONDS;
  }

  if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS)
  {
    return 0;
  }

  uint64_t byte;
  uint8_t byte_count = 0;
  WriteReadReplyDatagram read_reply_datagram;
  read_reply_datagram.bytes = 0;
  for (uint8_t i=0; i<WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
  {
    byte = serialRead();
    read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
  }

  return reverseData(read_reply_datagram.data);
}

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
