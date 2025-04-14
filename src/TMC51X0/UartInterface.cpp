// ----------------------------------------------------------------------------
// UartInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "UartInterface.hpp"


using namespace tmc51x0;

void UartInterface::setup(UartParameters uart_parameters)
{
  interface_mode = UartMode;
  uart_parameters_ = uart_parameters;

  pinMode(uart_parameters_.enable_txrx_pin, OUTPUT);
  disableTxEnableRx();
}

void UartInterface::writeRegister(uint8_t register_address,
  uint32_t data)
{
  CopiWriteDatagram copi_write_datagram;
  copi_write_datagram.bytes = 0;
  copi_write_datagram.sync = SYNC;
  copi_write_datagram.node_address = uart_parameters_.node_address;
  copi_write_datagram.register_address = register_address;
  copi_write_datagram.rw = RW_WRITE;
  copi_write_datagram.data = reverseData(data);
  copi_write_datagram.crc = calculateCrc(copi_write_datagram, COPI_WRITE_DATAGRAM_SIZE);
  blockingWrite(copi_write_datagram, COPI_WRITE_DATAGRAM_SIZE);
}

uint32_t UartInterface::readRegister(uint8_t register_address)
{
  CopiReadDatagram copi_read_datagram;
  copi_read_datagram.bytes = 0;
  copi_read_datagram.sync = SYNC;
  copi_read_datagram.node_address = uart_parameters_.node_address;
  copi_read_datagram.register_address = register_address;
  copi_read_datagram.rw = RW_READ;
  copi_read_datagram.crc = calculateCrc(copi_read_datagram, COPI_READ_DATAGRAM_SIZE);
  blockingWrite(copi_read_datagram, COPI_READ_DATAGRAM_SIZE);
  CipoDatagram cipo_datagram;
  cipo_datagram.bytes = 0;
  cipo_datagram = blockingRead();
  return reverseData(cipo_datagram.data);
}

// private

template<typename Datagram>
void UartInterface::blockingWrite(Datagram & datagram,
  uint8_t datagram_size)
{
  enableTxDisableRx();

  delayMicroseconds(ENABLE_DELAY_MICROSECONDS);

  uint8_t write_byte;
  for (uint8_t i=0; i<datagram_size; ++i)
  {
    write_byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    serialWrite(write_byte);
  }
  serialFlush();

  disableTxEnableRx();
}

UartInterface::CipoDatagram UartInterface::blockingRead()
{
  CipoDatagram cipo_datagram;
  cipo_datagram.bytes = 0;

  uint64_t read_byte;


  // clear the serial receive buffer if necessary
  while (serialAvailable() > 0)
  {
    read_byte = serialRead();
  }

  uint32_t reply_delay = 0;
  while ((serialAvailable() < CIPO_DATAGRAM_SIZE) and
    (reply_delay < REPLY_DELAY_MAX_MICROSECONDS))
  {
    delayMicroseconds(REPLY_DELAY_INC_MICROSECONDS);
    reply_delay += REPLY_DELAY_INC_MICROSECONDS;
  }

  if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS)
  {
    Serial.println("Read timeout!");
    return cipo_datagram;
  }

  for (uint8_t i=0; i<CIPO_DATAGRAM_SIZE; ++i)
  {
    read_byte = serialRead();
    cipo_datagram.bytes |= (read_byte << (i * BITS_PER_BYTE));
  }

  return cipo_datagram;
}

int UartInterface::serialAvailable()
{
  if (uart_parameters_.uart_ptr != nullptr)
  {
    return uart_parameters_.uart_ptr->available();
  }
  return 0;
}

size_t UartInterface::serialWrite(uint8_t c)
{
  if (uart_parameters_.uart_ptr != nullptr)
  {
    return uart_parameters_.uart_ptr->write(c);
  }
  return 0;
}

int UartInterface::serialRead()
{
  if (uart_parameters_.uart_ptr != nullptr)
  {
    return uart_parameters_.uart_ptr->read();
  }
  return 0;
}

void UartInterface::serialFlush()
{
  if (uart_parameters_.uart_ptr != nullptr)
  {
    return uart_parameters_.uart_ptr->flush();
  }
}

uint32_t UartInterface::reverseData(uint32_t data)
{
  uint32_t reversed_data = 0;
  uint8_t right_shift;
  uint8_t left_shift;
  for (uint8_t i=0; i<DATA_SIZE; ++i)
  {
    right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    left_shift = i * BITS_PER_BYTE;
    reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
  }
  return reversed_data;
}

template<typename Datagram>
uint8_t UartInterface::calculateCrc(Datagram & datagram,
  uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t current_byte;
  for (uint8_t i=0; i<(datagram_size - 1); ++i)
  {
    current_byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j=0; j<BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (current_byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      current_byte = current_byte >> 1;
    }
  }
  return crc;
}

#if defined(ARDUINO_ARCH_ESP32)
void digitalWriteFast(uint8_t pin, uint8_t val)
{
  val ? GPIO.out_w1ts = (1 << pin) : GPIO.out_w1tc = (1 << pin);
}
#elif defined(ARDUINO_ARCH_MBED)
void digitalWriteFast(uint8_t pin, uint8_t val)
{
  digitalWrite(pin, val);
}
#endif

void UartInterface::enableTxDisableRx()
{
  digitalWriteFast(uart_parameters_.enable_txrx_pin, ENABLE_TX_DISABLE_RX_PIN_VALUE);
}

void UartInterface::disableTxEnableRx()
{
  digitalWriteFast(uart_parameters_.enable_txrx_pin, DISABLE_TX_ENABLE_RX_PIN_VALUE);
}
