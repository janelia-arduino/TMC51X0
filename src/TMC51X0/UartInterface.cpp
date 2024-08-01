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
  uart_parameters_ = uart_parameters;

  pinMode(uart_parameters_.enable_tx_pin, OUTPUT);
  disableTx();

  pinMode(uart_parameters_.enable_rx_pin, OUTPUT);
  disableRx();

  uart_parameters_.uart_ptr->end();
  uart_parameters_.uart_ptr->begin(uart_parameters_.baud_rate);
}

void UartInterface::writeRegister(uint8_t register_address,
  uint32_t data)
{
  CopiWriteDatagram copi_write_datagram;
  copy_write_datagram.bytes = 0;
  copy_write_datagram.sync = SYNC;
  copi_write_datagram.node_address = uart_parameters_.node_address;
  copi_write_datagram.register_address = register_address;
  copi_write_datagram.rw = RW_WRITE;
  copi_write_datagram.data = reverseData(data);
  copi_write_datagram.crc = calculateCrc(copi_write_datagram, COPI_WRITE_DATAGRAM_SIZE);
  write(copi_write_datagram);
}

uint32_t UartInterface::readRegister(uint8_t register_address)
{
  CopiReadDatagram copi_read_datagram;
  copi_write_datagram.node_address = uart_parameters_.node_address;
  copi_datagram.register_address = register_address;
  copi_datagram.rw = RW_READ;
  CipoDatagram cipo_datagram = writeRead(copi_datagram);
  // cipo data is returned on second read
  cipo_datagram = writeRead(copi_datagram);
  return cipo_datagram.data;
}

// private

void UartInterface::write(CopiWriteDatagram copi_write_datagram)
{
  uint8_t write_byte;
  enableTx();
  for (uint8_t i=0; i<COPI_WRITE_DATAGRAM_SIZE; ++i)
  {
    write_byte = (copi_write_datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    serialWrite(write_byte);
  }
  disableTx();
}

// UartInterface::CipoDatagram UartInterface::writeRead(CopiDatagram copi_datagram)
// {
//   uint8_t write_byte, read_byte;
//   CipoDatagram cipo_datagram;
//   cipo_datagram.bytes = 0x0;
//   beginTransaction();
//   for (int i=(UART_DATAGRAM_SIZE - 1); i>=0; --i)
//   {
//     write_byte = (copi_datagram.bytes >> (8*i)) & 0xff;
//     read_byte = uart_ptr_->transfer(write_byte);
//     cipo_datagram.bytes |= ((uint32_t)read_byte) << (8*i);
//   }
//   endTransaction();
//   noInterrupts();
//   uart_status_ = cipo_datagram.uart_status;
//   interrupts();
//   return cipo_datagram;
// }

int UartInterface::serialAvailable()
{
  if (uart_parameters_.hs_uart_ptr != nullptr)
  {
    return uart_parameters_.hs_uart_ptr->available();
  }
// #if SOFTWARE_SERIAL_INCLUDED
//   else if (software_serial_ptr_ != nullptr)
//   {
//     return software_serial_ptr_->available();
//   }
// #endif
  return 0;
}

size_t UartInterface::serialWrite(uint8_t c)
{
  if (uart_parameters_.hs_uart_ptr != nullptr)
  {
    return uart_parameters_.hs_uart_ptr->write(c);
  }
// #if SOFTWARE_SERIAL_INCLUDED
//   else if (software_serial_ptr_ != nullptr)
//   {
//     return software_serial_ptr_->write(c);
//   }
// #endif
  return 0;
}

int UartInterface::serialRead()
{
  if (uart_parameters_.hs_uart_ptr != nullptr)
  {
    return uart_parameters_.hs_uart_ptr->read();
  }
// #if SOFTWARE_SERIAL_INCLUDED
//   else if (software_serial_ptr_ != nullptr)
//   {
//     return software_serial_ptr_->read();
//   }
// #endif
  return 0;
}

void UartInterface::serialFlush()
{
  if (uart_parameters_.hs_uart_ptr != nullptr)
  {
    return uart_parameters_.hs_uart_ptr->flush();
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

void UartInterface::enableTx()
{
  digitalWrite(uart_parameters_.enable_tx_pin, uart_parameters_.enable_tx_polarity);
}

void UartInterface::disableTx()
{
  digitalWrite(uart_parameters_.enable_tx_pin, !uart_parameters_.enable_tx_polarity);
}

void UartInterface::enableRx()
{
  digitalWrite(uart_parameters_.enable_rx_pin, uart_parameters_.enable_rx_polarity);
}

void UartInterface::disableRx()
{
  digitalWrite(uart_parameters_.enable_rx_pin, !uart_parameters_.enable_rx_polarity);
}
