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

  uart_parameters_.uart_ptr->begin();
}

void UartInterface::writeRegister(uint8_t register_address,
  uint32_t data)
{
}

uint32_t UartInterface::readRegister(uint8_t register_address)
{
  return 0;
}

// private

// UartInterface::CipoDatagram UartInterface::writeRead(CopiDatagram copi_datagram)
// {
//   uint8_t byte_write, byte_read;
//   CipoDatagram cipo_datagram;
//   cipo_datagram.bytes = 0x0;
//   beginTransaction();
//   for (int i=(UART_DATAGRAM_SIZE - 1); i>=0; --i)
//   {
//     byte_write = (copi_datagram.bytes >> (8*i)) & 0xff;
//     byte_read = uart_ptr_->transfer(byte_write);
//     cipo_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
//   }
//   endTransaction();
//   noInterrupts();
//   uart_status_ = cipo_datagram.uart_status;
//   interrupts();
//   return cipo_datagram;
// }

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

void UartInterface::beginTransaction()
{
  uart_ptr_->beginTransaction(uart_settings_);
  enableChipSelect();
}

void UartInterface::endTransaction()
{
  disableChipSelect();
  uart_ptr_->endTransaction();
}
