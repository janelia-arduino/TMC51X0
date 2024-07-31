// ----------------------------------------------------------------------------
// UARTInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "UARTInterface.hpp"


using namespace tmc51x0;

UARTInterface::UARTInterface()
{
  // enable_tx_pin_ = -1;
  // enable_rx_pin_ = -1;
}

void UARTInterface::setup(HardwareSerial & uart,
  size_t enable_tx_pin,
  size_t enable_rx_pin,
  size_t enable_tx_polarity,
  size_t enable_rx_polarity)
{
}

void UARTInterface::writeRegister(uint8_t register_address,
  uint32_t data)
{
}

uint32_t UARTInterface::readRegister(uint8_t register_address)
{
  return 0;
}

// private

// UARTInterface::PociDatagram UARTInterface::writeRead(PicoDatagram pico_datagram)
// {
//   uint8_t byte_write, byte_read;
//   PociDatagram poci_datagram;
//   poci_datagram.bytes = 0x0;
//   beginTransaction();
//   for (int i=(UART_DATAGRAM_SIZE - 1); i>=0; --i)
//   {
//     byte_write = (pico_datagram.bytes >> (8*i)) & 0xff;
//     byte_read = uart_ptr_->transfer(byte_write);
//     poci_datagram.bytes |= ((uint32_t)byte_read) << (8*i);
//   }
//   endTransaction();
//   noInterrupts();
//   uart_status_ = poci_datagram.uart_status;
//   interrupts();
//   return poci_datagram;
// }

// void UARTInterface::enableChipSelect()
// {
//   digitalWrite(chip_select_pin_, LOW);
// }

// void UARTInterface::disableChipSelect()
// {
//   digitalWrite(chip_select_pin_, HIGH);
// }

// void UARTInterface::beginTransaction()
// {
//   uart_ptr_->beginTransaction(uart_settings_);
//   enableChipSelect();
// }

// void UARTInterface::endTransaction()
// {
//   disableChipSelect();
//   uart_ptr_->endTransaction();
// }
