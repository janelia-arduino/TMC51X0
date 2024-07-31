// ----------------------------------------------------------------------------
// UartInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_INTERFACE_UART_HPP
#define TMC51X0_INTERFACE_UART_HPP
#include <Arduino.h>

#include "Interface.hpp"
#include "Constants.hpp"


namespace tmc51x0
{
struct UartParameters
{
  HardwareSerial * hs_uart_ptr;
  uint32_t baud_rate;
  uint8_t node_address;
  int8_t enable_tx_pin;
  int8_t enable_rx_pin;
  uint8_t enable_tx_polarity;
  uint8_t enable_rx_polarity;

  UartParameters(HardwareSerial & hs_uart_,
    uint32_t baud_rate_,
    uint8_t node_address_,
    size_t enable_tx_pin_,
    size_t enable_rx_pin_,
    size_t enable_tx_polarity_,
    size_t enable_rx_polarity_)
  {
    hs_uart_ptr = &hs_uart_;
    baud_rate = baud_rate_;
    node_address = node_address_;
    enable_tx_pin = enable_tx_pin_;
    enable_rx_pin = enable_rx_pin_;
    enable_tx_polarity = enable_tx_polarity_;
    enable_rx_polarity = enable_rx_polarity_;
  };

  UartParameters()
  {
    hs_uart_ptr = nullptr;
    baud_rate = BAUD_RATE_DEFAULT;
    node_address = NODE_ADDRESS_DEFAULT;
    enable_tx_pin = PIN_DEFAULT;
    enable_rx_pin = PIN_DEFAULT;
    enable_tx_polarity = ENABLE_TX_POLARITY_DEFAULT;
    enable_rx_polarity = ENABLE_RX_POLARITY_DEFAULT;
  };

  bool operator==(const UartParameters & rhs) const
  {
    if ((this->hs_uart_ptr == rhs.hs_uart_ptr) &&
      (this->baud_rate == rhs.baud_rate) &&
      (this->node_address == rhs.node_address) &&
      (this->enable_tx_pin == rhs.enable_tx_pin) &&
      (this->enable_rx_pin == rhs.enable_rx_pin) &&
      (this->enable_tx_polarity == rhs.enable_tx_polarity) &&
      (this->enable_rx_polarity == rhs.enable_rx_polarity))
    {
      return true;
    }
    return false;
  }
  bool operator!=(const UartParameters & rhs) const
  {
    return !(*this == rhs);
  }

private:
  const static uint32_t BAUD_RATE_DEFAULT = 115200;
  const static uint8_t NODE_ADDRESS_DEFAULT = 0;
  const static size_t PIN_DEFAULT = 255;
  const static uint8_t ENABLE_TX_POLARITY_DEFAULT = HIGH;
  const static uint8_t ENABLE_RX_POLARITY_DEFAULT = LOW;

};

class UartInterface : public Interface
{
public:
  UartInterface();

  void setup(UartParameters uart_parameters);

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  UartParameters uart_parameters_;

  const static uint8_t WRITE_PICO_DATAGRAM_SIZE = 8;
  const static uint8_t READ_PICO_DATAGRAM_SIZE = 4;

  // PICO Datagrams
  union PicoWriteDatagram
  {
    struct
    {
      uint64_t sync : 4;
      uint64_t reserved : 4;
      uint64_t node_address : 8;
      uint64_t register_address : 7;
      uint64_t rw : 1;
      uint64_t data : 32;
      uint64_t crc : 8;
    };
    uint64_t bytes;
  };
  union PicoReadDatagram
  {
    struct
    {
      uint32_t sync : 4;
      uint32_t reserved : 4;
      uint32_t node_address : 8;
      uint32_t register_address : 7;
      uint32_t rw : 1;
      uint32_t crc : 8;
    };
    uint32_t bytes;
  };
  const static uint8_t RW_READ = 0;
  const static uint8_t RW_WRITE = 1;

  const static uint8_t POCI_DATAGRAM_SIZE = 8;
  // // POCI Datagrams
  // union PociDatagram
  // {
  //   struct
  //   {
  //     uint64_t data : 32;
  //     UartStatus uart_status;
  //     uint64_t reserved : 24;
  //   };
  //   uint64_t bytes;
  // };

  // uint8_t uart_buffer_[UART_DATAGRAM_SIZE];

  // PociDatagram writeRead(PicoDatagram pico_datagram);

  // void enableChipSelect();
  // void disableChipSelect();
  // void beginTransaction();
  // void endTransaction();
};
}
#endif
