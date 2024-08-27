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
  Stream * stream_ptr;
  uint8_t node_address;
  int8_t enable_tx_pin;
  int8_t enable_rx_pin;
  uint8_t enable_tx_polarity;
  uint8_t enable_rx_polarity;

  UartParameters(Stream & uart_,
    uint8_t node_address_,
    size_t enable_tx_pin_,
    size_t enable_rx_pin_,
    size_t enable_tx_polarity_,
    size_t enable_rx_polarity_)
  {
    stream_ptr = &uart_;
    node_address = node_address_;
    enable_tx_pin = enable_tx_pin_;
    enable_rx_pin = enable_rx_pin_;
    enable_tx_polarity = enable_tx_polarity_;
    enable_rx_polarity = enable_rx_polarity_;
  };

  UartParameters()
  {
    stream_ptr = nullptr;
    node_address = NODE_ADDRESS_DEFAULT;
    enable_tx_pin = PIN_DEFAULT;
    enable_rx_pin = PIN_DEFAULT;
    enable_tx_polarity = ENABLE_TX_POLARITY_DEFAULT;
    enable_rx_polarity = ENABLE_RX_POLARITY_DEFAULT;
  };

  bool operator==(const UartParameters & rhs) const
  {
    if ((this->stream_ptr == rhs.stream_ptr) &&
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
  const static uint8_t NODE_ADDRESS_DEFAULT = 0;
  const static size_t PIN_DEFAULT = 255;
  const static uint8_t ENABLE_TX_POLARITY_DEFAULT = HIGH;
  const static uint8_t ENABLE_RX_POLARITY_DEFAULT = LOW;

};

class UartInterface : public Interface
{
public:
  void setup(UartParameters uart_parameters);

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  UartParameters uart_parameters_;

  const static uint8_t BYTE_MAX_VALUE = 0xFF;
  const static uint8_t BITS_PER_BYTE = 8;

  const static uint8_t SYNC = 0b101;
  const static uint8_t DATA_SIZE = 4;

  // Copi Datagrams
  const static uint8_t COPI_WRITE_DATAGRAM_SIZE = 8;
  union CopiWriteDatagram
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

  const static uint8_t COPI_READ_DATAGRAM_SIZE = 4;
  union CopiReadDatagram
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

  // Cipo Datagrams
  const static uint8_t CIPO_DATAGRAM_SIZE = 8;
  union CipoDatagram
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

  const static uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
  const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;

  template<typename Datagram>
  void blockingWrite(Datagram & datagram,
    uint8_t datagram_size);
  CipoDatagram blockingRead();

  int serialAvailable();
  size_t serialWrite(uint8_t c);
  int serialRead();
  void serialFlush();

  uint32_t reverseData(uint32_t data);
  template<typename Datagram>
  uint8_t calculateCrc(Datagram & datagram,
    uint8_t datagram_size);

  void enableTx();
  void disableTx();
  void enableRx();
  void disableRx();
};
}
#endif
