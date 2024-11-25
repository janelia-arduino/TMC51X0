// ----------------------------------------------------------------------------
// UartInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_INTERFACE_HPP
#define TMC51X0_UART_INTERFACE_HPP
#include <Arduino.h>

#include "UartParameters.hpp"
#include "Interface.hpp"


namespace tmc51x0
{
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

  const static uint8_t ENABLE_TX_DISABLE_RX_PIN_VALUE = HIGH;
  const static uint8_t DISABLE_TX_ENABLE_RX_PIN_VALUE = LOW;

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
  const static uint32_t ENABLE_DELAY_MICROSECONDS = 10;

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

  void enableTxDisableRx();
  void disableTxEnableRx();
};
}
#endif
