// ----------------------------------------------------------------------------
// InterfaceUART.hpp
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
class InterfaceUART : public Interface
{
public:
  InterfaceUART();

  void setup(HardwareSerial & uart,
    size_t enable_tx_pin,
    size_t enable_rx_pin,
    size_t enable_tx_polarity=HIGH,
    size_t enable_rx_polarity=LOW);

  void writeRegister(uint8_t register_address,
    uint32_t data);
  uint32_t readRegister(uint8_t register_address);

private:
  // // UART
  // HardwareSerial * uart_ptr_;
  // uint32_t baud_rate_;
  // uint8_t address_;
  // int8_t enable_tx_pin_;
  // int8_t enable_rx_pin_;
  // uint8_t enable_tx_polarity_;
  // uint8_t enable_rx_polarity_;

  // struct UartStatus
  // {
  //   uint8_t reset_flag : 1;
  //   uint8_t driver_error : 1;
  //   uint8_t sg2 : 1;
  //   uint8_t standstill : 1;
  //   uint8_t velocity_reached : 1;
  //   uint8_t position_reached : 1;
  //   uint8_t status_stop_l : 1;
  //   uint8_t status_stop_r : 1;
  // };
  // UartStatus uart_status_;

  // const static uint8_t UART_DATAGRAM_SIZE = 5;

  // // MOSI Datagrams
  // union MosiDatagram
  // {
  //   struct
  //   {
  //     uint64_t data : 32;
  //     uint64_t register_address : 7;
  //     uint64_t rw : 1;
  //     uint64_t reserved : 24;
  //   };
  //   uint64_t bytes;
  // };
  // const static uint8_t UART_RW_READ = 0;
  // const static uint8_t UART_RW_WRITE = 1;

  // // MISO Datagrams
  // union MisoDatagram
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

  // MisoDatagram writeRead(MosiDatagram mosi_datagram);

  // void enableChipSelect();
  // void disableChipSelect();
  // void beginTransaction();
  // void endTransaction();
};
}
#endif
