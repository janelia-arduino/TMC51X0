#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

const uint32_t UART_BAUD_RATE = 115200;

const uint16_t DELAY = 2000;

void setup()
{
#if defined(ARDUINO_ARCH_RP2040)
  Serial2.setTX(TX_PIN);
  Serial2.setRX(RX_PIN);
#endif
  Serial2.begin(UART_BAUD_RATE);
}

void loop()
{
  Serial2.write(7);
  Serial2.write(7);
  Serial2.write(7);
  Serial2.write(7);
  delay(DELAY);
}
