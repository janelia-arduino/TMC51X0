#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial1;
#else
SerialUART & uart = Serial1;
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

// ENABLE_TX_PIN and ENABLE_RX_PIN may be the same pin
const uint32_t UART_BAUD_RATE = 115200;
const uint8_t NODE_ADDRESS = 0;
const uint8_t ENABLE_TX_PIN = 15;
const uint8_t ENABLE_RX_PIN = 14;
const uint8_t ENABLE_TX_POLARITY = HIGH;
const uint8_t ENABLE_RX_POLARITY = LOW;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif

  tmc51x0::UartParameters uart_parameters(uart,
    UART_BAUD_RATE,
    NODE_ADDRESS,
    ENABLE_TX_PIN,
    ENABLE_RX_PIN,
    ENABLE_TX_POLARITY,
    ENABLE_RX_POLARITY);
  tmc5160.setupUart(uart_parameters);

  enabled = false;
}

void loop()
{
  if (enabled)
  {
    tmc5160.driver.disable();
  }
  else
  {
    tmc5160.driver.enable();
  }
  enabled = not enabled;

  tmc5160.printer.readAndPrintIoin();

  delay(DELAY);
}
