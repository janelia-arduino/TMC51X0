#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

// Remove after testing
const uint8_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_VALUE = HIGH;
const uint8_t MUX_ADDRESS_0_PIN = 6;
const uint8_t MUX_ADDRESS_0_VALUE = LOW;
const uint8_t MUX_ADDRESS_1_PIN = 3;
const uint8_t MUX_ADDRESS_1_VALUE = LOW;
const uint8_t MUX_ADDRESS_2_PIN = 2;
const uint8_t MUX_ADDRESS_2_VALUE = LOW;

// UART Parameters
const uint32_t UART_BAUD_RATE = 115200;
const uint8_t NODE_ADDRESS = 0;
const pin_size_t ENABLE_TXRX_PIN = 14;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 tmc5160;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  // Remove after testing
  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, LOW);
  delay(5000);
  digitalWrite(ENABLE_POWER_PIN, HIGH);
  delay(5000);

  pinMode(MUX_ADDRESS_0_PIN, OUTPUT);
  digitalWrite(MUX_ADDRESS_0_PIN, MUX_ADDRESS_0_VALUE);
  pinMode(MUX_ADDRESS_1_PIN, OUTPUT);
  digitalWrite(MUX_ADDRESS_1_PIN, MUX_ADDRESS_1_VALUE);
  pinMode(MUX_ADDRESS_2_PIN, OUTPUT);
  digitalWrite(MUX_ADDRESS_2_PIN, MUX_ADDRESS_2_VALUE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc51x0::UartParameters uart_parameters(uart,
    NODE_ADDRESS,
    ENABLE_TXRX_PIN);
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
