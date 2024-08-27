#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
pin_size_t TX_PIN = 4;
pin_size_t RX_PIN = 5;
#endif

const uint8_t ENABLE_VCC_PIN = 22;
const uint8_t ENABLE_VCC_POLARITY = HIGH;

// ENABLE_TX_PIN and ENABLE_RX_PIN may be the same pin
const uint32_t UART_BAUD_RATE = 115200;
const uint8_t NODE_ADDRESS = 0;
const uint8_t ENABLE_TX_POLARITY = HIGH;
const uint8_t ENABLE_RX_POLARITY = LOW;

const uint8_t PRISM_COUNT = 7;
const uint8_t ENABLE_TX_PINS[PRISM_COUNT] = {15, 13, 11, 9, 7, 3, 1};
const uint8_t ENABLE_RX_PINS[PRISM_COUNT] = {14, 12, 10, 8, 6, 2, 0};

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 2000;

// Instantiate TMC51X0
TMC51X0 prisms[PRISM_COUNT];
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  pinMode(ENABLE_VCC_PIN, OUTPUT);
  digitalWrite(ENABLE_VCC_PIN, ENABLE_VCC_POLARITY);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  for (size_t i=0; i<PRISM_COUNT; ++i)
  {
    tmc51x0::UartParameters uart_parameters(uart,
      NODE_ADDRESS,
      ENABLE_TX_PINS[i],
      ENABLE_RX_PINS[i],
      ENABLE_TX_POLARITY,
      ENABLE_RX_POLARITY);
    prisms[i].setupUart(uart_parameters);
  }

  enabled = false;
}

void loop()
{
  // for (size_t i=0; i<PRISM_COUNT; ++i)
  for (size_t i=0; i<1; ++i)
  {
    Serial.print("prism ");
    Serial.print(i);
    Serial.println(":");
    TMC51X0 & prism = prisms[i];
    // if (enabled)
    // {
    //   prism.driver.disable();
    // }
    // else
    // {
    //   prism.driver.enable();
    // }
    prism.printer.readAndPrintIoin();
    delay(DELAY);
  }
  Serial.println("--------------------------");

  enabled = not enabled;

}
