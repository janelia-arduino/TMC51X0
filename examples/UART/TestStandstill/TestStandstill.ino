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
const uint8_t NODE_ADDRESS = 1;
const uint8_t ENABLE_TX_POLARITY = HIGH;
const uint8_t ENABLE_RX_POLARITY = LOW;

const uint8_t ENABLE_TX_PIN = 15;
const uint8_t ENABLE_RX_PIN = 14;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 4000;

// driver constants
const uint8_t HOLD_CURRENT = 0;
const uint8_t HOLD_DELAY = 0;

// Instantiate TMC51X0
TMC51X0 tmc5130;

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

  tmc51x0::UartParameters uart_parameters(uart,
    NODE_ADDRESS,
    ENABLE_TX_PIN,
    ENABLE_RX_PIN,
    ENABLE_TX_POLARITY,
    ENABLE_RX_POLARITY);
  tmc5130.setupUart(uart_parameters);

  tmc5130.driver.enableStealthChop();
  tmc5130.driver.writeHoldCurrent(HOLD_CURRENT);
  tmc5130.driver.writeHoldDelay(HOLD_DELAY);
  tmc5130.driver.enable();

  delay(DELAY);
}

void loop()
{
  Serial.println("standstill mode = NORMAL");
  tmc5130.driver.writeStandstillMode(tmc51x0::Driver::NORMAL);
  delay(DELAY);

  Serial.println("standstill mode = FREEWHEELING");
  tmc5130.driver.writeStandstillMode(tmc51x0::Driver::FREEWHEELING);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_LS");
  tmc5130.driver.writeStandstillMode(tmc51x0::Driver::PASSIVE_BRAKING_LS);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_HS");
  tmc5130.driver.writeStandstillMode(tmc51x0::Driver::PASSIVE_BRAKING_HS);
  delay(DELAY);

  Serial.println("--------------------------");
}