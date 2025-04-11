#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const tmc51x0::UartParameters uart_parameters =
{
  .uart_ptr = &uart,
  .enable_txrx_pin = 14
};
const uint32_t UART_BAUD_RATE = 115200;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 4000;

// driver constants
const uint8_t HOLD_CURRENT = 0;
const uint8_t HOLD_DELAY = 0;

// global variables
TMC51X0 tmc5130;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc5130.setupUart(uart_parameters);

  while (!tmc5130.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(DELAY);
  }

  tmc5130.driver.enableStealthChop();
  tmc5130.driver.writeHoldCurrent(HOLD_CURRENT);
  tmc5130.driver.writeHoldDelay(HOLD_DELAY);
  tmc5130.driver.enable();

  delay(DELAY);
}

void loop()
{
  Serial.println("standstill mode = NORMAL");
  tmc5130.driver.writeStandstillMode(tmc51x0::NORMAL);
  delay(DELAY);

  Serial.println("standstill mode = FREEWHEELING");
  tmc5130.driver.writeStandstillMode(tmc51x0::FREEWHEELING);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_LS");
  tmc5130.driver.writeStandstillMode(tmc51x0::PASSIVE_BRAKING_LS);
  delay(DELAY);

  Serial.println("standstill mode = PASSIVE_BRAKING_HS");
  tmc5130.driver.writeStandstillMode(tmc51x0::PASSIVE_BRAKING_HS);
  delay(DELAY);

  Serial.println("--------------------------");
}
