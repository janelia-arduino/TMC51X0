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
TMC51X0 stepper;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  stepper.setupUart(uart_parameters);

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(DELAY);
  }

  stepper.driver.enableStealthChop();
  stepper.driver.writeHoldCurrent(HOLD_CURRENT);
  stepper.driver.writeHoldDelay(HOLD_DELAY);
  stepper.driver.enable();

  delay(DELAY);
}

void loop()
{
  Serial.println("standstill mode = NormalMode");
  stepper.driver.writeStandstillMode(tmc51x0::NormalMode);
  delay(DELAY);

  Serial.println("standstill mode = FreewheelingMode");
  stepper.driver.writeStandstillMode(tmc51x0::FreewheelingMode);
  delay(DELAY);

  Serial.println("standstill mode = PassiveBrakingLsMode");
  stepper.driver.writeStandstillMode(tmc51x0::PassiveBrakingLsMode);
  delay(DELAY);

  Serial.println("standstill mode = PassiveBrakingHsMode");
  stepper.driver.writeStandstillMode(tmc51x0::PassiveBrakingHsMode);
  delay(DELAY);

  Serial.println("--------------------------");
}
