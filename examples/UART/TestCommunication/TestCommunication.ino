#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const auto uart_parameters =
  tmc51x0::UartParameters{}
    .withUart(&uart)
    .withEnableTxRxPin(14);

const uint32_t UART_BAUD_RATE = 115200;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 stepper;
bool enabled;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  stepper.setupUart(uart_parameters);

  while (stepper.controller.stepAndDirectionMode())
  {
    Serial.println("Step and Direction mode enabled so SPI/UART motion commands will not work!");
    delay(LOOP_DELAY);
  }

  enabled = false;
}

void loop()
{
  if (enabled)
  {
    stepper.driver.disable();
  }
  else
  {
    stepper.driver.enable();
  }
  enabled = not enabled;

  stepper.printer.readAndPrintIoin();

  delay(LOOP_DELAY);
}
