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
const uint16_t LOOP_DELAY = 2000;

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
    delay(LOOP_DELAY);
  }
}

void loop()
{
  stepper.printer.readAndPrintGconf();
  stepper.printer.readAndPrintRampStat();
  stepper.printer.readAndPrintChopconf();

  delay(LOOP_DELAY);
  Serial.println("--------------------------");
}
