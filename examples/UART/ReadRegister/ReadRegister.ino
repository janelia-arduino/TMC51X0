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
const uint16_t LOOP_DELAY = 2000;

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
    delay(LOOP_DELAY);
  }
}

void loop()
{
  tmc5130.printer.readAndPrintGconf();
  tmc5130.printer.readAndPrintRampStat();
  tmc5130.printer.readAndPrintChopconf();

  delay(LOOP_DELAY);
  Serial.println("--------------------------");
}
