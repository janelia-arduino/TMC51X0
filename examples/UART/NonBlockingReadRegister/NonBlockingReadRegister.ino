#include <TMC51X0.hpp>

#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial &uart = Serial2;
#else
SerialUART &uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const auto uart_parameters = tmc51x0::UartParameters{}
                                 .withUart(&uart)
                                 .withEnableTxRxPin(14)
                                 .withBaudRate(115200)
                                 .withReplyTimeoutUs(2000)
                                 .withMaxRetries(1)
                                 .withDrainLimit(256);

const uint32_t UART_BAUD_RATE = 115200;
const uint32_t SERIAL_BAUD_RATE = 115200;
const uint32_t READ_INTERVAL_MS = 2000;

TMC51X0 stepper;

bool read_in_flight = false;
uint32_t next_read_ms = 0;

const char *uartErrorName(tmc51x0::UartError error) {
  switch (error) {
  case tmc51x0::UartError::None:
    return "None";
  case tmc51x0::UartError::Busy:
    return "Busy";
  case tmc51x0::UartError::NotInitialized:
    return "NotInitialized";
  case tmc51x0::UartError::ReplyTimeout:
    return "ReplyTimeout";
  case tmc51x0::UartError::CrcMismatch:
    return "CrcMismatch";
  case tmc51x0::UartError::UnexpectedFrame:
    return "UnexpectedFrame";
  case tmc51x0::UartError::RxGarbage:
    return "RxGarbage";
  }
  return "Unknown";
}

void startReadIfDue() {
  if (read_in_flight || (millis() < next_read_ms)) {
    return;
  }

  auto &bus = stepper.uartBus();
  auto start = bus.startRead(tmc51x0::Registers::GconfAddress);
  if (!start.ok()) {
    Serial.print("startRead failed: ");
    Serial.println(uartErrorName(start.error));
    next_read_ms = millis() + READ_INTERVAL_MS;
    return;
  }

  read_in_flight = true;
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif

  uart.begin(UART_BAUD_RATE);
  stepper.setupUart(uart_parameters);

  next_read_ms = millis();
}

void loop() {
  auto &bus = stepper.uartBus();
  bus.poll();

  startReadIfDue();

  if (!read_in_flight || !bus.done()) {
    return;
  }

  auto result = bus.takeReadResult();
  read_in_flight = false;
  next_read_ms = millis() + READ_INTERVAL_MS;

  if (!result.ok()) {
    Serial.print("UART read failed: ");
    Serial.println(uartErrorName(result.error));
    return;
  }

  Serial.print("GCONF = 0x");
  Serial.println(result.value, HEX);
  Serial.println("--------------------------");
}
