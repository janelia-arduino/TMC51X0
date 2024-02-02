#include <TMC51X0.h>

// This example will not work on Arduino boards without HardwareSerial ports,
// such as the Uno, Nano, and Mini.
//
// See this reference for more details:
// https://www.arduino.cc/reference/en/language/functions/communication/serial/

const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 3000;

// Instantiate TMC51X0
TMC51X0 stepper_controller_driver;
uint8_t version;


void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  stepper_controller_driver.setup(CHIP_SELECT_PIN);

  stepper_controller_driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
}

void loop()
{
  version = stepper_controller_driver.getVersion();
  Serial.print("Stepper controller driver version: ");
  Serial.print(version, _HEX);
  Serial.println();
  delay(DELAY);
}
