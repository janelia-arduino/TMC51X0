#include <TMC51X0.hpp>

const uint8_t CHIP_SELECT_PIN = 10;
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 250;

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
  Serial.print(version, HEX);
  Serial.println();
  delay(DELAY);
}
