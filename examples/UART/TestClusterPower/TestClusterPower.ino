#include <TMC51X0.hpp>


const uint8_t ENABLE_VCC_PIN = 22;
const uint8_t ENABLE_VCC_POLARITY = HIGH;

const uint16_t DELAY = 5000;
uint8_t enable_state;

void setup()
{
  pinMode(ENABLE_VCC_PIN, OUTPUT);
  enable_state = ENABLE_VCC_POLARITY;
  digitalWrite(ENABLE_VCC_PIN, enable_state);
}

void loop()
{
  delay(DELAY);
  enable_state = !enable_state;
  digitalWrite(ENABLE_VCC_PIN, enable_state);
}
