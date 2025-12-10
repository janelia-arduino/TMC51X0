#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const auto spi_parameters =
  tmc51x0::SpiParameters{}
    .withSpi(&spi)
    .withChipSelectPin(8);

const auto switch_parameters =
  tmc51x0::SwitchParameters{}
    .withLeftStopEnabled(false)
    .withRightStopEnabled(false)
    .withInvertLeftPolarity(false)
    .withInvertRightPolarity(false)
    .withSwapLeftRight(false)
    .withLatchLeftActive(false)
    .withLatchLeftInactive(false)
    .withLatchRightActive(false)
    .withLatchRightInactive(false)
    .withLatchEncoderEnabled(false);

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 stepper;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  stepper.setupSpi(spi_parameters);

  stepper.controller.setupSwitches(switch_parameters);
}

void loop()
{
  stepper.printer.readAndPrintRampStat();

  bool left_switch_active = stepper.controller.leftSwitchActive();
  Serial.print("left_switch_active = ");
  Serial.println(left_switch_active);

  bool right_switch_active = stepper.controller.rightSwitchActive();
  Serial.print("right_switch_active = ");
  Serial.println(right_switch_active);

  bool left_latch_active = stepper.controller.leftLatchActive();
  Serial.print("left_latch_active = ");
  Serial.println(left_latch_active);

  bool right_latch_active = stepper.controller.rightLatchActive();
  Serial.print("right_latch_active = ");
  Serial.println(right_latch_active);

  bool left_stop_event = stepper.controller.leftStopEvent();
  Serial.print("left_stop_event = ");
  Serial.println(left_stop_event);

  bool right_stop_event = stepper.controller.rightStopEvent();
  Serial.print("right_stop_event = ");
  Serial.println(right_stop_event);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
