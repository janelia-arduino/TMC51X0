#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
size_t SCK_PIN = 18;
size_t TX_PIN = 19;
size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const tmc51x0::SpiParameters spi_parameters =
{
  .spi_ptr = &spi,
  .chip_select_pin = 8
};

const tmc51x0::SwitchParameters switch_parameters =
{
  .left_stop_enabled = false,
  .right_stop_enabled = false,
  .invert_left_polarity = false,
  .invert_right_polarity = false,
  .swap_left_right = false,
  .latch_left_active = false,
  .latch_left_inactive = false,
  .latch_right_active = false,
  .latch_right_inactive = false,
  .latch_encoder_enabled = false
};

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 1000;

// global variables
TMC51X0 tmc5130;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5130.setupSpi(spi_parameters);

  tmc5130.controller.setupSwitches(switch_parameters);
}

void loop()
{
  tmc5130.printer.readAndPrintRampStat();

  bool left_switch_active = tmc5130.controller.leftSwitchActive();
  Serial.print("left_switch_active = ");
  Serial.println(left_switch_active);

  bool right_switch_active = tmc5130.controller.rightSwitchActive();
  Serial.print("right_switch_active = ");
  Serial.println(right_switch_active);

  bool left_latch_active = tmc5130.controller.leftLatchActive();
  Serial.print("left_latch_active = ");
  Serial.println(left_latch_active);

  bool right_latch_active = tmc5130.controller.rightLatchActive();
  Serial.print("right_latch_active = ");
  Serial.println(right_latch_active);

  bool left_stop_event = tmc5130.controller.leftStopEvent();
  Serial.print("left_stop_event = ");
  Serial.println(left_stop_event);

  bool right_stop_event = tmc5130.controller.rightStopEvent();
  Serial.print("right_stop_event = ");
  Serial.println(right_stop_event);

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
