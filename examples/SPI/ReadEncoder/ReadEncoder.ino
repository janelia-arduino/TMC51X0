#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
uint16_t SCK_PIN = 18;
uint16_t TX_PIN = 19;
uint16_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

// SPI Parameters
const tmc51x0::SpiParameters spi_parameters(
  spi,
  1000000, // clock_rate
  10); // chip_select_pin

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 100;

const tmc51x0::Encoder::FractionalMode FRACTIONAL_MODE = tmc51x0::Encoder::BINARY;
// 256 encoder single signal pulses per revolution
// 256*4 = 1024 quadrature encoder pulses per revolution
// 51200 microsteps per revolution
// 51200/1024 = 50.0 microsteps per encoder pulse
const int16_t MICROSTEPS_PER_PULSE_INTEGER = 50;
const int16_t MICROSTEPS_PER_PULSE_FRACTIONAL = 0;

const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5160;
int32_t encoder_actual_position;
tmc51x0::Registers::EncStatus encoder_status;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  spi.begin();
  tmc5160.setupSpi(spi_parameters);

  tmc5160.encoder.writeFractionalMode(FRACTIONAL_MODE);
  tmc5160.encoder.writeMicrostepsPerPulse(MICROSTEPS_PER_PULSE_INTEGER, MICROSTEPS_PER_PULSE_FRACTIONAL);
  // tmc5160.encoder.writeActualPosition(tmc5160.converter.positionRealToEncoder(INITIAL_POSITION));
}

void loop()
{
  encoder_actual_position = tmc5160.encoder.readActualPosition();
  Serial.print("encoder_actual_position: ");
  Serial.println(encoder_actual_position);
  encoder_status = tmc5160.encoder.readAndClearStatus();
  Serial.print("encoder_status.n_event: ");
  Serial.print(encoder_status.n_event);
  Serial.print(" , encoder_status.deviation_warn: ");
  Serial.println(encoder_status.deviation_warn);
  delay(DELAY);
}
