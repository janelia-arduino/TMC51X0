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
  spi,
  1000000, // clock_rate
  14 // chip_select_pin
};

const tmc51x0::EncoderParameters encoder_parameters =
{
  tmc51x0::BINARY, // fractional_mode
  64, // microsteps_per_pulse_integer
  0 // microsteps_per_pulse_fractional
};
// 200 encoder single signal pulses per revolution
// 200*4 = 800 quadrature encoder pulses per revolution
// 200 motor fullsteps per revolution
// 256 microsteps per fullstep
// 200*256 = 51200 microsteps per revolution
// 51200/800 = 64.0 microsteps per encoder pulse

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 500;

// Instantiate TMC51X0
TMC51X0 tmc5130;
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
  tmc5130.setupSpi(spi_parameters);

  tmc5130.encoder.setup(encoder_parameters);
}

void loop()
{
  encoder_actual_position = tmc5130.encoder.readActualPosition();
  Serial.print("encoder_actual_position: ");
  Serial.println(encoder_actual_position);
  encoder_status = tmc5130.encoder.readAndClearStatus();
  Serial.print("encoder_status.n_event: ");
  Serial.print(encoder_status.n_event);
  Serial.print(" , encoder_status.deviation_warn (only works on TMC5160): ");
  Serial.println(encoder_status.deviation_warn);
  delay(DELAY);
}
