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

const tmc51x0::EncoderParameters encoder_parameters =
{
  .fractional_mode = tmc51x0::BinaryMode,
  .microsteps_per_pulse_integer = 64,
  .microsteps_per_pulse_fractional = 0
};
// 200 encoder single signal pulses per revolution
// 200*4 = 800 quadrature encoder pulses per revolution
// 200 motor fullsteps per revolution
// 256 microsteps per fullstep
// 200*256 = 51200 microsteps per revolution
// 51200/800 = 64.0 microsteps per encoder pulse

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 500;

// global variables
TMC51X0 stepper;
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
  stepper.setupSpi(spi_parameters);

  stepper.encoder.setup(encoder_parameters);

  while (!stepper.communicating())
  {
    Serial.println("No communication detected, check motor power and connections.");
    delay(LOOP_DELAY);
  }
}

void loop()
{
  encoder_actual_position = stepper.encoder.readActualPosition();
  Serial.print("encoder_actual_position: ");
  Serial.println(encoder_actual_position);
  encoder_status = stepper.encoder.readAndClearStatus();
  Serial.print("encoder_status.n_event: ");
  Serial.print(encoder_status.n_event);
  Serial.print(" , encoder_status.deviation_warn (only works on TMC5160): ");
  Serial.println(encoder_status.deviation_warn);
  delay(LOOP_DELAY);
}
