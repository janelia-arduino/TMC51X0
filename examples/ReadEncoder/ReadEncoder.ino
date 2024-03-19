#include <TMC51X0.hpp>


#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;
#else
SPIClass & spi = SPI;
#endif

const uint8_t CHIP_SELECT_PIN = 10;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 100;

// Instantiate TMC51X0
TMC51X0 stepper;
uint32_t register_data;
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
  stepper.setup(spi, CHIP_SELECT_PIN);
}

void loop()
{
  encoder_actual_position = stepper.encoder.readActualPosition();
  Serial.print("encoder_actual_position: ");
  Serial.println(encoder_actual_position);
  encoder_status = stepper.encoder.readAndClearStatus();
  Serial.print("encoder_status.n_event: ");
  Serial.print(encoder_status.n_event);
  Serial.print(" , encoder_status.deviation_warn: ");
  Serial.println(encoder_status.deviation_warn);
  delay(DELAY);
}
