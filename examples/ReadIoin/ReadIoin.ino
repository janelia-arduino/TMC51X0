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
const uint8_t HARDWARE_ENABLE_PIN = 4;

const long SERIAL_BAUD_RATE = 115200;
const int DELAY = 1000;

// Instantiate TMC51X0
TMC51X0 stepper;
bool enabled;

void printRegisterPortion(const char * str, uint32_t value, bool hex=false)
{
  Serial.print(str);
  if (not hex)
  {
    Serial.print(": 0b");
    Serial.print(value, BIN);
  }
  else
  {
    Serial.print(": 0x");
    Serial.print(value, HEX);
  }
  Serial.println();
}

void printRegister(uint32_t register_data)
{
  tmc51x0::Registers::Ioin ioin;
  ioin.bytes = register_data;
  printRegisterPortion("ioin", ioin.bytes, true);
  printRegisterPortion("refl_step", ioin.refl_step);
  printRegisterPortion("refr_dir", ioin.refr_dir);
  printRegisterPortion("encb_dcen_cfg4", ioin.encb_dcen_cfg4);
  printRegisterPortion("enca_dcin_cfg5", ioin.enca_dcin_cfg5);
  printRegisterPortion("drv_enn", ioin.drv_enn);
  printRegisterPortion("enc_n_dco_cfg6", ioin.enc_n_dco_cfg6);
  printRegisterPortion("sd_mode", ioin.sd_mode);
  printRegisterPortion("swcomp_in", ioin.swcomp_in);
  printRegisterPortion("version", ioin.version, true);

  Serial.println("--------------------------");
}

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif
  stepper.setup(spi, CHIP_SELECT_PIN);
  stepper.driver.setHardwareEnablePin(HARDWARE_ENABLE_PIN);
  enabled = false;
}

void loop()
{
  if (enabled)
  {
    stepper.driver.disable();
  }
  else
  {
    stepper.driver.enable();
  }
  enabled = not enabled;
  printRegister(stepper.registers.read(tmc51x0::Registers::IOIN));
  delay(DELAY);
}
