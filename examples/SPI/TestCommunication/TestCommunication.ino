/*
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             ((uint32_t)digitalPinToPinName(pin))
#define IO_REG_TYPE                     uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, pin)          digitalReadFast((PinName)pin)
#define DIRECT_WRITE_LOW(base, pin)     digitalWriteFast((PinName)pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWriteFast((PinName)pin, HIGH)
#define DIRECT_MODE_INPUT(base, pin)    pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0))
#define DIRECT_MODE_OUTPUT(base, pin)   pin_function((PinName)pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0))

#define pin_size_t                      IO_REG_TYPE
#define DIRECT_PIN_READ(base, pin)      digitalRead(pin)
*/

#if defined(ARDUINO_ARCH_STM32)
#include "../../../../src/TMC51X0.hpp"
#include "../../../../src/EQ6IO.hpp"

#elif defined(ESP32)
#include "../../../../src/TMC51X0.hpp"
#include "../../../../src/EQ6IO.hpp"

#else
#include "TMC51X0.hpp"
#endif

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 & spi = SPI;
pin_size_t SCK_PIN = 18;
pin_size_t TX_PIN = 19;
pin_size_t RX_PIN = 20;

#elif defined(ARDUINO_ARCH_STM32)
//             MOSI   MISO   SCLK   SSEL
//SPIClass SPI_1(PA_7,  PA_6,  PA_5,  PB_0);    // RA
//SPIClass SPI_2(PB_15, PB_14, PB_13, PB_12);   // DEC
SPIClass RA_SPI(RA_MOSI,  RA_MISO,  RA_SCK,  RA_CSN);    // RA
SPIClass DEC_SPI(DEC_MOSI, DEC_MISO, DEC_SCK, DEC_CSN);   // DEC
#define spi RA_SPI
#define spi2 DEC_SPI

#elif defined(ESP32)
#define RA_MOSI   SPI_MOSI
#define RA_MISO   SPI_MISO
#define RA_SCK    SPI_SCK
#define DEC_MOSI  SPI_MOSI
#define DEC_MISO  SPI_MISO
#define DEC_SCK   SPI_SCK

//SPIClass RA_SPI  = NULL;
//SPIClass DEC_SPI = NULL;

// ESP32 SPIClass
// 4 SPI peripherals: SPI0, SPI1, SPI2(HSPI), and SPI3(VSPI).
// SP0 and SP1 are used internally
// Default Pins
// ============
//  SPI	MOSI	  MISO	  SCLK	  CS
// VSPI	GPIO 23	GPIO 19	GPIO 18	GPIO 5
// HSPI	GPIO 13	GPIO 12	GPIO 14	GPIO 15

//uninitialized pointers to SPI objects
SPIClass spi(HSPI);
SPIClass spi2(VSPI); //= NULL;

#else
SPIClass& spi = SPI1;
#endif

// SPI Parameters
const uint32_t    SPI_CLOCK_RATE = 1000000;
const pin_size_t  RA_CHIP_SELECT_PIN = RA_CSN;
const pin_size_t  DEC_CHIP_SELECT_PIN = DEC_CSN;
const pin_size_t  ENABLE_HARDWARE_PIN = RADEC_EN;

#if defined(ARDUINO_ARCH_STM32)
//                       RX    TX
HardwareSerial  Serial1(PB7, PB6);
#endif

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 5000;

// converter constants
// internal clock is ~12MHz
const uint8_t CLOCK_FREQUENCY_MHZ = 12;
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49
// one "real unit" in this example is one millimeters of linear travel
constexpr uint32_t MICROSTEPS_PER_REAL_UNIT = 4881;

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 50; // percent
const uint8_t RUN_CURRENT = 20; // percent
const uint8_t PWM_OFFSET = 20; // percent
const uint8_t PWM_GRADIENT = 5; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::REVERSE;

const uint8_t STEALTH_CHOP_THRESHOLD = 100; // millimeters/s
const uint8_t COOL_STEP_THRESHOLD = 100; // millimeters/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 90; // millimeters/s
const int8_t STALL_GUARD_THRESHOLD = 1;

// controller constants
const uint32_t START_VELOCITY = 1; // millimeters/s
const uint32_t FIRST_ACCELERATION = 10;  // millimeters/(s^2)
const uint32_t FIRST_VELOCITY = 10; // millimeters/s
const uint32_t MAX_ACCELERATION = 2; // millimeters/(s^2)
const uint32_t MAX_DECELERATION = 25;  // millimeters/(s^2)
const uint32_t FIRST_DECELERATION = 20;  // millimeters/(s^2)
const uint32_t MAX_VELOCITY = 20; // millimeters/s
const uint32_t STOP_VELOCITY = 5; // millimeters/s

const int32_t MIN_TARGET_POSITION = 20;  // millimeters
// const int32_t MAX_TARGET_POSITION = 180;  // millimeters
const int32_t MAX_TARGET_POSITION = 600;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5160;

bool enabled;

#if defined(ARDUINO_ARCH_STM32)
#define LED_BLUE    DEC_DIAG0
#define LED_GREEN   DEC_DIAG1
#endif

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);
  Serial1.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_ARCH_STM32)  
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
#endif
//while(1){
  Serial.println("blue");
  Serial1.println("blue");
  #if defined(ARDUINO_ARCH_STM32)
  digitalWrite(LED_BLUE, HIGH);
  delay(1000);
  digitalWrite(LED_BLUE, LOW);
  delay(1000);
  #endif
  Serial.println("green");
  Serial1.println("green");
  
#if defined(ARDUINO_ARCH_STM32)
  digitalWrite(LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(LED_GREEN, LOW);
  delay(1000);
#endif

#if defined(ARDUINO_ARCH_ESP32)


//  SPIClass spi  = *RA_SPI;
//  SPIClass spi2 = *DEC_SPI;
//hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_SS);  //SCLK, MISO, MOSI, SS

  spi.begin(  RA_SCK,  RA_MISO,  RA_MOSI,  RA_CSN);
//  spi2.begin(DEC_SCK, DEC_MISO, DEC_MOSI, DEC_CSN);
  #endif
//}

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(TX_PIN);
  spi.setRX(RX_PIN);
#endif


  tmc51x0::SpiParameters spi_parameters(spi, SPI_CLOCK_RATE, RA_CHIP_SELECT_PIN);
  tmc5160.setupSpi(spi_parameters);

  tmc51x0::ConverterParameters converter_parameters =
    {
      CLOCK_FREQUENCY_MHZ,
      MICROSTEPS_PER_REAL_UNIT
    };
  tmc5160.converter.setup(converter_parameters);

  tmc5160.driver.setEnableHardwarePin(ENABLE_HARDWARE_PIN);
  tmc5160.driver.enable();
  enabled = true;
}

void loop()
{
/*
  if (enabled)
  {
    tmc5160.driver.disable();
  }
  else
  {
    tmc5160.driver.enable();
  }
  enabled = not enabled;
*/
  tmc51x0::Registers::IholdIrun iholdirun;
  iholdirun.bytes = 0x0006140C;
  tmc5160.printer.printRegister(iholdirun);

  tmc51x0::Registers::Pwmconf pwmconf;
  pwmconf.bytes = 0x0006140C;
  tmc5160.printer.printRegister(pwmconf);

  tmc5160.printer.readAndPrintIoin();
  tmc5160.printer.readAndPrintGconf();
  tmc5160.printer.readClearAndPrintGstat();
  tmc5160.printer.readAndPrintSwMode();
  tmc5160.printer.readAndPrintRampStat();
  tmc5160.printer.readAndPrintChopconf();
  tmc5160.printer.readAndPrintDrvStatus();
  tmc5160.printer.getStoredAndPrintPwmconf();
  tmc5160.printer.readAndPrintPwmScale();
  tmc5160.printer.readAndPrintPwmAuto();

  delay(DELAY);
}
