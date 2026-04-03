#include <TMC51X0.hpp>

#if defined(ARDUINO_ARCH_RP2040)
SPIClassRP2040 &spi = SPI1;
const size_t SCK_PIN = 10;
const size_t COPI_PIN = 11;
const size_t CIPO_PIN = 12;
#else
SPIClass &spi = SPI;
#endif

const size_t CHIP_SELECT_PIN = 8;
const size_t POWER_ENABLE_PIN = 15;
const uint8_t POWER_ENABLE_ACTIVE_STATE = HIGH;

const auto spi_parameters =
    tmc51x0::SpiParameters{}.withSpi(&spi).withChipSelectPin(CHIP_SELECT_PIN);

const auto converter_parameters =
    tmc51x0::ConverterParameters{}
        .withClockFrequencyMHz(16)
        .withMicrostepsPerRealPositionUnit(4881);

const auto driver_parameters_real =
    tmc51x0::DriverParameters{}
        .withRunCurrent(75)
        .withHoldCurrent(0)
        .withHoldDelay(0)
        .withPwmOffset(25)
        .withPwmGradient(15)
        .withMotorDirection(tmc51x0::ForwardDirection)
        .withStandstillMode(tmc51x0::PassiveBrakingLsMode)
        .withStealthChopThreshold(250);

const auto controller_parameters_real =
    tmc51x0::ControllerParameters{}
        .withRampMode(tmc51x0::PositionMode)
        .withMaxVelocity(20)
        .withMaxAcceleration(20)
        .withStartVelocity(1)
        .withStopVelocity(5)
        .withFirstVelocity(10)
        .withFirstAcceleration(40)
        .withMaxDeceleration(30)
        .withFirstDeceleration(50);

const auto home_parameters_real =
    tmc51x0::HomeParameters{}
        .withRunCurrent(50)
        .withHoldCurrent(20)
        .withTargetPosition(-500)
        .withVelocity(20)
        .withAcceleration(2)
        .withZeroWaitDuration(100);

const auto stall_parameters_real =
    tmc51x0::StallParameters{}
        .withStallGuardThreshold(10)
        .withCoolStepThreshold(10);

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t POWER_OFF_DELAY_MS = 2000;
const uint16_t POWER_STABILIZE_DELAY_MS = 2000;
const uint16_t LOOP_DELAY_MS = 250;
const uint16_t PAUSE_DELAY_MS = 4000;

TMC51X0 stepper;
tmc51x0::HomeParameters home_parameters_chip;
tmc51x0::StallParameters stall_parameters_chip;

void waitForCommunication() {
  while (!stepper.communicating()) {
    Serial.println("No communication detected.");
    delay(LOOP_DELAY_MS);
  }
}

void waitForZeroVelocity() {
  stepper.controller.beginRampToZeroVelocity();
  while (!stepper.controller.zeroVelocity()) {
    Serial.println("Waiting for zero velocity.");
    delay(LOOP_DELAY_MS);
  }
  stepper.controller.endRampToZeroVelocity();
}

void printSnapshot(const char *label) {
  int32_t xactual = stepper.controller.readActualPosition();
  int32_t vactual = stepper.controller.readActualVelocity();
  int32_t xreal = stepper.converter.positionChipToReal(xactual);
  int32_t vreal = stepper.converter.velocityChipToReal(vactual);

  Serial.print("snapshot: ");
  Serial.println(label);
  Serial.print("  xactual_raw: ");
  Serial.println(xactual);
  Serial.print("  vactual_raw: ");
  Serial.println(vactual);
  Serial.print("  position_mm: ");
  Serial.println(xreal);
  Serial.print("  velocity_mm_per_s: ");
  Serial.println(vreal);
  Serial.print("  vmax_raw: ");
  Serial.println(stepper.registers.read(tmc51x0::Registers::VmaxAddress));
  Serial.print("  amax_raw: ");
  Serial.println(stepper.registers.read(tmc51x0::Registers::AmaxAddress));
  Serial.print("  sg_result: ");
  Serial.println(stepper.driver.readStallGuardResult());
  Serial.print("  home_failed: ");
  Serial.println(stepper.homeFailed());
  Serial.print("  homed: ");
  Serial.println(stepper.homed());
  Serial.println("--------------------------");
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);

  stepper.setEnablePowerPin(POWER_ENABLE_PIN);
  stepper.setEnablePowerPolarity(POWER_ENABLE_ACTIVE_STATE);
  stepper.disablePower();
  delay(POWER_OFF_DELAY_MS);
  stepper.enablePower();
  delay(POWER_STABILIZE_DELAY_MS);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK(SCK_PIN);
  spi.setTX(COPI_PIN);
  spi.setRX(CIPO_PIN);
#endif
  spi.begin();

  stepper.setupSpi(spi_parameters, tmc51x0::Registers::DeviceModel::TMC5130A);
  stepper.converter.setup(converter_parameters);

  auto driver_parameters_chip =
      stepper.converter.driverParametersRealToChip(driver_parameters_real);
  stepper.driver.setup(driver_parameters_chip);

  auto controller_parameters_chip =
      stepper.converter.controllerParametersRealToChip(
          controller_parameters_real);
  stepper.controller.setup(controller_parameters_chip);

  home_parameters_chip =
      stepper.converter.homeParametersRealToChip(home_parameters_real);
  stall_parameters_chip =
      stepper.converter.stallParametersRealToChip(stall_parameters_real);

  waitForCommunication();

  while (stepper.controller.stepAndDirectionMode()) {
    Serial.println("Step/dir mode enabled.");
    delay(LOOP_DELAY_MS);
  }

  stepper.driver.enable();
  waitForZeroVelocity();
  stepper.controller.zeroActualPosition();
  printSnapshot("setup complete");
}

void loop() {
  Serial.println("Starting home-to-stall...");
  stepper.beginHomeToStall(home_parameters_chip, stall_parameters_chip);

  while ((!stepper.homed()) && (!stepper.homeFailed())) {
    (void)stepper.recoverIfUnhealthy();
    printSnapshot("homing");
    delay(LOOP_DELAY_MS);
  }

  printSnapshot("home finished");
  stepper.endHome();
  delay(PAUSE_DELAY_MS);
}
