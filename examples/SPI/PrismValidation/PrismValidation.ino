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

const auto spi_parameters = tmc51x0::SpiParameters{}
                                .withSpi (&spi)
                                .withChipSelectPin (CHIP_SELECT_PIN);

const auto converter_parameters = tmc51x0::ConverterParameters{}
                                      .withClockFrequencyMHz (16)
                                      .withMicrostepsPerRealPositionUnit (51200)
                                      .withSecondsPerRealVelocityUnit (60);
// one "real unit" in this example is one motor rotation
// velocity is reported in rotations per minute

const auto driver_parameters_real = tmc51x0::DriverParameters{}
                                        .withGlobalCurrentScaler (100)
                                        .withRunCurrent (50)
                                        .withHoldCurrent (0)
                                        .withHoldDelay (0)
                                        .withPwmOffset (15)
                                        .withPwmGradient (5)
                                        .withChopperMode (tmc51x0::SpreadCycleMode)
                                        .withStealthChopThreshold (40)
                                        .withCoolStepThreshold (50)
                                        .withCoolStepMin (1)
                                        .withCoolStepMax (0)
                                        .withCoolStepEnabled (true)
                                        .withHighVelocityThreshold (200)
                                        .withHighVelocityFullstepEnabled (true)
                                        .withHighVelocityChopperSwitchEnabled (true)
                                        .withStallGuardThreshold (0)
                                        .withStallGuardFilterEnabled (true);

const auto controller_parameters_real = tmc51x0::ControllerParameters{}
                                            .withRampMode (tmc51x0::VelocityPositiveMode)
                                            .withStopMode (tmc51x0::HardMode)
                                            .withMaxVelocity (120)
                                            .withMaxAcceleration (45);

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t COMMUNICATION_RETRY_DELAY_MS = 1000;
const uint16_t POWER_OFF_DELAY_MS = 2000;
const uint16_t POWER_STABILIZE_DELAY_MS = 2000;
const uint16_t MOTION_SETTLE_DELAY_MS = 3000;
const uint16_t STOP_SETTLE_DELAY_MS = 1500;
const uint32_t ZERO_VELOCITY_TIMEOUT_MS = 12000;
const uint16_t ZERO_VELOCITY_POLL_DELAY_MS = 250;

TMC51X0 stepper;

void
printStopDiagnostics (const char *label)
{
  uint32_t rampmode_raw = stepper.registers.read (tmc51x0::Registers::RampmodeAddress);
  int32_t vactual_raw = stepper.controller.readActualVelocity ();
  uint32_t vstart_raw = stepper.registers.read (tmc51x0::Registers::VstartAddress);
  uint32_t vmax_raw = stepper.registers.read (tmc51x0::Registers::VmaxAddress);
  uint32_t vstop_raw = stepper.registers.read (tmc51x0::Registers::VstopAddress);

  tmc51x0::Registers::RampStat ramp_stat;
  ramp_stat.raw = stepper.registers.read (tmc51x0::Registers::RampStatAddress);

  tmc51x0::Registers::DrvStatus drv_status;
  drv_status.raw = stepper.registers.read (tmc51x0::Registers::DrvStatusAddress);

  Serial.print ("stop_diagnostics: ");
  Serial.println (label);
  Serial.print ("  rampmode_raw: ");
  Serial.println (rampmode_raw);
  Serial.print ("  vactual_raw: ");
  Serial.println (vactual_raw);
  Serial.print ("  vstart_raw: ");
  Serial.println (vstart_raw);
  Serial.print ("  vmax_raw: ");
  Serial.println (vmax_raw);
  Serial.print ("  vstop_raw: ");
  Serial.println (vstop_raw);
  Serial.print ("  ramp_stat_raw: 0x");
  Serial.println (ramp_stat.raw, HEX);
  Serial.print ("  ramp_stat.velocity_reached: ");
  Serial.println (ramp_stat.velocity_reached ());
  Serial.print ("  ramp_stat.position_reached: ");
  Serial.println (ramp_stat.position_reached ());
  Serial.print ("  ramp_stat.vzero: ");
  Serial.println (ramp_stat.vzero ());
  Serial.print ("  ramp_stat.t_zerowait_active: ");
  Serial.println (ramp_stat.t_zerowait_active ());
  Serial.print ("  drv_status_raw: 0x");
  Serial.println (drv_status.raw, HEX);
  Serial.print ("  drv_status.stst: ");
  Serial.println (drv_status.stst ());
  Serial.println ("--------------------------");
}

void
waitForCommunication ()
{
  while (!stepper.communicating ())
    {
      Serial.println ("No communication detected, check motor power and SPI wiring.");
      delay (COMMUNICATION_RETRY_DELAY_MS);
    }
}

bool
waitForZeroVelocity ()
{
  uint32_t start_ms = millis ();

  stepper.controller.beginRampToZeroVelocity ();
  while (!stepper.controller.zeroVelocity ())
    {
      Serial.println ("Waiting for the motor to reach zero velocity.");
      delay (ZERO_VELOCITY_POLL_DELAY_MS);
      if ((millis () - start_ms) >= ZERO_VELOCITY_TIMEOUT_MS)
        {
          Serial.print ("Zero-velocity wait timed out after ms: ");
          Serial.println (millis () - start_ms);
          printStopDiagnostics ("timeout");
          stepper.controller.endRampToZeroVelocity ();
          return false;
        }
    }
  stepper.controller.endRampToZeroVelocity ();
  Serial.print ("Zero velocity reached after ms: ");
  Serial.println (millis () - start_ms);
  printStopDiagnostics ("zero reached");
  return true;
}

void
printVelocityAndPosition ()
{
  int32_t actual_velocity_chip = stepper.controller.readActualVelocity ();
  int32_t actual_velocity_real = stepper.converter.velocityChipToReal (actual_velocity_chip);
  Serial.print ("actual_velocity_rpm: ");
  Serial.println (actual_velocity_real);

  int32_t actual_position_chip = stepper.controller.readActualPosition ();
  int32_t actual_position_real = stepper.converter.positionChipToReal (actual_position_chip);
  Serial.print ("actual_position_rotations: ");
  Serial.println (actual_position_real);
}

void
printStatusSnapshot ()
{
  uint8_t version = stepper.readVersion ();
  Serial.print ("version: 0x");
  Serial.println (version, HEX);

  stepper.printer.readClearAndPrintGstat ();
  stepper.printer.readAndPrintIoin ();
  stepper.printer.readAndPrintRampStat ();
  stepper.printer.readAndPrintDrvStatus ();
  stepper.printer.readAndPrintPwmScale ();
  printVelocityAndPosition ();
  Serial.println ("--------------------------");
}

void
runVelocityPhase (const char *label,
                  tmc51x0::RampMode ramp_mode,
                  uint16_t dwell_ms)
{
  Serial.print ("Commanded phase: ");
  Serial.println (label);
  stepper.driver.enable ();
  stepper.controller.writeRampMode (ramp_mode);
  delay (dwell_ms);
  printStatusSnapshot ();
}

void
runStopPhase (const char *label)
{
  Serial.print ("Commanded phase: ");
  Serial.println (label);
  stepper.controller.writeRampMode (tmc51x0::HoldMode);
  bool reached_zero_velocity = waitForZeroVelocity ();
  delay (STOP_SETTLE_DELAY_MS);
  if (!reached_zero_velocity)
    {
      Serial.println ("Proceeding after zero-velocity timeout.");
    }
  printStatusSnapshot ();
}

void
runRecoveryDrill ()
{
  Serial.println ("Phase 7: recovery drill after controlled chip power-cycle");
  stepper.controller.writeRampMode (tmc51x0::HoldMode);
  stepper.driver.disable ();
  delay (STOP_SETTLE_DELAY_MS);

  stepper.notePossibleMirrorDrift ();
  Serial.print ("mirror_resync_required_before_reset: ");
  Serial.println (stepper.mirrorResyncRequired ());

  stepper.disablePower ();
  delay (POWER_OFF_DELAY_MS);
  stepper.enablePower ();
  delay (POWER_STABILIZE_DELAY_MS);

  bool recovery_ok = stepper.recoverFromDeviceReset ();
  Serial.print ("recover_from_device_reset_ok: ");
  Serial.println (recovery_ok);

  bool readable_ok = stepper.resyncReadableConfiguration ();
  Serial.print ("resync_readable_configuration_ok: ");
  Serial.println (readable_ok);

  Serial.print ("mirror_resync_required_after_recovery: ");
  Serial.println (stepper.mirrorResyncRequired ());

  printStatusSnapshot ();
}

void
setup ()
{
  Serial.begin (SERIAL_BAUD_RATE);

  stepper.setEnablePowerPin (POWER_ENABLE_PIN);
  stepper.setEnablePowerPolarity (POWER_ENABLE_ACTIVE_STATE);
  stepper.disablePower ();
  delay (POWER_OFF_DELAY_MS);
  stepper.enablePower ();
  delay (POWER_STABILIZE_DELAY_MS);

#if defined(ARDUINO_ARCH_RP2040)
  spi.setSCK (SCK_PIN);
  spi.setTX (COPI_PIN);
  spi.setRX (CIPO_PIN);
#endif
  spi.begin ();

  stepper.setupSpi (spi_parameters, tmc51x0::Registers::DeviceModel::TMC5130A);
  stepper.converter.setup (converter_parameters);

  tmc51x0::DriverParameters driver_parameters_chip = stepper.converter.driverParametersRealToChip (driver_parameters_real);
  stepper.driver.setup (driver_parameters_chip);

  tmc51x0::ControllerParameters controller_parameters_chip = stepper.converter.controllerParametersRealToChip (controller_parameters_real);
  stepper.controller.setup (controller_parameters_chip);

  waitForCommunication ();

  while (stepper.controller.stepAndDirectionMode ())
    {
      Serial.println ("Step/dir mode is enabled, so SPI motion commands will not work.");
      delay (COMMUNICATION_RETRY_DELAY_MS);
    }

  stepper.driver.enable ();
  (void)waitForZeroVelocity ();
  stepper.controller.zeroActualPosition ();

  Serial.println ("Starting PrismValidation SPI validation loop.");
}

void
loop ()
{
  Serial.println ("Phase 1: idle baseline");
  printStatusSnapshot ();
  delay (STOP_SETTLE_DELAY_MS);

  runVelocityPhase ("forward velocity mode",
                    tmc51x0::VelocityPositiveMode,
                    MOTION_SETTLE_DELAY_MS);

  runStopPhase ("ramp to zero after forward motion");

  runVelocityPhase ("reverse velocity mode",
                    tmc51x0::VelocityNegativeMode,
                    MOTION_SETTLE_DELAY_MS);

  runStopPhase ("ramp to zero after reverse motion");

  Serial.println ("Phase 5: hold mode baseline before disable");
  stepper.controller.writeRampMode (tmc51x0::HoldMode);
  delay (STOP_SETTLE_DELAY_MS);
  printStatusSnapshot ();

  Serial.println ("Phase 6: disable outputs and verify communication remains healthy");
  stepper.controller.writeRampMode (tmc51x0::HoldMode);
  bool reached_zero_velocity = waitForZeroVelocity ();
  if (!reached_zero_velocity)
    {
      Serial.println ("Disabling outputs after zero-velocity timeout.");
    }
  stepper.driver.disable ();
  delay (STOP_SETTLE_DELAY_MS);
  printStatusSnapshot ();

  runRecoveryDrill ();
  delay (STOP_SETTLE_DELAY_MS);
}
