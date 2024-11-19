#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

const tmc51x0::UartParameters uart_parameters =
{
  uart,
  0, // node_address
  14 // enable_txrx_pin
};
const uint32_t UART_BAUD_RATE = 115200;

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t DELAY = 1000;

const tmc51x0::ConverterParameters converter_parameters =
{
  16, // clock_frequency_mhz
  256 // microsteps_per_real_unit
};
// external clock is 16MHz
// 256 microsteps per fullstep
// one "real unit" in this example is one fullstep of the motor shaft

// driver constants
const uint8_t GLOBAL_CURRENT_SCALAR = 100; // percent
const uint8_t RUN_CURRENT = 50; // percent
const uint8_t PWM_OFFSET = 20; // percent
const uint8_t PWM_GRADIENT = 5; // percent
const tmc51x0::Driver::MotorDirection MOTOR_DIRECTION = tmc51x0::Driver::FORWARD;
const uint8_t STEALTH_CHOP_THRESHOLD = 50; // fullsteps/s
const uint8_t COOL_STEP_THRESHOLD = 200; // fullsteps/s
const uint8_t MIN_COOL_STEP = 1;
const uint8_t MAX_COOL_STEP = 0;
const uint8_t HIGH_VELOCITY_THRESHOLD = 600; // fullsteps/s
// const int8_t STALL_GUARD_THRESHOLD = -20;

// controller constants
const int32_t MIN_TARGET_VELOCITY = 50;  // fullsteps/s
const int32_t MAX_TARGET_VELOCITY = 500; // fullsteps/s
const int32_t TARGET_VELOCITY_INC = 50;  // fullsteps/s
const uint32_t MAX_ACCELERATION = 50;  // fullsteps/(s^2)
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 tmc5130;
uint32_t target_velocity;
tmc51x0::Controller::RampMode ramp_mode = tmc51x0::Controller::VELOCITY_POSITIVE;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  tmc5130.setupUart(uart_parameters);

  tmc5130.converter.setup(converter_parameters);
  tmc5130.driver.writeGlobalCurrentScaler(tmc5130.converter.percentToGlobalCurrentScaler(GLOBAL_CURRENT_SCALAR));
  tmc5130.driver.writeRunCurrent(tmc5130.converter.percentToCurrentSetting(RUN_CURRENT));
  tmc5130.driver.writePwmOffset(tmc5130.converter.percentToPwmSetting(PWM_OFFSET));
  tmc5130.driver.writePwmGradient(tmc5130.converter.percentToPwmSetting(PWM_GRADIENT));
  tmc5130.driver.writeMotorDirection(MOTOR_DIRECTION);

  tmc5130.driver.writeStealthChopThreshold(tmc5130.converter.velocityRealToTstep(STEALTH_CHOP_THRESHOLD));

  // tmc5130.driver.writeCoolStepThreshold(tmc5130.converter.velocityRealToTstep(COOL_STEP_THRESHOLD));
  // tmc5130.driver.enableCoolStep(MIN_COOL_STEP, MAX_COOL_STEP);

  // tmc5130.driver.writeHighVelocityThreshold(tmc5130.converter.velocityRealToTstep(HIGH_VELOCITY_THRESHOLD));
  // tmc5130.driver.writeStallGuardThreshold(STALL_GUARD_THRESHOLD);

  tmc5130.controller.writeMaxAcceleration(tmc5130.converter.accelerationRealToChip(MAX_ACCELERATION));
  tmc5130.controller.writeRampMode(ramp_mode);
  tmc5130.controller.writeActualPosition(tmc5130.converter.positionRealToChip(INITIAL_POSITION));

  tmc5130.driver.enable();

  tmc5130.controller.rampToZeroVelocity();
  while (!tmc5130.controller.zeroVelocity())
  {
    Serial.println("Waiting for zero velocity.");
    delay(DELAY);
  }

  target_velocity = MIN_TARGET_VELOCITY;
  tmc5130.controller.writeMaxVelocity(tmc5130.converter.velocityRealToChip(target_velocity));

  delay(DELAY);
}

void loop()
{
  if (tmc5130.controller.velocityReached())
  {
    Serial.print("Target velocity ");
    Serial.print(target_velocity);
    Serial.println(" reached!");

    target_velocity += TARGET_VELOCITY_INC;
    if (target_velocity > MAX_TARGET_VELOCITY)
    {
      target_velocity = MIN_TARGET_VELOCITY;
      if (ramp_mode == tmc51x0::Controller::VELOCITY_POSITIVE)
      {
        ramp_mode = tmc51x0::Controller::VELOCITY_NEGATIVE;
      }
      else
      {
        ramp_mode = tmc51x0::Controller::VELOCITY_POSITIVE;
      }
      tmc5130.controller.writeRampMode(ramp_mode);
    }
    tmc5130.controller.writeMaxVelocity(tmc5130.converter.velocityRealToChip(target_velocity));
  }
  else
  {
    Serial.println("Target velocity not reached yet.");
  }
  Serial.println("--------------------------");
  delay(DELAY);
}
