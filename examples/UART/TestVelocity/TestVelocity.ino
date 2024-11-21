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

const tmc51x0::DriverParameters driver_parameters_real =
{
  50, // global_current_scalar (percent)
  50, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  20, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::FORWARD, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  50, // stealth_chop_threshold (fullsteps/s)
  true, // stealth_chop_enabled
  200, // cool_step_threshold (fullsteps/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  600, // high_velocity_threshold (fullsteps/s)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  1, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};

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

  tmc51x0::DriverParameters driver_parameters_chip = tmc5130.converter.driverParametersRealToChip(driver_parameters_real);
  tmc5130.driver.setup(driver_parameters_chip);

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
