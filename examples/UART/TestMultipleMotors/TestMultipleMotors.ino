#include <TMC51X0.hpp>


#if !defined(ARDUINO_ARCH_RP2040)
HardwareSerial & uart = Serial2;
#else
SerialUART & uart = Serial2;
size_t TX_PIN = 4;
size_t RX_PIN = 5;
#endif

// Optional power enable
const size_t ENABLE_POWER_PIN = 15;
const uint8_t ENABLE_POWER_VALUE = HIGH;

const uint8_t MOTOR_COUNT = 7;

tmc51x0::UartParameters uart_parameters =
{
  uart,
  0 // node_address
};
const uint8_t ENABLE_TXRX_PINS[MOTOR_COUNT] = {14, 13, 12, 11, 10, 9, 8};
const uint32_t UART_BAUD_RATE = 115200;

const uint8_t MUX_ADDRESS_0_PIN = 6;
const uint8_t MUX_ADDRESS_1_PIN = 3;
const uint8_t MUX_ADDRESS_2_PIN = 2;
const uint8_t MUX_ADDRESS_0_VALUES[MOTOR_COUNT] = {0, 1, 0, 1, 0, 1, 0};
const uint8_t MUX_ADDRESS_1_VALUES[MOTOR_COUNT] = {0, 0, 1, 1, 0, 0, 1};
const uint8_t MUX_ADDRESS_2_VALUES[MOTOR_COUNT] = {0, 0, 0, 0, 1, 1, 1};

const uint32_t SERIAL_BAUD_RATE = 115200;
const uint16_t LOOP_DELAY = 2000;

const tmc51x0::ConverterParameters converter_parameters =
{
  16, // clock_frequency_mhz
  4881 // microsteps_per_real_unit
};
// external clock is 16MHz
// 200 fullsteps per revolution for many steppers * 256 microsteps per fullstep
// 10.49 millimeters per revolution leadscrew -> 51200 / 10.49 ~= 4881
// one "real unit" in this example is one millimeters of linear travel

const tmc51x0::DriverParameters driver_parameters_real =
{
  50, // global_current_scaler (percent)
  20, // run_current (percent)
  0, // hold_current (percent)
  0, // hold_delay (percent)
  20, // pwm_offset (percent)
  5, // pwm_gradient (percent)
  false, // automatic_current_control_enabled
  tmc51x0::REVERSE, // motor_direction
  tmc51x0::NORMAL, // standstill_mode
  tmc51x0::SPREAD_CYCLE, // chopper_mode
  10, // stealth_chop_threshold (millimeters/s)
  true, // stealth_chop_enabled
  50, // cool_step_threshold (millimeters/s)
  1, // cool_step_min
  0, // cool_step_max
  true, // cool_step_enabled
  90, // high_velocity_threshold (rotations/min)
  true, // high_velocity_fullstep_enabled
  true, // high_velocity_chopper_switch_enabled
  1, // stall_guard_threshold
  false, // stall_guard_filter_enabled
  true // short_to_ground_protection_enabled
};

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
const int32_t MAX_TARGET_POSITION = 600;  // millimeters
const tmc51x0::Controller::RampMode RAMP_MODE = tmc51x0::Controller::POSITION;
const int32_t INITIAL_POSITION = 0;

// Instantiate TMC51X0
TMC51X0 motors[MOTOR_COUNT];
uint32_t target_position;

void setup()
{
  Serial.begin(SERIAL_BAUD_RATE);

  delay(5000);

  Serial.println("Disabling VCC");

  pinMode(ENABLE_POWER_PIN, OUTPUT);
  digitalWrite(ENABLE_POWER_PIN, ENABLE_POWER_VALUE);
  pinMode(MUX_ADDRESS_0_PIN, OUTPUT);
  pinMode(MUX_ADDRESS_1_PIN, OUTPUT);
  pinMode(MUX_ADDRESS_2_PIN, OUTPUT);

#if defined(ARDUINO_ARCH_RP2040)
  uart.setTX(TX_PIN);
  uart.setRX(RX_PIN);
#endif
  uart.begin(UART_BAUD_RATE);

  randomSeed(analogRead(A0));

  for (size_t i=0; i<MOTOR_COUNT; ++i)
  {
    TMC51X0 & motor = motors[i];

    uart_parameters.enable_txrx_pin = ENABLE_TXRX_PINS[i];
    motor.setupUart(uart_parameters);

    motor.converter.setup(converter_parameters);
    tmc51x0::DriverParameters driver_parameters_chip = motor.converter.driverParametersRealToChip(driver_parameters_real);

    digitalWrite(MUX_ADDRESS_0_PIN, MUX_ADDRESS_0_VALUES[i]);
    digitalWrite(MUX_ADDRESS_1_PIN, MUX_ADDRESS_1_VALUES[i]);
    digitalWrite(MUX_ADDRESS_2_PIN, MUX_ADDRESS_2_VALUES[i]);

    motor.driver.setup(driver_parameters_chip);

    motor.controller.writeFirstAcceleration(motor.converter.accelerationRealToChip(FIRST_ACCELERATION));
    motor.controller.writeFirstVelocity(motor.converter.velocityRealToChip(FIRST_VELOCITY));
    motor.controller.writeMaxAcceleration(motor.converter.accelerationRealToChip(MAX_ACCELERATION));
    motor.controller.writeMaxDeceleration(motor.converter.accelerationRealToChip(MAX_DECELERATION));
    motor.controller.writeFirstDeceleration(motor.converter.accelerationRealToChip(FIRST_DECELERATION));
    motor.controller.writeStopVelocity(motor.converter.velocityRealToChip(STOP_VELOCITY));
    motor.controller.writeRampMode(RAMP_MODE);
    motor.controller.writeActualPosition(motor.converter.positionRealToChip(INITIAL_POSITION));

    motor.driver.enable();

    motor.controller.writeStartVelocity(motor.converter.velocityRealToChip(START_VELOCITY));
    motor.controller.writeMaxVelocity(motor.converter.velocityRealToChip(MAX_VELOCITY));

    target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
    Serial.print("motor ");
    Serial.print(i);
    Serial.print(" target position: ");
    Serial.println(target_position);
    motor.controller.writeTargetPosition(motor.converter.positionRealToChip(target_position));
  }
}

void loop()
{
  for (size_t i=0; i<MOTOR_COUNT; ++i)
  {
    digitalWrite(MUX_ADDRESS_0_PIN, MUX_ADDRESS_0_VALUES[i]);
    digitalWrite(MUX_ADDRESS_1_PIN, MUX_ADDRESS_1_VALUES[i]);
    digitalWrite(MUX_ADDRESS_2_PIN, MUX_ADDRESS_2_VALUES[i]);

    TMC51X0 & motor = motors[i];
    // uint8_t version = motor.readVersion();
    // Serial.print("version: 0x");
    // Serial.println(version, HEX);
    motor.printer.readAndPrintRampStat();
    if (motor.controller.positionReached())
    {
      Serial.print("motor ");
      Serial.print(i);
      Serial.println(":");
      Serial.println("reached target position!");
      target_position = random(MIN_TARGET_POSITION, MAX_TARGET_POSITION);
      Serial.print("new target position: ");
      Serial.println(target_position);
      motor.controller.writeTargetPosition(motor.converter.positionRealToChip(target_position));
      Serial.println("--------------------------");
    }
  }

  Serial.println("--------------------------");
  delay(LOOP_DELAY);
}
