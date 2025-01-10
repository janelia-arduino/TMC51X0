// ----------------------------------------------------------------------------
// TMC51X0.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HPP
#define TMC51X0_HPP
#include <Arduino.h>


#if defined(ARDUINO_ARCH_STM32)
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
#endif

#if defined(ARDUINO_ARCH_ESP32)
#define pin_size_t uint16_t
#endif

#include "./TMC51X0/SpiInterface.hpp"
#include "./TMC51X0/UartInterface.hpp"

#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"


struct TMC51X0
{
  void setupSpi(tmc51x0::SpiParameters spi_parameters);
  void setupUart(tmc51x0::UartParameters uart_parameters);

  uint8_t readVersion();

  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

private:
  tmc51x0::SpiInterface interface_spi_;
  tmc51x0::UartInterface interface_uart_;
  void initialize();
};

#endif
