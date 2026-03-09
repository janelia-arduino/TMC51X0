// ----------------------------------------------------------------------------
// TMC51X0.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_HPP
#define TMC51X0_HPP
#include <Arduino.h>

#include "TMC51X0/SpiInterface.hpp"
#include "TMC51X0/UartInterface.hpp"
#include "HomeParameters.hpp"
#include "SwitchParameters.hpp"
#include "StallParameters.hpp"

#include "Registers.hpp"
#include "Converter.hpp"
#include "Driver.hpp"
#include "Controller.hpp"
#include "Encoder.hpp"
#include "Printer.hpp"

struct TMC51X0
{
  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

  TMC51X0 ();

  void setupSpi (
      tmc51x0::SpiParameters spi_parameters,
      tmc51x0::Registers::DeviceModel expected_device_model = tmc51x0::Registers::DeviceModel::Unknown);
  void setupUart (
      tmc51x0::UartParameters uart_parameters,
      tmc51x0::Registers::DeviceModel expected_device_model = tmc51x0::Registers::DeviceModel::Unknown);

  // Advanced access to the underlying UART interface (for non-blocking polling).
  tmc51x0::UartInterface &
  uartInterface ()
  {
    return interface_uart_;
  }
  const tmc51x0::UartInterface &
  uartInterface () const
  {
    return interface_uart_;
  }

  // Preferred family-style alias for async / event-loop driven UART access.
  tmc51x0::UartBus &
  uartBus ()
  {
    return interface_uart_;
  }
  const tmc51x0::UartBus &
  uartBus () const
  {
    return interface_uart_;
  }

  uint8_t readVersion ();
  bool communicating ();

  void setEnablePowerPin (size_t enable_power_pin);
  void setEnablePowerPolarity (uint8_t pin_value_when_enabled);
  void enablePower ();
  void disablePower ();

  void notePossibleMirrorDrift ();
  bool mirrorResyncRequired () const;
  tmc51x0::Registers::DeviceModel deviceModel () const;
  bool resyncReadableConfiguration ();

  // Call reinitialize after cycling power to the chip. This reseeds the
  // software-side register mirror to known reset defaults, clears reset flags,
  // and replays the library setup state.
  void reinitialize ();
  bool recoverFromDeviceReset ();
  bool recoverIfNeeded ();

  void beginHomeToSwitch (tmc51x0::HomeParameters home_parameters,
                          tmc51x0::SwitchParameters switch_parameters);
  void beginHomeToStall (tmc51x0::HomeParameters home_parameters,
                         tmc51x0::StallParameters stall_parameters);
  void endHome ();
  bool homed ();

private:
  tmc51x0::SpiInterface interface_spi_;
  tmc51x0::UartInterface interface_uart_;
  size_t enable_power_pin_;
  uint8_t pin_value_when_enabled_;
  void initialize ();
  bool updateDeviceModelFromVersion_ (uint8_t version);
  bool finishSetupOrRecovery_ ();
};

#endif
