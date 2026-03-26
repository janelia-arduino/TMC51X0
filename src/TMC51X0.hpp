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

namespace tmc51x0 {
struct HealthStatus {
  bool communication_ok{false};
  bool reset{false};
  bool driver_error{false};
  bool charge_pump_undervoltage{false};
  bool mirror_resync_required{false};
};
}

struct TMC51X0 {
  tmc51x0::Registers registers;
  tmc51x0::Converter converter;
  tmc51x0::Driver driver;
  tmc51x0::Controller controller;
  tmc51x0::Encoder encoder;
  tmc51x0::Printer printer;

  TMC51X0();

  void setupSpi(tmc51x0::SpiParameters spi_parameters,
                tmc51x0::Registers::DeviceModel expected_device_model =
                  tmc51x0::Registers::DeviceModel::Unknown);
  void setupUart(tmc51x0::UartParameters uart_parameters,
                 tmc51x0::Registers::DeviceModel expected_device_model =
                   tmc51x0::Registers::DeviceModel::Unknown);

  // Advanced access to the underlying UART interface (for non-blocking polling).
  tmc51x0::UartInterface& uartInterface() {
    return interface_uart_;
  }
  const tmc51x0::UartInterface& uartInterface() const {
    return interface_uart_;
  }

  // Preferred family-style alias for async / event-loop driven UART access.
  tmc51x0::UartBus& uartBus() {
    return interface_uart_;
  }
  const tmc51x0::UartBus& uartBus() const {
    return interface_uart_;
  }

  uint8_t readVersion();
  bool communicating();

  void setEnablePowerPin(size_t enable_power_pin);
  void setEnablePowerPolarity(uint8_t pin_value_when_enabled);
  void enablePower();
  void disablePower();

  void notePossibleMirrorDrift();
  bool mirrorResyncRequired() const;
  tmc51x0::Registers::DeviceModel deviceModel() const;
  bool resyncReadableConfiguration();
  tmc51x0::HealthStatus readHealthStatus() {
    tmc51x0::HealthStatus status;
    tmc51x0::Registers::Gstat gstat = registers.readAndClearGstat();
    status.reset = gstat.reset();
    status.driver_error = gstat.drv_err();
    status.charge_pump_undervoltage = gstat.uv_cp();
    status.communication_ok = communicating();
    status.mirror_resync_required = mirrorResyncRequired();
    return status;
  }

  // Call reinitialize after cycling power to the chip. This reseeds the
  // software-side register mirror to known reset defaults, clears reset flags,
  // and replays the library setup state.
  void reinitialize();
  bool recoverFromDeviceReset();
  bool recoverIfNeeded();
  bool recoverIfUnhealthy() {
    tmc51x0::HealthStatus status = readHealthStatus();
    if (!status.communication_ok || status.reset || status.driver_error ||
        status.charge_pump_undervoltage) {
      notePossibleMirrorDrift();
    }
    return recoverIfNeeded();
  }

  void beginHomeToSwitch(tmc51x0::HomeParameters home_parameters,
                         tmc51x0::SwitchParameters switch_parameters);
  void beginHomeToStall(tmc51x0::HomeParameters home_parameters,
                        tmc51x0::StallParameters stall_parameters);
  void endHome();
  bool homed();
  bool homeFailed() const {
    return home_failed_;
  }

private:
  enum class HomeMode : uint8_t {
    None = 0,
    Switch,
    Stall,
  };

  tmc51x0::SpiInterface interface_spi_;
  tmc51x0::UartInterface interface_uart_;
  size_t enable_power_pin_;
  uint8_t pin_value_when_enabled_;
  HomeMode home_mode_;
  bool home_succeeded_;
  bool home_failed_;
  void initialize();
  bool updateDeviceModelFromVersion_(uint8_t version);
  bool finishSetupOrRecovery_();
  void resetHomeTracking_();
};

#endif
