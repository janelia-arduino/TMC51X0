// ----------------------------------------------------------------------------
// UartInterface.hpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------

#ifndef TMC51X0_UART_INTERFACE_HPP
#define TMC51X0_UART_INTERFACE_HPP
#include <Arduino.h>

#include "UartParameters.hpp"
#include "Interface.hpp"

#include "Result.hpp"
#include "UartEngine.hpp"

namespace tmc51x0 {
class UartInterface : public Interface {
public:
  void setup(UartParameters uart_parameters);

  // --------------------------------------------------------------------------
  // Backward-compatible blocking API
  // --------------------------------------------------------------------------

  // These call the blocking wrappers and update last_uart_error_.
  void writeRegister(uint8_t register_address, uint32_t data) override;
  uint32_t readRegister(uint8_t register_address) override;

  // --------------------------------------------------------------------------
  // Blocking API with explicit error reporting
  // --------------------------------------------------------------------------

  Result<void> writeRegisterResult(uint8_t register_address,
                                   uint32_t data) override;
  Result<uint32_t> readRegisterResult(uint8_t register_address) override;

  UartError getLastUartError() const {
    return last_uart_error_;
  }

  UartError lastError() const {
    return getLastUartError();
  }

  // --------------------------------------------------------------------------
  // Non-blocking API (poll-driven)
  // --------------------------------------------------------------------------

  Result<void> startWriteRegister(uint8_t register_address, uint32_t data);
  Result<void> startReadRegister(uint8_t register_address);

  // Family-style aliases for the non-blocking API.
  Result<void> startWrite(uint8_t register_address, uint32_t data) {
    return startWriteRegister(register_address, data);
  }
  Result<void> startRead(uint8_t register_address) {
    return startReadRegister(register_address);
  }

  void poll(uint32_t now_us);
  void poll();

  bool busy() const {
    return uart_engine_.busy();
  }
  bool resultReady() const {
    return uart_engine_.resultReady();
  }
  bool done() const {
    return resultReady();
  }

  Result<void> takeWriteResult();
  Result<uint32_t> takeReadResult();

private:
  UartParameters uart_parameters_;
  UartError last_uart_error_{UartError::None};

  UartEngine uart_engine_;

  // Engine callbacks
  static int engineSerialAvailable_(void* ctx);
  static int engineSerialRead_(void* ctx);
  static size_t engineSerialWrite_(void* ctx, uint8_t b);
  static void engineSerialFlush_(void* ctx);
  static void engineSetTxEnable_(void* ctx, bool enable);

  bool txrxPinEnabled() const;

  int serialAvailable();
  size_t serialWrite(uint8_t c);
  int serialRead();
  void serialFlush();

  void enableTxDisableRx();
  void disableTxEnableRx();
};

// Low-friction public bus abstraction for family-wide API parity.
using UartBus = UartInterface;
}
#endif
