// ----------------------------------------------------------------------------
// UartInterface.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "UartInterface.hpp"

using namespace tmc51x0;

void UartInterface::setup(UartParameters uart_parameters) {
  interface_mode = UartMode;
  uart_parameters_ = uart_parameters;
  last_uart_error_ = UartError::None;

  if (txrxPinEnabled()) {
    pinMode(uart_parameters_.enable_txrx_pin, OUTPUT);
    disableTxEnableRx();
  }

  UartEngine::Callbacks callbacks;
  callbacks.ctx = this;
  callbacks.available = &UartInterface::engineSerialAvailable_;
  callbacks.read = &UartInterface::engineSerialRead_;
  callbacks.write = &UartInterface::engineSerialWrite_;
  callbacks.flush = &UartInterface::engineSerialFlush_;
  callbacks.set_tx_enable =
    txrxPinEnabled() ? &UartInterface::engineSetTxEnable_ : nullptr;

  UartEngine::Config config;
  config.baud_rate = uart_parameters_.baud_rate;
  config.reply_timeout_us = uart_parameters_.reply_timeout_us;
  config.enable_delay_us = uart_parameters_.enable_delay_us;
  config.max_retries = uart_parameters_.max_retries;
  config.tx_complete_delay_us = uart_parameters_.tx_complete_delay_us;
  config.drain_limit = uart_parameters_.drain_limit;

  uart_engine_.configure(callbacks, config);
}

void UartInterface::writeRegister(uint8_t register_address, uint32_t data) {
  (void)writeRegisterResult(register_address, data);
}

uint32_t UartInterface::readRegister(uint8_t register_address) {
  Result<uint32_t> r = readRegisterResult(register_address);
  return r.ok() ? r.value : 0;
}

Result<void> UartInterface::writeRegisterResult(uint8_t register_address,
                                                uint32_t data) {
  // Blocking wrapper built on the poll-driven engine.
  Result<void> start = startWriteRegister(register_address, data);
  if (!start.ok()) {
    return start;
  }

  while (!uart_engine_.resultReady()) {
    uart_engine_.poll(micros());
    if (!uart_engine_.resultReady()) {
      delayMicroseconds(1);
    }
  }

  return takeWriteResult();
}

Result<uint32_t> UartInterface::readRegisterResult(uint8_t register_address) {
  Result<uint32_t> result;

  Result<void> start = startReadRegister(register_address);
  if (!start.ok()) {
    result.error = start.error;
    last_uart_error_ = result.error;
    return result;
  }

  while (!uart_engine_.resultReady()) {
    uart_engine_.poll(micros());
    if (!uart_engine_.resultReady()) {
      delayMicroseconds(1);
    }
  }

  return takeReadResult();
}

Result<void> UartInterface::startWriteRegister(uint8_t register_address,
                                               uint32_t data) {
  Result<void> r;
  last_uart_error_ = UartError::None;

  if (uart_parameters_.uart_ptr == nullptr) {
    last_uart_error_ = UartError::NotInitialized;
    r.error = last_uart_error_;
    return r;
  }

  r = uart_engine_.startWrite(
    uart_parameters_.node_address, register_address, data);
  last_uart_error_ = r.error;
  return r;
}

Result<void> UartInterface::startReadRegister(uint8_t register_address) {
  Result<void> r;
  last_uart_error_ = UartError::None;

  if (uart_parameters_.uart_ptr == nullptr) {
    last_uart_error_ = UartError::NotInitialized;
    r.error = last_uart_error_;
    return r;
  }

  r = uart_engine_.startRead(uart_parameters_.node_address, register_address);
  last_uart_error_ = r.error;
  return r;
}

void UartInterface::poll(uint32_t now_us) {
  uart_engine_.poll(now_us);
}

void UartInterface::poll() {
  uart_engine_.poll(micros());
}

Result<void> UartInterface::takeWriteResult() {
  Result<void> r = uart_engine_.takeWriteResult();
  last_uart_error_ = r.error;
  return r;
}

Result<uint32_t> UartInterface::takeReadResult() {
  Result<uint32_t> r = uart_engine_.takeReadResult();
  last_uart_error_ = r.error;
  return r;
}

// --------------------------------------------------------------------------
// Engine callbacks
// --------------------------------------------------------------------------

int UartInterface::engineSerialAvailable_(void* ctx) {
  return static_cast<UartInterface*>(ctx)->serialAvailable();
}

int UartInterface::engineSerialRead_(void* ctx) {
  return static_cast<UartInterface*>(ctx)->serialRead();
}

size_t UartInterface::engineSerialWrite_(void* ctx, uint8_t b) {
  return static_cast<UartInterface*>(ctx)->serialWrite(b);
}

void UartInterface::engineSerialFlush_(void* ctx) {
  static_cast<UartInterface*>(ctx)->serialFlush();
}

void UartInterface::engineSetTxEnable_(void* ctx, bool enable) {
  UartInterface* self = static_cast<UartInterface*>(ctx);
  if (enable) {
    self->enableTxDisableRx();
  } else {
    self->disableTxEnableRx();
  }
}

// --------------------------------------------------------------------------
// Low-level UART access
// --------------------------------------------------------------------------

bool UartInterface::txrxPinEnabled() const {
  return uart_parameters_.enable_txrx_pin != NO_PIN;
}

int UartInterface::serialAvailable() {
  if (uart_parameters_.uart_ptr != nullptr) {
    return uart_parameters_.uart_ptr->available();
  }
  return 0;
}

size_t UartInterface::serialWrite(uint8_t c) {
  if (uart_parameters_.uart_ptr != nullptr) {
    return uart_parameters_.uart_ptr->write(c);
  }
  return 0;
}

int UartInterface::serialRead() {
  if (uart_parameters_.uart_ptr != nullptr) {
    return uart_parameters_.uart_ptr->read();
  }
  return -1;
}

void UartInterface::serialFlush() {
  if (uart_parameters_.uart_ptr != nullptr) {
    uart_parameters_.uart_ptr->flush();
  }
}

void UartInterface::enableTxDisableRx() {
  if (!txrxPinEnabled()) {
    return;
  }
  digitalWrite(uart_parameters_.enable_txrx_pin, HIGH);
}

void UartInterface::disableTxEnableRx() {
  if (!txrxPinEnabled()) {
    return;
  }
  digitalWrite(uart_parameters_.enable_txrx_pin, LOW);
}
