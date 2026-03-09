# Migration notes

This update keeps the existing UART API working while aligning the public surface with the intended family conventions.

## What changed

- `tmc51x0::UartError` now has an explicit `uint8_t` underlying type.
- `TMC51X0::uartBus()` was added as the preferred family-style alias for async UART access.
- `TMC51X0::uartInterface()` remains available for compatibility.
- `UartInterface` now exposes family-style async aliases:
  - `startRead(...)`
  - `startWrite(...)`
  - `done()`
  - `lastError()`
- `UartParameters` now exposes `withDrainLimit(...)` for stale-RX handling.
- RX drain overflow now surfaces as `UartError::RxGarbage` instead of being silently absorbed and later reported ambiguously.
- The software-side register mirror now updates only after explicit successful transport operations. Failed UART reads/writes no longer poison the stored cache with fallback values.
- Reset seeding is now chip-aware. `TMC51X0::setupSpi(...)` and `setupUart(...)` optionally accept an expected `Registers::DeviceModel` so the mirror can start from the correct reset defaults even before the first successful identity read.
- The register mirror now tracks confidence per entry via `Registers::MirrorConfidence` (`Unknown`, `ResetDefault`, `AssumedWritten`, `ReadVerified`).
- SPI now latches the transport-level `reset_flag` and marks the mirror as requiring recovery instead of silently trusting the old cache.
- `TMC51X0::reinitialize()` now performs the stronger reset-recovery flow: reseed, clear reset flags, replay desired configuration state, and verify a safe subset of readable configuration registers.
- Recovery now replays the latest high-level configuration writes instead of only the original `setup(...)` parameter structs. Direct calls like `driver.writeRunCurrent(...)`, `controller.writeMaxVelocity(...)`, and `encoder.writeFractionalMode(...)` now survive `recoverFromDeviceReset()`.
- Runtime motion state is still intentionally conservative after recovery: actual/target position are re-seeded and in-flight motion is not resumed automatically.
- New recovery helpers are available on `TMC51X0`:
  - `notePossibleMirrorDrift()`
  - `mirrorResyncRequired()`
  - `recoverFromDeviceReset()`
  - `recoverIfNeeded()`
  - `resyncReadableConfiguration()`
- Advanced recovery code can inspect `stepper.registers.storedConfidence(...)`, `stepper.registers.resyncRequired()`, and `stepper.registers.deviceModel()` directly when needed.
- See [docs/RECOVERY.md](./docs/RECOVERY.md) for the recommended reset / re-sync workflow.

## Existing code compatibility

Existing blocking code continues to work:

```cpp
stepper.setupUart(uart_parameters);
auto value = stepper.registers.read(tmc51x0::Registers::GconfAddress);
```

Existing async code that used `uartInterface()` also continues to work:

```cpp
auto &uart = stepper.uartInterface();
uart.poll();
```

## Preferred style going forward

Use `uartBus()` when writing new non-blocking code so this library matches the broader UART-driver family more closely:

```cpp
auto &bus = stepper.uartBus();
auto start = bus.startRead(tmc51x0::Registers::GconfAddress);
if (start.ok())
  {
    // keep polling until bus.done()
  }
```

If you have external power supervision or watchdog logic, the preferred recovery pattern after any suspected drift is:

```cpp
stepper.notePossibleMirrorDrift();

// Optionally power-cycle the driver externally here.

if (stepper.mirrorResyncRequired())
  {
    bool ok = stepper.recoverFromDeviceReset();
    if (!ok)
      {
        // Communication is still not healthy.
      }
  }
```
