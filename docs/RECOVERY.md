# Recovery after reset or suspected mirror drift

`TMC51X0` keeps a software-side mirror of chip registers so higher-level code can
perform read-modify-write updates even for write-only registers. That mirror can
still become uncertain if the chip resets outside the library or if SPI reports
its reset flag.

This update hardens the mirror and adds an explicit recovery path.

## What changed

- reset seeding is now chip-aware (`TMC5130A` vs `TMC5160A`)
- the mirror now tracks confidence per register:
  - `Unknown`
  - `ResetDefault`
  - `AssumedWritten`
  - `ReadVerified`
- SPI now latches the transport-level reset flag and marks the mirror as needing
  re-synchronization
- the library can re-read a safe subset of readable configuration registers to
  verify post-reset state

## Recommended usage

If you already know the target chip, pass it to `setupSpi(...)` or
`setupUart(...)` so reset defaults are deterministic even before the first
successful identity read:

```cpp
stepper.setupSpi(
    spi_parameters,
    tmc51x0::Registers::DeviceModel::TMC5160A);
```

If your hardware supervisor, power-good logic, or fault handling code suspects
mirror drift, mark the mirror uncertain and then recover after the external
reset or power cycle:

```cpp
stepper.notePossibleMirrorDrift();

// Optionally power-cycle the chip externally here.

if (stepper.mirrorResyncRequired())
  {
    bool ok = stepper.recoverFromDeviceReset();
    if (!ok)
      {
        // Communication is still not healthy.
      }
  }
```

For polling-style maintenance code, `recoverIfNeeded()` offers a single call:

```cpp
(void)stepper.recoverIfNeeded();
```

## Advanced inspection

Advanced code can inspect the lower-level mirror directly:

```cpp
auto confidence = stepper.registers.storedConfidence(
    tmc51x0::Registers::ChopconfAddress);
```

Readable configuration registers can also be refreshed explicitly:

```cpp
bool ok = stepper.resyncReadableConfiguration();
```
