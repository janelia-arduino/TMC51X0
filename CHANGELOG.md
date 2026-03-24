# Changelog

## 4.0.0

`TMC51X0` v4 is a release-focused cleanup and hardening update rather than a
transport redesign.

Highlights:

- typed UART result surface via `tmc51x0::Result<T>` and `tmc51x0::UartError`
- `uartBus()` as the preferred family-style async UART alias, with
  `uartInterface()` retained for compatibility
- explicit reset-recovery and register-mirror semantics
- conservative recovery helpers including `recoverIfNeeded()`,
  `recoverIfUnhealthy()`, and `readHealthStatus()`
- stronger stall-home completion semantics so ambiguous stops are not silently
  accepted as successful homing
- expanded release-facing documentation for setup, UART, migration, recovery,
  and hardware validation
- example-selection tooling that builds, uploads, and tests examples without
  editing `platformio.ini`

Known validation status at release time should be read from
`docs/HARDWARE_VALIDATION.md`.
