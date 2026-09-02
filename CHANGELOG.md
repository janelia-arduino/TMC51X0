# Changelog

## 4.1.0

Four TMC5130A correctness fixes, all found on bench hardware while bringing up
a TMC5130A + RP2040 board with 0.15 ohm sense resistors. Three of them could
move a motor that nothing had asked to move.

**`CHOPCONF.vsense` is now reachable.** `Registers::Chopconf` followed the
TMC5160 layout, where bit 17 is reserved; on the TMC5130 that bit is `vsense`
and it selects the full-scale sense resistor voltage, scaling every coil
current. On a board with low-value shunts, leaving it at the reset default
drives about 1.8x the requested current, and `IHOLD_IRUN` is write-only so
nothing reports it. Added `Registers::Chopconf::vsense()`,
`tmc51x0::SenseVoltageMode`, `DriverParameters::withSenseVoltageMode()`, and
`Driver::writeSenseVoltageMode()`. The write is a no-op unless the device model
is `TMC5130A`, since the bit is reserved on the TMC5160.

**`CHOPCONF`'s seeded reset default is device-model aware.** The mirror was
seeded with the TMC5160's `0x10410150` for both parts. Bits 23:20 are `TPFD` on
the TMC5160 and `SYNC` on the TMC5130, where a non-zero value *enables*
chopSync -- so every high-level `CHOPCONF` write, being a read-modify-write of
the mirror, turned chopSync on for a TMC5130. `setDeviceModel()` now also
re-seeds the model-dependent defaults when the model is learned after seeding,
which is what `setupSpi()` does when the caller does not declare it. Values
already written or read back are never overwritten by a re-seed.

**`Controller::initialize()` no longer leaves the motor commanded to turn.**
The default `ControllerParameters` were `VelocityPositiveMode` with
`max_velocity` 10, and `initialize()` applies them -- so the chip was commanded
to turn from the moment the transport came up, about 9.5 microsteps/s at 16 MHz.
The defaults are now `HoldMode` with `max_velocity` 0. `reinitialize()` also no
longer replays a *velocity* ramp mode, because recovery restores configuration
and does not resume motion; a position ramp mode is still replayed faithfully,
and the configured `max_velocity` is left intact either way.

**`VSTOP` and `D1` can no longer be written as zero.** The datasheet forbids
both in positioning mode, "even if V1=0", and the chip does not fault on a
zero -- it crawls at about `VSTOP` and never reaches the target. They are easy
to zero by accident through `controllerParametersRealToChip()`, because one chip
unit of acceleration is 116.4 microsteps/s^2 at 16 MHz and a small real value
rounds down: this struct's own default of 10, passed in real units, produced
`D1` = 0. `writeStopVelocity()` and `writeFirstDeceleration()` now clamp to 1,
which is harmless in velocity mode where neither value is used.

**A reversed encoder can be expressed in one signed number.** `ENC_CONST` is a
signed fixed-point value whose fractional part is always *added* to the integer
part, so -12.8 microsteps per pulse is -13 + 0.2 and never -12 + 0.8. The
documented way to reverse an encoder was "use integer < 0", which makes the
obvious pairing produce -11.2 -- a 12.5% scale error that reads as a working
encoder with a slightly wrong resolution, with nothing to report it. Added
`EncoderParameters::withMicrostepsPerPulse(FractionalMode, int32_t scaled)` and
`Encoder::writeMicrostepsPerPulseScaled()`, which take the value in the
fractional mode's own units (`DecimalMode` -> /10000, `BinaryMode` -> /65536)
and do the split with floor division. The existing two-argument form is
unchanged for callers who want raw register semantics. An encoder mounted on the
far end of a shaft is a common enough condition to be worth an API that cannot
be got wrong.

Also in this release:

- `beginHomeToSwitch()` and `beginHomeToStall()` re-apply the caller's own
  driver and controller configuration instead of default-constructing it.
  Default-constructing silently reset every field the homing sequence does not
  go on to set explicitly -- including the new `sense_voltage_mode`.
- Agent guidance is tool-agnostic: `.codex/` is now `.agents/`, reached through
  `AGENTS.md` and `CLAUDE.md` shims at the repo root.

Behaviour changes to be aware of when upgrading: the `ControllerParameters`
defaults, the velocity-ramp-mode replay, and the `VSTOP`/`D1` clamp. See
`MIGRATION.md`.

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
