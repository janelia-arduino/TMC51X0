# TMC51X0 Codex guidance

This file is the canonical project guidance for Codex in this repository.

## Session start

Read this file first.

If `.codex/handoff.local.md` exists, read it before planning or editing. Treat it as current local context, not permanent project policy.

If the task is multi-step, risky, likely to span more than one session, or changes documentation workflow / architecture, read `.codex/PLANS.md` before editing and keep it updated as work progresses.

For this repository, the most useful files to inspect early are usually:

- `README.md`
- `.metadata/README.org`
- `MIGRATION.md`
- `docs/UART.md`
- `docs/RECOVERY.md`
- `tools/version_sync.py`
- `pixi.toml`
- `.github/workflows/ci.yml`

For core behavior changes, also inspect the relevant native tests under `test/native/` before editing.

## Project state and priorities

`TMC51X0` is already close to release-ready. Default posture: make small, reviewable patches and avoid broad redesign.

Current priorities:

1. README workflow cleanup.
   - Current canonical README source is still `.metadata/README.org`.
   - Preferred end state is a root `README.org` plus committed generated `README.md`.
   - If that source-of-truth moves, update the workflow docs and version-sync tooling in the same patch.
2. Expand the README so it clearly explains:
   - SPI vs UART setup
   - caller-owned transport setup
   - optional TX/RX-enable pin behavior
   - typed `Result<T>` / `UartError` semantics
   - blocking vs non-blocking UART usage
   - reset-recovery / mirror semantics
   - TMC5130A vs TMC5160A differences
   - build / upload / example workflow
3. Rewrite `MIGRATION.md` into a real v3 -> v4 migration guide for both humans and LLM-assisted refactors.
4. Add `docs/HARDWARE_VALIDATION.md` and use real bench results to drive only the final firmware fixes.
5. Finish with minor family-style and tooling consistency polish.

Lower priority:

- naming / docs / workflow consistency with `TMC2209`

Not a priority:

- transport redesign
- speculative API churn
- automatic reconstruction of lost in-flight motion after reset

## Architecture boundaries

Preserve these repo-level decisions unless the task explicitly requires changing them:

- keep the modular split between registers, interfaces, driver, controller, encoder, converter, and printer
- keep SPI / UART lifecycle caller-owned
- do not take hidden ownership of `Serial`, `Stream`, or `SPI` setup/teardown
- keep optional `enable_txrx_pin` behavior opt-in and safe by default
- if `enable_txrx_pin` is not configured, do not touch direction-control pins
- keep register/protocol code explicit and portable
- no C++ bitfields
- no union punning
- no debug printing in the library core

Treat `TMC51X0` as the cleaner family reference repository. If `TMC2209` is available through `.codex/workspace.dirs`, use it as a read-only style / naming / tooling reference, not as a reason to force a worse architecture onto `TMC51X0`.

## Mirror, UART, and recovery rules

Preserve these semantics unless the task explicitly requires changing them and you also update tests/docs:

- `tmc51x0::Result<T>` / `tmc51x0::UartError` are the typed UART result surface
- `TMC51X0::uartBus()` is the preferred family-style alias for async UART work
- `TMC51X0::uartInterface()` remains for compatibility
- blocking UART wrappers flow through the same poll-driven UART engine as the non-blocking path
- the software-side register mirror is best-known intended state with confidence metadata, not guaranteed chip truth
- mirror entries should update only after successful transport outcomes
- chip-aware reset defaults for `TMC5130A` vs `TMC5160A` must remain intact
- reset recovery is conservative and configuration-oriented
- high-level desired configuration is replayed after reset recovery
- raw `registers.write(...)` calls are not automatically tracked as replayable desired state
- recovery restores safe configuration state; it does not auto-resume arbitrary motion history

After suspected drift or reset, the intended flow is:

1. mark uncertainty with `notePossibleMirrorDrift()` if needed
2. perform any required external reset / power-cycle outside the library
3. call `recoverIfNeeded()` or `recoverFromDeviceReset()`
4. use `resyncReadableConfiguration()` only when the task truly needs explicit post-recovery verification

Be explicit in docs whenever behavior is conservative, optimistic, or only best-effort.

## Coding conventions

Follow the existing code style and `.clang-format` configuration. Do not manually restyle unrelated code.

Prefer the smallest correct change that satisfies the request.

Preserve backward compatibility unless the task explicitly requires a break.

When behavior changes, update the relevant docs and native tests in the same patch.

When touching example selection or build workflow, keep `tools/pio_task.py`, `platformio.ini`, `README.md`, and CI expectations aligned.

When touching the README workflow, do not update only one of the files. Keep these consistent in the same patch:

- canonical Org source (`.metadata/README.org` today, root `README.org` in the desired end state)
- `README.md`
- `tools/version_sync.py`
- relevant `pixi.toml` task descriptions
- any README workflow documentation

Do not change version metadata casually.

Do not add new dependencies unless they clearly simplify a real repo need.

## Validation

Prefer Pixi commands if Pixi is available:

- Version metadata check: `pixi run check-version`
- Formatting check: `pixi run format-check`
- Native tests: `pixi run test`
- Representative SPI build: `pixi run build examples/SPI/TestCommunication pico`
- Representative UART build: `pixi run build examples/UART/TestCommunication pico`
- Additional representative build: `pixi run build examples/SPI/TestCommunication teensy40`

Direct equivalents when using Python + PlatformIO without Pixi:

- Version metadata check: `python tools/version_sync.py check`
- Formatting check: `python tools/clang_format_all.py --check`
- Native tests: `python tools/pio_task.py test --env native`
- Representative SPI build: `python tools/pio_task.py build --example examples/SPI/TestCommunication --env pico`
- Representative UART build: `python tools/pio_task.py build --example examples/UART/TestCommunication --env pico`
- Additional representative build: `python tools/pio_task.py build --example examples/SPI/TestCommunication --env teensy40`

Useful extra build coverage when the patch could affect portability:

- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env esp32`
- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env giga_r1_m7`
- `python tools/pio_task.py build --example examples/SPI/TestCommunication --env nanoatmega328`

CI currently covers:

- version metadata check
- native tests
- `examples/SPI/TestCommunication` on `pico`, `teensy40`, `esp32`, `giga_r1_m7`, `nanoatmega328`
- `examples/UART/TestCommunication` on `pico`, `teensy40`

There is no separate repo lint or type-check step beyond formatting / metadata validation. Do not invent a fake lint or type-check command; state clearly when those categories are not applicable.

If a command is unavailable in the current environment, say what you could not run and why.

## Risky areas

Changes in these areas deserve extra care and usually need tests/docs updates in the same patch:

- `src/TMC51X0/UartEngine.hpp`
- `src/TMC51X0/UartInterface.*`
- `src/Result.hpp`
- `src/TMC51X0/TMC51X0.cpp`
- `src/TMC51X0/Registers.cpp`
- `src/Registers.hpp`
- `tools/pio_task.py`
- `tools/version_sync.py`
- `README.md` / `.metadata/README.org` / future `README.org`

## Plans

Use `.codex/PLANS.md` for work that is multi-step, risky, or likely to span multiple sessions.

Keep the current goal, milestones, assumptions, validation notes, and progress log current.

Before ending a longer session, leave a concise note in `.codex/handoff.local.md` if the local state or next step changed.

## Patch summary rule

For each completed task, report:

- what changed
- which files changed
- what validation actually ran
- what was not run
- remaining risks, assumptions, or known limitations
