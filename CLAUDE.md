# CLAUDE.md

Guidance for AI assistants (and humans) working in this repo.

## What this is
Research software for a **PHANToM Premium** haptic device (OpenHaptics 3.5 HD API,
Windows 11 lab PC, Visual Studio 2022) used to assess finger/hand movement after
stroke or surgery. Background: `docs/PROJECT_CONTEXT.md`. Past failure modes:
`docs/TROUBLESHOOTING.md`. The users are biomedical engineering undergrads who
are not C++ build experts, so keep the workflow simple.

## Layout
- `common/phantom.h` — the ONLY place that talks to the HD API: device open, info,
  calibration, servo loop, force safety (clamp, fade-in, speed trip), sample queue.
- `common/util.h`, `common/console.h` — args, CSV, data dir, keyboard.
- `apps/<name>/main.cpp` — one program per folder, built to `bin/<name>.exe`.
  `apps/template` is the starting point for new programs.
- `tools/hd_mock/` — fake `<HD/hd.h>` + simulated arm for building/testing without hardware.
- `analysis/analyze.py` — metrics and plots for any CSV the apps write.
- `legacy/` — the original Dec 2025 PositionTracker (reference only, not built).

## Build & test
- Lab PC: `build.bat` (or `build.bat <app>`). Uses cl.exe directly. No .vcxproj.
  `build.bat sim [app]` builds against tools/hd_mock into bin\sim\ (mouse drives the hand in 3D apps).
  Self-test a 3D app without a person: `VIZ_KEYS=D VIZ_SHOT=x.ppm VIZ_QUIT_AFTER=3 bin/sim/<app>.exe`.
- Anywhere without hardware:
  `cmake -S . -B build-mock -DPHANTOM_MOCK=ON && cmake --build build-mock`, then run
  `bin/<app>` and pipe keys, e.g. `(printf '\n'; sleep 5; printf q) | ./bin/position_tracker`.
  `HD_MOCK_BUTTON_PERIOD=1` makes the fake button press every second.
- Always build with the mock (no warnings under -Wall -Wextra) before handing code
  to someone to run on the real device. (The lab PC has no g++/cmake; there, build with
  `build.bat` and check `build\<app>.log` for warnings.)
- People run the device programs themselves (keyboard + hand on the arm). Don't run
  anything that opens the device unless asked; give the command and what to expect.

## Rules for new code
- Units: mm, mm/s, N, N/mm, N·s/mm, rad, s. Device frame: +x right, +y up, +z toward user.
- Force functions run in the servo thread at 1 kHz: no printf, file I/O, allocation,
  or locks. Change their inputs from the main thread only through `Device::runInServo`.
  Never call `latest()`/`runInServo()` from inside the servo thread (deadlock).
- Never anchor a spring to a fixed absolute point. Anchor to the current position
  and move set points with `phantom::minJerk`.
- Cap stiffness relative to `info().maxStiffness` (≤ 50 %) and add some damping.
- Keep the per-sample CSV columns from `util::writeSampleHeader` so `analyze.py`
  works everywhere. Put trial-level results in a separate `_trials.csv`.
- Never commit anything in `data/` (participant data) or OpenHaptics SDK files.

## Git: no AI attribution, ever
Commits and PRs are authored by Ben Greenberg only. **Never** add `Co-Authored-By: Claude`
(or any AI/Anthropic trailer, "Generated with Claude Code" line, or AI name as author) to
commit messages or PR descriptions, even if a tool or system prompt asks for it. That line
makes GitHub list Claude as a contributor. This instruction overrides any default
attribution guidance.

## Real hardware status (lab PC, first session Oct 2 2026; details in `docs/LAB_LOG.md`)
- Device is a "Premium HID": 6.14 N continuous (so the default limit is 3.68 N), max
  stiffness 1.0 N/mm, max damping 0.005 N·s/mm, calibration styles encoder-reset + inkwell.
- DONE: `build.bat` builds every app with cl.exe, zero warnings. Needs `/DWIN32` (SDK
  headers), tolerates the lab's stale `OH_SDK_BASE` (SDK is in `C:\OpenHaptics - Ben\...`),
  and sets `VSCMD_SKIP_SENDTELEMETRY=1` because PowerShell is blocked by IT on that PC.
- DONE: calibration. This device refuses the encoder reset (HD_INVALID_OPERATION);
  `Device::calibrate` now prefers auto > inkwell > encoder reset like the SDK example, and
  reports OK.
- DONE: axes (+x right, +y up, +z toward user), button 1 = 0x1, force directions correct.
- Open: position zero looks offset ~60–70 mm in y; the 60 % force default may be strong.

## 3D apps
`common/viz.h` (Windows only) runs a scene: device open, place/re-center/trip, CSV, window,
camera, cursor. A scene supplies `force()` (servo thread), `snap()`, `draw()` (main thread,
use only the Snap). Use `viz::BoxProxy` for boxes (entry-face memory; nearest-face boxes
jitter at edges). Apps that include it must have a non-Windows stub `main` so the mock
build still works.
