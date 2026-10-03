# Lab log

What was tried on the real device, what happened, and what was changed because of it.
Newest first. Raw recordings stay in `data/` on the lab PC (never committed).

## 2026-10-02 (evening, Mac): simulator check of the lab session's work

No device. Everything checked against the simulator (tools/hd_mock).
- **Mac/Linux 3D window was never wired up.** viz.h expected CMake to fetch GLFW, but CMakeLists.txt
  didn't, so every 3D app only printed "needs a 3D window". CMake now downloads GLFW 3.4 on non-Windows
  builds. All seven 3D apps open and render on the Mac.
- **New test hook:** `VIZ_SWEEP=1` moves the simulated hand along a figure-eight through the room
  (button held 1 s in every 4), so an unattended run actually touches everything.
- **Sweep results (20 s per app):** 1000 Hz throughout, no NaN, no safety trips, force never above the
  3.68 N limit. Force never *rose* faster than 65 mN per tick, except one 0.49 N step when the stylus hit
  a wall at 430 mm/s (wall damping, as designed). The large one-tick changes were all *releases*
  (force to zero): re-centering the room with button 1, or sliding off an object's edge while pressing
  hard. In surfaces the scripted hand pushed past the force limit 30 mm into the friction block and
  released 3.68 N when it slid out the side; a person pressing that hard would feel the same lunge as
  slipping off a real table.
- **Bug fixed (console.h, Mac/Linux only):** after end of input, `while (keyPressed())` loops in
  position_tracker and position_matching spun forever (caused by the Sept 28 change). End of input is
  now reported exactly once. Windows keyboard input was never affected.
- All 15 programs run to completion in the simulator; position_matching completes 6/6 trials.
- analyze.py now summarises reach_game `_trials.csv` (by direction and by block).

## 2026-10-02 — first session on the lab PC (first real build, first real run)

**Setup found on the lab PC**
- Device: **"Premium HID"** (3D Systems), serial 231201270, 6 DOF in / 6 DOF out.
  37.27 N peak, **6.14 N continuous**, max stiffness 1.0 N/mm, max damping 0.005 N·s/mm,
  usable workspace x[-80, 80] y[-50, 70] z[-50, 70] mm. Calibration styles: encoder-reset **and** inkwell.
- OpenHaptics 3.5 Developer Edition is installed at `C:\OpenHaptics - Ben\Developer\3.5.0`,
  but the system variable `OH_SDK_BASE` points to `C:\OpenHaptics\Developer\3.5.0`, which does not exist.
- Visual Studio 2022 Community with MSVC 14.44. PHANToM Device Drivers and 3D Systems Touch drivers installed.
- University IT blocks **both** Windows PowerShell and PowerShell 7 (error 0x800704EC, "blocked by group
  policy"). Use Command Prompt. Git for Windows is installed per-user.

**Build problems and fixes** (all in `build.bat`)
1. `OH_SDK_BASE` set but stale -> "OpenHaptics not found". build.bat now falls back to the standard path and
   to `C:\OpenHaptics - Ben\...` when the variable points nowhere.
2. Hundreds of `error C4430 ... 'HDAPI'` in `hdDevice.h`. The SDK headers only define `HDAPI` when `WIN32` is
   defined; Visual Studio projects add it, plain `cl` on x64 only defines `_WIN32`. Added `/DWIN32`.
3. "Windows cannot find powershell.exe" popup on every build: VS's `vcvars64.bat` launches PowerShell to send
   telemetry. build.bat now sets `VSCMD_SKIP_SENDTELEMETRY=1`.

**Run results**
| Program | Result |
|---|---|
| device_check | First run: `[HD error] encoder reset: HD_INVALID_OPERATION (0x0102)`. This Premium refuses the encoder reset. The SDK's own Calibration example prefers inkwell over encoder reset when both are offered; `phantom.h` now does the same (encoder reset only on devices that offer nothing else; inkwell/auto run with the scheduler briefly started, forces off). Second run: **Calibration status: OK** immediately, servo **1000 Hz**, force limit 3.68 N. |
| axis_check | **PASS** +x right, +y up, +z toward you. Button 1 = bit 0x1; no second button seen. Ranges suggest the zero point is offset (y never above -15 mm with the stylus "in the middle"), see open items. |
| force_direction | +x, +y, -y, -z PASS; -x and +z UNCLEAR (stylus barely moved). The CSV shows the commanded force was correct for all six (e.g. -x sent -0.30 N in x), and the opposite pulse on the same axis passed, so no flipped motor: the stylus was held too firmly. |
| virtual_box | Solid, no buzz (k = 0.2 N/mm). |
| haptic_playground | Sphere good. Cube jittered: it pushed out through the *nearest* face every tick, which flips between faces near edges. Fixed by keeping the face the stylus entered by (`viz::BoxProxy`). The ball was never reached (recording: closest approach 27 mm); moved it to the front. |

**Open items**
1. Position zero looks offset by roughly 60–70 mm in y (and the usable box is bigger than reported). The device
   said "calibrated" without asking for the inkwell, so it may be using a stored calibration. Check with
   PHANToM Test / Phantom Configuration, or put the stylus in the inkwell and see if positions jump. All force
   programs work relative to where the stylus is when forces start, so this is safe, but absolute positions in
   recordings are shifted until it is understood.
2. The 60 % default force limit is 3.68 N on this device (much stronger than the 1.4 N Premium 1.5 the code was
   first written for). Fine so far; lower `safety.maxForce` for participants if it feels too strong.
3. damping_field not run yet.
