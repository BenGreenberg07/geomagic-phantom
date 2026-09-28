# Why programs "didn't work" or "started spazzing" — and how this repo avoids it

This is based on the `POSITIONTRACKER` folder from December 2025 (build logs,
`.vcxproj`, source) and on how the OpenHaptics HD API behaves.

## Part 1: "I uploaded something new and it didn't work" — the build setup

What the build logs show:

| Evidence (from `x64/Debug/*.log`, `*.tlog`) | What it means |
|---|---|
| Project lives in `C:\OpenHaptics\Developer\3.5.0\examples\HD\console\POSITIONTRACKER` | The new program was made by copying an SDK example folder in place. |
| `warning MSB8028: The intermediate directory (x64\Debug\) contains files shared from another project (FrictionlessPlane.vcxproj)` | Two projects (FrictionlessPlane and PositionTracker) write into the **same** build folder. Visual Studio says outright that clean/rebuild can then misbehave. `FrictionlessPlane.exe` sits right next to `PositionTracker.exe`. |
| Object files go into `PositionTracker___Win32_Debug\` while the exe is built for x64 | The project still carries its original 32-bit settings from an old (2015 / VC6-upgrade) template. It works, but it's confusing. |
| 8 build configurations (`Debug`, `Release`, `DebugAcademicEdition`, … × Win32/x64) | Picking the wrong one quietly builds against different libraries or into a different folder. |
| Include/lib paths hard-coded to `C:\OPENHAPTICS\DEVELOPER\3.5.0\...` plus `.\INCLUDE` | Move the folder and it stops building. |

The usual ways this goes wrong:
* **You run an old exe.** The build fails (or builds into another folder), but an
  older `.exe` is still there, so you double-click it and nothing changed.
* **Visual Studio builds the wrong project.** In a big `.sln`, the "startup project"
  or the one you right-click isn't the one you edited.
* **Copied example = copied settings.** The name changes but the project GUID,
  output folders and the `.vcxproj.user` settings stay the same.

**What this repo does instead:** there are no `.sln`/`.vcxproj` files. Every
program is one `apps/<name>/main.cpp`. `build.bat` finds the compiler,
compiles each one with the same flags, and writes `bin\<name>.exe`. If a build
fails you get the compiler error and a `BUILD FAILED` message — it doesn't
quietly leave an old exe behind for you to run. The repo lives outside the
OpenHaptics folder, so the SDK stays untouched.

## Part 2: "It started spazzing" — force rendering

Haptic code runs in a **servo loop at 1000 Hz**. Whatever force you return goes
straight to the motors 1 ms later. Small mistakes become violent motion. Common
causes, roughly most-likely first:

1. **Premium not calibrated this power-up.** Premium encoders are relative, so
   after every power-on the arm has to be held in its reset position while the
   encoders are zeroed. Skip that and every position is offset, so a "spring to
   the centre" or a "wall at y=0" ends up somewhere unexpected and the arm
   gets yanked there. → every app in this repo runs the encoder reset at startup
   (see `Device::calibrate`), and apps that drive forces refuse to run uncalibrated.
2. **Springs anchored to a fixed point.** For example, `F = k * (origin - pos)`.
   The moment forces turn on, the arm is 100 mm away and gets the full force in
   one tick. → anchor to *where the arm is* when forces start, move targets
   along smooth (minimum-jerk) paths, and fade forces in over 1 s (done
   automatically by `phantom.h`).
3. **Wrong units / stiffness too high.** HD uses **millimetres and newtons**,
   so stiffness is **N/mm**. `k = 1` sounds small but is 1000 N/m. With
   stiffness above what the device can render (`HD_NOMINAL_MAX_STIFFNESS`), a
   wall buzzes or chatters. → `phantom.h` prints the device limits. Apps cap k
   at 20–50 % of the maximum and add a little damping (N·s/mm).
4. **No force limit.** The Premium 1.5 can only sustain about 1.4 N (peak
   ~8.5 N). If you ask for more, the driver raises
   `HD_EXCEEDED_MAX_FORCE`/`..._VELOCITY` and cuts forces, so it jerks and
   then goes limp. → forces are clamped to 60 % of continuous by default.
5. **Threading bugs.** In the original tracker, the servo thread writes
   `currentPos` and the GLUT thread reads it with no synchronisation (torn
   reads). For position logging this only gives the odd wrong sample. For a
   force target written by the UI thread, though, a half-written vector means
   a force spike. → only touch servo-thread data through
   `Device::runInServo()` / `latest()`, which use `hdScheduleSynchronous`
   exactly as the SDK intends.
6. **Two programs using the device.** Leaving PHANToM Test or an example running
   while you start your program gives init errors or fighting callbacks.
   → `Device::open` prints a clear message.
7. **Slow or blocking code in the callback** (`printf`, file writes,
   allocation). The servo loop misses deadlines, which feels like rough or
   gritty forces and can end in `HD_SCHEDULER_...` errors. → apps log through
   a lock-free queue and write files on the main thread.

## Part 3: the original PositionTracker

It worked for what it did, but for movement analysis:
* it logged **one sample every 5 s** (from the GLUT idle loop), so speed,
  smoothness and path can't be computed. Human movement needs ≥100 Hz, and 1 kHz is free.
* no error checks: if `hdInitDevice` failed it went on silently.
* `hdEnable(HD_FORCE_OUTPUT)` in a program that never renders forces.
* no calibration step.

`apps/position_tracker` is the replacement. The original source is kept in
`legacy/PositionTracker.cpp` for reference.

## Quick checklist when something misbehaves

1. Close every other haptics program.
2. Run `bin\device_check.exe` and do the calibration when asked.
3. Move the arm and check the axes: +x right, +y up, +z toward you. Values should
   change smoothly and stay near 0 in the centre of the workspace.
4. Press `S` for the weak test spring. It should pull gently back to where you pressed S.
5. Only then run your own program. If it trips, the message says why (speed, NaN,
   driver force error).
