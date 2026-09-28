# Project context

Summarised from the lab notes (BG Notes, Matthew & Ben Notes, "Soft Robots and
Muscle Endpoint Mechanics") so that anyone new, human or AI, can catch up.

## Goal

Use a **PHANToM Premium** haptic device to **assess and eventually train hand /
finger movement after stroke or surgery**. Longer term: a **fingertip interface**
(thimble / end effector) so the device measures and drives a finger rather than a
stylus. This ties into the PI's interest in **muscle endpoint force mechanics** of a
single digit and soft robotic hand devices.

## The hardware

* PHANToM Premium (3D Systems / SensAble), **6-DOF** according to the Feb 24 meeting
  with Josh (3D Systems). `device_check` prints the model, DOF, force and
  stiffness limits. Record them here once seen: `____`.
* Uses **encoder-reset calibration**, so it needs calibrating at every power-up.
* **Touch drivers do not work with the Premium.** Use the PHANToM Device Driver
  (v5.1.7) + OpenHaptics 3.5.0.
* One axis was getting squeaky (Nov 2025), so check it before long sessions.
* The PHANToM does **not have a force sensor**. The "force" we log is the force
  we *command*. When the user is pressing against a virtual wall/spring and not
  moving, that equals the force they apply (Newton's 3rd law at equilibrium),
  which is how "contact force" gets measured with this device.
* Force budget is small: continuous force ~1.4 N on a Premium 1.5 (check the real
  number). That's enough for guiding a relaxed finger and for walls/springs. It's not
  enough to resist a strong grip.

## End-effector work (Spring 2026)

* Flexible thimble prototypes (PLA, 4 slots), scaled 110–130 % for fit. TPU and
  nylon 11/12 suggested for flexibility and durability. O-ring retention.
* Attachment moved higher up the arm to reduce droop/torque. Two-finger pinch
  variant (Apr 28). CAD files are in the team Google Drive.
* Open questions: gimbal for the thimble (none exists for our device yet), whether
  the stylus's gimbal encoders can still be read with a thimble attached, and a
  wiring schematic from 3D Systems before taking anything apart.
* **Software note:** when the end effector changes, the **tip point moves**.
  `HD_CURRENT_POSITION` is the gimbal centre as configured. If the fingertip sits
  some distance from it, the fingertip position = position + rotation × offset,
  using `HD_CURRENT_TRANSFORM`. Not implemented yet, so measure the offset first.

## Software status

| Thing | Status |
|---|---|
| Drivers + OpenHaptics + VS2022 on lab PC | working (Nov 2025) |
| SDK HL/HD examples | build & run (interesting ones: Simple Rigid Body Dynamics, Point Manipulation, SimplePinch, Simple Haptic Scene) |
| Original PositionTracker | worked, 1 sample / 5 s (see `legacy/`) |
| This repo: `device_check`, `position_tracker`, `position_matching`, `template` | written Sept 2026, **tested only on the simulator** so far |
| Remote access | Microsoft Remote Desktop from a Mac works (Matthew, Apr 2026) |
| Other repo | `matthewdkim2025/research-2026` holds the SDK files themselves |

## Candidate experiments (roughly easiest first)

1. **Free-movement kinematics.** Record reaching/tracing with `position_tracker`
   and compute speed, path length and smoothness (SPARC, sub-movement count). These
   are standard post-stroke recovery metrics. No forces needed.
2. **Proprioceptive position matching** (`position_matching`). The device
   passively moves the finger to a target, and the participant reproduces it with
   vision blocked. Output is absolute, constant and variable error. Proprioceptive
   deficits are common after stroke. (Ben's idea, Nov 2025.)
3. **Force-matching / endpoint force production.** Push into a virtual wall to
   reach target force levels in different directions. This measures endpoint force
   capability per direction and links to the muscle endpoint mechanics interest.
4. **Haptic perception thresholds.** Stiffness or force JND with a staircase procedure.
5. **Visuomotor reaching with a display** (centre-out targets). Needs a graphics
   window. Options: the OpenHaptics GLUT utilities, or a separate Python display
   reading the position over a socket.
6. **Force-field adaptation** (e.g. velocity-dependent curl field), for motor
   learning. Needs careful safety tuning.

## References from the notes

* https://jneuroengrehab.biomedcentral.com/articles/10.1186/s12984-015-0020-x
* https://pmc.ncbi.nlm.nih.gov/articles/PMC3844272/
* https://pmc.ncbi.nlm.nih.gov/articles/PMC7014637/
* Soft robots for hand function: https://pmc.ncbi.nlm.nih.gov/articles/PMC11111837/ ,
  https://www.liebertpub.com/doi/epub/10.1089/soro.2019.0135 , review https://pubmed.ncbi.nlm.nih.gov/39444223/
* Smoothness metric: Balasubramanian et al. 2015, "On the analysis of movement
  smoothness", J NeuroEng Rehabil (SPARC).
* Minimum-jerk trajectories: Flash & Hogan 1985.

## Data handling

`data/` is git-ignored on purpose. Participant recordings should never be pushed
to GitHub. Use coded IDs (`--id P01`), not names, and follow whatever the IRB
protocol says about storage.
