#pragma once
// adl_core.h - task switching, trial timing and the protocol for the ADL simulator.
//
// One AdlCore owns all seven task models. step() runs once per servo tick (1 kHz):
// it steps the active model, times the trials, and returns the force for the hand.
// No graphics, no allocation, no printing, so it is also driven directly by the
// offline self-test (main.cpp --selftest).
//
// Timing follows the paper (time from onset to cessation of the task force):
//   first action  (IN / CLOSE / PRESS): starts when the task force first exceeds
//                 0.05 N, stops when the goal is reached (seated, closed, clicked...)
//   second action (OUT / OPEN): starts when the button grips the object again, stops
//                 when it is out / open.
// One "cycle" = first + second action (just the press for the remote).

#include <cmath>

#include "adl_models.h"

namespace adl {

struct Result {
    int task = 0, dir = 0;      // dir 0 = first action (in/close/press), 1 = second (out/open)
    int cycle = 0, rep = 0;     // cycle counter (all tasks), repetition within this task
    bool protocol = false;
    double tStart = 0, tGoal = 0, time = 0;
    double peakSim = 0;         // peak simulated object force during the action, N (incl. pushing on walls)
    double peakProfile = 0;     // peak while the object moved along its task profile, N
    double peakHand = 0;        // peak force on the hand, N
    double path = 0;            // stylus path length during the action, mm
    int jams = 0;               // ticks spent jammed by rotation (shown as seconds / 1000)
};

const int kProtoOrder[NTASK] = {FORK, KEY, PLUG, VZIP, HZIP, ATM, REMOTE};  // table order
const int kProtoReps = 3;
const int kRing = 16;

// Everything the screen and the autopilot need, copied out of the servo thread.
struct View {
    int task = KEY, stage = HAND;
    bool protoActive = false, protoDone = false;
    int protoIdx = 0, rep = 0, cycle = 0;
    double pauseLeft = 0;       // protocol: seconds until the next task
    bool rollOn = true, jam = false;
    double roll = 0;            // rad, relative to the trial start
    bool timing = false;
    int timingDir = 0;
    double timer = 0;
    double objF = 0, handF = 0, gainRamp = 0, gain = 1;
    Vec3 p;                     // stylus (room coords)
    // object state for drawing
    bool held = false;
    Vec3 objTip, handle;
    double depth = 0;
    int atm = 0;
    double zipS = 0;
    Vec3 tabEnd;
    double capX = 0;
    bool clicked = false;
    Fork::Dent dents[32];
    int nDents = 0;
    // results ring (newest at (nResults-1) % kRing)
    Result ring[kRing];
    int nResults = 0;
};

struct AdlCore {
    Insertion key, plug, atm;
    Fork fork;
    Zipper vzip, hzip;
    Remote remote;

    int task = KEY;
    int pending = -1;           // task switch requested (applied on the next tick)
    bool protoActive = false, protoDone = false;
    int protoIdx = 0;
    int rep = 0, cycle = 0;
    double pauseLeft = 0;
    bool rollOn = true;
    double roll0 = 0, rollNow = 0;
    bool rollZeroPending = true;
    double gainRamp = 0;        // 0 -> 1 over 0.5 s after a task switch / reset
    double forceGain = 1.0;

    // timing
    bool timing = false;
    int timingDir = 0;
    double t0 = 0, peakSim = 0, peakProfile = 0, peakHand = 0, path = 0;
    int jamTicks = 0;
    Vec3 lastP;
    bool haveLastP = false;
    double tNow = 0;

    Result ring[kRing];
    int nResults = 0;

    // last outputs (for the view)
    double lastObjF = 0, lastHandF = 0;
    bool lastJam = false;
    Vec3 lastPos;

    AdlCore() {
        key.setup(KEY); plug.setup(PLUG); atm.setup(ATM);
        fork.setup(); vzip.setup(true); hzip.setup(false); remote.setup();
    }

    void resetTask(int t) {
        switch (t) {
        case KEY: key.reset(); break;
        case PLUG: plug.reset(); break;
        case ATM: atm.reset(); break;
        case FORK: fork.reset(); break;
        case VZIP: vzip.reset(); break;
        case HZIP: hzip.reset(); break;
        default: remote.reset();
        }
    }
    void setTask(int t) {
        task = t; resetTask(t);
        gainRamp = 0; timing = false; rollZeroPending = true; rep = 0;
    }
    // Room placed again (ENTER): restart the current cycle only, keep protocol progress.
    void restartCycle() { resetTask(task); gainRamp = 0; timing = false; rollZeroPending = true; }

    void startProtocol() {
        protoActive = true; protoDone = false; protoIdx = 0; pauseLeft = 0;
        pending = kProtoOrder[0];
    }
    void onKey(int vk) {  // servo thread (via runInServo)
        if (vk >= '1' && vk <= '7') { protoActive = false; pending = vk - '1'; }
        if (vk == 'A') { if (protoActive) protoActive = false; else startProtocol(); }
        if (vk == 'R') rollOn = !rollOn;
        if (vk == 'C') restartCycle();
        if (vk == 'N' && protoActive) { rep = kProtoReps; pauseLeft = 0.01; }
    }

    int stage() const {
        switch (task) {
        case KEY: return key.stage;
        case PLUG: return plug.stage;
        case ATM: return atm.stage;
        case FORK: return fork.stage;
        case VZIP: return vzip.stage;
        case HZIP: return hzip.stage;
        default: return remote.stage;
        }
    }

    void record(int dir, double t) {
        Result& r = ring[nResults % kRing];
        r = Result();
        r.task = task; r.dir = dir; r.cycle = cycle; r.rep = rep; r.protocol = protoActive;
        r.tStart = t0; r.tGoal = t; r.time = t - t0;
        r.peakSim = peakSim; r.peakProfile = peakProfile; r.peakHand = peakHand; r.path = path; r.jams = jamTicks;
        nResults++;
        timing = false;
    }
    void startTiming(int dir, double t) {
        timing = true; timingDir = dir; t0 = t; peakSim = 0; peakProfile = 0; peakHand = 0; path = 0; jamTicks = 0;
    }
    void cycleDone() {
        cycle++; rep++; rollZeroPending = true;
        if (protoActive && rep >= kProtoReps) pauseLeft = 2.0;
    }

    // One servo tick. in.p/in.v/in.btn/in.press/in.t/in.dt/in.kc/in.bc/in.cursorR filled by the caller;
    // gimbalRoll = raw stylus twist (rad). Returns the force on the hand.
    Vec3 step(Input in, double gimbalRoll, int* phaseOut = nullptr) {
        tNow = in.t;
        if (pending >= 0) { setTask(pending); pending = -1; }
        // protocol: pause between tasks, then advance
        if (pauseLeft > 0) {
            pauseLeft -= in.dt;
            if (pauseLeft <= 0) {
                pauseLeft = 0;
                if (++protoIdx >= NTASK) { protoActive = false; protoDone = true; }
                else setTask(kProtoOrder[protoIdx]);
            }
        }
        rollNow = gimbalRoll;
        if (rollZeroPending && stage() == HAND) { roll0 = gimbalRoll; rollZeroPending = false; }
        in.roll = gimbalRoll - roll0;
        in.rollOn = rollOn;
        in.gain = forceGain;
        gainRamp = std::fmin(1.0, gainRamp + in.dt / 0.5);

        Output o;
        bool hasOut = true;
        switch (task) {
        case KEY: o = key.step(in); break;
        case PLUG: o = plug.step(in); break;
        case ATM: o = atm.step(in); break;
        case FORK: o = fork.step(in); break;
        case VZIP: o = vzip.step(in); break;
        case HZIP: o = hzip.step(in); break;
        default: o = remote.step(in); hasOut = false; break;
        }
        Vec3 f = o.f * gainRamp;
        if (!f.finite()) f = Vec3();
        // soft cap (the device clamp at 3.68 N should never be reached)
        double fn = f.norm();
        if (fn > 3.5) f = f * (3.5 / fn);
        o.objF = std::fmin(o.objF * gainRamp, f.norm());  // log what is actually delivered

        int st = stage();
        bool jam = (task == KEY && key.jam) || (task == PLUG && plug.jam) || (task == ATM && atm.jam);
        // --- timing ---
        if (paused()) {
            // between protocol tasks: nothing is timed
        } else {
            if (!timing && o.objF > 0.05 && (st == HAND || st == PUSHING) && !o.ev.goalIn) startTiming(0, in.t);
            if (o.ev.regrip) startTiming(1, in.t);
            if (timing) {
                peakSim = std::fmax(peakSim, o.objF);
                if (o.onProfile) peakProfile = std::fmax(peakProfile, o.objF);
                peakHand = std::fmax(peakHand, f.norm());
                if (haveLastP) path += (in.p - lastP).norm();
                if (jam) jamTicks++;
            }
            if (o.ev.goalIn) {
                if (!timing) startTiming(0, in.t);  // e.g. the ATM finishes after the hand let go
                if (timingDir == 0) record(0, in.t);
                if (!hasOut) cycleDone();
            }
            if (o.ev.goalOut && timing && timingDir == 1) { record(1, in.t); cycleDone(); }
        }
        lastP = in.p; haveLastP = true;
        lastObjF = o.objF; lastHandF = f.norm(); lastJam = jam; lastPos = in.p;
        if (phaseOut) *phaseOut = 10 * (task + 1) + st;  // CSV phase: task*10 + stage (11..77)
        return f;
    }
    bool paused() const { return pauseLeft > 0; }

    void fill(View& v) const {
        v.task = task; v.stage = stage();
        v.protoActive = protoActive; v.protoDone = protoDone; v.protoIdx = protoIdx;
        v.rep = rep; v.cycle = cycle; v.pauseLeft = pauseLeft;
        v.rollOn = rollOn; v.jam = lastJam; v.roll = rollNow - roll0;
        v.timing = timing; v.timingDir = timingDir; v.timer = timing ? tNow - t0 : 0;
        v.objF = lastObjF; v.handF = lastHandF; v.gainRamp = gainRamp; v.gain = forceGain;
        v.p = lastPos;
        v.held = false; v.depth = 0; v.atm = 0;
        const Insertion* ins = task == KEY ? &key : task == PLUG ? &plug : task == ATM ? &atm : nullptr;
        if (ins) {
            v.held = ins->cp.alpha > 0.5;
            v.objTip = ins->cp.attached() ? ins->proxyTip : ins->objTip();
            v.handle = ins->handle();
            v.depth = ins->sp;
            v.atm = ins->atm;
        } else if (task == FORK) {
            v.held = fork.cp.alpha > 0.5;
            v.objTip = fork.cp.attached() ? fork.proxyTip : fork.objTip();
            v.handle = fork.handle();
            v.depth = fork.sp;
            v.nDents = fork.nDents;
            for (int i = 0; i < fork.nDents; ++i) v.dents[i] = fork.dents[i];
        } else if (task == VZIP || task == HZIP) {
            const Zipper& z = task == VZIP ? vzip : hzip;
            v.held = z.cp.alpha > 0.5;
            v.zipS = z.s; v.tabEnd = z.tabEnd; v.handle = z.handle();
        } else {
            v.capX = remote.x; v.clicked = remote.clicked;
        }
        int first = nResults > kRing ? nResults - kRing : 0;
        for (int i = first; i < nResults; ++i) v.ring[i % kRing] = ring[i % kRing];
        v.nResults = nResults;
    }
};

}  // namespace adl
