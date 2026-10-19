// adl_tasks - haptic simulation of the 7 ADL tasks in Smaby et al. 2004
// ("Identification of key pinch forces required to complete functional tasks",
// J Rehabil Res Dev 41(2):215-224): fork into putty, key in/out, plug in/out,
// vertical and horizontal zipper close/open, ATM card in/out, doorbell remote.
//
// HOW IT WORKS
//   The task object starts in your hand (attached to the stylus, like the robot's
//   end-effector in the paper). Push it in / pull the zipper closed / press the
//   button: you feel that task's force profile, scaled into the arm's limits (see
//   adl_models.h). When the first action is done and you ease off, the object stays
//   where it is (in the lock, outlet, slot, putty, or the zipper left closed). Press
//   the stylus button on it to take hold again, then pull it out / open the zipper.
//   Each action is timed like the paper (onset of task force -> goal) and saved.
//
//   Light rotation: the object twists with the stylus. The key (10 deg), plug
//   (16 deg, the tolerance the paper found) and card (12 deg) jam at the entrance if
//   they are twisted too far; R turns this off. --roll-axis picks the gimbal axis.
//
// KEYS (click the 3D window first)
//   1-7     practice a task: 1 fork, 2 key, 3 plug, 4 vertical zipper, 5 horizontal zipper,
//           6 ATM card, 7 remote button
//   A       start / stop the protocol: all tasks in the paper's order, 3 cycles each
//   N       protocol: skip to the next task      C   restart the current cycle
//   R       rotation tolerance on / off           ENTER  place the room around the stylus
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// OPTIONS
//   --force-gain 1.0   scale every task force (use 0.5 for a first run on the device)
//   --roll-axis 2      gimbal axis used for the stylus twist (0, 1 or 2)
//   --k 0.25           coupling stiffness N/mm (capped at 30% of the device max)
//   --selftest         run every task with a simulated arm and hand, no device needed; exit code 0 = pass
//   --device "<name>"  --skip-calibration  --allow-uncalibrated
//
// OUTPUT (data/)
//   adl_tasks_<stamp>.csv         every servo sample (phase = task*10 + stage, trial = cycle)
//   adl_tasks_<stamp>_trials.csv  one row per timed action, with the robot reference times
//                                 from Table 1, simulated and implied real peak force, and the
//                                 pinch force that force would need (for the future pinch device)
//   stage: 1 in hand, 2 pushing/closing, 3 done (easing off), 4 left in place, 5 pulling/opening,
//          6 ATM reader moving the card, 7 waiting (zipper: go to the tab)
//
// SIMULATOR TESTS: ADL_AUTOPILOT=1 runs the protocol with a scripted hand (build.bat sim);
// ADL_SHOTS=<dir> saves a screenshot of every task.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "phantom.h"
#include "util.h"
#include "viz.h"
#include "adl_models.h"
#include "adl_core.h"
#include "adl_autopilot.h"

using adl::Vec3;

// ============================================================================
// Self-test: every task, 3 cycles, simulated arm + autopilot hand. No device, no window.
// ============================================================================
namespace selftest {

struct Arm {  // like tools/hd_mock: point mass, hand = spring + damper to a speed-limited target
    Vec3 x, v, hand;
    double m = 0.12, kh = 0.15, bh = 0.004, fric = 0.05;
    void step(const Vec3& handTarget, const Vec3& fDev, double dt) {
        Vec3 d = handTarget - hand;
        double n = d.norm(), lim = 400 * dt;
        hand = n > lim ? hand + d * (lim / n) : handTarget;
        Vec3 f = (hand - x) * kh - v * bh + fDev;
        f = f - Vec3(std::tanh(v.x / 2), std::tanh(v.y / 2), std::tanh(v.z / 2)) * fric;
        v += f * (1000.0 / m * dt);
        x += v * dt;
    }
};

struct Stats {
    int cycles = 0, ins = 0, outs = 0, nan = 0;
    double maxF = 0, maxRise = 0, maxSpeed = 0, peakIn = 0, peakOut = 0;
    double sumIn = 0, sumOut = 0;
};

inline int runTask(adl::AdlCore& core, int task, bool protocol, Stats& st, adl::Autopilot& ap, double maxTime) {
    Arm arm;
    double dt = 0.001, t = 0;
    if (protocol) core.startProtocol(); else core.pending = task;
    bool prevBtn = false;
    double lastF = 0;
    int startResults = core.nResults;
    adl::View v;
    int startCycle = core.cycle;
    while (t < maxTime) {
        core.fill(v);
        Vec3 tgt;
        int btn = 0;
        double roll = 0;
        ap.drive(core, v, t, tgt, btn, roll);
        adl::Input in;
        in.p = arm.x; in.v = arm.v; in.btn = btn != 0; in.press = in.btn && !prevBtn; prevBtn = in.btn;
        in.kc = 0.25; in.bc = 0.0015; in.cursorR = 4; in.t = t; in.dt = dt;
        Vec3 f = core.step(in, roll);
        if (!f.finite()) { st.nan++; f = Vec3(); }
        double fn = f.norm();
        if (core.gainRamp >= 1.0) {  // ignore the 0.5 s fade-in after a task switch
            st.maxF = std::fmax(st.maxF, fn);
            st.maxRise = std::fmax(st.maxRise, fn - lastF);
        }
        static const char* trace = std::getenv("ADL_TRACE");  // ADL_TRACE=1: print what happens (debugging)
        if (trace) {
            static int every = 0;
            bool spike = core.gainRamp >= 1.0 && fn - lastF > 0.12;
            if (spike || ++every % 250 == 0)
                std::printf("%s t=%.3f task=%d stage=%d sub=%d p=(%.1f %.1f %.1f) tgt=(%.1f %.1f %.1f) f=%.2f objF=%.2f "
                            "depth=%.2f btn=%d held=%d atm=%d zip=%.1f\n", spike ? "SPIKE" : "     ", t, core.task,
                            core.stage(), ap.sub, arm.x.x, arm.x.y, arm.x.z, tgt.x, tgt.y, tgt.z, fn, core.lastObjF, v.depth, btn,
                            v.held ? 1 : 0, v.atm, v.zipS);
        }
        lastF = fn;
        arm.step(tgt, f, dt);
        st.maxSpeed = std::fmax(st.maxSpeed, arm.v.norm());
        t += dt;
        if (protocol ? core.protoDone : core.cycle - startCycle >= 3) break;
    }
    for (int i = startResults; i < core.nResults; ++i) {
        const adl::Result& r = core.ring[i % adl::kRing];
        if (r.dir == 0) { st.ins++; st.peakIn = std::fmax(st.peakIn, r.peakProfile); st.sumIn += r.time; }
        else { st.outs++; st.peakOut = std::fmax(st.peakOut, r.peakProfile); st.sumOut += r.time; }
    }
    st.cycles = core.cycle - startCycle;
    return (int)(t * 1000);
}

inline int run() {
    std::printf("ADL self-test: simulated arm (0.12 kg, hand 0.15 N/mm), autopilot hand, 3 cycles per task.\n");
    std::printf("Force map: F_sim = %.1f * (F_real / %.1f)^%.1f\n\n", adl::kMapTop, adl::kMapRef, adl::kMapExp);
    std::printf("%-18s %6s %4s %4s %7s %7s %7s %7s %7s %7s %6s %6s\n", "task", "cycles", "in", "out", "peakIn", "target",
                "peakOut", "target", "maxF", "rise/t", "mm/s", "sim s");
    int failures = 0;
    int only = std::getenv("ADL_ONLY") ? std::atoi(std::getenv("ADL_ONLY")) - 1 : -1;  // ADL_ONLY=2: just the key
    for (int task = 0; task < adl::NTASK; ++task) {
        if (only >= 0 && task != only) continue;
        adl::AdlCore core;
        adl::Autopilot ap;
        Stats st;
        int ms = runTask(core, task, false, st, ap, 120);
        const adl::TaskInfo& ti = adl::info(task);
        double tIn = adl::toSim(ti.realIn), tOut = adl::toSim(ti.realOut);
        bool hasOut = task != adl::REMOTE;
        bool ok = st.cycles >= 3 && st.nan == 0 && st.maxF <= 3.501 && st.maxRise <= 0.15 && st.maxSpeed < 600;
        // the measured peak must reach the target (within 15 %) and not overshoot it much (35 %, the hand
        // spring adds speed-dependent damping on top of the profile)
        bool peakOk = st.peakIn >= 0.85 * tIn && st.peakIn <= 1.35 * tIn;
        if (hasOut && tOut > 0) peakOk = peakOk && st.peakOut >= 0.85 * tOut && st.peakOut <= 1.35 * tOut;
        if (task == adl::FORK) peakOk = st.peakIn >= 0.85 * tIn && st.peakIn <= 1.5 * tIn;  // putty is speed dependent
        ok = ok && peakOk;
        if (ap.blockedChecks > 0) ok = ok && ap.blockedOk == ap.blockedChecks;
        if (!ok) failures++;
        std::printf("%-18s %6d %4d %4d %7.2f %7.2f %7.2f %7.2f %7.2f %7.3f %6.0f %6.1f  %s", ti.name, st.cycles, st.ins,
                    st.outs, st.peakIn, tIn, st.peakOut, tOut, st.maxF, st.maxRise, st.maxSpeed, ms / 1000.0,
                    ok ? "PASS" : "FAIL");
        if (ap.blockedChecks) std::printf("  (blocked entry %d/%d)", ap.blockedOk, ap.blockedChecks);
        std::printf("\n");
    }
    // the whole protocol in one go
    if (only < 0) {
        adl::AdlCore core;
        adl::Autopilot ap;
        Stats st;
        int ms = runTask(core, 0, true, st, ap, 900);
        int expected = adl::kProtoReps * adl::NTASK;
        int expectedRows = adl::kProtoReps * (2 * (adl::NTASK - 1) + 1);
        bool ok = core.protoDone && st.ins + st.outs == expectedRows && st.nan == 0 && st.maxF <= 3.501;
        if (!ok) failures++;
        std::printf("\nprotocol: %d/%d cycles, %d/%d timed actions, %.0f s simulated, max %.2f N  %s\n",
                    core.cycle, expected, st.ins + st.outs, expectedRows, ms / 1000.0, st.maxF, ok ? "PASS" : "FAIL");
    }
    std::printf("\n%s (%d failure%s)\n", failures ? "SELF-TEST FAILED" : "SELF-TEST PASSED", failures,
                failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

}  // namespace selftest

#if VIZ_AVAILABLE
#include "adl_draw.h"

// ============================================================================
// The scene
// ============================================================================
struct AdlScene : viz::SceneBase {
    static constexpr bool kButtonPlaces = false;  // the button is part of the task
    static constexpr bool kDrawRoom = false;      // we draw our own stations
    static constexpr bool kDrawCursor = false;    // and our own hand / fingertip

    adl::AdlCore core;          // servo thread
    bool prevBtn = false;
    int rollAxis = 2;

    struct Snap { adl::View v; };

    // --- main thread ---
    FILE* trials = nullptr;
    int written = 0;
    bool autopilot = false;
    adl::Autopilot ap;
    int camTask = -1;
    std::string shotDir;
    bool shotTaken[adl::NTASK] = {};
    double frameT = 0;
    adl_draw::Renderer gfx;

    std::string info() const {
        char buf[512];
        std::snprintf(buf, sizeof buf,
                      "app: adl_tasks (Smaby et al. 2004 ADL tasks)\nforce_map: F_sim = %.2f * (F_real / %.1f)^%.2f\n"
                      "force_gain: %.2f\nroll_axis: %d\n", adl::kMapTop, adl::kMapRef, adl::kMapExp, core.forceGain,
                      rollAxis);
        std::string s = buf;
        for (int t = 0; t < adl::NTASK; ++t) {
            const adl::TaskInfo& ti = adl::info(t);
            std::snprintf(buf, sizeof buf, "task_%d: %s | real peak in %.1f N -> sim %.2f N | out %.1f N -> sim %.2f N\n", t + 1,
                          ti.name, ti.realIn, adl::toSim(ti.realIn), ti.realOut, adl::toSim(ti.realOut));
            s += buf;
        }
        return s;
    }

    // ---------------- servo thread ----------------
    viz::Vec3 force(viz::State& s, const viz::Vec3& p, double dt) {
        adl::Input in;
        in.p = p; in.v = s.vel;
        in.btn = s.button1(); in.press = in.btn && !prevBtn; prevBtn = in.btn;
        in.kc = k;
        in.bc = maxDamping > 0 ? std::fmin(0.0015, 0.5 * maxDamping) : 0.0015;
        in.cursorR = cursorR; in.t = s.t; in.dt = dt;
        int phase = 0;
        Vec3 f = core.step(in, s.gimbal[rollAxis], &phase);
        s.phase = phase;
        s.trial = core.cycle;
        return f;
    }
    void reset() { core.restartCycle(); }
    void key(int vk) { core.onKey(vk); }
    void snap(Snap& o) const { core.fill(o.v); }

    // ---------------- main thread ----------------
    void onStart(const std::filesystem::path& csv) {
        std::filesystem::path p = csv;
        p.replace_filename(csv.stem().string() + "_trials.csv");
        trials = std::fopen(p.string().c_str(), "w");
        if (trials) {
            std::fprintf(trials,
                         "mode,cycle,rep,task_id,task,action,time_s,t_start_s,t_goal_s,robot_ref_mean_s,robot_ref_sd_s,"
                         "robot_t1_slow_s,robot_t2_med_s,robot_t3_fast_s,ratio_to_robot_mean,real_peak_N,"
                         "sim_peak_target_N,sim_peak_profile_N,sim_peak_any_N,implied_real_peak_N,pinch_req_mu056_N,pinch_req_mu033_N,"
                         "max_hand_force_N,path_mm,jammed_s,force_gain,rotation_on,simulator\n");
            std::fflush(trials);
            std::printf("Trial results -> %s\n", p.string().c_str());
        }
        autopilot = std::getenv("ADL_AUTOPILOT") != nullptr && viz::ui().sim;
        if (const char* d = std::getenv("ADL_SHOTS")) shotDir = d;
    }
    void writeResult(const adl::Result& r, bool rotOn) {
        const adl::TaskInfo& ti = adl::info(r.task);
        const adl::RefTime& ref = r.dir == 0 ? ti.refIn : ti.refOut;
        double real = r.dir == 0 ? ti.realIn : ti.realOut;
        double g = core.forceGain > 0 ? core.forceGain : 1;
        double implied = adl::toReal(r.peakProfile / g);  // the profile peak is what compares to the paper
        const char* action = r.dir == 0 ? ti.inName : ti.outName;
        if (trials) {
            char refs[160] = ",,,,,";
            if (ref.mean > 0)
                std::snprintf(refs, sizeof refs, "%.2f,%.2f,%.2f,%.2f,%.2f,%.3f", ref.mean, ref.sd, ref.t1, ref.t2, ref.t3,
                              r.time / ref.mean);
            char realStr[16] = "";
            if (real > 0) std::snprintf(realStr, sizeof realStr, "%.1f", real);
            std::fprintf(trials, "%s,%d,%d,%d,%s,%s,%.4f,%.4f,%.4f,%s,%s,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%.1f,%.3f,%.2f,%d,%d\n",
                         r.protocol ? "protocol" : "practice", r.cycle, r.rep, r.task + 1, ti.name, action, r.time,
                         r.tStart, r.tGoal, refs, realStr,
                         adl::toSim(real) * g, r.peakProfile, r.peakSim, implied, implied / (2 * 0.56), implied / (2 * 0.33), r.peakHand,
                         r.path, r.jams / 1000.0, core.forceGain, rotOn ? 1 : 0, viz::ui().sim ? 1 : 0);
            std::fflush(trials);
        }
        if (ref.mean > 0)
            std::printf("%-16s %6.2f s   (robot reference %.1f s, SD %.1f)   peak %.2f N sim = %.1f N real\n", action, r.time,
                        ref.mean, ref.sd, r.peakProfile, implied);
        else
            std::printf("%-16s %6.2f s   peak %.2f N sim\n", action, r.time, r.peakProfile);
    }
    void mainTick(const Snap& sn) {
        const adl::View& v = sn.v;
        if (v.nResults < written) written = 0;
        int first = v.nResults - written > adl::kRing ? v.nResults - adl::kRing : written;
        for (int i = first; i < v.nResults; ++i) writeResult(v.ring[i % adl::kRing], v.rollOn);
        written = v.nResults;
        if (v.task != camTask) {  // camera preset per task (you can still rotate afterwards)
            camTask = v.task;
            adl_draw::cameraFor(v.task);
        }
        if (autopilot && v.protoDone) viz::ui().quit = true;
    }
    void onStop() { if (trials) std::fclose(trials); trials = nullptr; }

    bool simHand(const Snap& sn, double elapsed, viz::Vec3& target, int& buttons) {
        if (!autopilot) return false;
        static bool started = false;
        if (!started && elapsed > 1.0) {  // start the protocol (or ADL_TASK=n: practise one task) once placed
            started = true;
            const char* one = std::getenv("ADL_TASK");
            int vk = one ? '0' + (int)adl::clampd(std::atoi(one), 1, 7) : 'A';
            viz::ui().keys[viz::ui().nKeys++] = vk;
        }
        double roll = 0;
        ap.drive(core, sn.v, elapsed, target, buttons, roll);
#ifdef HD_MOCK
        double gim[3] = {0, 0, 0};
        gim[rollAxis] = roll;
        hdMockSetGimbal(gim);
#endif
        return true;
    }

    void draw(const Snap& sn, const viz::Vec3& cursor) const {
        const_cast<adl_draw::Renderer&>(gfx).drawScene(core, sn.v, cursor);
    }
    void drawHud(const Snap& sn) const {
        const_cast<adl_draw::Renderer&>(gfx).drawHud(sn.v);
        // per-task screenshots for the tests (taken before the buffer swap)
        if (!shotDir.empty() && sn.v.timing && sn.v.timer > 0.3 && !sn.v.protoDone) {
            bool& done = const_cast<bool&>(shotTaken[sn.v.task]);
            if (!done) {
                done = true;
                std::string path = shotDir + "/adl_" + std::to_string(sn.v.task + 1) + "_" + adl::info(sn.v.task).name + ".ppm";
                for (char& c : path) if (c == ' ') c = '_';
                viz::saveScreenshot(path.c_str());
            }
        }
    }
};
#endif

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    if (args.flag("--selftest")) return selftest::run();
#if VIZ_AVAILABLE
    static AdlScene scene;  // static: a few kB of task state
    scene.core.forceGain = adl::clampd(args.num("--force-gain", 1.0), 0.1, 1.0);
    scene.rollAxis = (int)adl::clampd(args.num("--roll-axis", 2), 0, 2);
    scene.cursorR = 3;
    return viz::run(argc, argv, scene, "adl_tasks",
                    "ADL tasks (Smaby et al. 2004). 1-7 pick a task, A = protocol, R = rotation, C = restart cycle.\n"
                    "Push the object in; ease off and it stays; press the stylus button on it, then pull it out.");
#else
    std::printf("adl_tasks needs a 3D window (Windows, or the CMake build with GLFW). --selftest works anywhere.\n");
    return 0;
#endif
}
