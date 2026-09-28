// position_matching - proprioceptive position-matching task (a.k.a. joint
// position sense / "passive-to-active" matching), a standard stroke-assessment
// paradigm.
//
// Per trial:
//   GUIDE_OUT  the device gently moves the (relaxed) hand from HOME to a hidden
//              target along a smooth minimum-jerk path
//   HOLD       holds it there so the participant can "memorise" the position
//   GUIDE_BACK moves it back to HOME
//   RELEASE    forces fade to zero
//   MATCH      participant actively moves to where they think the target was
//              and presses the stylus button (vision should be occluded)
//   RETURN     the device gently brings the hand back to HOME
//
// Before trial 1 the participant moves to a comfortable start pose and presses
// the button: that point becomes HOME and all targets are offsets from it.
//
// Options:
//   --id P01         label for file names
//   --dist 40        target distance from HOME, mm
//   --reps 3         repetitions of the 6 directions (+/-x, +/-y, +/-z)
//   --k 0.2          guiding stiffness, N/mm (capped at 50% of device max)
//   --b 0.0015       guiding damping, N*s/mm
//   --move 1.5       guided movement duration, s
//   --hold 2.0       hold-at-target duration, s
//   --max-force 0    N, 0 = 60% of device continuous force
//   --seed 0         random order seed (0 = from clock)
//   --device "<name>"  --skip-calibration
//
// Keys: Q = abort (forces off immediately).
//
// Output: data/<id>_<stamp>_matching.csv          (every servo sample, with phase/trial)
//         data/<id>_<stamp>_matching_trials.csv   (one row per trial: target, response, error)

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

enum Phase { WAIT_HOME, GUIDE_OUT, HOLD, GUIDE_BACK, RELEASE, MATCH, RETURN, FINISH, DONE };
static const char* kPhaseName[] = {"WAIT_HOME", "GUIDE_OUT", "HOLD", "GUIDE_BACK", "RELEASE",
                                   "MATCH", "RETURN", "FINISH", "DONE"};

struct Trial {
    std::string label;
    Vec3 offset;    // relative to HOME, mm
    Vec3 target;    // absolute, mm (filled in when the trial starts)
    Vec3 response;  // absolute, mm
    double matchTime = 0;
};

struct Config {
    double k = 0.2, b = 0.0015;
    double moveT = 1.5, holdT = 2.0;
    double settleT = 0.5;    // hold at HOME before releasing
    double releaseT = 0.5;   // force fade-out
    double minMatchT = 0.5;  // ignore button presses right after release
    double returnT = 1.5;
    double returnFadeT = 0.3;
};

// Runs entirely in the servo thread (deterministic 1 kHz timing). The main
// thread only watches the atomics.
class Task {
public:
    Config cfg;
    std::vector<Trial> trials;
    std::atomic<int> phaseOut{WAIT_HOME}, trialOut{0}, completed{0};
    std::atomic<bool> abort{false};

    Vec3 update(State& s) {
        bool press = s.button1() && !prevBtn_;
        prevBtn_ = s.button1();
        if (abort.load(std::memory_order_relaxed) && phase_ != DONE) enter(DONE, s.t);

        double tp = s.t - phaseStart_;
        Vec3 sp, spv;
        double gain = 1.0;
        Trial* tr = trialIdx_ < (int)trials.size() ? &trials[trialIdx_] : nullptr;

        switch (phase_) {
        case WAIT_HOME:
            gain = 0;
            if (press) { home_ = s.pos; beginTrial(0, s.t); }
            break;
        case GUIDE_OUT:
            phantom::minJerk(home_, tr->target, cfg.moveT, tp, sp, spv);
            if (tp >= cfg.moveT) enter(HOLD, s.t);
            break;
        case HOLD:
            sp = tr->target;
            if (tp >= cfg.holdT) enter(GUIDE_BACK, s.t);
            break;
        case GUIDE_BACK:
            phantom::minJerk(tr->target, home_, cfg.moveT, tp, sp, spv);
            if (tp >= cfg.moveT) enter(RELEASE, s.t);
            break;
        case RELEASE:
            sp = home_;
            gain = tp < cfg.settleT ? 1.0 : 1.0 - (tp - cfg.settleT) / cfg.releaseT;
            if (tp >= cfg.settleT + cfg.releaseT) enter(MATCH, s.t);
            break;
        case MATCH:
            gain = 0;
            if (press && tp >= cfg.minMatchT) {
                tr->response = s.pos;
                tr->matchTime = tp;
                completed.store(trialIdx_ + 1, std::memory_order_release);
                returnFrom_ = s.pos;
                enter(RETURN, s.t);
            }
            break;
        case RETURN:
            phantom::minJerk(returnFrom_, home_, cfg.returnT, tp, sp, spv);
            gain = std::min(1.0, tp / cfg.returnFadeT);
            if (tp >= cfg.returnT + cfg.settleT) {
                if (trialIdx_ + 1 < (int)trials.size()) beginTrial(trialIdx_ + 1, s.t);
                else enter(FINISH, s.t);
            }
            break;
        case FINISH:
            sp = home_;
            gain = 1.0 - tp / cfg.releaseT;
            if (tp >= cfg.releaseT) enter(DONE, s.t);
            break;
        case DONE:
            gain = 0;
            break;
        }

        s.phase = phase_;
        s.trial = trialIdx_;
        if (gain <= 0) return Vec3();
        return phantom::springDamper(s, sp, spv, cfg.k, cfg.b) * gain;
    }

    Vec3 home() const { return home_; }

private:
    void enter(Phase p, double t) {
        phase_ = p;
        phaseStart_ = t;
        phaseOut.store(p, std::memory_order_release);
    }
    void beginTrial(int i, double t) {
        trialIdx_ = i;
        trials[i].target = home_ + trials[i].offset;
        trialOut.store(i, std::memory_order_release);
        enter(GUIDE_OUT, t);
    }

    Phase phase_ = WAIT_HOME;
    double phaseStart_ = 0;
    int trialIdx_ = 0;
    bool prevBtn_ = false;
    Vec3 home_, returnFrom_;
};

static const char* instructions(int phase) {
    switch (phase) {
    case WAIT_HOME: return "Participant: move to a comfortable START position and press the button.";
    case GUIDE_OUT: return "Relax - the device is moving your hand. (Eyes closed / vision blocked.)";
    case HOLD: return "Remember this position...";
    case GUIDE_BACK: return "Relax - moving back to start.";
    case RELEASE: return "Releasing...";
    case MATCH: return "GO: move to the remembered position and press the button.";
    case RETURN: return "Relax - returning to start.";
    case FINISH: return "Finishing...";
    case DONE: return "Done.";
    }
    return "";
}

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    if (args.flag("--help")) {
        std::printf("usage: position_matching [--id P01] [--dist 40] [--reps 3] [--k 0.2] [--b 0.0015]\n"
                    "                         [--move 1.5] [--hold 2] [--max-force N] [--seed N]\n");
        return 0;
    }
    std::string id = args.get("--id", "test");
    double dist = args.num("--dist", 40);
    int reps = (int)args.num("--reps", 3);
    unsigned seed = (unsigned)args.num("--seed", 0);
    if (seed == 0) seed = (unsigned)std::random_device{}();

    Task task;
    task.cfg.k = args.num("--k", task.cfg.k);
    task.cfg.b = args.num("--b", task.cfg.b);
    task.cfg.moveT = args.num("--move", task.cfg.moveT);
    task.cfg.holdT = args.num("--hold", task.cfg.holdT);

    // Build the trial list: each block = the 6 directions in random order.
    struct Dir { const char* label; Vec3 v; };
    const Dir dirs[] = {{"+x_right", Vec3(1, 0, 0)}, {"-x_left", Vec3(-1, 0, 0)}, {"+y_up", Vec3(0, 1, 0)},
                        {"-y_down", Vec3(0, -1, 0)}, {"+z_toward", Vec3(0, 0, 1)}, {"-z_away", Vec3(0, 0, -1)}};
    std::mt19937 rng(seed);
    for (int r = 0; r < reps; ++r) {
        std::vector<int> order = {0, 1, 2, 3, 4, 5};
        std::shuffle(order.begin(), order.end(), rng);
        for (int i : order) {
            Trial t;
            t.label = dirs[i].label;
            t.offset = dirs[i].v * dist;
            task.trials.push_back(t);
        }
    }

    if (task.trials.empty()) {
        std::printf("No trials (--reps must be >= 1).\n");
        return 1;
    }

    phantom::Device dev;
    dev.safety.maxForce = args.num("--max-force", 0);
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }
    double kMax = dev.info().maxStiffness > 0 ? 0.5 * dev.info().maxStiffness : 0.5;
    if (task.cfg.k > kMax) {
        std::printf("k=%.3f N/mm is above 50%% of device max; using %.3f.\n", task.cfg.k, kMax);
        task.cfg.k = kMax;
    }

    dev.setForceFunction([&](State& s) { return task.update(s); });
    if (!dev.start(true)) return 1;

    auto dir = util::dataDir(argv[0]);
    std::string base = id + "_" + util::stamp() + "_matching";
    auto rawPath = dir / (base + ".csv");
    auto trialsPath = dir / (base + "_trials.csv");
    FILE* raw = util::openCsv(rawPath);
    if (!raw) return 1;
    util::writeSampleHeader(raw);
    char extra[512];
    std::snprintf(extra, sizeof extra,
                  "app: position_matching\nid: %s\nseed: %u\ndist_mm: %.1f\nreps: %d\nk_N_per_mm: %.4f\n"
                  "b_Ns_per_mm: %.5f\nmove_s: %.2f\nhold_s: %.2f\nphases: 0=WAIT_HOME 1=GUIDE_OUT 2=HOLD "
                  "3=GUIDE_BACK 4=RELEASE 5=MATCH 6=RETURN 7=FINISH 8=DONE\n",
                  id.c_str(), seed, dist, reps, task.cfg.k, task.cfg.b, task.cfg.moveT, task.cfg.holdT);
    util::writeInfoFile(rawPath, dev, extra);

    std::printf("%zu trials, %.0f mm targets. Q = abort.\n\n", task.trials.size(), dist);
    dev.startRecording();
    dev.enableForces(true);

    int shownPhase = -1, shownDone = 0;
    bool aborted = false;
    while (task.phaseOut.load() != DONE) {
        State s;
        while (dev.popSample(s)) util::writeSample(raw, s, 0);

        int ph = task.phaseOut.load(std::memory_order_acquire);
        if (ph != shownPhase) {
            shownPhase = ph;
            if (ph == GUIDE_OUT)
                std::printf("\n--- Trial %d/%zu ---\n", task.trialOut.load() + 1, task.trials.size());
            std::printf("  [%s] %s\n", kPhaseName[ph], instructions(ph));
        }
        int done = task.completed.load(std::memory_order_acquire);
        for (; shownDone < done; ++shownDone) {
            const Trial& t = task.trials[shownDone];
            Vec3 e = t.response - t.target;
            std::printf("  result: %-10s error %5.1f mm  (dx %+5.1f dy %+5.1f dz %+5.1f)  %.1f s\n",
                        t.label.c_str(), e.norm(), e.x, e.y, e.z, t.matchTime);
        }

        while (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27) {
                dev.enableForces(false);
                task.abort = true;
                aborted = true;
                std::printf("\nABORTED by operator - forces off.\n");
            }
        }
        if (dev.tripped() != phantom::Trip::None) {
            std::printf("\nSAFETY TRIP: %s. Forces off, task aborted.\n", phantom::tripName(dev.tripped()));
            dev.enableForces(false);
            task.abort = true;
            aborted = true;
        }
        util::sleepMs(10);
    }

    dev.enableForces(false);
    util::sleepMs(50);
    dev.stopRecording();
    State s;
    while (dev.popSample(s)) util::writeSample(raw, s, 0);
    std::fclose(raw);
    dev.close();

    // One row per completed trial.
    int n = task.completed.load();
    FILE* tf = util::openCsv(trialsPath);
    if (tf) {
        std::fprintf(tf, "trial,direction,offset_x_mm,offset_y_mm,offset_z_mm,target_x_mm,target_y_mm,target_z_mm,"
                         "resp_x_mm,resp_y_mm,resp_z_mm,err_x_mm,err_y_mm,err_z_mm,abs_err_mm,match_time_s\n");
        for (int i = 0; i < n; ++i) {
            const Trial& t = task.trials[i];
            Vec3 e = t.response - t.target;
            std::fprintf(tf, "%d,%s,%.1f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", i + 1,
                         t.label.c_str(), t.offset.x, t.offset.y, t.offset.z, t.target.x, t.target.y, t.target.z,
                         t.response.x, t.response.y, t.response.z, e.x, e.y, e.z, e.norm(), t.matchTime);
        }
        std::fclose(tf);
    }

    double sum = 0;
    for (int i = 0; i < n; ++i) sum += (task.trials[i].response - task.trials[i].target).norm();
    std::printf("\n%s: %d/%zu trials completed", aborted ? "Aborted" : "Finished", n, task.trials.size());
    if (n) std::printf(", mean absolute error %.1f mm", sum / n);
    std::printf("\nRaw samples: %s\nTrials:      %s\n", rawPath.string().c_str(), trialsPath.string().c_str());
    return 0;
}
