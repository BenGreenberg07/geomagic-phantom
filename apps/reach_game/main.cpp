// reach_game - center-out reaching task with per-trial movement metrics.
//
// The classic motor-control task: hold the stylus on the gray HOME ball, an
// orange TARGET appears on a 35 mm circle in front of you (8 directions in the
// frontal x-y plane), reach it and hold still, return home, repeat. 3 rounds x
// 8 directions = 24 trials, shuffled within each round (fixed seed, so every
// session uses the same order). A short buzz says "got it". The room walls are
// the only other forces, so the movement itself is not assisted.
//
// Per trial (data/reach_game_<stamp>_trials.csv):
//   reaction_time_s   target shown -> movement onset (filtered speed > 30 mm/s)
//   movement_time_s   onset -> first entry into the target (6 mm radius)
//   path_length_mm    distance travelled from onset to first entry
//   straight_distance_mm, path_ratio (1.0 = perfectly straight)
//   peak_speed_mm_s, speed_peaks (number of submovements; 1 = one smooth reach)
//
// Keys (click the 3D window first):
//   ENTER / button 1  place the room (home = where the stylus is) and start over
//   X, F, arrows/drag, wheel, Q: see common/viz.h
//
// Options: --k 0.25  --b 0.001  --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/reach_game_<stamp>.csv (every sample) + _trials.csv (one row per trial)
//   phase: 0 no room, 3 go/hold home, 4 target shown (reaction),
//          5 moving, 6 holding on target, 7 all done;  trial: 0-based trial index

#include <cmath>
#include <cstdio>
#include <filesystem>

#include "phantom.h"
#include "util.h"
#include "viz.h"

#if VIZ_AVAILABLE

using viz::State;
using viz::Vec3;

struct ReachGame : viz::SceneBase {
    static const int kDirs = 8, kTrials = 24;
    enum { HOME = 3, REACT = 4, MOVE = 5, HOLD = 6, DONE = 7 };
    const double ringR = 35, tol = 6, holdHome = 0.5, holdTarget = 0.3, onsetSpeed = 30;

    struct Result {
        int block = 0, trial = 0, dir = 0;
        Vec3 target;
        double tShow = 0, rt = 0, mt = 0, path = 0, straight = 0, peak = 0;
        int peaks = 0;
    };

    Vec3 targetPos(int dir) const {
        double a = dir * 2 * viz::kPi / kDirs;
        return Vec3(ringR * std::cos(a), ringR * std::sin(a), 0);
    }

    struct Snap {
        int st = HOME, trial = 0, dir = 0, resultCount = 0;
        double holdFrac = 0;
        Result results[kTrials];
    };

    // --- servo thread
    int order[kTrials] = {};
    int block = 0, trial = 0, st = HOME;
    double holdT = 0, tShow = 0, tOnset = 0, tArrive = -1, buzzUntil = -1;
    Vec3 prevP, onsetP;
    double path = 0, pathAtArrive = 0, peak = 0, fs = 0, runMax = 0, runMin = 0;
    bool rising = true;
    int peaks = 0;
    Result results[kTrials];
    int resultCount = 0;
    unsigned seed = 12345;

    // --- main thread
    FILE* trialsCsv = nullptr;
    int written = 0;

    void reset() {
        block++;
        for (int r = 0; r < kTrials / kDirs; ++r) {  // shuffle each round of 8 (Fisher-Yates, LCG)
            int* o = order + r * kDirs;
            for (int i = 0; i < kDirs; ++i) o[i] = i;
            for (int i = kDirs - 1; i > 0; --i) {
                seed = seed * 1664525u + 1013904223u;
                int j = (int)((seed >> 8) % (unsigned)(i + 1));
                int t = o[i]; o[i] = o[j]; o[j] = t;
            }
        }
        trial = 0; st = HOME; holdT = 0; resultCount = 0; buzzUntil = -1;
    }

    void snap(Snap& o) const {
        o.st = st; o.trial = trial; o.resultCount = resultCount;
        o.dir = trial < kTrials ? order[trial] : 0;
        o.holdFrac = st == HOME ? holdT / holdHome : (st == HOLD ? holdT / holdTarget : 0);
        for (int i = 0; i < resultCount; ++i) o.results[i] = results[i];
    }

    Vec3 force(State& s, const Vec3& p, double dt) {
        Vec3 f = viz::roomForce(room, p, s.vel, cursorR, k, b);
        double t = s.t;
        fs += (s.vel.norm() - fs) * 0.05;  // ~8 Hz low-pass of speed
        Vec3 tgt = targetPos(trial < kTrials ? order[trial] : 0);

        if (st == HOME) {
            if (p.norm() < tol) {
                holdT += dt;
                if (holdT >= holdHome) {
                    st = REACT; tShow = t; holdT = 0; tArrive = -1;
                    path = 0; pathAtArrive = 0; peak = 0; peaks = 0;
                }
            } else {
                holdT = 0;
            }
        }
        if (st == REACT && fs > onsetSpeed) {
            st = MOVE; tOnset = t; prevP = p; onsetP = p; runMax = fs; rising = true;
        }
        if (st == MOVE || st == HOLD) {
            path += (p - prevP).norm();
            prevP = p;
            if (fs > peak) peak = fs;
            // Count speed peaks with 20 % hysteresis (submovements).
            if (rising) {
                if (fs > runMax) runMax = fs;
                if (runMax > onsetSpeed && fs < 0.8 * runMax) { peaks++; rising = false; runMin = fs; }
            } else {
                if (fs < runMin) runMin = fs;
                if (fs > 1.25 * runMin + 5) { rising = true; runMax = fs; }
            }
            if ((p - tgt).norm() < tol) {
                if (st == MOVE) { st = HOLD; holdT = 0; }
                if (tArrive < 0) { tArrive = t; pathAtArrive = path; }
                holdT += dt;
                if (holdT >= holdTarget) {
                    Result& r = results[resultCount++];
                    r.block = block; r.trial = trial; r.dir = order[trial]; r.target = tgt;
                    r.tShow = tShow; r.rt = tOnset - tShow; r.mt = tArrive - tOnset;
                    r.path = pathAtArrive; r.straight = (tgt - onsetP).norm(); r.peak = peak;
                    r.peaks = peaks + (rising && runMax > onsetSpeed ? 1 : 0);  // a peak still in progress
                    buzzUntil = t + 0.06;
                    trial++;
                    st = trial >= kTrials ? DONE : HOME;
                    holdT = 0;
                }
            } else if (st == HOLD) {
                st = MOVE; holdT = 0;
            }
        }
        if (t < buzzUntil) f += Vec3(0, 0.3 * std::sin(2 * viz::kPi * 150 * t), 0);  // "got it"
        s.phase = st;
        s.trial = trial;
        return f;
    }

    void onStart(const std::filesystem::path& csv) {
        std::filesystem::path p = csv;
        p.replace_filename(csv.stem().string() + "_trials.csv");
        trialsCsv = std::fopen(p.string().c_str(), "w");
        if (!trialsCsv) { std::printf("Could not open %s\n", p.string().c_str()); return; }
        std::fprintf(trialsCsv, "block,trial,direction,target_x_mm,target_y_mm,target_z_mm,t_show_s,reaction_time_s,"
                                "movement_time_s,path_length_mm,straight_distance_mm,path_ratio,peak_speed_mm_s,speed_peaks\n");
        std::printf("Trial results -> %s\n", p.string().c_str());
    }
    void mainTick(const Snap& sn) {
        if (sn.resultCount < written) written = 0;  // room re-placed: new block
        for (; written < sn.resultCount; ++written) {
            const Result& r = sn.results[written];
            double ratio = r.straight > 0 ? r.path / r.straight : 0;
            if (trialsCsv) {
                std::fprintf(trialsCsv, "%d,%d,%d,%.1f,%.1f,%.1f,%.4f,%.4f,%.4f,%.2f,%.2f,%.3f,%.1f,%d\n", r.block, r.trial,
                             r.dir, r.target.x, r.target.y, r.target.z, r.tShow, r.rt, r.mt, r.path, r.straight, ratio,
                             r.peak, r.peaks);
                std::fflush(trialsCsv);
            }
            std::printf("trial %2d/%d  dir %d  RT %.2f s  MT %.2f s  path ratio %.2f  peak %.0f mm/s  peaks %d\n",
                        r.trial + 1, kTrials, r.dir, r.rt, r.mt, ratio, r.peak, r.peaks);
        }
    }
    void onStop() { if (trialsCsv) std::fclose(trialsCsv); trialsCsv = nullptr; }

    void draw(const Snap& sn, const Vec3&) const {
        Vec3 home;
        bool homeActive = sn.st == HOME;
        viz::drawSphere(home, tol, homeActive ? 0.3 + 0.6 * sn.holdFrac : 0.5, homeActive ? 0.5 + 0.4 * sn.holdFrac : 0.5,
                        0.5, 0.8);
        for (int i = 0; i < sn.resultCount; ++i) viz::drawSphere(sn.results[i].target, 1.5, 0.3, 0.8, 0.4);
        if (sn.st == REACT || sn.st == MOVE || sn.st == HOLD) {
            Vec3 tgt = targetPos(sn.dir);
            viz::drawLine(home, tgt, 0.45, 0.45, 0.5);
            if (sn.st == HOLD) viz::drawSphere(tgt, tol, 0.3, 0.9, 0.4, 0.85);
            else viz::drawSphere(tgt, tol, 1.0, 0.55, 0.1, 0.85);
            viz::drawShadow(tgt, tol * 0.8, room.floor);
        }
        viz::drawShadow(home, tol * 0.8, room.floor);
    }

    void status(const Snap& sn, char* buf, size_t n) const {
        const char* what = sn.st == HOME ? "go to the gray home ball and hold"
                           : sn.st == REACT ? "reach the ORANGE target"
                           : sn.st == MOVE  ? "reach the ORANGE target"
                           : sn.st == HOLD  ? "hold still..."
                                            : "DONE - results saved (ENTER = new block)";
        std::snprintf(buf, n, "trial %d/%d - %s", sn.trial < kTrials ? sn.trial + 1 : kTrials, kTrials, what);
    }
};

int main(int argc, char** argv) {
    static ReachGame scene;  // static: the result arrays are a few kB
    return viz::run(argc, argv, scene, "reach_game",
                    "Hold the gray home ball, reach each orange target and hold still, return home. 24 trials.");
}

#else
int main() { std::printf("reach_game needs a 3D window: build on Windows, or with CMake (it fetches GLFW) elsewhere.\n"); return 0; }
#endif
