// force_direction - checks the motors push the way the code thinks. Gentle forces.
//
// Run after axis_check passes. For each of +x, -x, +y, -y, +z, -z it sends one
// smooth force pulse (sin^2 shape, 0 -> peak -> 0) while you hold the stylus
// LOOSELY, like a pen, in the middle of the workspace. It measures which way
// the stylus drifted furthest during the pulse and compares that with the direction it
// pushed. A mismatch means force and position disagree about the axes, and
// every spring/wall in the other programs would push the wrong way.
//
// Default pulse: 0.3 N peak over 1.5 s, the weight of a ~30 g object. Some
// damping is added so the stylus can't build up speed. The safety layer still
// clamps force, fades in, and trips on speed.
//
// Options:
//   --peak 0.3      pulse peak force, N (capped at the safety force limit)
//   --time 1.5      pulse duration, s (at least 0.5)
//   --b 0.001       damping during the pulse, N*s/mm (capped at 50% of device max)
//   --device "<name>"  --skip-calibration  --allow-uncalibrated
// Keys: ENTER = send the next pulse, Q = quit (forces off immediately).
// Output: data/force_direction_<stamp>.csv (every servo sample; trial = pulse #, phase 1 = pushing)

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

namespace {

// Everything the force function uses. After start() the main thread only
// touches it through runInServo.
struct Pulse {
    bool armed = false;     // set by main thread; the servo starts the pulse next tick
    bool active = false;
    bool finished = false;
    int index = -1;
    Vec3 dir;               // unit vector
    double peak = 0.3, T = 1.5, b = 0.001;
    double t0 = 0;
    Vec3 startPos;
    double along = 0;       // largest displacement along dir during the pulse, mm (signed)
    double across = 0;      // sideways displacement at that moment, mm
};

struct Dir { const char* name; Vec3 dir; };
const Dir kDirs[6] = {
    {"+x (right)", Vec3(1, 0, 0)},      {"-x (left)", Vec3(-1, 0, 0)},
    {"+y (up)", Vec3(0, 1, 0)},         {"-y (down)", Vec3(0, -1, 0)},
    {"+z (toward you)", Vec3(0, 0, 1)}, {"-z (away)", Vec3(0, 0, -1)},
};

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// Returns true if Q/ESC/end of input was pressed; sets *enter if ENTER was.
bool pollKeys(bool* enter) {
    while (console::keyPressed()) {
        int c = console::readKey();
        if (c == 'q' || c == 'Q' || c == 27 || c < 0) return true;
        if ((c == '\n' || c == '\r') && enter) *enter = true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    Pulse p;
    p.peak = args.num("--peak", 0.3);
    p.T = std::fmax(0.5, args.num("--time", 1.5));
    p.b = args.num("--b", 0.001);

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }
    if (dev.info().maxDamping > 0 && p.b > 0.5 * dev.info().maxDamping) p.b = 0.5 * dev.info().maxDamping;

    dev.setForceFunction([&](State& s) -> Vec3 {
        s.trial = p.index;
        if (p.armed) {
            p.armed = false; p.active = true; p.t0 = s.t; p.startPos = s.pos; p.along = 0; p.across = 0;
        }
        if (!p.active) { s.phase = 0; return Vec3(); }
        s.phase = 1;
        double tau = (s.t - p.t0) / p.T;
        Vec3 d = s.pos - p.startPos;
        double a = dot(d, p.dir);
        if (std::fabs(a) > std::fabs(p.along)) { p.along = a; p.across = (d - p.dir * a).norm(); }
        if (tau >= 1) {
            p.active = false;
            p.finished = true;
            return Vec3();
        }
        double shape = std::sin(3.14159265358979 * tau);
        return p.dir * (p.peak * shape * shape) - s.vel * p.b;
    });

    // start() fills in safety.maxForce, which we need to cap the peak. Before
    // start() nothing runs in the servo thread, so writing p here is safe.
    if (!dev.start(true)) return 1;
    double peak = std::fmin(p.peak, dev.safety.maxForce);
    double damping = p.b, T = p.T;
    dev.runInServo([&] { p.peak = peak; });
    dev.enableForces(true);  // net force stays zero until a pulse starts

    auto csvPath = util::dataDir(argv[0]) / ("force_direction_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    char extra[160];
    std::snprintf(extra, sizeof extra, "pulse_peak_N: %.3f\npulse_time_s: %.2f\ndamping_Ns_mm: %.5f\n", peak, T,
                  damping);
    util::writeInfoFile(csvPath, dev, extra);
    dev.startRecording();
    auto drainToCsv = [&] { State s; while (dev.popSample(s)) util::writeSample(csv, s, 0); };

    std::printf("\nPulse: %.2f N peak over %.1f s. Hold the stylus LOOSELY in the middle of the workspace.\n", peak,
                T);
    const char* verdict[6] = {"not run", "not run", "not run", "not run", "not run", "not run"};
    double along[6] = {}, across[6] = {};
    bool quit = false;

    for (int i = 0; i < 6 && !quit; ++i) {
        std::printf("\n[%d/6] Next push: %s.  ENTER = go, Q = quit\n", i + 1, kDirs[i].name);
        bool go = false;
        while (!go && !quit) {
            drainToCsv();
            quit = pollKeys(&go);
            util::sleepMs(20);
        }
        if (quit) break;

        Vec3 dir = kDirs[i].dir;
        dev.runInServo([&] { p.dir = dir; p.index = i; p.finished = false; p.armed = true; });
        bool finished = false;
        while (!finished && !quit) {
            drainToCsv();
            quit = pollKeys(nullptr);
            if (dev.tripped() != phantom::Trip::None) {
                std::printf("\nSAFETY TRIP: %s. Stopping.\n", phantom::tripName(dev.tripped()));
                quit = true;
            }
            std::printf("\r  pushing...  F %.2f N ", dev.latest().force.norm());
            std::fflush(stdout);
            util::sleepMs(20);
            dev.runInServo([&] { finished = p.finished; along[i] = p.along; across[i] = p.across; });
        }
        if (quit) break;
        // A pass needs a clear drift along the push that is bigger than the sideways drift.
        if (along[i] > 2 && along[i] > across[i]) verdict[i] = "PASS";
        else if (along[i] < -2) verdict[i] = "OPPOSITE - check axis / force sign";
        else verdict[i] = "UNCLEAR (held too tight? try --peak 0.5)";
        std::printf("\r  moved %+.1f mm along the push, %.1f mm sideways -> %s\n", along[i], across[i], verdict[i]);
    }

    dev.enableForces(false);
    util::sleepMs(50);
    drainToCsv();
    dev.stopRecording();
    std::fclose(csv);
    dev.close();

    std::printf("\n================ SUMMARY ================\n");
    for (int i = 0; i < 6; ++i) std::printf(" %-16s %+6.1f mm  %s\n", kDirs[i].name, along[i], verdict[i]);
    std::printf("=========================================\nSaved %s\n", csvPath.string().c_str());
    return 0;
}
