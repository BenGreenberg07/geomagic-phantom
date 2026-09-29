// virtual_box - you are inside an invisible box with solid walls.
//
// Press ENTER (or stylus button 1) and a box appears CENTERED where the stylus is,
// so nothing pushes until you reach a wall. Each wall is a one-sided spring
// with a little damping: it pushes back only when you are through it, never pulls.
// This is the classic first haptic demo and a good check that stiffness and
// damping feel stable (no buzzing) on this device.
//
// Keys:
//   ENTER  place / re-center the box here   (stylus button 1 does the same)
//   X      remove the box (forces go to zero)
//   Q      quit (forces off immediately)
//
// Options:
//   --size 40      half-width of the box, mm (so default is 80 mm across)
//   --k 0.2        wall stiffness, N/mm (capped at 30% of the device's max)
//   --b 0.001      damping inside a wall, N*s/mm (capped at 50% of the device's max)
//   --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/virtual_box_<stamp>.csv (phase: 0 no box, 1 inside, 2 touching a wall)

#include <cstdio>

#include "phantom.h"
#include "util.h"

using phantom::State;
using phantom::Vec3;

namespace {

// One axis of the box: push back only past a wall, with damping only while in it.
double wallForce(double p, double v, double center, double half, double k, double b) {
    double lo = center - half, hi = center + half;
    if (p > hi) { double f = -k * (p - hi) - b * v; return f < 0 ? f : 0; }
    if (p < lo) { double f = k * (lo - p) - b * v; return f > 0 ? f : 0; }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    double half = args.num("--size", 40);
    double k = args.num("--k", 0.2);
    double b = args.num("--b", 0.001);
    if (half < 5) half = 5;

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }
    if (dev.info().maxStiffness > 0 && k > 0.3 * dev.info().maxStiffness) k = 0.3 * dev.info().maxStiffness;
    if (dev.info().maxDamping > 0 && b > 0.5 * dev.info().maxDamping) b = 0.5 * dev.info().maxDamping;

    // Servo-thread state. The main thread only sets the request flags via runInServo.
    bool placeRequested = false, removeRequested = false;
    bool placed = false, prevBtn = false;
    Vec3 center;
    dev.setForceFunction([&](State& s) -> Vec3 {
        bool press = s.button1() && !prevBtn;
        prevBtn = s.button1();
        if (placeRequested || press) { center = s.pos; placed = true; placeRequested = false; }
        if (removeRequested) { placed = false; removeRequested = false; }
        if (!placed) { s.phase = 0; return Vec3(); }
        Vec3 f(wallForce(s.pos.x, s.vel.x, center.x, half, k, b), wallForce(s.pos.y, s.vel.y, center.y, half, k, b),
               wallForce(s.pos.z, s.vel.z, center.z, half, k, b));
        s.phase = (f.x != 0 || f.y != 0 || f.z != 0) ? 2 : 1;
        return f;
    });

    if (!dev.start(true)) return 1;
    dev.enableForces(true);

    auto csvPath = util::dataDir(argv[0]) / ("virtual_box_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    char extra[128];
    std::snprintf(extra, sizeof extra, "box_half_mm: %.1f\nwall_k_N_mm: %.4f\nwall_b_Ns_mm: %.5f\n", half, k, b);
    util::writeInfoFile(csvPath, dev, extra);
    dev.startRecording();

    std::printf("Virtual box, %.0f mm across, walls k=%.3f N/mm, b=%.4f N*s/mm.\n"
                "ENTER or button 1 = place the box here, X = remove, Q = quit.\n\n",
                2 * half, k, b);
    for (;;) {
        State s;
        while (dev.popSample(s)) util::writeSample(csv, s, 0);
        if (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) break;
            if (c == '\n' || c == '\r') { dev.runInServo([&] { placeRequested = true; }); std::printf("\nbox placed\n"); }
            if (c == 'x' || c == 'X') { dev.runInServo([&] { removeRequested = true; }); std::printf("\nbox removed\n"); }
        }
        if (dev.tripped() != phantom::Trip::None) {
            std::printf("\nSAFETY TRIP: %s\n", phantom::tripName(dev.tripped()));
            break;
        }
        State now = dev.latest();
        const char* where = now.phase == 0 ? "no box " : now.phase == 1 ? "inside " : "WALL   ";
        std::printf("\r%s | pos %6.1f %6.1f %6.1f mm | force %5.2f N ", where, now.pos.x, now.pos.y, now.pos.z,
                    now.force.norm());
        std::fflush(stdout);
        util::sleepMs(30);
    }

    dev.enableForces(false);
    util::sleepMs(50);
    State s;
    while (dev.popSample(s)) util::writeSample(csv, s, 0);
    dev.stopRecording();
    std::fclose(csv);
    dev.close();
    std::printf("\nSaved %s\n", csvPath.string().c_str());
    return 0;
}
