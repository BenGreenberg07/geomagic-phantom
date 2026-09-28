// position_tracker - records the arm continuously to CSV. No forces, ever.
//
// Replaces the original PositionTracker (one sample every 5 s via the GLUT idle
// loop). This one logs every servo tick (~1000 Hz) with velocity, joint and
// gimbal angles, so you can compute speed profiles, smoothness, path length...
//
//   SPACE  bump the "marker" column (use it to separate trials/segments)
//   Q      stop and save
//
// Options:
//   --id P01            participant / session label used in the file name
//   --hz 1000           rows per second to keep (1000 = every tick; 100 = every 10th)
//   --device "<name>"   --skip-calibration
//
// Output: data/<id>_<date>_<time>_tracker.csv (+ .info.txt)
// Analyse with: python analysis/analyze.py data/<file>.csv

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    if (args.flag("--help")) {
        std::printf("usage: position_tracker [--id P01] [--hz 1000] [--device name] [--skip-calibration]\n");
        return 0;
    }
    std::string id = args.get("--id", "test");
    double hz = args.num("--hz", 1000);
    long long every = hz >= 1000 ? 1 : (long long)std::lround(1000.0 / (hz > 0 ? hz : 1000));

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.start(false)) return 1;  // false = motors never driven
    if (!dev.info().calibrated)
        std::printf("WARNING: device not calibrated - positions may be offset.\n");

    auto csvPath = util::dataDir(argv[0]) / (id + "_" + util::stamp() + "_tracker.csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) return 1;
    util::writeSampleHeader(csv);
    util::writeInfoFile(csvPath, dev, "app: position_tracker\nid: " + id + "\nkeep_every_nth_tick: " + std::to_string(every) + "\n");

    std::printf("Recording to %s\nSPACE = next marker, Q = stop.\n\n", csvPath.string().c_str());
    dev.startRecording();

    int marker = 0;
    long long rows = 0;
    bool prevBtn = false;
    double lastT = 0;
    phantom::State last;
    bool quit = false;
    int loops = 0;
    while (!quit) {
        phantom::State s;
        while (dev.popSample(s)) {
            if (s.tick % every == 0) { util::writeSample(csv, s, marker); ++rows; }
            if (s.button1() && !prevBtn) std::printf("\n  button 1 pressed at t=%.3f s\n", s.t);
            prevBtn = s.button1();
            lastT = s.t;
            last = s;
        }
        while (console::keyPressed()) {
            int c = console::readKey();
            if (c == 'q' || c == 'Q' || c == 27 || c < 0) quit = true;
            if (c == ' ') std::printf("\n  marker -> %d at t=%.3f s\n", ++marker, lastT);
        }
        if (dev.tripped() == phantom::Trip::ServoStopped) {
            std::printf("\nServo loop stopped (device communication error). Saving what we have.\n");
            quit = true;
        }
        if (++loops % 25 == 0) {
            std::printf("\rt=%7.1f s  rows=%lld  marker=%d  pos=(%6.1f %6.1f %6.1f)  %4.0f Hz ", lastT, rows, marker,
                        last.pos.x, last.pos.y, last.pos.z, dev.servoRateHz());
            std::fflush(stdout);
        }
        util::sleepMs(20);
    }

    dev.stopRecording();
    phantom::State s;
    while (dev.popSample(s)) if (s.tick % every == 0) { util::writeSample(csv, s, marker); ++rows; }
    std::fclose(csv);
    dev.close();

    std::printf("\n\nSaved %lld rows (%.1f s) to %s\n", rows, lastT, csvPath.string().c_str());
    if (dev.droppedSamples()) std::printf("WARNING: %llu samples dropped (disk too slow?)\n",
                                          (unsigned long long)dev.droppedSamples());
    return 0;
}
