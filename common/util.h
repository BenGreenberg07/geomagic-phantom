#pragma once
// util.h - command-line args, timestamps, data folder, CSV writing.

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

#include "phantom.h"

namespace util {

// Tiny "--name value" / "--flag" parser.
class Args {
public:
    Args(int argc, char** argv) : v_(argv + 1, argv + argc) {}
    bool flag(const std::string& name) const {
        for (auto& a : v_) if (a == name) return true;
        return false;
    }
    std::string get(const std::string& name, const std::string& def) const {
        for (size_t i = 0; i + 1 < v_.size(); ++i) if (v_[i] == name) return v_[i + 1];
        return def;
    }
    double num(const std::string& name, double def) const {
        std::string s = get(name, "");
        return s.empty() ? def : std::atof(s.c_str());
    }
private:
    std::vector<std::string> v_;
};

inline std::tm localNow() {
    std::time_t now = std::time(nullptr);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &now);
#else
    localtime_r(&now, &tm);
#endif
    return tm;
}

// "20260928_141503" - used in file names so runs never overwrite each other.
inline std::string stamp() {
    std::tm tm = localNow();
    char b[32];
    std::strftime(b, sizeof b, "%Y%m%d_%H%M%S", &tm);
    return b;
}

// "2026-09-28 14:15:03"
inline std::string humanTime() {
    std::tm tm = localNow();
    char b[32];
    std::strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S", &tm);
    return b;
}

// <repo>/data when the exe lives in <repo>/bin (or bin/sim), otherwise ./data. Created if missing.
inline std::filesystem::path dataDir(const char* argv0) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path exeDir = fs::absolute(argv0, ec).lexically_normal().parent_path();
    if (exeDir.filename() == "sim" && exeDir.parent_path().filename() == "bin") exeDir = exeDir.parent_path();  // bin\sim
    fs::path dir = (exeDir.filename() == "bin") ? exeDir.parent_path() / "data" : fs::current_path() / "data";
    fs::create_directories(dir, ec);
    return dir;
}

// Opens a CSV with a large buffer (the servo loop produces 1000 rows/s).
inline FILE* openCsv(const std::filesystem::path& p) {
    FILE* f = std::fopen(p.string().c_str(), "w");
    if (!f) { std::fprintf(stderr, "Could not open %s for writing\n", p.string().c_str()); return nullptr; }
    std::setvbuf(f, nullptr, _IOFBF, 1 << 20);
    return f;
}

// Standard per-sample columns shared by every app, so one analysis script reads them all.
inline void writeSampleHeader(FILE* f) {
    std::fprintf(f, "t_s,x_mm,y_mm,z_mm,vx_mm_s,vy_mm_s,vz_mm_s,fx_N,fy_N,fz_N,"
                    "joint0_rad,joint1_rad,joint2_rad,gimbal0_rad,gimbal1_rad,gimbal2_rad,"
                    "button1,button2,phase,trial,marker\n");
}
inline void writeSample(FILE* f, const phantom::State& s, int marker) {
    std::fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%d,%d,%d,%d\n",
                 s.t, s.pos.x, s.pos.y, s.pos.z, s.vel.x, s.vel.y, s.vel.z, s.force.x, s.force.y, s.force.z,
                 s.joint[0], s.joint[1], s.joint[2], s.gimbal[0], s.gimbal[1], s.gimbal[2],
                 s.button1() ? 1 : 0, s.button2() ? 1 : 0, s.phase, s.trial, marker);
}

// Small text file next to the CSV recording who/what/when (session metadata).
inline void writeInfoFile(const std::filesystem::path& csv, const phantom::Device& dev, const std::string& extra) {
    std::filesystem::path p = csv;
    p.replace_extension(".info.txt");
    FILE* f = std::fopen(p.string().c_str(), "w");
    if (!f) return;
    const phantom::Info& i = dev.info();
    std::fprintf(f, "started: %s\ndevice: %s (%s) serial %s\ncalibrated: %s\n"
                    "force_limit_N: %.3f\nspeed_trip_mm_s: %.1f\n%s",
                 humanTime().c_str(), i.model.c_str(), i.vendor.c_str(), i.serial.c_str(),
                 i.calibrated ? "yes" : "NO", dev.safety.maxForce, dev.safety.maxSpeed, extra.c_str());
    std::fclose(f);
}

inline void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

}  // namespace util
