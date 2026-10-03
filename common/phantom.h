#pragma once
// phantom.h - a small, *safe* layer over the OpenHaptics HD API.
//
// Every program in apps/ goes through this file instead of calling hd* directly.
// The raw HD examples leave all safety to you, and most "the arm suddenly goes
// crazy" problems come from one of these (see docs/TROUBLESHOOTING.md):
//   * the Premium was not calibrated (encoder reset) this power-up, so positions
//     are offset and any spring/wall is far from where the arm really is
//   * a spring anchored at a fixed point (often the origin) instead of where the
//     arm is when forces turn on -> full-force jump on the first servo tick
//   * stiffness in the wrong units: HD uses newtons and millimetres, so k is N/mm
//     (k = 1 N/mm with a 50 mm error is 50 N)
//   * reading/writing shared variables from the graphics thread without syncing
//
// What this layer does for you:
//   * prints device info and runs calibration before anything moves
//   * runs ONE servo callback (~1 kHz) that calls your force function
//   * clamps force magnitude, fades forces in, and trips (forces off, latched)
//     if the arm moves faster than a speed limit, the force is NaN, or the
//     driver reports a force error
//   * records every servo sample into a lock-free queue so the main thread can
//     write a CSV without ever blocking the servo loop
//
// Units everywhere: position mm, velocity mm/s, force N, angles rad, time s.
// Device frame (Premium, facing the device): +x right, +y up, +z toward you.

#include <HD/hd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "console.h"

namespace phantom {

// ---------------------------------------------------------------------------
// Vec3: plain 3-vector with the few operations force laws need.
// ---------------------------------------------------------------------------
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    static Vec3 from(const double* v) { return Vec3(v[0], v[1], v[2]); }

    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    double norm() const { return std::sqrt(x * x + y * y + z * z); }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// ---------------------------------------------------------------------------
// One servo-loop sample. Plain data so it can live in the lock-free queue.
// ---------------------------------------------------------------------------
struct State {
    long long tick = 0;     // servo tick counter since start()
    double t = 0;           // s since start()
    Vec3 pos;               // mm   HD_CURRENT_POSITION
    Vec3 vel;               // mm/s HD_CURRENT_VELOCITY (driver-filtered)
    Vec3 force;             // N    force actually sent this tick, after safety
    double joint[3] = {};   // rad  HD_CURRENT_JOINT_ANGLES (base, shoulder, elbow)
    double gimbal[3] = {};  // rad  HD_CURRENT_GIMBAL_ANGLES (wrist/gimbal)
    int buttons = 0;        // HD_CURRENT_BUTTONS bitmask (HD_DEVICE_BUTTON_1, _2)
    // App-defined tags. Your force function may set these; they get logged.
    int phase = -1;
    int trial = -1;

    bool button1() const { return (buttons & HD_DEVICE_BUTTON_1) != 0; }
    bool button2() const { return (buttons & HD_DEVICE_BUTTON_2) != 0; }
};

// Called on the servo thread ~1000x per second. Keep it FAST: no printing, no
// file I/O, no memory allocation, no locks. Return the force you want (N).
using ForceFn = std::function<Vec3(State&)>;

struct Info {
    std::string model, vendor, serial;
    double maxForce = 0;            // N, peak       (HD_NOMINAL_MAX_FORCE)
    double maxContinuousForce = 0;  // N, sustained  (HD_NOMINAL_MAX_CONTINUOUS_FORCE)
    double maxStiffness = 0;        // N/mm          (HD_NOMINAL_MAX_STIFFNESS)
    double maxDamping = 0;          // N*s/mm        (HD_NOMINAL_MAX_DAMPING)
    double workspace[6] = {};       // mm, usable box: minX minY minZ maxX maxY maxZ
    int inputDOF = 0, outputDOF = 0;
    int calibrationStyles = 0;
    bool calibrated = false;
};

struct Safety {
    double maxForce = 0;     // N. 0 = 60% of the device's continuous force
    double maxSpeed = 1000;  // mm/s. Faster than this while forces are on -> trip
    double rampTime = 1.0;   // s to fade forces in each time they are enabled
};

enum class Trip { None = 0, Speed, NotFinite, DriverForceError, ServoStopped };

inline const char* tripName(Trip t) {
    switch (t) {
    case Trip::None: return "none";
    case Trip::Speed: return "arm moved faster than safety.maxSpeed while forces were on";
    case Trip::NotFinite: return "force function returned NaN/inf";
    case Trip::DriverForceError: return "driver reported a force/velocity error (see last error)";
    case Trip::ServoStopped: return "servo loop stopped (communication/timer error)";
    }
    return "?";
}

// Prints and clears any pending HD errors. Returns true if there was one.
inline bool reportErrors(const char* where) {
    bool any = false;
    for (int i = 0; i < 16; ++i) {
        HDErrorInfo e = hdGetError();
        if (!HD_DEVICE_ERROR(e)) break;
        std::fprintf(stderr, "[HD error] %s: %s (code 0x%04x, internal %d)\n", where,
                     hdGetErrorString(e.errorCode), (unsigned)e.errorCode, e.internalErrorCode);
        any = true;
    }
    return any;
}

inline bool isForceError(HDerror c) {
    return c == HD_EXCEEDED_MAX_FORCE || c == HD_EXCEEDED_MAX_FORCE_IMPULSE ||
           c == HD_EXCEEDED_MAX_VELOCITY || c == HD_FORCE_ERROR;
}
inline bool isSchedulerError(HDerror c) {
    return c == HD_COMM_ERROR || c == HD_TIMER_ERROR || c == HD_ILLEGAL_BEGIN || c == HD_ILLEGAL_END;
}

// ---------------------------------------------------------------------------
// Single-producer / single-consumer ring buffer (servo thread -> main thread).
// ---------------------------------------------------------------------------
class SampleQueue {
public:
    explicit SampleQueue(size_t capacityPow2 = size_t(1) << 17)  // ~130 s at 1 kHz
        : buf_(capacityPow2), mask_(capacityPow2 - 1) {}
    bool push(const State& s) {
        size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= buf_.size()) return false;
        buf_[h & mask_] = s;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }
    bool pop(State& s) {
        size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        s = buf_[t & mask_];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }
private:
    std::vector<State> buf_;
    size_t mask_;
    std::atomic<size_t> head_{0}, tail_{0};
};

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------
class Device {
public:
    Safety safety;

    Device() = default;
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;
    ~Device() { close(); }

    // Initialise the device, print its info, and calibrate.
    // configName: name from "Phantom Configuration"; empty/null = default device.
    bool open(const char* configName = HD_DEFAULT_DEVICE, bool skipCalibration = false) {
        if (configName && !*configName) configName = HD_DEFAULT_DEVICE;
        hhd_ = hdInitDevice(configName);
        HDErrorInfo e = hdGetError();
        if (HD_DEVICE_ERROR(e)) {
            std::fprintf(stderr, "\nCould not open the haptic device: %s\n", hdGetErrorString(e.errorCode));
            std::fprintf(stderr,
                "  * Is another program using it? Only ONE program can open the device at a time.\n"
                "    Close PHANToM Test / Touch Smart Setup / any OpenHaptics example first.\n"
                "  * Open 'Phantom Configuration' and check a device exists and its Test passes.\n"
                "  * If you have several configurations, pass --device \"<name>\".\n");
            hhd_ = HD_INVALID_HANDLE;
            return false;
        }
        readInfo();
        printInfo();
        if (!calibrate(skipCalibration)) return false;
        return true;
    }

    const Info& info() const { return info_; }

    // Set before start(). See ForceFn for the rules.
    void setForceFunction(ForceFn f) { forceFn_ = std::move(f); }

    // Start the servo loop. forces=false: read-only tracking, motors never driven.
    bool start(bool forces) {
        if (hhd_ == HD_INVALID_HANDLE) return false;
        forcesWanted_ = forces;
        if (safety.maxForce <= 0)
            safety.maxForce = info_.maxContinuousForce > 0 ? 0.6 * info_.maxContinuousForce : 1.0;
        if (forces) {
            hdEnable(HD_FORCE_OUTPUT);
            hdEnable(HD_MAX_FORCE_CLAMPING);
            std::printf("Force limit: %.2f N  | speed trip: %.0f mm/s | fade-in: %.1f s\n",
                        safety.maxForce, safety.maxSpeed, safety.rampTime);
        } else {
            hdDisable(HD_FORCE_OUTPUT);
        }
        t0_ = std::chrono::steady_clock::now();
        handle_ = hdScheduleAsynchronous(&Device::servoCallback, this, HD_MAX_SCHEDULER_PRIORITY);
        hdStartScheduler();
        running_ = true;  // so close() cleans up even if the start failed
        if (reportErrors("starting scheduler")) return false;
        // Wait for the first sample so latest() is meaningful right away.
        for (int i = 0; i < 200 && tickCount_.load() == 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        return true;
    }

    // Stop servo loop and release the device. Safe to call more than once.
    void close() {
        if (running_) {
            forcesOn_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));  // let zero force go out
            hdStopScheduler();
            hdUnschedule(handle_);
            running_ = false;
        }
        if (hhd_ != HD_INVALID_HANDLE) {
            hdDisableDevice(hhd_);
            hhd_ = HD_INVALID_HANDLE;
        }
    }

    // Turn forces on (with fade-in) or off (immediately). Needs start(true).
    void enableForces(bool on) { forcesOn_.store(on); }
    bool forcesEnabled() const { return forcesOn_.load(); }

    // Recording: every servo sample goes into the queue; drain it with popSample().
    void startRecording() { recording_.store(true); }
    void stopRecording() { recording_.store(false); }
    bool popSample(State& s) { return queue_.pop(s); }
    uint64_t droppedSamples() const { return dropped_.load(); }

    // Thread-safe copy of the most recent sample (runs a tiny job in the servo thread).
    State latest() {
        State s;
        if (running_) {
            CopyJob job{this, &s};
            hdScheduleSynchronous(&Device::copyCallback, &job, HD_DEFAULT_SCHEDULER_PRIORITY);
        }
        return s;
    }

    // Run fn inside the servo thread, between ticks. Use it to change anything
    // your force function reads (targets, stiffness...) without data races.
    // Do NOT call latest() or runInServo() from inside fn (it would deadlock).
    void runInServo(const std::function<void()>& fn) {
        if (!running_) { fn(); return; }
        hdScheduleSynchronous(&Device::runCallback, const_cast<std::function<void()>*>(&fn),
                              HD_DEFAULT_SCHEDULER_PRIORITY);
    }

    Trip tripped() const { return static_cast<Trip>(trip_.load()); }
    void resetTrip() { trip_.store(0); }
    HDerror lastError() const { return lastError_.load(); }
    double servoRateHz() const { return rateHz_.load(); }
    bool running() const { return running_; }

private:
    struct CopyJob { Device* dev; State* out; };

    static HDCallbackCode HDCALLBACK copyCallback(void* p) {
        auto* j = static_cast<CopyJob*>(p);
        *j->out = j->dev->last_;
        return HD_CALLBACK_DONE;
    }
    static HDCallbackCode HDCALLBACK runCallback(void* p) {
        (*static_cast<std::function<void()>*>(p))();
        return HD_CALLBACK_DONE;
    }

    void doTrip(Trip t) {
        int expected = 0;
        trip_.compare_exchange_strong(expected, static_cast<int>(t));
    }

    static HDCallbackCode HDCALLBACK servoCallback(void* p) {
        Device* d = static_cast<Device*>(p);
        State s;
        s.tick = d->tick_++;
        s.t = std::chrono::duration<double>(std::chrono::steady_clock::now() - d->t0_).count();

        hdBeginFrame(d->hhd_);
        double v[3];
        hdGetDoublev(HD_CURRENT_POSITION, v); s.pos = Vec3::from(v);
        hdGetDoublev(HD_CURRENT_VELOCITY, v); s.vel = Vec3::from(v);
        hdGetDoublev(HD_CURRENT_JOINT_ANGLES, s.joint);
        hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, s.gimbal);
        hdGetIntegerv(HD_CURRENT_BUTTONS, &s.buttons);

        Vec3 f;
        if (d->forcesWanted_) {
            bool on = d->forcesOn_.load(std::memory_order_relaxed) && d->trip_.load() == 0;
            if (on) {
                if (!d->wasOn_) { d->onSince_ = s.t; d->wasOn_ = true; }
                if (d->forceFn_) f = d->forceFn_(s);
                if (!f.finite()) { d->doTrip(Trip::NotFinite); f = Vec3(); }
                // Fade in so a force that is "already large" never arrives in one tick.
                double ramp = d->safety.rampTime > 0 ? (s.t - d->onSince_) / d->safety.rampTime : 1.0;
                if (ramp < 1.0) f = f * (ramp < 0 ? 0 : ramp);
                double mag = f.norm();
                if (mag > d->safety.maxForce) f = f * (d->safety.maxForce / mag);
                if (s.vel.norm() > d->safety.maxSpeed) { d->doTrip(Trip::Speed); f = Vec3(); }
                if (d->trip_.load() != 0) f = Vec3();
            } else {
                d->wasOn_ = false;
                // Still let the task see the state (e.g. to wait for a button).
                if (d->forceFn_ && d->callFnWhenOff_) d->forceFn_(s);
            }
            double out[3] = {f.x, f.y, f.z};
            hdSetDoublev(HD_CURRENT_FORCE, out);
        } else if (d->forceFn_) {
            d->forceFn_(s);  // tracking-only apps can still tag samples
        }
        s.force = f;
        hdEndFrame(d->hhd_);

        for (int i = 0; i < 4; ++i) {
            HDErrorInfo e = hdGetError();
            if (!HD_DEVICE_ERROR(e)) break;
            d->lastError_.store(e.errorCode);
            if (isForceError(e.errorCode)) d->doTrip(Trip::DriverForceError);
            if (isSchedulerError(e.errorCode)) {
                d->doTrip(Trip::ServoStopped);
                return HD_CALLBACK_DONE;
            }
        }

        d->last_ = s;
        if (d->recording_.load(std::memory_order_relaxed) && !d->queue_.push(s)) d->dropped_++;

        d->tickCount_.store(d->tick_, std::memory_order_relaxed);
        if (s.tick % 1000 == 0) {
            if (s.tick > 0) d->rateHz_.store(1000.0 / (s.t - d->rateT_));
            d->rateT_ = s.t;
        }
        return HD_CALLBACK_CONTINUE;
    }

    void readInfo() {
        auto str = [](HDenum e) { HDstring s = hdGetString(e); return std::string(s ? s : "?"); };
        info_.model = str(HD_DEVICE_MODEL_TYPE);
        info_.vendor = str(HD_DEVICE_VENDOR);
        info_.serial = str(HD_DEVICE_SERIAL_NUMBER);
        hdGetDoublev(HD_NOMINAL_MAX_FORCE, &info_.maxForce);
        hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE, &info_.maxContinuousForce);
        hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &info_.maxStiffness);
        hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &info_.maxDamping);
        hdGetDoublev(HD_USABLE_WORKSPACE_DIMENSIONS, info_.workspace);
        hdGetIntegerv(HD_INPUT_DOF, &info_.inputDOF);
        hdGetIntegerv(HD_OUTPUT_DOF, &info_.outputDOF);
        hdGetIntegerv(HD_CALIBRATION_STYLE, &info_.calibrationStyles);
        reportErrors("reading device info");
    }

    void printInfo() const {
        const Info& i = info_;
        std::printf("----------------------------------------------------------------\n");
        std::printf(" Device        : %s (%s), serial %s\n", i.model.c_str(), i.vendor.c_str(), i.serial.c_str());
        std::printf(" DOF in/out    : %d / %d\n", i.inputDOF, i.outputDOF);
        std::printf(" Max force     : %.2f N peak, %.2f N continuous\n", i.maxForce, i.maxContinuousForce);
        std::printf(" Max stiffness : %.3f N/mm   max damping: %.4f N*s/mm\n", i.maxStiffness, i.maxDamping);
        std::printf(" Workspace (mm): x[%.0f, %.0f] y[%.0f, %.0f] z[%.0f, %.0f]\n", i.workspace[0], i.workspace[3],
                    i.workspace[1], i.workspace[4], i.workspace[2], i.workspace[5]);
        std::printf(" Calibration   :%s%s%s\n", (i.calibrationStyles & HD_CALIBRATION_ENCODER_RESET) ? " encoder-reset" : "",
                    (i.calibrationStyles & HD_CALIBRATION_INKWELL) ? " inkwell" : "",
                    (i.calibrationStyles & HD_CALIBRATION_AUTO) ? " auto" : "");
        std::printf("----------------------------------------------------------------\n");
    }

    // Mirrors the OpenHaptics "Calibration" example, including its preference
    // auto > inkwell > encoder reset. Called from open(), before start().
    bool calibrate(bool skip) {
        int styles = info_.calibrationStyles;
        // The lab's "Premium HID" reports encoder-reset AND inkwell, and refuses the
        // encoder reset (HD_INVALID_OPERATION), so only reset devices that offer nothing else.
        if ((styles & (HD_CALIBRATION_AUTO | HD_CALIBRATION_INKWELL)) && !skip) return calibrateInServo(styles);
        if ((styles & HD_CALIBRATION_ENCODER_RESET) && !skip) {
            // PHANToM Premium: encoders are relative, so after every power-up the
            // arm must be held in its reset position while we zero them.
            std::printf("\nCALIBRATION (encoder reset)\n"
                        "  Hold the arm in its reset position (the SDK says: all links\n"
                        "  orthogonal, the same pose PHANToM Test uses) and keep it still.\n"
                        "  Then press ENTER.\n");
            console::waitEnter();
            hdUpdateCalibration(HD_CALIBRATION_ENCODER_RESET);
            if (reportErrors("encoder reset")) return false;
        } else if (skip) {
            std::printf("Calibration skipped (--skip-calibration).\n");
        }

        HDenum status = hdCheckCalibration();
        if (status == HD_CALIBRATION_NEEDS_UPDATE && (styles & (HD_CALIBRATION_AUTO | HD_CALIBRATION_INKWELL))) {
            if (!(styles & HD_CALIBRATION_AUTO)) {
                std::printf("Put the stylus in the inkwell, then press ENTER.\n");
                console::waitEnter();
            }
            hdUpdateCalibration((styles & HD_CALIBRATION_AUTO) ? HD_CALIBRATION_AUTO : HD_CALIBRATION_INKWELL);
            status = hdCheckCalibration();
        }
        reportErrors("checking calibration");
        info_.calibrated = (status == HD_CALIBRATION_OK);
        std::printf("Calibration status: %s\n\n",
                    info_.calibrated ? "OK"
                    : status == HD_CALIBRATION_NEEDS_MANUAL_INPUT ? "NEEDS MANUAL INPUT (hold reset position and rerun)"
                                                                  : "NEEDS UPDATE");
        return true;
    }

    // Inkwell / auto calibration needs the servo loop running, so start the
    // scheduler just for this and stop it again (start() restarts it). Forces are
    // never enabled here.
    static HDCallbackCode HDCALLBACK calStatusCallback(void* p) {
        hdBeginFrame(hdGetCurrentDevice());
        *static_cast<HDenum*>(p) = hdCheckCalibration();
        hdEndFrame(hdGetCurrentDevice());
        return HD_CALLBACK_DONE;
    }
    static HDCallbackCode HDCALLBACK calUpdateCallback(void* p) {
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) hdUpdateCalibration(*static_cast<HDenum*>(p));
        return HD_CALLBACK_DONE;
    }

    bool calibrateInServo(int styles) {
        HDenum style = (styles & HD_CALIBRATION_AUTO) ? HD_CALIBRATION_AUTO : HD_CALIBRATION_INKWELL;
        hdStartScheduler();
        if (reportErrors("starting scheduler for calibration")) { hdStopScheduler(); return false; }

        HDenum status = HD_CALIBRATION_NEEDS_UPDATE;
        hdScheduleSynchronous(calStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);
        if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
            std::printf("\nCALIBRATION (%s)\n"
                        "  Put the stylus in its inkwell (the holder on the base) and keep it still.\n"
                        "  Waiting for the device to report calibrated... (press any key to give up)\n",
                        style == HD_CALIBRATION_AUTO ? "auto" : "inkwell");
        }
        // Poll like the SDK example: apply an update whenever the driver asks for one.
        while (status != HD_CALIBRATION_OK) {
            if (status == HD_CALIBRATION_NEEDS_UPDATE)
                hdScheduleSynchronous(calUpdateCallback, &style, HD_DEFAULT_SCHEDULER_PRIORITY);
            if (reportErrors("updating calibration")) break;
            if (console::keyPressed()) { console::readKey(); break; }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            hdScheduleSynchronous(calStatusCallback, &status, HD_DEFAULT_SCHEDULER_PRIORITY);
        }
        hdStopScheduler();
        reportErrors("checking calibration");
        info_.calibrated = (status == HD_CALIBRATION_OK);
        std::printf("Calibration status: %s\n\n",
                    info_.calibrated ? "OK"
                    : status == HD_CALIBRATION_NEEDS_MANUAL_INPUT ? "NEEDS MANUAL INPUT (put the stylus in the inkwell and rerun)"
                                                                  : "NEEDS UPDATE");
        return true;
    }

    // --- set in main thread before start(), then read-only
    HHD hhd_ = HD_INVALID_HANDLE;
    HDSchedulerHandle handle_ = 0;
    Info info_;
    ForceFn forceFn_;
    bool forcesWanted_ = false;
    bool callFnWhenOff_ = true;
    bool running_ = false;
    std::chrono::steady_clock::time_point t0_;

    // --- shared between threads
    std::atomic<bool> forcesOn_{false};
    std::atomic<bool> recording_{false};
    std::atomic<int> trip_{0};
    std::atomic<HDerror> lastError_{HD_SUCCESS};
    std::atomic<uint64_t> dropped_{0};
    std::atomic<long long> tickCount_{0};
    std::atomic<double> rateHz_{0};
    SampleQueue queue_;

    // --- servo thread only
    long long tick_ = 0;
    State last_;
    bool wasOn_ = false;
    double onSince_ = 0, rateT_ = 0;
};

// Handy helpers for force laws -------------------------------------------------

// Minimum-jerk interpolation from a to b over duration T (the classic smooth
// point-to-point human reaching profile, Flash & Hogan 1985).
inline void minJerk(const Vec3& a, const Vec3& b, double T, double t, Vec3& pos, Vec3& vel) {
    double tau = T > 0 ? t / T : 1.0;
    if (tau <= 0) { pos = a; vel = Vec3(); return; }
    if (tau >= 1) { pos = b; vel = Vec3(); return; }
    double t3 = tau * tau * tau;
    double s = t3 * (10 - 15 * tau + 6 * tau * tau);
    double ds = 30 * tau * tau * (1 - 2 * tau + tau * tau) / T;
    pos = a + (b - a) * s;
    vel = (b - a) * ds;
}

// Spring-damper pulling toward a (moving) set point. k: N/mm, b: N*s/mm.
inline Vec3 springDamper(const State& s, const Vec3& target, const Vec3& targetVel, double k, double b) {
    return (target - s.pos) * k + (targetVel - s.vel) * b;
}

}  // namespace phantom
