// Simulated PHANToM for the mock <HD/hd.h>. See the header for what it does.

#define _USE_MATH_DEFINES  // M_PI on MSVC
#include <HD/hd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

struct Sim {
    // "Device" limits, roughly a Premium 1.5
    double maxForce = 8.5, maxCont = 1.4, maxStiff = 3.5, maxDamp = 0.005;
    // Physics: point mass + hand (weak spring to a slowly circling target + damping)
    double mass = 0.15;        // kg
    double handK = 0.02;       // N/mm
    double handB = 0.003;      // N*s/mm
    double pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
    double cmdForce[3] = {0, 0, 0}, frameForce[3] = {0, 0, 0};
    double t = 0;
    int buttons = 0;
    double buttonPeriod = 3.0;
    bool forceOut = true, calibrated = false;

    void step(double dt) {
        double w = 2 * M_PI * 0.3;
        double tgt[3] = {30 * std::cos(w * t), 20 * std::sin(w * t), 10 * std::sin(0.5 * w * t)};
        for (int i = 0; i < 3; ++i) {
            double f = handK * (tgt[i] - pos[i]) - handB * vel[i] + (forceOut ? cmdForce[i] : 0);
            vel[i] += f / mass * 1000.0 * dt;  // N/kg = m/s^2 -> mm/s^2
            pos[i] += vel[i] * dt;
        }
        t += dt;
        double ph = std::fmod(t, buttonPeriod);
        buttons = (ph > buttonPeriod - 0.06) ? 1 : 0;
    }
};

Sim g;
std::mutex gMu;
std::condition_variable gCv;
std::thread gThread;
std::atomic<bool> gRunning{false};

struct Async { HDSchedulerHandle id; HDSchedulerCallback cb; void* data; bool active; };
std::vector<Async> gAsync;
struct SyncJob { HDSchedulerCallback cb; void* data; bool done; };
std::vector<SyncJob*> gSync;
HDSchedulerHandle gNextId = 1;
std::vector<HDErrorInfo> gErrors;

void schedulerLoop() {
    using clk = std::chrono::steady_clock;
    auto next = clk::now();
    while (gRunning) {
        next += std::chrono::microseconds(1000);
        std::vector<Async> cbs;
        {
            std::lock_guard<std::mutex> l(gMu);
            cbs = gAsync;
        }
        for (auto& a : cbs)
            if (a.active && a.cb(a.data) == HD_CALLBACK_DONE) {
                std::lock_guard<std::mutex> l(gMu);
                for (auto& x : gAsync) if (x.id == a.id) x.active = false;
            }
        {
            std::unique_lock<std::mutex> l(gMu);
            auto jobs = gSync;
            gSync.clear();
            l.unlock();
            for (auto* j : jobs) {
                while (j->cb(j->data) == HD_CALLBACK_CONTINUE) {}
                l.lock();
                j->done = true;
                l.unlock();
            }
            gCv.notify_all();
        }
        g.step(0.001);
        std::this_thread::sleep_until(next);
    }
}

}  // namespace

extern "C" {

HHD hdInitDevice(HDstring) {
    if (const char* p = std::getenv("HD_MOCK_BUTTON_PERIOD")) g.buttonPeriod = std::atof(p);
    return 0;
}
void hdDisableDevice(HHD) {}
void hdMakeCurrentDevice(HHD) {}
HHD hdGetCurrentDevice(void) { return 0; }

void hdBeginFrame(HHD) { std::memset(g.frameForce, 0, sizeof g.frameForce); }
void hdEndFrame(HHD) { std::memcpy(g.cmdForce, g.frameForce, sizeof g.cmdForce); }

void hdGetBooleanv(HDenum, HDboolean* p) { *p = 0; }
void hdGetIntegerv(HDenum e, HDint* p) {
    switch (e) {
    case HD_CURRENT_BUTTONS: *p = g.buttons; break;
    case HD_INPUT_DOF: *p = 6; break;
    case HD_OUTPUT_DOF: *p = 3; break;
    case HD_CALIBRATION_STYLE: *p = HD_CALIBRATION_ENCODER_RESET; break;
    default: *p = 0;
    }
}
void hdGetDoublev(HDenum e, HDdouble* p) {
    switch (e) {
    case HD_CURRENT_POSITION: std::memcpy(p, g.pos, 3 * sizeof(double)); break;
    case HD_CURRENT_VELOCITY: std::memcpy(p, g.vel, 3 * sizeof(double)); break;
    case HD_CURRENT_FORCE: std::memcpy(p, g.cmdForce, 3 * sizeof(double)); break;
    case HD_CURRENT_JOINT_ANGLES:
        p[0] = std::atan2(g.pos[0], 200.0); p[1] = g.pos[1] / 300.0; p[2] = g.pos[2] / 300.0; break;
    case HD_CURRENT_GIMBAL_ANGLES: p[0] = 0.1 * std::sin(g.t); p[1] = 0.05; p[2] = 0.0; break;
    case HD_NOMINAL_MAX_FORCE: *p = g.maxForce; break;
    case HD_NOMINAL_MAX_CONTINUOUS_FORCE: *p = g.maxCont; break;
    case HD_NOMINAL_MAX_STIFFNESS: *p = g.maxStiff; break;
    case HD_NOMINAL_MAX_DAMPING: *p = g.maxDamp; break;
    case HD_USABLE_WORKSPACE_DIMENSIONS: {
        const double w[6] = {-190, -135, -85, 190, 135, 85};
        std::memcpy(p, w, sizeof w);
        break;
    }
    default: *p = 0;
    }
}
HDstring hdGetString(HDenum e) {
    switch (e) {
    case HD_DEVICE_MODEL_TYPE: return "MOCK PHANToM Premium (simulated)";
    case HD_DEVICE_VENDOR: return "mock";
    case HD_DEVICE_SERIAL_NUMBER: return "00000";
    default: return "";
    }
}
void hdSetDoublev(HDenum e, const HDdouble* p) {
    if (e == HD_CURRENT_FORCE) std::memcpy(g.frameForce, p, 3 * sizeof(double));
}

void hdEnable(HDenum c) { if (c == HD_FORCE_OUTPUT) g.forceOut = true; }
void hdDisable(HDenum c) { if (c == HD_FORCE_OUTPUT) g.forceOut = false; }
HDboolean hdIsEnabled(HDenum c) { return c == HD_FORCE_OUTPUT ? g.forceOut : 0; }

HDErrorInfo hdGetError(void) {
    std::lock_guard<std::mutex> l(gMu);
    if (gErrors.empty()) return HDErrorInfo{HD_SUCCESS, 0, 0};
    HDErrorInfo e = gErrors.back();
    gErrors.pop_back();
    return e;
}
HDstring hdGetErrorString(HDerror c) {
    switch (c) {
    case HD_SUCCESS: return "HD_SUCCESS";
    case HD_EXCEEDED_MAX_FORCE: return "HD_EXCEEDED_MAX_FORCE";
    case HD_EXCEEDED_MAX_VELOCITY: return "HD_EXCEEDED_MAX_VELOCITY";
    case HD_COMM_ERROR: return "HD_COMM_ERROR";
    default: return "HD_MOCK_ERROR";
    }
}

HDenum hdCheckCalibration(void) { return g.calibrated ? HD_CALIBRATION_OK : HD_CALIBRATION_NEEDS_MANUAL_INPUT; }
void hdUpdateCalibration(HDenum) { g.calibrated = true; }

void hdStartScheduler(void) {
    if (gRunning.exchange(true)) return;
    gThread = std::thread(schedulerLoop);
}
void hdStopScheduler(void) {
    if (!gRunning.exchange(false)) return;
    gThread.join();
}
HDSchedulerHandle hdScheduleAsynchronous(HDSchedulerCallback cb, void* d, HDushort) {
    std::lock_guard<std::mutex> l(gMu);
    gAsync.push_back(Async{gNextId, cb, d, true});
    return gNextId++;
}
void hdScheduleSynchronous(HDSchedulerCallback cb, void* d, HDushort) {
    if (!gRunning) { while (cb(d) == HD_CALLBACK_CONTINUE) {} return; }
    SyncJob job{cb, d, false};
    std::unique_lock<std::mutex> l(gMu);
    gSync.push_back(&job);
    gCv.wait(l, [&] { return job.done; });
}
void hdUnschedule(HDSchedulerHandle h) {
    std::lock_guard<std::mutex> l(gMu);
    for (auto& a : gAsync) if (a.id == h) a.active = false;
}

}  // extern "C"
