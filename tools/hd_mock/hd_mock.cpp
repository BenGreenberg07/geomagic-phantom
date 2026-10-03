// Simulated PHANToM for the mock <HD/hd.h>. See the header for what it does.

#define _USE_MATH_DEFINES  // M_PI on MSVC
#include <HD/hd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace {

struct Sim {
    // "Device" limits: the lab's Premium HID (docs/LAB_LOG.md, 2026-10-02)
    double maxForce = 37.27, maxCont = 6.14, maxStiff = 1.0, maxDamp = 0.005;
    double workspace[6] = {-80, -50, -50, 80, 70, 70};
    // Arm physics per axis: effective mass, Coulomb friction, viscous friction, and a
    // constant force from imperfect gravity balance. PLACEHOLDERS until bin\device_id
    // has measured the real arm; override with HD_MOCK_MASS / _FRICTION / _VISCOUS / _GRAVITY.
    double mass[3] = {0.12, 0.12, 0.12};      // kg
    double coulomb[3] = {0.05, 0.05, 0.05};   // N
    double viscous[3] = {0.0005, 0.0005, 0.0005};  // N*s/mm
    double gravity[3] = {0, 0, 0};            // N (y < 0 = the arm sags when let go)
    // Hand: a spring + damper from the stylus to where the hand "wants" to be.
    double handK = 0.02;       // N/mm (lazy hand for the circling / still modes)
    double handB = 0.003;      // N*s/mm
    double pos[3] = {0, 0, 0}, vel[3] = {0, 0, 0};
    double cmdForce[3] = {0, 0, 0}, frameForce[3] = {0, 0, 0};
    double t = 0;
    int buttons = 0;
    double buttonPeriod = 3.0;
    bool forceOut = true, calibrated = true;
    int calStyles = HD_CALIBRATION_ENCODER_RESET | HD_CALIBRATION_INKWELL;  // like the lab device
    double flip[3] = {1, 1, 1};  // HD_MOCK_FLIP=x (or y, z, xyz...): that motor pushes backwards
    bool stillHand = false;    // HD_MOCK_HAND=still: hand rests at the origin instead of circling
    // External hand (the 3D apps' mouse in the simulator, via hdMockSetHand()).
    bool extHand = false;
    double extTarget[3] = {0, 0, 0}, extSmooth[3] = {0, 0, 0};
    int extButtons = 0;

    void step(double dt) {
        double tgt[3];
        double hk = handK, hb = handB;
        if (extHand) {
            double want[3];
            {
                std::lock_guard<std::mutex> l(handMu);
                for (int i = 0; i < 3; ++i) want[i] = extTarget[i];
                buttons = extButtons;
            }
            // A real hand can't teleport: move toward the mouse at most 400 mm/s.
            double d[3], n = 0;
            for (int i = 0; i < 3; ++i) { d[i] = want[i] - extSmooth[i]; n += d[i] * d[i]; }
            n = std::sqrt(n);
            double maxStep = 400.0 * dt;
            for (int i = 0; i < 3; ++i) extSmooth[i] += (n > maxStep) ? d[i] * maxStep / n : d[i];
            for (int i = 0; i < 3; ++i) tgt[i] = extSmooth[i];
            hk = 0.15; hb = 0.004;  // a firm grip
        } else {
            double w = 2 * M_PI * 0.3;
            tgt[0] = 30 * std::cos(w * t); tgt[1] = 20 * std::sin(w * t); tgt[2] = 10 * std::sin(0.5 * w * t);
            if (stillHand) tgt[0] = tgt[1] = tgt[2] = 0;
            double ph = std::fmod(t, buttonPeriod);
            buttons = (ph > buttonPeriod - 0.06) ? 1 : 0;
        }
        for (int i = 0; i < 3; ++i) {
            double fric = coulomb[i] * std::tanh(vel[i] / 2.0) + viscous[i] * vel[i];  // smooth Coulomb
            double f = hk * (tgt[i] - pos[i]) - hb * vel[i] + (forceOut ? flip[i] * cmdForce[i] : 0) - fric + gravity[i];
            vel[i] += f / mass[i] * 1000.0 * dt;  // N/kg = m/s^2 -> mm/s^2
            pos[i] += vel[i] * dt;
        }
        t += dt;
    }
    std::mutex handMu;
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
    if (const char* p = std::getenv("HD_MOCK_FLIP"))
        for (int i = 0; i < 3; ++i) g.flip[i] = std::strchr(p, "xyz"[i]) ? -1 : 1;
    if (const char* p = std::getenv("HD_MOCK_HAND")) g.stillHand = std::strcmp(p, "still") == 0;
    if (const char* p = std::getenv("HD_MOCK_CAL"))  // "reset" = old encoder-reset-only device
        if (std::strcmp(p, "reset") == 0) { g.calStyles = HD_CALIBRATION_ENCODER_RESET; g.calibrated = false; }
    auto read3 = [](const char* name, double* v) {  // "0.1" (all axes) or "0.1,0.12,0.1"
        const char* p = std::getenv(name);
        if (!p) return;
        double a = 0, b = 0, c = 0;
        int n = std::sscanf(p, "%lf,%lf,%lf", &a, &b, &c);
        if (n == 1) v[0] = v[1] = v[2] = a;
        if (n == 3) { v[0] = a; v[1] = b; v[2] = c; }
    };
    read3("HD_MOCK_MASS", g.mass);
    read3("HD_MOCK_FRICTION", g.coulomb);
    read3("HD_MOCK_VISCOUS", g.viscous);
    read3("HD_MOCK_GRAVITY", g.gravity);
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
    case HD_OUTPUT_DOF: *p = 6; break;
    case HD_CALIBRATION_STYLE: *p = g.calStyles; break;
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
        std::memcpy(p, g.workspace, sizeof g.workspace);
        break;
    }
    default: *p = 0;
    }
}
HDstring hdGetString(HDenum e) {
    switch (e) {
    case HD_DEVICE_MODEL_TYPE: return "SIMULATED Premium HID (mock)";
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
void hdMockSetHand(const double target[3], int buttons) {
    std::lock_guard<std::mutex> l(g.handMu);
    if (!g.extHand) for (int i = 0; i < 3; ++i) g.extSmooth[i] = g.pos[i];  // start from where the arm is
    for (int i = 0; i < 3; ++i) g.extTarget[i] = target[i];
    g.extButtons = buttons;
    g.extHand = true;
}
void hdMockGetHand(double target[3]) {
    std::lock_guard<std::mutex> l(g.handMu);
    for (int i = 0; i < 3; ++i) target[i] = g.extHand ? g.extSmooth[i] : g.pos[i];
}

void hdUnschedule(HDSchedulerHandle h) {
    std::lock_guard<std::mutex> l(gMu);
    for (auto& a : gAsync) if (a.id == h) a.active = false;
}

}  // extern "C"
