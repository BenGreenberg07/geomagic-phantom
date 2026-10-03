#pragma once
// viz.h - shared 3D window + scene runner for the apps you can SEE and FEEL
// (grab_ball, surfaces, reach_game, mechanisms, physics_toys).
//
// A scene only describes its world; run() does everything else the same way in
// every app: open + calibrate the device, the place / re-center / trip logic,
// CSV logging, the window, camera, room, cursor, shadow and force arrow.
//
// Writing a scene (see apps/grab_ball for a full example):
//
//   struct MyScene : viz::SceneBase {
//       struct Snap { Vec3 ball; };                 // what draw() needs, copied each frame
//       Vec3 force(State& s, const Vec3& p, double dt);  // SERVO THREAD, ~1 kHz
//       void reset();                               // servo thread, when the room is placed
//       void key(int vk);                           // servo thread, for keys run() doesn't use
//       void snap(Snap& out) const;                 // servo thread, copy state for drawing
//       void draw(const Snap& sn, const Vec3& cursor) const;  // main thread: use sn + constants only
//   };
//
// force() gets p = stylus position relative to where the room was placed (mm).
// It runs in the servo loop: no printf, file I/O, allocation or locks (CLAUDE.md).
// Everything goes through phantom.h, so the force clamp, fade-in and speed trip apply.
//
// Keys handled here: ENTER (and stylus button 1 unless kButtonPlaces = false)
// places the room around the stylus; X removes it; F toggles the force arrow;
// arrows / mouse drag rotate; wheel zooms; Q / Esc quit. Other keys go to key().
//
// Windows only (Win32 + OpenGL 1.x ship with Windows).

#ifdef _WIN32

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "phantom.h"
#include "util.h"

#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")

namespace viz {

using phantom::State;
using phantom::Vec3;

const double kPi = 3.14159265358979;

inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double clampd(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline double get(const Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }
inline Vec3 axisVec(int i, double s = 1.0) { return Vec3(i == 0 ? s : 0, i == 1 ? s : 0, i == 2 ? s : 0); }

// One-sided penalty along unit normal n: push out by k*pen, damp only along n, never pull.
inline Vec3 contact(const Vec3& n, double pen, const Vec3& v, double k, double b) {
    double mag = k * pen - b * dot(v, n);
    return mag > 0 ? n * mag : Vec3();
}

// ---------------------------------------------------------------------------
// The room: floor + walls, mm relative to where it was placed.
// ---------------------------------------------------------------------------
struct Room {
    double x = 60, floor = -45, ceil = 45, z = 45;
};

// Walls and floor as one-sided springs for a stylus of radius r.
inline Vec3 roomForce(const Room& room, const Vec3& p, const Vec3& v, double r, double k, double b) {
    double lo[3] = {-room.x + r, room.floor + r, -room.z + r};
    double hi[3] = {room.x - r, room.ceil - r, room.z - r};
    Vec3 f;
    for (int i = 0; i < 3; ++i) {
        double pi = get(p, i);
        if (pi < lo[i]) f += contact(axisVec(i), lo[i] - pi, v, k, b);
        if (pi > hi[i]) f += contact(axisVec(i, -1), pi - hi[i], v, k, b);
    }
    return f;
}

// Solid sphere the stylus (radius r) can touch from outside.
inline Vec3 sphereForce(const Vec3& p, const Vec3& v, const Vec3& c, double R, double r, double k, double b) {
    Vec3 d = p - c;
    double dist = d.norm();
    if (dist >= R + r || dist < 1e-6) return Vec3();
    return contact(d * (1.0 / dist), R + r - dist, v, k, b);
}

// Solid box the stylus can touch from any side. It pushes out through the face
// the stylus ENTERED by and keeps that face until the stylus leaves; picking the
// nearest face every tick makes the force flip near edges (felt as jitter).
// One BoxProxy per box; it lives in the servo thread.
struct BoxProxy {
    int axis = -1;  // entry face axis, -1 = outside
    double sign = 1;
    Vec3 prevQ;
    bool havePrev = false;

    void reset() { axis = -1; havePrev = false; }
    // Returns the force; face (optional) gets 0..5 = +x,-x,+y,-y,+z,-z or -1.
    Vec3 force(const Vec3& p, const Vec3& v, const Vec3& c, const Vec3& half, double r, double k, double b,
               int* face = nullptr) {
        Vec3 q = p - c;
        double h[3] = {half.x + r, half.y + r, half.z + r};
        bool inside = std::fabs(q.x) < h[0] && std::fabs(q.y) < h[1] && std::fabs(q.z) < h[2];
        Vec3 f;
        if (inside) {
            if (axis < 0) {
                Vec3 ref = havePrev ? prevQ : q;
                bool nearest = !havePrev ||
                               (std::fabs(ref.x) < h[0] && std::fabs(ref.y) < h[1] && std::fabs(ref.z) < h[2]);
                if (nearest) ref = q;
                double best = -1e9;
                for (int i = 0; i < 3; ++i) {
                    // From outside: the face the previous position was furthest past.
                    // No usable previous position: the nearest face.
                    double score = std::fabs(get(ref, i)) - h[i];
                    if (score > best) { best = score; axis = i; sign = get(ref, i) >= 0 ? 1.0 : -1.0; }
                }
            }
            f = contact(axisVec(axis, sign), h[axis] - sign * get(q, axis), v, k, b);
        } else {
            axis = -1;
        }
        if (face) *face = axis < 0 ? -1 : axis * 2 + (sign > 0 ? 0 : 1);
        prevQ = q;
        havePrev = true;
        return f;
    }
};

// ---------------------------------------------------------------------------
// Drawing helpers (main thread only).
// ---------------------------------------------------------------------------
inline GLUquadric*& quadric() { static GLUquadric* q = nullptr; return q; }

inline void drawSphere(const Vec3& c, double r, double cr, double cg, double cb, double alpha = 1.0) {
    if (alpha < 1.0) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE); }
    glColor4d(cr, cg, cb, alpha);
    glPushMatrix();
    glTranslated(c.x, c.y, c.z);
    gluSphere(quadric(), r, 32, 20);
    glPopMatrix();
    if (alpha < 1.0) { glDisable(GL_BLEND); glDepthMask(GL_TRUE); }
}

inline void drawBox(const Vec3& c, const Vec3& half, double cr, double cg, double cb, double alpha = 1.0) {
    if (alpha < 1.0) { glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE); }
    glColor4d(cr, cg, cb, alpha);
    double hh[3] = {half.x, half.y, half.z};
    static const double corner[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    glBegin(GL_QUADS);
    for (int a = 0; a < 3; ++a)
        for (int s = -1; s <= 1; s += 2) {
            glNormal3d(a == 0 ? s : 0, a == 1 ? s : 0, a == 2 ? s : 0);
            int u = (a + 1) % 3, w = (a + 2) % 3;
            for (int k = 0; k < 4; ++k) {
                double p[3];
                p[a] = s * hh[a];
                p[u] = corner[k][0] * hh[u];
                p[w] = corner[k][1] * hh[w];
                glVertex3d(c.x + p[0], c.y + p[1], c.z + p[2]);
            }
        }
    glEnd();
    if (alpha < 1.0) { glDisable(GL_BLEND); glDepthMask(GL_TRUE); }
}

inline void drawLine(const Vec3& a, const Vec3& b, double cr, double cg, double cb, double width = 1.0) {
    glDisable(GL_LIGHTING);
    glLineWidth((GLfloat)width);
    glColor3d(cr, cg, cb);
    glBegin(GL_LINES);
    glVertex3d(a.x, a.y, a.z);
    glVertex3d(b.x, b.y, b.z);
    glEnd();
    glLineWidth(1);
    glEnable(GL_LIGHTING);
}

// Dark disc on the floor under a point: the main depth cue.
inline void drawShadow(const Vec3& p, double r, double floorY) {
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4d(0, 0, 0, 0.35);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(p.x, floorY + 0.2, p.z);
    for (int i = 0; i <= 24; ++i) {
        double a = i * 2 * kPi / 24;
        glVertex3d(p.x + r * std::cos(a), floorY + 0.2, p.z + r * std::sin(a));
    }
    glEnd();
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

inline void drawRoom(const Room& room, bool ghost) {
    glDisable(GL_LIGHTING);
    if (!ghost) {
        glColor3d(0.82, 0.84, 0.88);
        glBegin(GL_QUADS);
        glVertex3d(-room.x, room.floor, -room.z); glVertex3d(-room.x, room.floor, room.z);
        glVertex3d(room.x, room.floor, room.z);   glVertex3d(room.x, room.floor, -room.z);
        glEnd();
    }
    glColor3d(0.6, 0.63, 0.7);
    glBegin(GL_LINES);
    for (double x = -room.x; x <= room.x + 0.1; x += 10) {
        glVertex3d(x, room.floor + 0.1, -room.z); glVertex3d(x, room.floor + 0.1, room.z);
    }
    for (double z = -room.z; z <= room.z + 0.1; z += 10) {
        glVertex3d(-room.x, room.floor + 0.1, z); glVertex3d(room.x, room.floor + 0.1, z);
    }
    glEnd();
    glColor3d(0.35, 0.4, 0.5);
    double xs[2] = {-room.x, room.x}, ys[2] = {room.floor, room.ceil}, zs[2] = {-room.z, room.z};
    glBegin(GL_LINES);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            glVertex3d(-room.x, ys[i], zs[j]); glVertex3d(room.x, ys[i], zs[j]);
            glVertex3d(xs[i], room.floor, zs[j]); glVertex3d(xs[i], room.ceil, zs[j]);
            glVertex3d(xs[i], ys[j], -room.z); glVertex3d(xs[i], ys[j], room.z);
        }
    glEnd();
    glEnable(GL_LIGHTING);
}

// ---------------------------------------------------------------------------
// Window + input.
// ---------------------------------------------------------------------------
struct Ui {
    double yaw = 0, pitch = 20, dist = 260;
    bool dragging = false;
    int lastX = 0, lastY = 0;
    bool showForce = true;
    bool quit = false, place = false, remove = false;
    int width = 1000, height = 750;
    int keys[32] = {};  // other key presses for the scene, drained each frame
    int nKeys = 0;
};
inline Ui& ui() { static Ui u; return u; }

inline LRESULT CALLBACK wndProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    Ui& u = ui();
    switch (msg) {
    case WM_CLOSE: u.quit = true; return 0;
    case WM_SIZE: u.width = LOWORD(lp); u.height = HIWORD(lp); return 0;
    case WM_KEYDOWN:
        switch (wp) {
        case VK_ESCAPE: case 'Q': u.quit = true; break;
        case VK_RETURN: u.place = true; break;
        case 'X': u.remove = true; break;
        case 'F': u.showForce = !u.showForce; break;
        case VK_LEFT: u.yaw -= 5; break;
        case VK_RIGHT: u.yaw += 5; break;
        case VK_UP: u.pitch += 5; break;
        case VK_DOWN: u.pitch -= 5; break;
        default:
            if (u.nKeys < 32) u.keys[u.nKeys++] = (int)wp;
        }
        u.pitch = clampd(u.pitch, -10, 85);
        return 0;
    case WM_LBUTTONDOWN:
        u.dragging = true; u.lastX = (short)LOWORD(lp); u.lastY = (short)HIWORD(lp); SetCapture(h);
        return 0;
    case WM_LBUTTONUP: u.dragging = false; ReleaseCapture(); return 0;
    case WM_MOUSEMOVE:
        if (u.dragging) {
            int x = (short)LOWORD(lp), y = (short)HIWORD(lp);
            u.yaw += 0.4 * (x - u.lastX);
            u.pitch = clampd(u.pitch + 0.4 * (y - u.lastY), -10, 85);
            u.lastX = x; u.lastY = y;
        }
        return 0;
    case WM_MOUSEWHEEL:
        u.dist = clampd(u.dist - 0.15 * GET_WHEEL_DELTA_WPARAM(wp), 120, 600);
        return 0;
    }
    return DefWindowProc(h, msg, wp, lp);
}

inline HWND createWindow(const char* title, HDC& dc, HGLRC& rc) {
    WNDCLASSA wc = {};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = "PhantomViz";
    RegisterClassA(&wc);
    HWND h = CreateWindowA(wc.lpszClassName, title, WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
                           ui().width, ui().height, nullptr, nullptr, wc.hInstance, nullptr);
    if (!h) return nullptr;
    dc = GetDC(h);
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof pfd;
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 32;
    pfd.cDepthBits = 24;
    SetPixelFormat(dc, ChoosePixelFormat(dc, &pfd), &pfd);
    rc = wglCreateContext(dc);
    wglMakeCurrent(dc, rc);

    quadric() = gluNewQuadric();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    GLfloat ambient[4] = {0.35f, 0.35f, 0.38f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    return h;
}

// Clear + camera. Device frame = OpenGL frame (+x right, +y up, +z toward you),
// so yaw 0 looks at the room the way you face the device.
inline void beginFrame(bool tripped) {
    Ui& u = ui();
    int hgt = u.height > 0 ? u.height : 1;
    glViewport(0, 0, u.width, hgt);
    glClearColor(tripped ? 0.35f : 0.12f, 0.13f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)u.width / hgt, 10.0, 2000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    double yr = u.yaw * kPi / 180, pr = u.pitch * kPi / 180;
    gluLookAt(u.dist * std::sin(yr) * std::cos(pr), -10 + u.dist * std::sin(pr), u.dist * std::cos(yr) * std::cos(pr),
              0, -10, 0, 0, 1, 0);
    GLfloat lightPos[4] = {80.0f, 200.0f, 150.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
}

// ---------------------------------------------------------------------------
// Scene base + runner.
// ---------------------------------------------------------------------------
struct SceneBase {
    Room room;                   // constant after run() starts, so draw() may read it
    double cursorR = 4;          // stylus contact radius, mm
    double k = 0.25, b = 0.001;  // surface stiffness N/mm, damping N*s/mm (run() caps them)
    double maxStiffness = 0, maxDamping = 0;  // the device's nominal limits, filled in by run()
    // false: stylus button 1 belongs to the scene (e.g. grabbing); only ENTER places the room.
    static constexpr bool kButtonPlaces = true;

    void reset() {}
    void key(int) {}
    std::string info() const { return ""; }                            // extra lines for the .info.txt
    void onStart(const std::filesystem::path&) {}                      // main thread, gets the sample CSV path
    void onStop() {}                                                   // main thread, before exit
    template <class S> void mainTick(const S&) {}                      // main thread, every frame
    template <class S> void status(const S&, char* buf, size_t n) const { if (n) buf[0] = 0; }  // title text
};

// Runs a scene until the window closes. Returns the process exit code.
template <class Scene>
int run(int argc, char** argv, Scene& scene, const char* name, const char* help) {
    util::Args args(argc, argv);
    scene.k = args.num("--k", scene.k);
    scene.b = args.num("--b", scene.b);

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }
    if (dev.info().maxStiffness > 0 && scene.k > 0.3 * dev.info().maxStiffness) scene.k = 0.3 * dev.info().maxStiffness;
    if (dev.info().maxDamping > 0 && scene.b > 0.5 * dev.info().maxDamping) scene.b = 0.5 * dev.info().maxDamping;
    scene.maxStiffness = dev.info().maxStiffness;
    scene.maxDamping = dev.info().maxDamping;

    // Servo-thread runner state. The main thread touches it only through runInServo().
    struct Rt {
        bool placed = false, placeReq = false, removeReq = false, prevBtn = false;
        int presses = 0;
        Vec3 center;
        double lastT = -1;
    } rt;
    dev.setForceFunction([&](State& s) -> Vec3 {
        bool press = s.button1() && !rt.prevBtn;
        rt.prevBtn = s.button1();
        if (press) rt.presses++;
        if (rt.placeReq) { rt.center = s.pos; rt.placed = true; rt.placeReq = false; scene.reset(); }
        if (rt.removeReq) { rt.placed = false; rt.removeReq = false; }
        double dt = rt.lastT < 0 ? 0.001 : s.t - rt.lastT;
        rt.lastT = s.t;
        if (dt <= 0 || dt > 0.005) dt = 0.001;
        if (!rt.placed) { s.phase = 0; return Vec3(); }
        s.phase = 1;
        Vec3 f = scene.force(s, s.pos - rt.center, dt);
        if (s.phase == 1 && f.norm() > 0) s.phase = 2;
        return f;
    });
    if (!dev.start(true)) return 1;

    auto csvPath = util::dataDir(argv[0]) / (std::string(name) + "_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    char extra[256];
    std::snprintf(extra, sizeof extra, "surface_k_N_mm: %.4f\ncontact_b_Ns_mm: %.5f\n", scene.k, scene.b);
    util::writeInfoFile(csvPath, dev, extra + scene.info());
    scene.onStart(csvPath);
    dev.startRecording();

    HDC dc = nullptr;
    HGLRC rc = nullptr;
    HWND wnd = createWindow(name, dc, rc);
    if (!wnd) { std::printf("Could not open the 3D window.\n"); std::fclose(csv); dev.close(); return 1; }

    std::printf("%s: surfaces k=%.3f N/mm, b=%.4f N*s/mm.\n%s\n"
                "Click the 3D window, then ENTER%s = place the room around the stylus.\n"
                "X = remove, F = force arrow, arrows/drag = rotate, wheel = zoom, Q = quit.\n\n",
                name, scene.k, scene.b, help, Scene::kButtonPlaces ? " or stylus button 1" : "");

    // Place / re-center: forces off, move the room, forces back on (so they fade in again).
    auto place = [&] {
        dev.enableForces(false);
        util::sleepMs(5);
        dev.resetTrip();
        dev.runInServo([&] { rt.placeReq = true; });
        dev.enableForces(true);
        std::printf("room placed\n");
    };

    typename Scene::Snap snap;
    bool placed = false;
    Vec3 center;
    int presses = 0, seenPresses = 0, frame = 0;
    bool tripShown = false;
    char title[256], status[160];
    Ui& u = ui();
    while (!u.quit) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) { TranslateMessage(&msg); DispatchMessage(&msg); }
        State s;
        while (dev.popSample(s)) util::writeSample(csv, s, 0);

        dev.runInServo([&] { placed = rt.placed; center = rt.center; presses = rt.presses; scene.snap(snap); });
        if (Scene::kButtonPlaces && presses != seenPresses) u.place = true;
        seenPresses = presses;
        if (u.place) { u.place = false; place(); tripShown = false; }
        if (u.remove) {
            u.remove = false;
            dev.enableForces(false);
            dev.runInServo([&] { rt.removeReq = true; });
            std::printf("room removed\n");
        }
        for (int i = 0; i < u.nKeys; ++i) {
            int vk = u.keys[i];
            dev.runInServo([&] { scene.key(vk); });
        }
        u.nKeys = 0;
        scene.mainTick(snap);

        bool tripped = dev.tripped() != phantom::Trip::None;
        if (tripped && !tripShown) {
            dev.enableForces(false);
            std::printf("SAFETY TRIP: %s. Forces off. Press ENTER to place the room again.\n",
                        phantom::tripName(dev.tripped()));
            tripShown = true;
        }

        State now = dev.latest();
        Vec3 cur = now.pos - (placed ? center : now.pos);
        beginFrame(tripped);
        drawRoom(scene.room, !placed);
        if (placed) {
            scene.draw(snap, cur);
            drawShadow(cur, scene.cursorR * 1.2, scene.room.floor);
            drawLine(cur, Vec3(cur.x, scene.room.floor, cur.z), 0.7, 0.7, 0.7);
        }
        bool touching = now.phase >= 2;
        drawSphere(cur, scene.cursorR, 1.0, touching ? 0.25 : 1.0, touching ? 0.25 : 1.0);
        if (u.showForce && now.force.norm() > 0.01) drawLine(cur, cur + now.force * 15.0, 1.0, 0.9, 0.1, 3);  // 15 mm/N
        SwapBuffers(dc);

        if (++frame % 10 == 0) {
            if (tripped) {
                std::snprintf(title, sizeof title, "%s - SAFETY TRIP, forces off - press ENTER to restart", name);
            } else if (!placed) {
                std::snprintf(title, sizeof title, "%s - press ENTER%s to place the room", name,
                              Scene::kButtonPlaces ? " or the stylus button" : "");
            } else {
                scene.status(snap, status, sizeof status);
                std::snprintf(title, sizeof title, "%s - force %.2f N - %s", name, now.force.norm(), status);
            }
            SetWindowTextA(wnd, title);
        }
        util::sleepMs(10);
    }

    dev.enableForces(false);
    util::sleepMs(50);
    State s;
    while (dev.popSample(s)) util::writeSample(csv, s, 0);
    dev.stopRecording();
    std::fclose(csv);
    scene.mainTick(snap);
    scene.onStop();
    gluDeleteQuadric(quadric());
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(rc);
    ReleaseDC(wnd, dc);
    DestroyWindow(wnd);
    dev.close();
    std::printf("Saved %s\n", csvPath.string().c_str());
    return 0;
}

}  // namespace viz

#endif  // _WIN32
