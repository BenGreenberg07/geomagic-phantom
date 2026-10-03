// haptic_playground - a small 3D world you can SEE and FEEL.
//
// A window shows a room with a floor, walls, a hard cube (right), a soft sphere
// (left) and a ball (front, near you) you can push around. The white dot is the
// stylus; it turns red when it touches something, and its shadow on the floor helps you judge depth. The
// yellow line is the force the device is pushing on your hand.
//
// Press ENTER (or stylus button 1) and the room appears CENTERED where the
// stylus is, so nothing pushes until you reach for it. Forces fade in each
// time the room is placed. All forces go through phantom.h (clamp, fade-in,
// speed trip).
//
// Keys (click the 3D window first):
//   ENTER / button 1  place / re-center the room here (also resets the ball)
//   X                 remove the room (forces off)
//   R                 put the ball back
//   F                 show / hide the force arrow
//   arrows or drag    rotate the view;  mouse wheel = zoom
//   Q / Esc           quit (forces off immediately)
//
// Options:
//   --k 0.25    surface stiffness, N/mm (capped at 30% of the device's max)
//   --b 0.001   damping in contact, N*s/mm (capped at 50% of the device's max)
//   --device "<name>"  --skip-calibration  --allow-uncalibrated
// Output: data/haptic_playground_<stamp>.csv (phase: 0 no room, 1 free, 2 touching)
//
// Windows only (Win32 + OpenGL 1.x, both ship with Windows; no extra installs).

#include <cmath>
#include <cstdio>

#include "phantom.h"
#include "util.h"

#ifdef _WIN32

#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")

using phantom::State;
using phantom::Vec3;

namespace {

double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

// ---- the room, in mm relative to where the stylus was when it was placed ----
const double kRoomX = 60, kFloor = -45, kCeil = 45, kRoomZ = 45;
const double kCursorR = 4;                                   // stylus contact radius
const Vec3 kSphereC(-32, -27, -5); const double kSphereR = 18;  // soft sphere, sits on the floor
const Vec3 kCubeC(32, -30, -5);    const double kCubeH = 15;    // hard cube (half-width)
const double kBallR = 12;                                    // pushable ball, at the front:
const Vec3 kBallStart(0, kFloor + kBallR, 22);               // the back of the room is hard to reach
const double kBallMass = 0.03;  // kg
const double kBallDrag = 3.0;   // 1/s, rolling friction

// One-sided penalty along normal n: push out by k*pen, damp only along n, never pull.
Vec3 contact(const Vec3& n, double pen, const Vec3& v, double k, double b) {
    double mag = k * pen - b * dot(v, n);
    return mag > 0 ? n * mag : Vec3();
}

// Everything the servo loop owns. The main thread touches it only via runInServo().
struct World {
    double k = 0.25, b = 0.001;
    bool placed = false;
    Vec3 center;
    Vec3 ball = kBallStart, ballVel;
    bool placeRequested = false, removeRequested = false, resetBall = false;
    bool prevBtn = false;
    int buttonPresses = 0;  // the main thread watches this to place the room on button 1
    double lastT = -1;
    int cubeAxis = -1;      // face the stylus entered the cube by (-1 = outside)
    double cubeSign = 1;
    Vec3 prevQ;             // last stylus position relative to the cube

    Vec3 step(State& s) {
        bool press = s.button1() && !prevBtn;
        prevBtn = s.button1();
        if (press) buttonPresses++;
        if (placeRequested) {
            center = s.pos; placed = true; placeRequested = false; resetBall = true;
            cubeAxis = -1; prevQ = s.pos - center - kCubeC;
        }
        if (removeRequested) { placed = false; removeRequested = false; }
        if (resetBall) { ball = kBallStart; ballVel = Vec3(); resetBall = false; }
        double dt = (lastT < 0) ? 0.001 : s.t - lastT;
        lastT = s.t;
        if (dt <= 0 || dt > 0.005) dt = 0.001;
        if (!placed) { s.phase = 0; return Vec3(); }

        Vec3 p = s.pos - center, v = s.vel, f;
        // Room walls and floor (one-sided springs, like virtual_box).
        double lo[3] = {-kRoomX + kCursorR, kFloor + kCursorR, -kRoomZ + kCursorR};
        double hi[3] = {kRoomX - kCursorR, kCeil - kCursorR, kRoomZ - kCursorR};
        double pc[3] = {p.x, p.y, p.z};
        for (int i = 0; i < 3; ++i) {
            Vec3 n(i == 0, i == 1, i == 2);
            if (pc[i] < lo[i]) f += contact(n, lo[i] - pc[i], v, k, b);
            if (pc[i] > hi[i]) f += contact(n * -1.0, pc[i] - hi[i], v, k, b);
        }
        // Soft sphere: 40% of the stiffness.
        Vec3 d = p - kSphereC;
        double dist = d.norm();
        if (dist < kSphereR + kCursorR && dist > 1e-6) f += contact(d * (1.0 / dist), kSphereR + kCursorR - dist, v, 0.4 * k, b);
        // Hard cube: push out through the face the stylus ENTERED by, and keep that
        // face until it leaves. Picking the nearest face every tick made the force
        // flip between faces near edges and corners (felt as jitter).
        Vec3 q = p - kCubeC;
        double h = kCubeH + kCursorR, qa[3] = {q.x, q.y, q.z};
        if (std::fabs(q.x) < h && std::fabs(q.y) < h && std::fabs(q.z) < h) {
            if (cubeAxis < 0) {
                // The axis where the previous (outside) position was furthest past a face.
                double pq[3] = {prevQ.x, prevQ.y, prevQ.z}, best = -1e9;
                for (int i = 0; i < 3; ++i) {
                    double out = std::fabs(pq[i]) - h;
                    if (out > best) { best = out; cubeAxis = i; cubeSign = pq[i] >= 0 ? 1.0 : -1.0; }
                }
            }
            Vec3 n(cubeAxis == 0, cubeAxis == 1, cubeAxis == 2);
            f += contact(n * cubeSign, h - cubeSign * qa[cubeAxis], v, k, b);
        } else {
            cubeAxis = -1;
        }
        prevQ = q;
        // Ball: equal and opposite contact force, then roll it along the floor.
        Vec3 bf;
        d = p - ball;
        dist = d.norm();
        if (dist < kBallR + kCursorR && dist > 1e-6) {
            Vec3 fc = contact(d * (1.0 / dist), kBallR + kCursorR - dist, v - ballVel, k, b);
            f += fc;
            bf = fc * -1.0;
        }
        stepBall(bf, dt);

        s.phase = f.norm() > 0 ? 2 : 1;
        return f;
    }

    void stepBall(const Vec3& force, double dt) {
        // mm/s^2 = 1000 * N / kg. The ball stays on the floor, so only x and z move.
        ballVel.x += (1000.0 * force.x / kBallMass - kBallDrag * ballVel.x) * dt;
        ballVel.z += (1000.0 * force.z / kBallMass - kBallDrag * ballVel.z) * dt;
        double sp = std::sqrt(ballVel.x * ballVel.x + ballVel.z * ballVel.z);
        if (sp > 800) { ballVel.x *= 800 / sp; ballVel.z *= 800 / sp; }
        ball.x += ballVel.x * dt;
        ball.z += ballVel.z * dt;
        ball.y = kFloor + kBallR;
        // Bounce off the walls.
        if (ball.x < -kRoomX + kBallR) { ball.x = -kRoomX + kBallR; ballVel.x = 0.5 * std::fabs(ballVel.x); }
        if (ball.x > kRoomX - kBallR)  { ball.x = kRoomX - kBallR;  ballVel.x = -0.5 * std::fabs(ballVel.x); }
        if (ball.z < -kRoomZ + kBallR) { ball.z = -kRoomZ + kBallR; ballVel.z = 0.5 * std::fabs(ballVel.z); }
        if (ball.z > kRoomZ - kBallR)  { ball.z = kRoomZ - kBallR;  ballVel.z = -0.5 * std::fabs(ballVel.z); }
        // Bounce off the sphere (in the floor plane).
        double dx = ball.x - kSphereC.x, dz = ball.z - kSphereC.z, dd = std::sqrt(dx * dx + dz * dz);
        double minD = kSphereR + kBallR - 2;  // spheres touch a bit lower than their equators
        if (dd < minD && dd > 1e-6) {
            dx /= dd; dz /= dd;
            ball.x = kSphereC.x + dx * minD; ball.z = kSphereC.z + dz * minD;
            double vn = ballVel.x * dx + ballVel.z * dz;
            if (vn < 0) { ballVel.x -= 1.5 * vn * dx; ballVel.z -= 1.5 * vn * dz; }
        }
        // Bounce off the cube (in the floor plane).
        double qx = ball.x - kCubeC.x, qz = ball.z - kCubeC.z, hb = kCubeH + kBallR;
        if (std::fabs(qx) < hb && std::fabs(qz) < hb) {
            if (hb - std::fabs(qx) < hb - std::fabs(qz)) {
                ball.x = kCubeC.x + (qx >= 0 ? hb : -hb);
                if (ballVel.x * qx < 0) ballVel.x *= -0.5;
            } else {
                ball.z = kCubeC.z + (qz >= 0 ? hb : -hb);
                if (ballVel.z * qz < 0) ballVel.z *= -0.5;
            }
        }
    }
};

// What the window needs each frame, copied out of the servo thread.
struct View {
    bool placed = false;
    Vec3 center, ball;
    int buttonPresses = 0;
};

// ---- window + input ---------------------------------------------------------
struct Ui {
    double yaw = 0, pitch = 20, dist = 260;
    bool dragging = false;
    int lastX = 0, lastY = 0;
    bool showForce = true;
    bool quit = false, place = false, remove = false, resetBall = false;
    int width = 1000, height = 750;
};
Ui g_ui;

LRESULT CALLBACK wndProc(HWND h, UINT msg, WPARAM wp, LPARAM lp) {
    Ui& u = g_ui;
    switch (msg) {
    case WM_CLOSE: u.quit = true; return 0;
    case WM_SIZE: u.width = LOWORD(lp); u.height = HIWORD(lp); return 0;
    case WM_KEYDOWN:
        switch (wp) {
        case VK_ESCAPE: case 'Q': u.quit = true; break;
        case VK_RETURN: u.place = true; break;
        case 'X': u.remove = true; break;
        case 'R': u.resetBall = true; break;
        case 'F': u.showForce = !u.showForce; break;
        case VK_LEFT: u.yaw -= 5; break;
        case VK_RIGHT: u.yaw += 5; break;
        case VK_UP: u.pitch += 5; break;
        case VK_DOWN: u.pitch -= 5; break;
        }
        if (u.pitch > 85) u.pitch = 85;
        if (u.pitch < -10) u.pitch = -10;
        return 0;
    case WM_LBUTTONDOWN: u.dragging = true; u.lastX = (short)LOWORD(lp); u.lastY = (short)HIWORD(lp); SetCapture(h); return 0;
    case WM_LBUTTONUP: u.dragging = false; ReleaseCapture(); return 0;
    case WM_MOUSEMOVE:
        if (u.dragging) {
            int x = (short)LOWORD(lp), y = (short)HIWORD(lp);
            u.yaw += 0.4 * (x - u.lastX);
            u.pitch += 0.4 * (y - u.lastY);
            if (u.pitch > 85) u.pitch = 85;
            if (u.pitch < -10) u.pitch = -10;
            u.lastX = x; u.lastY = y;
        }
        return 0;
    case WM_MOUSEWHEEL:
        u.dist -= 0.15 * GET_WHEEL_DELTA_WPARAM(wp);
        if (u.dist < 120) u.dist = 120;
        if (u.dist > 600) u.dist = 600;
        return 0;
    }
    return DefWindowProc(h, msg, wp, lp);
}

HWND createGlWindow(HDC& dc, HGLRC& rc) {
    WNDCLASSA wc = {};
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = wndProc;
    wc.hInstance = GetModuleHandle(nullptr);
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.lpszClassName = "HapticPlayground";
    RegisterClassA(&wc);
    HWND h = CreateWindowA(wc.lpszClassName, "Haptic playground", WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT,
                           CW_USEDEFAULT, g_ui.width, g_ui.height, nullptr, nullptr, wc.hInstance, nullptr);
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
    return h;
}

// ---- drawing ----------------------------------------------------------------
GLUquadric* g_quad = nullptr;

void sphere(const Vec3& c, double r, double cr, double cg, double cb) {
    glColor3d(cr, cg, cb);
    glPushMatrix();
    glTranslated(c.x, c.y, c.z);
    gluSphere(g_quad, r, 32, 20);
    glPopMatrix();
}

void cube(const Vec3& c, double h) {
    static const double n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    glColor3d(0.95, 0.55, 0.15);
    glBegin(GL_QUADS);
    for (int f = 0; f < 6; ++f) {
        glNormal3d(n[f][0], n[f][1], n[f][2]);
        int a = f / 2;                 // axis this face is perpendicular to
        int u = (a + 1) % 3, w = (a + 2) % 3;
        double s = n[f][a];
        double corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
        for (int k = 0; k < 4; ++k) {
            int kk = s > 0 ? k : 3 - k;  // keep the winding outward
            double p[3];
            p[a] = s * h;
            p[u] = corners[kk][0] * h;
            p[w] = corners[kk][1] * h;
            glVertex3d(c.x + p[0], c.y + p[1], c.z + p[2]);
        }
    }
    glEnd();
}

// Dark disc on the floor under a point: the main depth cue.
void shadow(const Vec3& p, double r) {
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4d(0, 0, 0, 0.35);
    glBegin(GL_TRIANGLE_FAN);
    glVertex3d(p.x, kFloor + 0.2, p.z);
    for (int i = 0; i <= 24; ++i) {
        double a = i * 2 * 3.14159265 / 24;
        glVertex3d(p.x + r * std::cos(a), kFloor + 0.2, p.z + r * std::sin(a));
    }
    glEnd();
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

void drawRoom(bool ghost) {
    glDisable(GL_LIGHTING);
    // Floor with a 10 mm grid.
    if (!ghost) {
        glColor3d(0.82, 0.84, 0.88);
        glBegin(GL_QUADS);
        glVertex3d(-kRoomX, kFloor, -kRoomZ); glVertex3d(-kRoomX, kFloor, kRoomZ);
        glVertex3d(kRoomX, kFloor, kRoomZ);   glVertex3d(kRoomX, kFloor, -kRoomZ);
        glEnd();
    }
    glColor3d(0.6, 0.63, 0.7);
    glBegin(GL_LINES);
    for (double x = -kRoomX; x <= kRoomX + 0.1; x += 10) { glVertex3d(x, kFloor + 0.1, -kRoomZ); glVertex3d(x, kFloor + 0.1, kRoomZ); }
    for (double z = -kRoomZ; z <= kRoomZ + 0.1; z += 10) { glVertex3d(-kRoomX, kFloor + 0.1, z); glVertex3d(kRoomX, kFloor + 0.1, z); }
    glEnd();
    // Room edges.
    glColor3d(0.35, 0.4, 0.5);
    double xs[2] = {-kRoomX, kRoomX}, ys[2] = {kFloor, kCeil}, zs[2] = {-kRoomZ, kRoomZ};
    glBegin(GL_LINES);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            glVertex3d(-kRoomX, ys[i], zs[j]); glVertex3d(kRoomX, ys[i], zs[j]);
            glVertex3d(xs[i], kFloor, zs[j]);  glVertex3d(xs[i], kCeil, zs[j]);
            glVertex3d(xs[i], ys[j], -kRoomZ); glVertex3d(xs[i], ys[j], kRoomZ);
        }
    glEnd();
    glEnable(GL_LIGHTING);
}

void render(const Ui& u, const View& v, const State& s, bool tripped) {
    glViewport(0, 0, u.width, u.height > 0 ? u.height : 1);
    glClearColor(tripped ? 0.35f : 0.12f, 0.13f, 0.16f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(40.0, (double)u.width / (u.height > 0 ? u.height : 1), 10.0, 2000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    // Device frame = OpenGL frame (+x right, +y up, +z toward you), so the default
    // view (yaw 0) looks at the room the way you face the device.
    double yr = u.yaw * 3.14159265 / 180, pr = u.pitch * 3.14159265 / 180;
    gluLookAt(u.dist * std::sin(yr) * std::cos(pr), -10 + u.dist * std::sin(pr), u.dist * std::cos(yr) * std::cos(pr),
              0, -10, 0, 0, 1, 0);
    GLfloat lightPos[4] = {80.0f, 200.0f, 150.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    // Before the room is placed, draw it as a ghost around the stylus.
    Vec3 c = v.placed ? v.center : s.pos;
    Vec3 cur = s.pos - c;
    drawRoom(!v.placed);
    if (v.placed) {
        shadow(kSphereC, kSphereR * 0.9);
        shadow(kCubeC, kCubeH * 1.1);
        shadow(v.ball - c, kBallR * 0.9);
        shadow(cur, kCursorR * 1.2);
        sphere(kSphereC, kSphereR, 0.3, 0.55, 0.95);
        cube(kCubeC, kCubeH);
        sphere(v.ball - c, kBallR, 0.3, 0.85, 0.4);
        // A thin line from the stylus down to the floor.
        glDisable(GL_LIGHTING);
        glColor3d(0.7, 0.7, 0.7);
        glBegin(GL_LINES);
        glVertex3d(cur.x, cur.y, cur.z); glVertex3d(cur.x, kFloor, cur.z);
        glEnd();
        glEnable(GL_LIGHTING);
    }
    bool touching = s.phase == 2;
    sphere(cur, kCursorR, 1.0, touching ? 0.25 : 1.0, touching ? 0.25 : 1.0);
    if (u.showForce && s.force.norm() > 0.01) {
        Vec3 tip = cur + s.force * 15.0;  // 15 mm per newton
        glDisable(GL_LIGHTING);
        glLineWidth(3);
        glColor3d(1.0, 0.9, 0.1);
        glBegin(GL_LINES);
        glVertex3d(cur.x, cur.y, cur.z); glVertex3d(tip.x, tip.y, tip.z);
        glEnd();
        glLineWidth(1);
        glEnable(GL_LIGHTING);
    }
}

}  // namespace

int main(int argc, char** argv) {
    util::Args args(argc, argv);
    World world;
    world.k = args.num("--k", 0.25);
    world.b = args.num("--b", 0.001);

    phantom::Device dev;
    if (!dev.open(args.get("--device", "").c_str(), args.flag("--skip-calibration"))) return 1;
    if (!dev.info().calibrated && !args.flag("--allow-uncalibrated")) {
        std::printf("Refusing to drive forces on an uncalibrated device (pass --allow-uncalibrated to override).\n");
        return 1;
    }
    if (dev.info().maxStiffness > 0 && world.k > 0.3 * dev.info().maxStiffness) world.k = 0.3 * dev.info().maxStiffness;
    if (dev.info().maxDamping > 0 && world.b > 0.5 * dev.info().maxDamping) world.b = 0.5 * dev.info().maxDamping;

    dev.setForceFunction([&](State& s) { return world.step(s); });
    if (!dev.start(true)) return 1;

    auto csvPath = util::dataDir(argv[0]) / ("haptic_playground_" + util::stamp() + ".csv");
    FILE* csv = util::openCsv(csvPath);
    if (!csv) { dev.close(); return 1; }
    util::writeSampleHeader(csv);
    char extra[128];
    std::snprintf(extra, sizeof extra, "surface_k_N_mm: %.4f\ncontact_b_Ns_mm: %.5f\n", world.k, world.b);
    util::writeInfoFile(csvPath, dev, extra);
    dev.startRecording();

    HDC dc = nullptr;
    HGLRC rc = nullptr;
    HWND wnd = createGlWindow(dc, rc);
    if (!wnd) { std::printf("Could not open the 3D window.\n"); dev.close(); return 1; }
    g_quad = gluNewQuadric();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    GLfloat ambient[4] = {0.35f, 0.35f, 0.38f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    std::printf("Haptic playground: surfaces k=%.3f N/mm, b=%.4f N*s/mm.\n"
                "Click the 3D window, then ENTER or stylus button 1 = place the room around the stylus.\n"
                "X = remove, R = reset ball, F = force arrow, arrows/drag = rotate, wheel = zoom, Q = quit.\n\n",
                world.k, world.b);

    // Place / re-center: forces off, move the room, forces back on (so they fade in again).
    auto place = [&] {
        dev.enableForces(false);
        util::sleepMs(5);
        dev.resetTrip();
        dev.runInServo([&] { world.placeRequested = true; });
        dev.enableForces(true);
        std::printf("room placed\n");
    };

    View view;
    int seenPresses = 0;
    bool tripShown = false;
    char title[200];
    int frame = 0;
    while (!g_ui.quit) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
        State s;
        while (dev.popSample(s)) util::writeSample(csv, s, 0);

        dev.runInServo([&] {
            view.placed = world.placed;
            view.center = world.center;
            view.ball = world.ball;
            view.buttonPresses = world.buttonPresses;
        });
        if (view.buttonPresses != seenPresses) { seenPresses = view.buttonPresses; g_ui.place = true; }
        if (g_ui.place) { g_ui.place = false; place(); tripShown = false; }
        if (g_ui.remove) {
            g_ui.remove = false;
            dev.enableForces(false);
            dev.runInServo([&] { world.removeRequested = true; });
            std::printf("room removed\n");
        }
        if (g_ui.resetBall) { g_ui.resetBall = false; dev.runInServo([&] { world.resetBall = true; }); }

        bool tripped = dev.tripped() != phantom::Trip::None;
        if (tripped && !tripShown) {
            dev.enableForces(false);
            std::printf("SAFETY TRIP: %s. Forces off. Press ENTER to place the room again.\n",
                        phantom::tripName(dev.tripped()));
            tripShown = true;
        }

        State now = dev.latest();
        render(g_ui, view, now, tripped);
        SwapBuffers(dc);

        if (++frame % 10 == 0) {
            if (tripped)
                std::snprintf(title, sizeof title, "Haptic playground - SAFETY TRIP, forces off - press ENTER to restart");
            else if (!view.placed)
                std::snprintf(title, sizeof title, "Haptic playground - press ENTER or stylus button to place the room");
            else
                std::snprintf(title, sizeof title, "Haptic playground - force %.2f N - %s", now.force.norm(),
                              now.phase == 2 ? "touching" : "free");
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
    gluDeleteQuadric(g_quad);
    wglMakeCurrent(nullptr, nullptr);
    wglDeleteContext(rc);
    ReleaseDC(wnd, dc);
    DestroyWindow(wnd);
    dev.close();
    std::printf("Saved %s\n", csvPath.string().c_str());
    return 0;
}

#else  // not Windows (e.g. the mock build on a Mac)

int main() {
    std::printf("haptic_playground needs Windows (Win32 + OpenGL). Build it on the lab PC.\n");
    return 0;
}

#endif
