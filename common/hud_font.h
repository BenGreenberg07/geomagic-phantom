#pragma once
// hud_font.h - a tiny built-in stroke font for on-screen text in the 3D apps.
//
// No OS fonts, no files: every glyph is a few line strokes on a 4 x 6 grid, so it
// looks the same on Windows (Win32) and Mac/Linux (GLFW). Upper case only
// (lower case is drawn as upper case). Use it inside a 2D overlay:
//
//   hud::begin2D();                                  // pixel coordinates, origin bottom-left
//   hud::panel(10, 10, 300, 60, 0.0, 0.0, 0.0, 0.55); // translucent box
//   hud::text(20, 40, 18, "TIME 1.23 S", 1, 1, 1);    // x, y (baseline), height in px, colour
//   hud::end2D();
//
// Glyph encoding: strokes separated by spaces; each stroke is a run of points,
// two digits per point (x 0-4, y 0-6, y up). "0003264340 0343" is an A.

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <cstdio>
#include <cstring>

namespace hud {

inline const char* glyph(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    switch (c) {
    case 'A': return "0003264340 0343";
    case 'B': return "00063645443303 3342413000";
    case 'C': return "4536160501103041";
    case 'D': return "00063645413000";
    case 'E': return "40000646 0333";
    case 'F': return "460600 0333";
    case 'G': return "45361605011030414323";
    case 'H': return "0006 4046 0343";
    case 'I': return "1030 2026 1636";
    case 'J': return "4641301001";
    case 'K': return "0006 4602 1340";
    case 'L': return "060040";
    case 'M': return "0006234640";
    case 'N': return "00064046";
    case 'O': return "103041453616050110";
    case 'P': return "00063645443303";
    case 'Q': return "103041453616050110 2240";
    case 'R': return "00063645443303 2340";
    case 'S': return "453616050413334241301001";
    case 'T': return "0646 2620";
    case 'U': return "060110304146";
    case 'V': return "062046";
    case 'W': return "0610233046";
    case 'X': return "0046 0640";
    case 'Y': return "0623 4623 2320";
    case 'Z': return "06460040";
    case '0': return "103041453616050110 4105";
    case '1': return "1526 2620 1030";
    case '2': return "05163645440040";
    case '3': return "05163645443313 334241301001";
    case '4': return "30360242";
    case '5': return "460603334241301001";
    case '6': return "453616050110304142331302";
    case '7': return "064610";
    case '8': return "13040516364544331302011030414233";
    case '9': return "011030414536160504133344";
    case '.': return "1020211110";
    case ',': return "212010";
    case ':': return "2122 2425";
    case ';': return "2425 222110";
    case '-': return "1333";
    case '+': return "1333 2224";
    case '/': return "0046";
    case '(': return "36252130";
    case ')': return "16252110";
    case '%': return "0046 0515 3141";
    case '\'': return "2625";
    case '!': return "2622 2120";
    case '?': return "0516364544332322 2120";
    case '=': return "0242 0444";
    case '<': return "351331";
    case '>': return "153311";
    case '_': return "0040";
    case '#': return "1016 3036 0242 0444";
    case '*': return "1335 1533 2125";
    case '|': return "2026";
    default: return "";  // space and unknown characters: blank
    }
}

// Width in pixels of a string at the given glyph height.
inline double textWidth(const char* s, double h) { return std::strlen(s) * h; }

inline void strokes(double x, double y, double h, const char* s) {
    double u = h / 6.0;  // grid unit
    double cx = x;
    glBegin(GL_LINES);
    for (const char* c = s; *c; ++c) {
        const char* g = glyph(*c);
        double px = 0, py = 0;
        bool have = false;
        for (const char* q = g;; ++q) {
            if (*q == ' ' || *q == 0) { have = false; if (*q == 0) break; continue; }
            if (!q[1]) break;
            double nx = cx + (q[0] - '0') * u, ny = y + (q[1] - '0') * u;
            if (have) { glVertex2d(px, py); glVertex2d(nx, ny); }
            px = nx; py = ny; have = true;
            ++q;
        }
        cx += h;  // advance: 4 units of glyph + 2 units of space
    }
    glEnd();
}

// Text with a soft dark shadow so it reads on any background.
inline void text(double x, double y, double h, const char* s, double r, double g, double b, double a = 1.0) {
    double lw = h < 14 ? 1.5 : (h < 24 ? 2.0 : 3.0);
    glLineWidth((GLfloat)(lw + 2));
    glColor4d(0, 0, 0, 0.55 * a);
    strokes(x + 1, y - 1, h, s);
    glLineWidth((GLfloat)lw);
    glColor4d(r, g, b, a);
    strokes(x, y, h, s);
    glLineWidth(1);
}

inline void panel(double x, double y, double w, double h, double r, double g, double b, double a) {
    glColor4d(r, g, b, a);
    glBegin(GL_QUADS);
    glVertex2d(x, y); glVertex2d(x + w, y); glVertex2d(x + w, y + h); glVertex2d(x, y + h);
    glEnd();
    glColor4d(1, 1, 1, 0.12 * (a > 0 ? 1 : 0));
    glBegin(GL_LINE_LOOP);
    glVertex2d(x + 0.5, y + 0.5); glVertex2d(x + w - 0.5, y + 0.5); glVertex2d(x + w - 0.5, y + h - 0.5); glVertex2d(x + 0.5, y + h - 0.5);
    glEnd();
}

// Horizontal bar: value/maxv filled, with an optional marker line at markv.
inline void bar(double x, double y, double w, double h, double value, double maxv, double r, double g, double b,
                double markv = -1) {
    panel(x, y, w, h, 0.05, 0.05, 0.07, 0.8);
    double f = maxv > 0 ? value / maxv : 0;
    if (f < 0) f = 0;
    if (f > 1) f = 1;
    glColor4d(r, g, b, 0.95);
    glBegin(GL_QUADS);
    glVertex2d(x + 2, y + 2); glVertex2d(x + 2 + (w - 4) * f, y + 2);
    glVertex2d(x + 2 + (w - 4) * f, y + h - 2); glVertex2d(x + 2, y + h - 2);
    glEnd();
    if (markv >= 0 && maxv > 0) {
        double mx = x + 2 + (w - 4) * (markv / maxv > 1 ? 1 : markv / maxv);
        glColor4d(1, 1, 1, 0.8);
        glLineWidth(2);
        glBegin(GL_LINES); glVertex2d(mx, y - 2); glVertex2d(mx, y + h + 2); glEnd();
        glLineWidth(1);
    }
}

// Switch to a pixel-coordinate overlay (origin bottom-left) and back.
inline void begin2D(int w, int h) {
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LINE_BIT | GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, w, 0, h, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
}
inline void end2D() {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glPopAttrib();
}

}  // namespace hud
