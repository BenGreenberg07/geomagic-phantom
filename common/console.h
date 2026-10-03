#pragma once
// console.h - non-blocking keyboard input that works on Windows and Linux/macOS.
//   console::keyPressed()  true if a key is waiting
//   console::readKey()     the key (call only after keyPressed()), -1 on end of input
//   console::waitEnter()   block until ENTER

#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace console {

#ifdef _WIN32
inline bool keyPressed() { return _kbhit() != 0; }
inline int readKey() { return _getch(); }
inline void waitEnter() {
    for (;;) {
        int c = _getch();
        if (c == '\r' || c == '\n') return;
    }
}
#else
namespace detail {
struct RawMode {  // single-key input without ENTER, restored on exit
    termios old{};
    bool active = false;
    RawMode() {
        if (isatty(0) && tcgetattr(0, &old) == 0) {
            termios raw = old;
            raw.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(0, TCSANOW, &raw);
            active = true;
        }
    }
    ~RawMode() { if (active) tcsetattr(0, TCSANOW, &old); }
};
inline RawMode& raw() { static RawMode r; return r; }
inline bool& eof() { static bool e = false; return e; }
inline bool& eofTaken() { static bool t = false; return t; }  // has readKey() returned -1 yet?
inline int rawRead() {
    if (eof()) return -1;
    unsigned char c;
    if (read(0, &c, 1) != 1) { eof() = true; return -1; }
    return c;
}
}  // namespace detail

// End of input (piped/scripted runs only) is reported exactly once: keyPressed()
// stays true until readKey() has returned -1, then false, so apps see it and quit
// even if waitEnter() hit it first, and `while (keyPressed())` loops still end.
inline bool keyPressed() {
    detail::raw();
    if (detail::eof()) return !detail::eofTaken();
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    timeval tv{0, 0};
    return select(1, &fds, nullptr, nullptr, &tv) > 0;
}
inline int readKey() {
    detail::raw();
    int c = detail::rawRead();
    if (c < 0) detail::eofTaken() = true;
    return c;
}
inline void waitEnter() {
    detail::raw();
    for (;;) {
        int c = detail::rawRead();
        if (c == '\n' || c == '\r' || c < 0) return;
    }
}
#endif

}  // namespace console
