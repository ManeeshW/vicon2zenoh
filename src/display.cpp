#include "display.hpp"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <chrono>
#include <thread>
#include <string>
#include <algorithm>

// ── ANSI escape sequences ────────────────────────────────────────────────────
#define RST  "\033[0m"
#define BLD  "\033[1m"
#define DIM  "\033[2m"
#define CYN  "\033[36m"
#define YEL  "\033[33m"
#define GRN  "\033[32m"
#define WHT  "\033[97m"
#define RED  "\033[31m"
#define HOME "\033[H"
#define EOLN "\033[K"
#define HIDE "\033[?25l"
#define SHOW "\033[?25h"
#define CLR  "\033[2J"

// ── Box-drawing characters (UTF-8) ───────────────────────────────────────────
// Each is 1 visual column, 3 bytes in UTF-8.
#define BD_H  "═"   // U+2550 horizontal double
#define BD_h  "─"   // U+2500 horizontal single
#define BD_V  "║"   // U+2551 vertical double
#define BD_TL "╔"   // U+2554 top-left
#define BD_TR "╗"   // U+2557 top-right
#define BD_BL "╚"   // U+255A bottom-left
#define BD_BR "╝"   // U+255D bottom-right
#define BD_ML "╠"   // U+2560 mid-left
#define BD_MR "╣"   // U+2563 mid-right
#define BD_MM "╬"   // U+256C mid-cross
#define BD_TM "╦"   // U+2566 top-mid
#define BD_BM "╩"   // U+2569 bottom-mid

// ── Layout constants ─────────────────────────────────────────────────────────
// Three columns of CW visible chars each, four ║ borders.
// Total visual width = 3*CW + 4.
static constexpr int CW = 40;
static constexpr int TW = 3 * CW + 4;   // 124

// ── String helpers ───────────────────────────────────────────────────────────

// Pad/truncate plain-ASCII string to exactly w chars.
static std::string pad(const std::string& s, int w) {
    if ((int)s.size() >= w) return s.substr(0, w);
    return s + std::string(w - (int)s.size(), ' ');
}

// Repeat a UTF-8 box char n times (1 visual col per call).
static std::string rpt(const char* c, int n) {
    std::string r;
    for (int i = 0; i < n; i++) r += c;
    return r;
}

// Three-column border row: L══╬══╬══R
static std::string brow3(const char* L, const char* fill, const char* M, const char* R) {
    return std::string(L) + rpt(fill, CW) + M + rpt(fill, CW) + M + rpt(fill, CW) + R;
}

// Three-column data row: ║ a ║ b ║ c ║  (a,b,c are plain ASCII, padded to CW)
static std::string drow3(const std::string& a, const std::string& b, const std::string& c) {
    return BD_V + pad(a, CW) + BD_V + pad(b, CW) + BD_V + pad(c, CW) + BD_V;
}

// Full-width data row: ║ content (TW-2 visible chars) ║
static std::string frow(const std::string& s) {
    return std::string(BD_V) + pad(s, TW - 2) + BD_V;
}

// Full-width border row with optional centered title: L══ title ══R
static std::string full_brow(const char* L, const char* fill,
                              const char* title, const char* R) {
    int inner = TW - 2;
    std::string t = title && title[0] ? std::string(" ") + title + " " : "";
    int tlen = (int)t.size();
    int lf   = (inner - tlen) / 2;
    int rf   = inner - tlen - lf;
    if (rf < 0) { rf = 0; lf = inner - tlen; }
    return std::string(L) + rpt(fill, lf) + t + rpt(fill, rf) + R;
}

// Append a colored line followed by clear-to-EOL and newline.
static void emit(std::string& out, const char* color, const std::string& line) {
    if (color && color[0]) out += color;
    out += line;
    if (color && color[0]) out += RST;
    out += EOLN "\n";
}

// ── Formatting helpers ───────────────────────────────────────────────────────

static std::string timestamp() {
    auto now  = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buf[24];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return buf;
}

static std::string freq_cell(double hz, double lat_ms, bool has) {
    char buf[64];
    if (!has) {
        snprintf(buf, sizeof(buf), " Freq:  ---    Hz  [waiting...]");
    } else if (lat_ms > 0.0) {
        snprintf(buf, sizeof(buf), " Freq:%6.1f Hz  Lat:%5.1f ms", hz, lat_ms);
    } else {
        snprintf(buf, sizeof(buf), " Freq:%6.1f Hz", hz);
    }
    return buf;
}

static std::string pos_cell(const PoseSnap::Snap& s, int axis) {
    static const char ax[] = {'X', 'Y', 'Z'};
    char buf[32];
    if (s.has_data) snprintf(buf, sizeof(buf), "   %c: %+10.5f m", ax[axis], s.pos(axis));
    else            snprintf(buf, sizeof(buf), "   %c:     ---", ax[axis]);
    return buf;
}

static std::string rot_cell(const PoseSnap::Snap& s, int row) {
    char buf[48];
    if (s.has_data)
        snprintf(buf, sizeof(buf), "  [%+7.4f  %+7.4f  %+7.4f]",
                 s.rot(row, 0), s.rot(row, 1), s.rot(row, 2));
    else
        snprintf(buf, sizeof(buf), "  [   ---      ---      ---  ]");
    return buf;
}

// ── render() ─────────────────────────────────────────────────────────────────
void TUIDisplay::render() {
    // Snapshot all state (brief lock per call)
    auto bsnap   = ds_.base_pose.snap();
    auto rsnap   = ds_.rover_pose.snap();
    auto relsnap = ds_.rel_pose.snap();
    auto gtsnap  = ds_.gt_state.snap();
    double bhz   = ds_.base_freq.hz();
    double rhz   = ds_.rover_freq.hz();
    double relhz = ds_.rel_freq.hz();
    double gthz  = ds_.gt_freq.hz();

    std::string out;
    out.reserve(16384);
    out += HOME;   // move cursor to top-left without clearing (avoids flicker)

    // ── Title bar ─────────────────────────────────────────────────────────
    {
        std::string ts  = timestamp();
        std::string mid = " VICON2ZENOH  Live Pose Monitor ";
        int inner  = TW - 2;
        int remain = inner - (int)mid.size() - (int)ts.size() - 1;
        if (remain < 2) remain = 2;
        std::string bar = BD_TL + rpt(BD_H, 2) + mid + rpt(BD_H, remain) + ts + " " BD_TR;
        emit(out, CYN BLD, bar);
    }

    // ── Column header section ─────────────────────────────────────────────
    emit(out, CYN, brow3(BD_ML, BD_H, BD_TM, BD_MR));

    // Section name + measurement mode
    {
        char ba[64], ra[64], rela[64];
        snprintf(ba,   sizeof(ba),   " BASE  [%s]", ds_.base_label.c_str());
        snprintf(ra,   sizeof(ra),   " ROVER [%s]", ds_.rover_label.c_str());
        snprintf(rela, sizeof(rela), " RELATIVE  [%s]",
                 (ds_.meas_type_str == "base2rover") ? "base->rover" : "rover->base");
        emit(out, YEL BLD, drow3(ba, ra, rela));
    }

    // Zenoh keys
    {
        char bk[64], rk[64], relk[64];
        snprintf(bk,   sizeof(bk),   " Key: %s", ds_.base_key.c_str());
        snprintf(rk,   sizeof(rk),   " Key: %s", ds_.rover_key.c_str());
        snprintf(relk, sizeof(relk), " Key: %s", ds_.rel_key.c_str());
        emit(out, DIM, drow3(bk, rk, relk));
    }

    // Frequency / latency
    {
        std::string bf   = freq_cell(bhz,   ds_.base_cfg_lat_ms,  bsnap.has_data);
        std::string rf   = freq_cell(rhz,   ds_.rover_cfg_lat_ms, rsnap.has_data);
        std::string relf = freq_cell(relhz, ds_.rel_cfg_lat_ms,   relsnap.has_data);
        emit(out, GRN, drow3(bf, rf, relf));
    }

    // ── Position panel ────────────────────────────────────────────────────
    emit(out, CYN, brow3(BD_ML, BD_h, BD_MM, BD_MR));
    emit(out, BLD WHT, drow3(" Position (m)", " Position (m)", " Position (m)"));
    for (int ax = 0; ax < 3; ax++)
        emit(out, WHT, drow3(pos_cell(bsnap, ax), pos_cell(rsnap, ax), pos_cell(relsnap, ax)));

    // ── Rotation matrix panel ─────────────────────────────────────────────
    emit(out, nullptr, drow3("", "", ""));
    emit(out, BLD WHT, drow3(" Rotation Matrix", " Rotation Matrix", " Rotation Matrix"));
    for (int row = 0; row < 3; row++)
        emit(out, GRN, drow3(rot_cell(bsnap, row), rot_cell(rsnap, row), rot_cell(relsnap, row)));

    // ── Ground truth section ──────────────────────────────────────────────
    emit(out, CYN BLD, full_brow(BD_ML, BD_H, "GROUND TRUTH STATE", BD_MR));

    // GT key + frequency
    {
        char buf[TW + 16];
        if (ds_.gt_enabled)
            snprintf(buf, sizeof(buf), " Key: %-35s  Freq:%6.1f Hz%s",
                     ds_.gt_key.c_str(), gthz,
                     gtsnap.has_data ? "" : "  [waiting...]");
        else
            snprintf(buf, sizeof(buf), " GT state disabled in config");
        emit(out, GRN, frow(buf));
    }

    // GT vectors
    auto gt_vec = [&](const char* label, const Eigen::Vector3d& v, bool has) {
        char buf[TW + 8];
        if (has)
            snprintf(buf, sizeof(buf),
                     " %-16s  X: %+10.5f   Y: %+10.5f   Z: %+10.5f",
                     label, v(0), v(1), v(2));
        else
            snprintf(buf, sizeof(buf),
                     " %-16s  X:     ---         Y:     ---         Z:     ---", label);
        emit(out, WHT, frow(buf));
    };

    gt_vec("Position (m):",   gtsnap.pos,   gtsnap.has_data && ds_.gt_enabled);
    gt_vec("Velocity (m/s):", gtsnap.vel,   gtsnap.has_data && ds_.gt_enabled);
    gt_vec("Omega (rad/s):",  gtsnap.omega, gtsnap.has_data && ds_.gt_enabled);

    // GT rotation matrix
    emit(out, BLD WHT, frow(" Rotation Matrix:"));
    for (int row = 0; row < 3; row++) {
        char buf[64];
        if (gtsnap.has_data && ds_.gt_enabled)
            snprintf(buf, sizeof(buf), "  [%+7.4f  %+7.4f  %+7.4f]",
                     gtsnap.rot(row, 0), gtsnap.rot(row, 1), gtsnap.rot(row, 2));
        else
            snprintf(buf, sizeof(buf), "  [   ---      ---      ---  ]");
        emit(out, GRN, frow(buf));
    }

    // ── Footer ────────────────────────────────────────────────────────────
    emit(out, CYN BLD, full_brow(BD_BL, BD_H, "Ctrl+C to exit", BD_BR));

    fwrite(out.c_str(), 1, out.size(), stdout);
    fflush(stdout);
}

// ── TUIDisplay::run() ────────────────────────────────────────────────────────
void TUIDisplay::run() {
    fputs(HIDE CLR, stdout);   // hide cursor, clear screen once
    fflush(stdout);

    while (running_.load(std::memory_order_relaxed)) {
        render();
        std::this_thread::sleep_for(std::chrono::milliseconds(66));  // ~15 Hz
    }

    fputs(SHOW RST "\n\n", stdout);
    fflush(stdout);
}
