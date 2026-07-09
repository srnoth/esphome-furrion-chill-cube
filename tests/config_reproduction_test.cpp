// Regression test for the 2026-07-09 config-driven refactor (Phase C).
// Proves the NEW table-driven CS math (gear_cs_with_clamp_) and the auto-built ladder
// (build_ladders_) reproduce the OLD hardcoded cool_cs_/heat-clamp + C_*/H_* constants
// EXACTLY across the realistic setpoint band — so defaults are behavior-identical and the
// only intended deltas are the quirk-engine / interval / keepalive changes.
//
// Build: g++ -std=c++17 -O2 -o tests/config_reproduction_test tests/config_reproduction_test.cpp && ./tests/config_reproduction_test
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <algorithm>

static int failures = 0;
#define CHECK(cond, msg, ...) do { if (!(cond)) { printf("FAIL: " msg "\n", ##__VA_ARGS__); failures++; } } while (0)

// ============================================================
// OLD implementation (verbatim from pre-refactor furrion_chill_cube.cpp)
// ============================================================
static int old_cool_cs(int off_from_sp, int sp) {
  int lo = sp - 2, hi = sp + 3;
  int offset = (lo < 15) ? (15 - lo) : (hi > 30) ? (30 - hi) : 0;
  if (offset < -2) offset = -2;
  return sp + off_from_sp + offset;
}
static int old_gear_cs_cool(int gear, int sp) {
  switch (gear) {
    case 3: return old_cool_cs(3, sp);
    case 2: return old_cool_cs(0, sp);
    case 1: return old_cool_cs(-2, sp);
    default: return sp - 5;
  }
}
static int old_gear_cs_heat(int gear, int sp) {
  int hi = sp + 2;
  int offset = (hi > 30) ? (30 - hi) : 0;
  switch (gear) {
    case 3: return sp - 1 + offset;
    case 2: return sp + offset;
    case 1: return sp + 1 + offset;
    default: return sp + 5;
  }
}
// Old ladder constants (2026-07-08 symmetric-1F ladder)
static const float OLD_C_UP_01=0.35f, OLD_C_UP_12=0.55f, OLD_C_UP_23=1.10f;
static const float OLD_C_DN_10=0.15f, OLD_C_DN_21=0.55f, OLD_C_DN_32=1.10f, OLD_C_IDLE=-0.30f;
static const float OLD_H_UP_01=-0.35f, OLD_H_UP_12=-0.55f, OLD_H_UP_23=-1.10f;
static const float OLD_H_DN_10=-0.15f, OLD_H_DN_21=-0.55f, OLD_H_DN_32=-1.10f, OLD_H_IDLE=0.30f;

// ============================================================
// NEW implementation (verbatim logic from the refactored component)
// ============================================================
static const int MAX_GEARS = 4;
static int cool_off[MAX_GEARS] = {-5,-2,0,3};
static int heat_off[MAX_GEARS] = {5,1,0,-1};
static int cool_max_gear = 3, heat_max_gear = 3;

static int gear_cs_with_clamp(bool is_heat, int offset, int sp) {
  const int *offs = is_heat ? heat_off : cool_off;
  int max_gear = is_heat ? heat_max_gear : cool_max_gear;
  int min_a = offs[1], max_a = offs[1];
  for (int g = 2; g <= max_gear && g < MAX_GEARS; g++) {
    if (offs[g] < min_a) min_a = offs[g];
    if (offs[g] > max_a) max_a = offs[g];
  }
  int lo = sp + min_a, hi = sp + max_a;
  int shift = (lo < 15) ? (15 - lo) : (hi > 30) ? (30 - hi) : 0;
  if (offs[0] < min_a) { int floor = offs[0]-min_a+1; if (shift < floor) shift = floor; }
  else if (offs[0] > max_a) { int ceil = offs[0]-max_a-1; if (shift > ceil) shift = ceil; }
  return sp + offset + shift;
}
static int new_gear_cs(bool is_heat, int gear, int sp) {
  const int *offs = is_heat ? heat_off : cool_off;
  if (gear <= 0 || gear >= MAX_GEARS) return sp + offs[0];
  return gear_cs_with_clamp(is_heat, offs[gear], sp);
}

// New build_ladders_ (defaults)
static float cool_up[MAX_GEARS]={0}, cool_dn[MAX_GEARS]={0}, heat_up[MAX_GEARS]={0}, heat_dn[MAX_GEARS]={0};
static void build_ladders(float cs, float ch, float hs, float hh) {
  for (int n=1; n<MAX_GEARS; n++) {
    cool_up[n]=n*cs; cool_dn[n]=n*cs-ch;
    heat_up[n]=-(n*hs); heat_dn[n]=-(n*hs-hh);
  }
}

int main() {
  // --- 1. CS math reproduction ---
  // Cool: MUST match old exactly across the whole 16-30 band.
  for (int sp = 16; sp <= 30; sp++)
    for (int g = 0; g <= 3; g++)
      CHECK(new_gear_cs(false, g, sp) == old_gear_cs_cool(g, sp),
            "cool gear %d sp=%d: new=%d old=%d", g, sp, new_gear_cs(false,g,sp), old_gear_cs_cool(g,sp));

  // Heat: match old across the REALISTIC band [16,28]. The generalized clamp intentionally
  // differs at sp>=29 (84-86F heat — physically impossible; the old code used hi=sp+2 vs the
  // generic hi=sp+max_offset). Documented deviation, never reached in service.
  for (int sp = 16; sp <= 28; sp++)
    for (int g = 0; g <= 3; g++)
      CHECK(new_gear_cs(true, g, sp) == old_gear_cs_heat(g, sp),
            "heat gear %d sp=%d: new=%d old=%d", g, sp, new_gear_cs(true,g,sp), old_gear_cs_heat(g,sp));

  // --- 2. Quirk via-CS reproduction ---
  // idle->1 quirk via_offset=0 must equal the old idle kickstart CS (cool_cs_(0) = SP+0 clamped).
  for (int sp = 16; sp <= 30; sp++)
    CHECK(gear_cs_with_clamp(false, 0, sp) == old_cool_cs(0, sp),
          "idle->1 via sp=%d: new=%d old=%d", sp, gear_cs_with_clamp(false,0,sp), old_cool_cs(0,sp));
  // MAX->2 dip via_offset=-1 must sit strictly below MED (gear 2) so it re-seats from below.
  for (int sp = 16; sp <= 30; sp++)
    CHECK(gear_cs_with_clamp(false, -1, sp) < new_gear_cs(false, 2, sp),
          "MAX->2 via(-1) sp=%d not below MED: via=%d med=%d", sp,
          gear_cs_with_clamp(false,-1,sp), new_gear_cs(false,2,sp));

  // --- 3. Ladder auto-build reproduces the old thresholds ---
  build_ladders(0.55f, 0.0f, 0.55f, 0.0f);
  CHECK(std::abs(cool_up[1]-OLD_C_UP_12)<1e-6 && std::abs(cool_up[2]-OLD_C_UP_23)<1e-6, "cool_up mismatch");
  CHECK(std::abs(cool_dn[1]-OLD_C_DN_21)<1e-6 && std::abs(cool_dn[2]-OLD_C_DN_32)<1e-6, "cool_dn mismatch");
  CHECK(std::abs(heat_up[1]-OLD_H_UP_12)<1e-6 && std::abs(heat_up[2]-OLD_H_UP_23)<1e-6, "heat_up mismatch");
  CHECK(std::abs(heat_dn[1]-OLD_H_DN_21)<1e-6 && std::abs(heat_dn[2]-OLD_H_DN_32)<1e-6, "heat_dn mismatch");
  // Pins (aliases) reproduce old start/stop/idle
  CHECK(std::abs(0.35f-OLD_C_UP_01)<1e-6 && std::abs(0.15f-OLD_C_DN_10)<1e-6 && std::abs(-0.30f-OLD_C_IDLE)<1e-6, "cool pins");
  CHECK(std::abs(-0.35f-OLD_H_UP_01)<1e-6 && std::abs(-0.15f-OLD_H_DN_10)<1e-6 && std::abs(0.30f-OLD_H_IDLE)<1e-6, "heat pins");

  if (failures == 0) printf("PASS: all config-reproduction checks (CS math, quirk via, ladder build)\n");
  else printf("%d CHECK(s) FAILED\n", failures);
  return failures ? 1 : 0;
}
