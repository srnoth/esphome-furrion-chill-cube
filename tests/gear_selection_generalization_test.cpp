// Proves the v2 generalized N-gear selection (design_gear_engine_v2) reproduces the OLD hardcoded
// case-0/1/2/3 switch EXACTLY for cool_max_gear_ == 3 (both cool and heat), and behaves monotonically
// for a 4-gear ladder. Isolates the pure threshold logic: can_upshift assumed true, adaptive off
// (eff_diff == up_diff == diff), so this tests only the switch→loop generalization.
//
// Build: g++ -std=c++17 -O2 -o tests/gear_selection_generalization_test tests/gear_selection_generalization_test.cpp && ./tests/gear_selection_generalization_test
#include <cstdio>

static int failures = 0;
#define CHECK(cond, msg, ...) do { if (!(cond)) { printf("FAIL: " msg "\n", ##__VA_ARGS__); failures++; } } while (0)

// ---- Ladder (spacing S=0.55, h=0; pins = shipped 2026-07-08 values) ----
static const float S = 0.55f;
static float cool_up[8], cool_dn[8], heat_up[8], heat_dn[8];
static const float cool_start = 0.35f, cool_stop = 0.15f, cool_idle = -0.30f;
static const float heat_start = -0.35f, heat_stop = -0.15f, heat_idle = 0.30f;
static void build() {
  for (int n = 1; n < 8; n++) {
    cool_up[n] = n * S; cool_dn[n] = n * S;      // h=0
    heat_up[n] = -(n * S); heat_dn[n] = -(n * S);
  }
}

// ================= OLD (verbatim, M=3) =================
static bool old_in_band_cool(int g, float d) {
  const float C_UP_01=0.35f,C_UP_12=0.55f,C_UP_23=1.10f,C_DN_10=0.15f,C_DN_21=0.55f,C_DN_32=1.10f,C_IDLE=-0.30f;
  switch (g) {
    case 0: return d >= C_IDLE  && d <= C_UP_01;
    case 1: return d >= C_DN_10 && d <= C_UP_12;
    case 2: return d >= C_DN_21 && d <= C_UP_23;
    case 3: return d >= C_DN_32;
    default: return false;
  }
}
static bool old_in_band_heat(int g, float d) {
  const float H_UP_01=-0.35f,H_UP_12=-0.55f,H_UP_23=-1.10f,H_DN_10=-0.15f,H_DN_21=-0.55f,H_DN_32=-1.10f,H_IDLE=0.30f;
  switch (g) {
    case 0: return d >= H_UP_01 && d <= H_IDLE;
    case 1: return d >= H_UP_12 && d <= H_DN_10;
    case 2: return d >= H_UP_23 && d <= H_DN_21;
    case 3: return d <= H_DN_32;
    default: return false;
  }
}
// OLD active-gear step (can_upshift true, eff==up==diff)
static int old_step_cool(int g, float d) {
  const float C_UP_12=0.55f,C_UP_23=1.10f,C_DN_10=0.15f,C_DN_21=0.55f,C_DN_32=1.10f;
  int ng = g;
  switch (g) {
    case 1: if (d > C_UP_12) ng = 2; else if (d < C_DN_10) ng = 0; break;
    case 2: if (d > C_UP_23) ng = 3; else if (d < C_DN_21) ng = 1; break;
    case 3: if (d < C_DN_32) ng = 2; break;
  }
  return ng;
}
static int old_step_heat(int g, float d) {
  const float H_UP_12=-0.55f,H_UP_23=-1.10f,H_DN_10=-0.15f,H_DN_21=-0.55f,H_DN_32=-1.10f;
  int ng = g;
  switch (g) {
    case 1: if (d < H_UP_12) ng = 2; else if (d > H_DN_10) ng = 0; break;   // 1→0 on real diff
    case 2: if (d < H_UP_23) ng = 3; else if (d > H_DN_21) ng = 1; break;
    case 3: if (d > H_DN_32) ng = 2; break;
  }
  return ng;
}
static int old_pick_cool(float d) {
  const float C_UP_01=0.35f,C_UP_12=0.55f,C_UP_23=1.10f;
  if (d > C_UP_23) return 3; if (d > C_UP_12) return 2; if (d > C_UP_01) return 1; return 0;
}
static int old_pick_heat(float d) {
  const float H_UP_01=-0.35f,H_UP_12=-0.55f,H_UP_23=-1.10f;
  if (d < H_UP_23) return 3; if (d < H_UP_12) return 2; if (d < H_UP_01) return 1; return 0;
}

// ================= NEW (generalized, verbatim logic from the refactored component) =================
static bool new_in_band_cool(int g, float d, int M) {
  if (g < 0 || g > M) return false;
  if (g == 0) return d >= cool_idle && d <= cool_start;
  float dn = (g == 1) ? cool_stop : cool_dn[g-1];
  if (g == M) return d >= dn;
  return d >= dn && d <= cool_up[g];
}
static bool new_in_band_heat(int g, float d, int M) {
  if (g < 0 || g > M) return false;
  if (g == 0) return d >= heat_start && d <= heat_idle;
  float up = heat_up[g];
  float dn = (g == 1) ? heat_stop : heat_dn[g-1];
  if (g == M) return d <= dn;
  return d >= up && d <= dn;
}
static int new_step_cool(int g, float d, int M) {
  if (g < M && d > cool_up[g]) return g + 1;
  float dn = (g == 1) ? cool_stop : cool_dn[g-1];
  if (d < dn) return g - 1;
  return g;
}
static int new_step_heat(int g, float d, int M) {
  if (g < M && d < heat_up[g]) return g + 1;
  float dn = (g == 1) ? heat_stop : heat_dn[g-1];
  float dcmp = d;   // gear 1 STOP uses real diff; gears 2+ use eff — equal here (adaptive off)
  if (dcmp > dn) return g - 1;
  return g;
}
static int new_pick_cool(float d, int M) {
  for (int g = M; g >= 1; g--) { float e = (g <= 1) ? cool_start : cool_up[g-1]; if (d > e) return g; }
  return 0;
}
static int new_pick_heat(float d, int M) {
  for (int g = M; g >= 1; g--) { float e = (g <= 1) ? heat_start : heat_up[g-1]; if (d < e) return g; }
  return 0;
}

int main() {
  build();
  // Equivalence for M=3 across a fine diff sweep.
  for (int i = -200; i <= 300; i++) {
    float d = i * 0.01f;
    for (int g = 0; g <= 3; g++) {
      CHECK(old_in_band_cool(g, d) == new_in_band_cool(g, d, 3), "cool band g=%d d=%.2f", g, d);
      CHECK(old_in_band_heat(g, d) == new_in_band_heat(g, d, 3), "heat band g=%d d=%.2f", g, d);
    }
    for (int g = 1; g <= 3; g++) {
      CHECK(old_step_cool(g, d) == new_step_cool(g, d, 3), "cool step g=%d d=%.2f: old=%d new=%d",
            g, d, old_step_cool(g, d), new_step_cool(g, d, 3));
      CHECK(old_step_heat(g, d) == new_step_heat(g, d, 3), "heat step g=%d d=%.2f: old=%d new=%d",
            g, d, old_step_heat(g, d), new_step_heat(g, d, 3));
    }
    CHECK(old_pick_cool(d) == new_pick_cool(d, 3), "cool pick d=%.2f: old=%d new=%d",
          d, old_pick_cool(d), new_pick_cool(d, 3));
    CHECK(old_pick_heat(d) == new_pick_heat(d, 3), "heat pick d=%.2f: old=%d new=%d",
          d, old_pick_heat(d), new_pick_heat(d, 3));
  }

  // M=4 smoke: pick_from_below is monotonic non-decreasing in diff, reaches gear 4, and the top-gear
  // band has no upper bound (staying in gear 4 for very hot). Trips at 0.35/0.55/1.10/1.65.
  int prev = 0;
  for (int i = 0; i <= 300; i++) {
    float d = i * 0.01f;
    int p = new_pick_cool(d, 4);
    CHECK(p >= prev, "cool pick M=4 non-monotonic at d=%.2f (%d < %d)", d, p, prev);
    prev = p;
  }
  CHECK(new_pick_cool(2.0f, 4) == 4, "cool pick M=4 should reach gear 4 at d=2.0 (got %d)", new_pick_cool(2.0f, 4));
  CHECK(new_pick_cool(0.40f, 4) == 1, "cool pick M=4 g1 band (got %d)", new_pick_cool(0.40f, 4));
  CHECK(new_pick_cool(0.60f, 4) == 2, "cool pick M=4 g2 band (got %d)", new_pick_cool(0.60f, 4));
  CHECK(new_pick_cool(1.20f, 4) == 3, "cool pick M=4 g3 band (got %d)", new_pick_cool(1.20f, 4));
  CHECK(new_pick_cool(1.70f, 4) == 4, "cool pick M=4 g4 band (got %d)", new_pick_cool(1.70f, 4));
  CHECK(new_in_band_cool(4, 5.0f, 4), "cool gear 4 (top) must have no upper bound");
  CHECK(new_step_cool(4, 2.0f, 4) == 4, "cool gear 4 no upshift (got %d)", new_step_cool(4, 2.0f, 4));
  CHECK(new_step_cool(4, 1.0f, 4) == 3, "cool gear 4→3 downshift below cool_dn[3]=1.65 (got %d)", new_step_cool(4, 1.0f, 4));

  if (failures == 0) printf("PASS: generalized selection == old switch (M=3), M=4 monotonic + bounded\n");
  else printf("%d FAILURES\n", failures);
  return failures ? 1 : 0;
}
