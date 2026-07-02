// Room-drift windowed-slope estimator test
// ---------------------------------------------------------------------------
// Validates update_room_drift_() (furrion_chill_cube.cpp): a 3-min trailing-window slope that
// replaced the EMA of per-sample instantaneous rates. The EMA failed on 2026-07-02: after gear 4
// arrested a climb, the plateau stopped emitting samples (flat room → no state change) and a
// 5.5-min reporting gap froze the EMA at a stale positive value, so `warming` stayed true and the
// wound-up bias grabbed gear 5 into an overshoot. This test replays that exact trace and asserts
// the windowed slope reads <= 0 at the 15:50 plateau (gate closes), while a genuine climb keeps it
// > 0 (gate stays open → a real 4<->5 hunt is preserved).
//
// The estimator logic below MIRRORS the firmware (kept in lockstep by hand, like the other tests).
//
// Build: g++ -std=c++17 -O2 -o tests/drift_window_test tests/drift_window_test.cpp && ./tests/drift_window_test
// ---------------------------------------------------------------------------
#include <cstdio>
#include <cstdint>
#include <cmath>

// ── Constants (mirror the .cpp) ────────────────────────────────────────────────
static const uint32_t DRIFT_WINDOW_MS       = 180000;
static const uint32_t DRIFT_BASELINE_MIN_MS =  90000;
static const uint32_t DRIFT_BASELINE_MAX_MS = 360000;
static const uint32_t DRIFT_STALE_MS        = 240000;   // positive drift older than this isn't trusted
static const float    ADAPT_UPSHIFT_DRIFT_MIN_CPM = 0.0f;
static const float    ADAPT_DRIFT_ALPHA     = 0.3f;   // old EMA, kept only to demonstrate the bug

// Mirror of the firmware `warming` gate (adaptive_cool_eff_diff_): a positive drift counts as
// warming only while a sample no older than DRIFT_STALE_MS backs it; NAN stays permissive.
static bool gate_warming(float drift, uint32_t now, uint32_t last_sample_at) {
  bool fresh = (last_sample_at != 0) && (now - last_sample_at <= DRIFT_STALE_MS);
  return std::isnan(drift) || (drift > ADAPT_UPSHIFT_DRIFT_MIN_CPM && fresh);
}

static float f_to_c(float f) { return (f - 32.0f) * (5.0f / 9.0f); }

// ── Mirror of update_room_drift_() ─────────────────────────────────────────────
struct DriftRing {
  static const uint8_t N = 48;
  uint32_t at[N] = {0};
  float temp[N] = {0};
  uint8_t head = 0, count = 0;

  // Push (now, temp_c); return the 3-min windowed slope (°C/min) or NAN.
  float update(uint32_t now, float temp_c) {
    at[head] = now; temp[head] = temp_c;
    head = (uint8_t)((head + 1) % N);
    if (count < N) count++;

    float base_temp = NAN; uint32_t base_at = 0, best_err = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < count; i++) {
      uint32_t age = now - at[i];
      if (age < DRIFT_BASELINE_MIN_MS || age > DRIFT_BASELINE_MAX_MS) continue;
      uint32_t err = (age > DRIFT_WINDOW_MS) ? (age - DRIFT_WINDOW_MS) : (DRIFT_WINDOW_MS - age);
      if (err < best_err) { best_err = err; base_temp = temp[i]; base_at = at[i]; }
    }
    if (std::isnan(base_temp)) return NAN;
    float dt_min = (now - base_at) / 60000.0f;
    return (temp_c - base_temp) / dt_min;
  }
};

// ── Old EMA (dt gated to [5s, 5min]) — reproduces the frozen-stale bug ───────────
struct OldEma {
  float ema = NAN, prev_c = NAN; uint32_t prev_at = 0;
  float update(uint32_t now, float temp_c) {
    if (!std::isnan(prev_c) && prev_at != 0) {
      float dt_min = (now - prev_at) / 60000.0f;
      if (dt_min >= (5.0f/60.0f) && dt_min <= 5.0f) {
        float inst = (temp_c - prev_c) / dt_min;
        ema = std::isnan(ema) ? inst : ADAPT_DRIFT_ALPHA*inst + (1.0f-ADAPT_DRIFT_ALPHA)*ema;
      }
    }
    prev_c = temp_c; prev_at = now;
    return ema;
  }
};

static int failures = 0;
static void check(bool ok, const char *msg) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok) failures++;
}

// The real 2026-07-02 trace (setpoint 76F). t = seconds from 15:30:33; the 845->1172 jump is the
// 5.5-min plateau gap; gear went 4->5 at t=1172 (15:50:05).
struct Sample { uint32_t t_s; float f; };
static const Sample TRACE[] = {
  {0,75.68},{33,75.74},{152,75.8},{180,75.86},{185,75.92},{240,75.98},{272,76.1},{300,76.15},
  {305,76.21},{332,76.27},{360,76.28},{365,76.34},{420,76.4},{452,76.46},{480,76.52},{485,76.58},
  {512,76.64},{540,76.7},{572,76.76},{600,76.82},{605,76.88},{660,76.89},{665,76.95},{692,77.07},
  {720,77.12},{845,77.18},{1172,77.12},{1260,77.11},{1320,77.06},
};

int main() {
  printf("=== Windowed drift: real 2026-07-02 plateau+gap trace ===\n");
  DriftRing ring; OldEma old;
  float drift_at_upshift = NAN, ema_at_upshift = NAN, drift_mid_climb = NAN;
  for (auto &s : TRACE) {
    uint32_t now = s.t_s * 1000u;
    float d = ring.update(now, f_to_c(s.f));
    float e = old.update(now, f_to_c(s.f));
    if (s.t_s == 692) drift_mid_climb = d;   // 15:42:05, room climbing hard
    if (s.t_s == 1172) { drift_at_upshift = d; ema_at_upshift = e; }  // the moment 4->5 fired
  }
  printf("  windowed drift @15:42 climb = %+.4f C/min\n", drift_mid_climb);
  printf("  windowed drift @15:50 plateau = %+.4f C/min   (OLD ema = %+.4f)\n",
         drift_at_upshift, ema_at_upshift);
  check(drift_mid_climb > 0.02f,   "climb: windowed drift clearly positive (gate open, as intended)");
  check(drift_at_upshift <= 0.0f,  "plateau: windowed drift <= 0 -> gate CLOSES -> gear-5 grab blocked");
  check(ema_at_upshift  > 0.0f,    "plateau: OLD ema still > 0 (frozen stale) -> documents the bug");

  printf("\n=== Genuine hunt preserved: gear 4 losing ground, room climbing +0.06 C/min ===\n");
  DriftRing ring2; float min_drift = 1e9f; bool any = false;
  for (uint32_t m = 0; m <= 15; m++) {          // 1 sample/min, steady rise
    float d = ring2.update(m * 60000u, 24.0f + 0.06f * m);
    if (m >= 4 && !std::isnan(d)) { any = true; if (d < min_drift) min_drift = d; }
  }
  printf("  min windowed drift after warmup = %+.4f C/min\n", min_drift);
  check(any && min_drift > 0.02f, "steady climb: drift stays > 0 -> gate OPEN -> gear 5 still permitted");

  printf("\n=== Quiet plateau with 1 tick/min stays ~0 (no false upshift) ===\n");
  DriftRing ring3; float max_abs = 0.0f; bool any3 = false;
  float noise[] = {0.0f, 0.02f, -0.01f, 0.01f, -0.02f, 0.0f, 0.01f};  // ±0.02C jitter, flat mean
  for (uint32_t m = 0; m <= 6; m++) {
    float d = ring3.update(m * 60000u, 25.0f + noise[m]);
    if (m >= 4 && !std::isnan(d)) { any3 = true; if (std::fabs(d) > max_abs) max_abs = std::fabs(d); }
  }
  printf("  max |windowed drift| over flat plateau = %.4f C/min\n", max_abs);
  check(any3 && max_abs < 0.02f, "flat plateau: |drift| < 0.02 -> gate closed (no windup upshift)");

  printf("\n=== Staleness guard: a silent plateau can't grab a top gear off frozen climb data ===\n");
  // Climb to a positive drift, note the last sample time, then go quiet (no more samples).
  DriftRing ring4; float d = NAN; uint32_t last_at = 0;
  for (uint32_t m = 0; m <= 6; m++) { last_at = m*60000u; d = ring4.update(last_at, 24.0f + 0.06f*m); }
  bool warm_fresh = gate_warming(d, last_at + 60000u,  last_at);  // 1 min after last sample
  bool warm_stale = gate_warming(d, last_at + 300000u, last_at);  // 5 min of silence (> DRIFT_STALE_MS)
  printf("  frozen drift = %+.4f C/min; warming @+1min=%d  @+5min=%d\n", d, warm_fresh, warm_stale);
  check(d > 0.0f && warm_fresh,  "fresh positive drift: gate OPEN (genuine climb still upshifts)");
  check(!warm_stale,             "stale positive drift (5-min silence): gate CLOSED -> no top-gear grab");

  printf("\n%s (%d failure%s)\n", failures ? "FAILURES" : "ALL PASS", failures, failures==1?"":"s");
  return failures ? 1 : 0;
}
