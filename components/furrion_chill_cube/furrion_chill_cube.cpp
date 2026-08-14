#include "furrion_chill_cube.h"
#include "esphome/core/log.h"
#include <cmath>  // expf, isnan (adaptive integral decay)

#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

#ifdef USE_ESP32
#include <esp_system.h>  // esp_reset_reason() — gates the gear restore on warm vs cold boot
#endif

namespace esphome {
namespace furrion_chill_cube {

static const char *const TAG = "furrion_chill_cube";

// ============================================================
// IR Protocol Constants — RAC-PT1411HWRU
// ============================================================

static const uint16_t IR_HEADER_MARK = 4380;
static const uint16_t IR_HEADER_SPACE = 4370;
static const uint16_t IR_BIT_MARK = 540;
static const uint16_t IR_ZERO_SPACE = 540;
static const uint16_t IR_ONE_SPACE = 1620;
static const uint16_t IR_GAP_SPACE = 5480;
static const uint16_t IR_PACKET_SPACE = 10500;
static const uint16_t IR_CARRIER_FREQ = 38000;
// Inter-MESSAGE guard idle, prepended to each CS / mode frame. Distinct from
// IR_GAP_SPACE (5480µs, the INTRA-message gap between a frame's repeats) and
// larger than the 10500µs that already delimits the swing frame reliably. With
// non_blocking:false, consecutive transmit.perform() calls leave no idle between
// them, so a CS→MODE-ON→CS bracket (or the quick-kickstart's CS→MODE-ON) arrives
// jammed at ~intra-message spacing and the unit can merge/misparse the mode-on.
// That is invisible on a running unit (the mode frame is redundant there) but
// makes a cold OFF→ON power-on intermittently fail — the mode-on is its only
// shot. A leading idle on every frame guarantees a clean message boundary.
//
// 2026-06-20: raised 25ms → 100ms. 25ms reliably fixed the OFF→ON case but a
// running setpoint change (CS→MODE→CS bracket, where the new setpoint rides the
// middle MODE frame sandwiched between two CS frames) still intermittently failed
// to register — confirmed from the camper's recorder DB by repeated up/down
// jiggling on climate.camper_ac. 100ms is closer to the idle a physical remote
// leaves between distinct commands; the unit parses each frame cleanly. With
// non_blocking:false the CPU idle-waits while the RMT peripheral clocks each frame
// out in hardware (so IR timing is unaffected — only CPU responsiveness): a full
// CS→MODE→CS bracket is ~430ms (CS + MODE + the MODE's swing sub-frame + CS, each
// with a 100ms lead), and a fresh-start pass that also hits the HVAC-on path can
// stack two brackets (~0.8s) in one loop iteration. Acceptable on this ESP32-C3
// (RMT-driven IR, no hard real-time work; ESPHome feeds the task WDT) but worth a
// hardware check that a worst-case pass doesn't hiccup the API. A bracket only fires
// on a settled setpoint/mode change; the 30s heartbeat is a single frame.
static const uint32_t IR_INTER_MSG_GAP = 100000;  // 100ms

// Temperature encoding: non-linear Gray code lookup
// RAC-PT1411HWRU Celsius table (index 0=16°C, 14=30°C)
// Bits: [5]=FRAC, [4]=NEG, [3:0]=temp nibble
static const uint8_t TEMP_C_TABLE[] = {
    0x10, 0x00, 0x01, 0x03, 0x02, 0x06, 0x07, 0x05,
    0x04, 0x0C, 0x0D, 0x09, 0x08, 0x0A, 0x0B};

// RAC-PT1411HWRU Fahrenheit table (index 0=60°F, 26=86°F)
// Same flag bits as the C table. Source: esphome PR #14666
// (esphome/components/toshiba/toshiba.cpp RAC_PT1411HWRU_TEMPERATURE_F).
// In F-mode, message[9] also gets bit 0x01 (FLAG_FAH) set so the unit's
// panel displays in °F.
static const uint8_t TEMP_F_TABLE[] = {
    0x10, 0x30, 0x00, 0x20, 0x01, 0x21, 0x03, 0x23, 0x02,
    0x22, 0x06, 0x26, 0x07, 0x05, 0x25, 0x04, 0x24, 0x0C,
    0x2C, 0x0D, 0x2D, 0x09, 0x08, 0x28, 0x0A, 0x2A, 0x0B};

// Celsius setpoint range (Furrion accepts 16-30°C)
static const int FURRION_MIN_TEMP_C = 16;
static const int FURRION_MAX_TEMP_C = 30;

// Fahrenheit setpoint range (Toshiba F-protocol bounds, 60-86°F).
// Approximately equivalent to the C range; F resolution is 1°F.
static const int FURRION_MIN_TEMP_F = 60;
static const int FURRION_MAX_TEMP_F = 86;

// Gear controller constants — no fixed anchors; setpoint is dynamic

// Per-gear upshift hold times (ms) — indexed by target gear number.
// Standard name: MINIMUM DWELL TIMES (move constraints) for the stage sequencer —
// the upshift-side inter-stage timers of classic staged-equipment control.
// HOLD_MS[0] is unused (gear 0 never upshifts via can_upshift_to) but
// kept for direct array indexing by gear number without offset.
//                        0      1       2       3        4        5        6        7
static const uint32_t HOLD_MS[] = {0, 180000, 180000, 300000, 300000, 600000, 600000, 600000};
// (v2: the OFF→gear clamp duration is now the from_gear:-1 quirk's own duration — default 305s in
// climate.py — so the old fixed CLAMP_DURATION_MS constant is gone.)
// Idle→LOW/MED restart is now a configurable transition quirk (idle→Cool1 via SP+0), held for
// quirk_duration_ms_ and re-asserted every quirk_transmit_interval_ms_ so the restart CS outlasts
// the compressor's anti-short-cycle lockout (~3 min). See session_log 2026-05-20 (a 10s idle kick
// died inside the lockout shadow) and the 2026-07-09 quirk-engine redesign. The from-idle Cool2
// restart quirk was dropped: the shorter cs_transmit_interval_ms_ (10s) re-asserts SP+0 densely
// enough to cross the lockout on its own. CS heartbeat cadence is cs_transmit_interval_ms_.
static const uint32_t GEAR_INTERVAL_MS = 60000;  // 60s fallback gear-pass cadence
// Setpoint debounce: a temperature change is held this long (steady state, no further
// change) before it is committed to IR. Coalesces a rapid multi-step adjustment (user
// holding temp-down: 68→67→66→65→64→63 as 5 separate control() calls) into ONE transmit
// of the final value, instead of a burst of overlapping CS→MODE→CS brackets that collide
// on the wire and cause steps to be missed. Only the final setpoint matters to the
// open-loop unit. Tune here: long enough to swallow a button-mash, short enough that a
// single deliberate change still acts promptly. Does NOT delay HA's display (the climate
// state updates immediately in control()); only the IR is deferred. Mode/fan changes are
// NOT debounced — they flush immediately.
static const uint32_t SETPOINT_SETTLE_MS = 2500;  // 2.5s
// Mode switch lockouts are now configurable member variables (mode_switch_*_ms_, mode_switch_temp_offset_c_)

// Heating deadbands (diff = room - target, negative = cold; more negative = higher heat gear)
//
// ── 2026-07-08 SYMMETRIC 1°F LADDER (mirror of cool; heat side WINTER-VALIDATED, not field-tested) ──
// The MIRROR of the cool redesign below. The GEAR-modulation boundaries (1↔2, 2↔3) are now SYMMETRIC
// single lines — up-trip == down-trip, i.e. ZERO programmed hysteresis — spaced ~1°F (0.55°C) apart.
// Rationale (2026-07-08 design discussion, full thread in session_log): in THIS system short-cycling
// cannot occur (the Furrion's own ~5-min min-cycle gates the compressor; large cabin thermal inertia +
// low-noise sensors mean the temp signal never chatters across a line), so programmed h earns nothing
// and only widens the swing. The 1°F rung is RUNWAY, not anti-chatter: it gives each modulation gear
// ~1°F (symmetric) to arrest a climb / catch a descent — e.g. MED catching MAX's ~9A wind-down
// momentum instead of dropping straight through to LOW. The slow integral (bias_h_) is UNCHANGED and
// still centers whichever boundary the load selects on the heat setpoint.
//   • 0↔1 (start/stop) + idle: NOT on the 1°F grid. H_DN_10 is PINNED near setpoint by the heat→cool
//     pong constraint (2026-04-13) and run_heat_mode_ evaluates 1→0 on REAL diff so bias can't move it.
//   • Asymmetric runway (descent ≠ ascent) is NOT used — no data yet shows descent needs more room.
//     Re-add only if a symmetric 1°F descent runway still drops MAX→MED→LOW on the hottest days
//     (momentum is the one directional asymmetry; that drop-through is its signature).
// Up-rail (start/upshift): -0.35 / -0.55 / -1.10 ; Down-rail (stop/downshift): -0.15 / -0.55 / -1.10.
// Monotonic (more negative = higher gear). ⚠️ UNTESTABLE until heating season — validate the pong
// margin + centering on the first real heat cycle before trusting.
// Ladder thresholds are now CONFIGURABLE members (heat_start_/heat_stop_/heat_idle_ pins +
// heat_up_[]/heat_dn_[] modulation trips auto-built from heat_spacing_/heat_hyst_ in
// build_ladders_()). Each run_*_mode_ / gear_in_band_* aliases them back to the old H_*/C_*
// names at function entry so the selection logic stays byte-identical. Defaults reproduce the
// 2026-07-08 symmetric-1°F ladder. NOTE the two PINNED boundaries kept off the uniform grid:
//   • start/stop (0↔1): compressor start/stop hysteresis; stop (H_DN_10=-0.15) is heat→cool
//     pong-critical — stops heat 0.27°F below setpoint so the ~0.85°F post-drop carry lands
//     inside mode_switch_temp_offset_c_ (2026-04-13). run_heat_mode_ evaluates 1→0 on REAL diff
//     so bias_h_ can't move it.
//   • idle (0→-1).

// Cooling deadbands (diff = room - target, positive = hot)
//
// ── 2026-07-08 SYMMETRIC 1°F LADDER (PROVISIONAL — pending a real hot cool-day cycle) ──
// The GEAR-modulation boundaries (1↔2 = LOW↔MED, 2↔3 = MED↔MAX) are now SYMMETRIC single lines —
// up-trip == down-trip, ZERO programmed hysteresis — spaced ~1°F (0.55°C) apart. Replaces the
// 2026-07-07 S=0.25 / h=0.20 hysteretic ladder (and the 07-08 asymmetric 3→2 early-handoff hack).
// Rationale (2026-07-08 design discussion — full thread in session_log):
//   • Short-cycling CANNOT occur here: we don't drive the compressor directly (the Furrion's internal
//     ~5-min min-cycle gates it), and cabin thermal inertia + low-noise sensors mean the temp signal
//     never chatters across a line. So programmed hysteresis earns nothing and only WIDENS the swing.
//   • The 1°F rung is RUNWAY, not anti-chatter: it gives MED a full ~1°F (both directions) to arrest a
//     climb before punting to MAX, and to catch MAX's ~9A wind-down momentum on the descent instead of
//     overshooting through MED to LOW (the 07-08 hot-day 1↔3 thrash). Ascent == descent (symmetric).
//   • The slow integral (bias_c_) is UNCHANGED — it centers whichever boundary the load selects on
//     setpoint (~1°F / 9-18 min). No rate-trigger / rate-adaptive gain was added: a fast upshift that
//     doesn't recenter buys ~0.5°F off the peak for a self-decaying offset — not worth the machinery.
//   • Effective hysteresis still exists for free: upshift uses rate-gated up_diff (bias stripped when
//     not warming) while downshift uses full eff_diff, so a wound bias holds the higher gear a bit —
//     load-adaptive, no constant needed.
// 0↔1 (compressor start/stop) + idle KEPT off the 1°F grid: preserves the tight overnight LOW↔idle
// hunt (0.6°F band) and the stop pin (carry lands at setpoint; mirror of heat's pong-pinned -0.15).
// Asymmetric-runway is shelved (no data shows it's needed); re-add only if 1°F symmetric still drops
// through on hot days. Up-rail 0.35 / 0.55 / 1.10 ; Down-rail 0.15 / 0.55 / 1.10 — monotonic, mirror heat.
// (cool ladder values moved to configurable members — see the heat note above; defaults
// reproduce the symmetric-1°F cool ladder: start 0.35 / stop 0.15 / idle -0.30, rungs 0.55·n).

// ── Phase 2 adaptive equilibrium-gear controller (cool mode) ────────────────────
// A slow integral floats the cool ladder's operating point to the gear that sustains
// the current load (which is NOT observable from outside temp — bodies, the CO2 vent
// fan, solar gain). PI control: the static ladder is the proportional inner loop; this
// integral removes its steady-state offset. See PHASE2_ADAPTIVE_DESIGN.md.
static const float ADAPT_KI = 0.06f;          // integral gain: bias_c (°C) per (°C-error · min). Primary tuning knob.
static const float ADAPT_BIAS_C_MAX = 2.0f;   // authority clamp (~2-3 gears) + runaway backstop
static const float ADAPT_DEADBAND_C = 0.15f;  // don't integrate noise / tiny offset
// Idle decay time constant. Idle is BLIND: the compressor is off, so unlike an active gear (where a
// stale bias self-corrects the instant load moves — the room drifts, e flips sign, the integrator
// unwinds) there is NO feedback to catch a bias that has gone stale. This decay is the only stand-in
// for that missing feedback, so it must run on the timescale over which the real load actually changes
// while we're blind (HOURS), NOT the timescale of a normal cooling cycle (minutes). It is emphatically
// NOT anti-windup. A natural glide to idle (2↔1↔0 hunts) already unwinds the bias via e<0 on the way
// down, so the ONLY path that reaches idle with a heavily-wound bias is a discontinuity — SP jumped or
// the unit was toggled off mid-load — where the physical load is UNCHANGED and the bias should be kept.
// τ=30 was pathological: it sat on the cycle timescale and shed ~1/3 of a still-valid bias across a
// ~12-min SP-nudge idle (2026-07-20 incident: re-engaged underpowered → ~1°F overshoot for ~25 min).
// τ=180 makes short idles near-lossless (~6%/12min) while still fully forgetting an overnight idle.
static const float ADAPT_DECAY_TAU_MIN = 180.0f; // idle memory-fade; hours, not minutes (see note above)

// Approach-side predictive re-engagement (fix-setpoint-transition-integral §Deferred design).
// APPROACH_DRIFT_MIN_CPM: minimum sustained drift magnitude (°C/min) treated as a real approach —
// below this the room is considered parked (storage: no fire). 0.02 °C/min ≈ 0.036 °F/min, well
// under the observed morning free-warm (~0.09 °C/min) and above sensor jitter on the 3-min window.
// APPROACH_RETRY_MS: cooldown after an aborted approach (drift died) before re-arming, so a
// transient gust can't cycle the compressor in storage. Hold caps are multiples of the lead:
// abort when the recomputed crossing recedes past 2× lead (setpoint moved away / cloud), and
// hard-cap the hold at 3× lead (drift overestimated).
static const float APPROACH_DRIFT_MIN_CPM = 0.02f;
static const uint32_t APPROACH_RETRY_MS = 600000;  // 10 min
// Crossing-preload guards (bug-check 2026-08-10; re-derived for engagement sampling 2026-08-14).
// DRIFT_CAP: the 3-min drift window can catch a door/cooking transient on the sampled pass —
// uncapped, 0.25 °C/min × kd 15 pegs the bias at the full ±2.0 authority clamp on a mild day
// (~20-35 min gear-climb + 1.8-2.7 °F undershoot, a small cousin of the 08-05 incident shape).
// NOTE the cap now guards ENGAGEMENT free-rise drift, which runs HIGHER than the old handover
// sample (trip mornings fired at 0.18-0.28 free vs 0.111 max handover) — so 0.12 clips routinely
// on fast rises, making floor = kd 15 × 0.12 = 1.8 the operative value there. Accepted
// deliberately (bug-check round 1): the k_d≈20 calibration pairs were handover-based, so kd 15
// against the larger free-rise drift is less conservative than the 0.75 derate suggests; the cap
// is what re-bounds it, an over-floored crossing self-corrects in minutes via the never-frozen
// e<0 unwind, and max() still under-serves the retained-bias case. Revisit kd AND the cap
// together against trip engagement-drift pairs (the old MIN_HOLD gate died with handover
// sampling — its "engage-time flux over-predicts ~2×" rationale is now the design, bounded here).
static const float CROSSING_PRELOAD_DRIFT_CAP_CPM = 0.12f;
// Raise-side bias freeze horizon (Stephen 2026-08-14). A comfort raise re-crosses in tens of
// minutes (08-14 live: 5°F raise = 40 min at ~0.07 °C/min free rise) — freeze fully across it.
// Past this cap the interlude is an away-raise: the load regime itself may have changed, and the
// τ=180 min blind-idle decay (which this freeze suspends) is the designed behavior — release the
// freeze and let it resume from the frozen value.
static const uint32_t RAISE_FREEZE_MAX_MS = 90UL * 60UL * 1000UL;  // 90 min
// Displacement arming gate (incident 2026-08-05): approach may fire only after the room has
// genuinely TRAVELED — risen ≥ APPROACH_ARM_RISE_C above its trailing-window trough (cool;
// heat mirrors as fall-below-peak). Both real approach regimes clear it fast (the 08-05 midday
// SP-raise recovery hit +1.2 °C ten minutes in, 39 min before its crossing; a morning sun ramp
// clears within ~20–60 min), while parked-room quantization noise is zero-mean (~±0.2 °C) and
// cannot accumulate it at ANY rate floor — rate distributions overlap (noise tail 0.077 vs
// design ~0.09 °C/min), displacement distributions do not. Replay 2026-08-05: blocks all 15
// evening churn engagements AND the 4 off→blast cycles. 1.2 (not 1.0) so the ~1.03 °C warm-back
// an OFF-entry blast manufactures for itself can never re-arm the loop that made it.
static const float APPROACH_ARM_RISE_C = 1.2f;
static const uint32_t APPROACH_ARM_SAMPLE_MS = 300000;   // 5-min ring cadence (12 slots ≈ 60 min)
static const uint32_t APPROACH_ARM_WINDOW_MS = 3900000;  // 65-min validity horizon per sample
// Natural-off gate (incident 2026-08-05, semantics settled by Stephen 2026-08-06): idle→OFF fires
// once the integral has wound down near zero AND the room sits past SP by the mode-switch offset
// (~1 °F) — a wound integral marks the sub-SP room as self-inflicted (integral lag): hold at idle
// while it unwinds rather than going OFF, whose only road back is a 305 s clamped cold start. No
// drift condition: a room PARKED below SP−1°F with an unwound bias goes properly OFF (an early
// "still falling" leg made never-OFF the parked-room default — rejected). A room caught mid
// warm-back goes OFF too and re-enters through the normal above-SP clamped start — the
// pre-approach status quo, accepted. Unwind at gear 0 is the τ=180 min idle DECAY (the e<0
// integration path only runs at active gears; a very large bias ≳0.9 instead bleeds off fast
// through sub-SP gear-1 pulses via the eff-based 0→1 re-engage), so the hold can still last a
// few hours after a heavy day regime. (Both unwind paths are suspended while a raise freeze is
// armed — which is why the bias leg below treats a frozen bias as unwound; see raise_freeze_c_at_.)
// OFF also fires on any user tap past SP and through the
// HEAT_COOL handoff door (opposite-mode demand bypasses the bias leg — see the gate blocks).
// One NVS quantum (GEAR_PREF_BIAS_QUANTUM_C): a warm-reset-restored "effectively zero" bias
// floors at 0.1, so a smaller eps would block OFF for ~2 h after every reboot.
static const float NATURAL_OFF_BIAS_EPS_C = 0.1f;
static const float ADAPT_DT_CAP_MIN = 5.0f;   // clamp dt across stalls/reboots
// Fan feedforward scale (°C eff_diff per fan-gear) is now the configurable member gear_step_c_
// (default 0.25 = behavior-parity). ⚠️ Latent inconsistency preserved: its comment historically
// claimed "= modulation spacing S", but S moved to 0.55 on 2026-07-08 while this stayed 0.25, so
// fan feedforward is ~2.2× under-scaled vs a real gear rung. Left at 0.25 for parity; retune via
// YAML (gear_step_c) if the vent-fan feedforward proves too weak in testing.
static const uint32_t FAN_EDGE_FREEZE_MS = 180000; // freeze integral 3 min after a vent-fan edge
// Quantum for persisting bias_c_ to flash: the integral drifts a tiny amount every
// pass, so saving it raw would queue an NVS write per pass. Quantized to 0.1°C a
// write happens only every few minutes even while the bias is actively ramping
// (ADAPT_KI=0.06 → ≥~28 min per step at typical sub-deadband-adjacent errors),
// and a restore lands within half a quantum of the true equilibrium.
static const float GEAR_PREF_BIAS_QUANTUM_C = 0.1f;
// Room-drift estimator: 3-minute trailing-window slope of inside temp (°C/min), consumed by the
// rate-gated upshift below. Replaces an EMA of per-sample instantaneous rates that (a) froze at a
// stale positive value when a plateau went quiet (a flat room stops emitting samples exactly when
// we need drift≈0) and (b) lagged the thermal turnaround — both let a wound-up bias grab gear 5
// into an overshoot (2026-07-02 incident). The windowed slope is less noisy (a large ΔT washes out
// the 0.06 °F sensor quantization) and reads ≤0 within ~1 min of a turnaround. The baseline is the
// buffered sample nearest DRIFT_WINDOW_MS old, accepted anywhere in [MIN, MAX]; the wide MAX makes
// the slope robust to reporting gaps instead of freezing.
static const uint32_t DRIFT_WINDOW_MS       = 180000;  // target baseline age (3 min)
static const uint32_t DRIFT_BASELINE_MIN_MS =  90000;  // reject baselines younger than this (dt too small/noisy)
static const uint32_t DRIFT_BASELINE_MAX_MS = 360000;  // ignore baselines older than this; also the gap tolerance (6 min)
// The windowed slope is refreshed ONLY when a sample arrives. A room that plateaus can then go
// quiet (a flat room emits no state change) exactly when we need drift≈0, leaving room_drift_cpm_
// frozen at its last climbing value. So the gate trusts a POSITIVE (warming) reading only while a
// sample no older than this backs it; a stale positive is treated as not-warming. Chosen well above
// the ~2-min normal inter-sample gap (a genuinely warming room updates fast) and below the 5.5-min
// gap that fooled the old EMA on 2026-07-02.
static const uint32_t DRIFT_STALE_MS        = 240000;  // 4 min
// Rate-gated upshift (derivative action). The learned integral bias may DRIVE an upshift only while
// the room is strictly WARMING (drift > this). When the current gear is already holding or cooling
// the room, a stronger gear isn't needed, so upshift decisions fall back to the unbiased real diff —
// a wound-up integral can't grab a higher gear straight into an overshoot. This is the fix for the
// hot-day "gear-5 windup": the integral would wind to ~0.6°C and leap to the 13 A top gear while
// gear 4 was already holding/cooling the room, overshoot ~1°F, then collapse back down through 3+
// gears (analysis 2026-06-30). Effect, by closed-loop sim sweep (tests/adaptive_test.cpp):
//   • load < gear-4 capacity → collapses the "gear-3 + gear-5 pulses" pattern into a clean 3↔4 pair
//     (fewer transitions, tighter swing, SAME steady offset);
//   • load > gear-4 capacity → no effect: the room keeps warming at gear 4 (drift > 0) so the gate
//     stays open and the genuinely-needed gear 5 / 4↔5 hunt is untouched.
// Threshold is exactly 0, NOT positive: a positive value locks gear 4 and rides a large steady
// offset at the gear-4-capacity knife-edge (it suppresses the corrective top-gear pulses too — the
// sweep showed +0.01 → +1.45°F offset). 0 stays permeable there. Negative values are inert
// (equilibrium warming is ≥ 0). Downshifts and the integral's own error term are NOT gated.
static const float ADAPT_UPSHIFT_DRIFT_MIN_CPM = 0.0f;

// Mode-switching protection lives in the 0→-1 gate and -1→active gate
// (see mode_switch_idle_ms_, mode_switch_event_ms_, mode_switch_temp_offset_c_, mode_switch_off_ms_)

// Diagnostic compressor-output percentage for a gear. Single source for the
// gear-pass publishes AND setup()'s boot-restore publish, so a restored gear
// can't show a mismatched percentage.
// Compressor output % for the diagnostic sensor: linear gear/max_gear. Reproduces the old 3-gear
// 33.3/66.6/100 (max_gear 3) exactly and scales to any N (e.g. 4 gears → 25/50/75/100).
static float gear_output_pct(int gear, int max_gear) {
  if (gear <= 0 || max_gear <= 0) return 0.0f;
  return 100.0f * (float)gear / (float)max_gear;
}

// ============================================================
// Configuration Setters
// ============================================================

void FurrionChillCube::set_outside_lockout_temp(float temp_f) {
  // Convert Fahrenheit to Celsius
  outside_lockout_temp_c_ = (temp_f - 32.0f) * (5.0f / 9.0f);
}

void FurrionChillCube::set_mode_switch_idle_min(int min) {
  mode_switch_idle_ms_ = (uint32_t)min * 60000;
}
void FurrionChillCube::set_mode_switch_event_min(int min) {
  mode_switch_event_ms_ = (uint32_t)min * 60000;
}
void FurrionChillCube::set_mode_switch_temp_offset(float offset_c) {
  // Value already converted to °C at config time (Python validator handles F/C suffix)
  mode_switch_temp_offset_c_ = offset_c;
}
// set_mode_switch_off_ms is inline in the header (time-typed via YAML).

// ============================================================
// Active IR Mode Persistence
// ============================================================

// A "warm" reset is one where only the ESP rebooted (OTA, esp_restart, crash/WDT):
// the Furrion kept power and is still running the last IR-commanded mode/gear/CS,
// so the saved gear IS the unit's true state. A cold boot (power-on, brownout)
// means the camper likely lost power too — the unit's internal board resumed at
// the user's real target (the failover anchor, see project_failover_invariant),
// so gear 0 — not the saved gear — matches the hardware. Default cold when unsure.
static bool is_warm_reset_() {
#ifdef USE_ESP32
  switch (esp_reset_reason()) {
    case ESP_RST_SW:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return true;
    default:
      return false;
  }
#else
  return false;
#endif
}

void FurrionChillCube::set_active_ir_mode_(climate::ClimateMode mode) {
  climate::ClimateMode prev = active_ir_mode_;
  if (prev == mode) return;
  active_ir_mode_ = mode;
  uint8_t m = (mode == climate::CLIMATE_MODE_HEAT) ? 1 :
              (mode == climate::CLIMATE_MODE_COOL) ? 2 : 0;
  mode_pref_.save(&m);

  // Timed vane positioning hook. The ONLY known vane anchor is the unit's power-on
  // re-home, so we trigger strictly on an OFF->active transition (Stephen's call).
  // Leaving an active mode (-> OFF, or a mode change that routes through OFF) aborts
  // any in-progress homing. Note: setup()'s boot-restore assigns active_ir_mode_
  // DIRECTLY (not via this setter), so a reboot never spuriously re-homes the vane.
  if (mode == climate::CLIMATE_MODE_OFF) {
    abort_vent_positioning_();
  } else if (prev == climate::CLIMATE_MODE_OFF) {
    maybe_start_vent_positioning_(mode == climate::CLIMATE_MODE_HEAT);
  }
}

// Persist the active mode's gear + the adaptive bias for the warm-reboot restore.
// Called at the end of every gear pass: every gear mutation inside the pass funnels
// through here, and the paths that zero the gear OUTSIDE a pass (failsafe, forced
// OFF, user OFF, kickstart abort) all save mode OFF via set_active_ir_mode_() —
// which makes the gear pref irrelevant on restore — so one call site suffices.
// No-op when neither field moved, so steady state costs zero flash writes; on the
// ESP32's wear-leveled NVS even the worst case (a gear change every HOLD_MS plus a
// bias quantum step every few minutes) is decades below the flash endurance budget.
// ESP32-only, matching is_warm_reset_(): on other platforms (ESP8266) preferences
// live in a single fixed flash sector with NO wear leveling — these writes would be
// real sector wear — and is_warm_reset_() always reports cold there, so the saved
// gear would never be read back anyway. Skip the writes; behavior = pre-persistence.
void FurrionChillCube::save_gear_pref_() {
#ifdef USE_ESP32
  int8_t g = -1;
  float bias_active = bias_c_;  // default cool; overridden for heat below
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
    g = (int8_t) heat_gear_;
    bias_active = bias_h_;
  } else if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
    g = (int8_t) cool_gear_;
  }
  float bias_q = isnan(bias_active)
                     ? 0.0f
                     : roundf(bias_active / GEAR_PREF_BIAS_QUANTUM_C) * GEAR_PREF_BIAS_QUANTUM_C;
  if (g == last_saved_gear_ && bias_q == last_saved_bias_c_) return;
  GearPrefData d{g, bias_q};
  gear_pref_.save(&d);
  last_saved_gear_ = g;
  last_saved_bias_c_ = bias_q;
#endif
}

// ============================================================
// IR Protocol Encoding
// ============================================================

void FurrionChillCube::encode_(remote_base::RemoteTransmitData *data,
                                const uint8_t *msg, uint8_t len, uint8_t repeat) {
  data->set_carrier_frequency(IR_CARRIER_FREQ);
  for (uint8_t copy = 0; copy <= repeat; copy++) {
    data->item(IR_HEADER_MARK, IR_HEADER_SPACE);
    for (uint8_t byte = 0; byte < len; byte++) {
      for (uint8_t bit = 0; bit < 8; bit++) {
        data->mark(IR_BIT_MARK);
        if (msg[byte] & (1 << (7 - bit))) {
          data->space(IR_ONE_SPACE);
        } else {
          data->space(IR_ZERO_SPACE);
        }
      }
    }
    data->item(IR_BIT_MARK, IR_GAP_SPACE);
  }
}

// ============================================================
// IR Transmission Methods
// ============================================================

bool FurrionChillCube::transmit_mode_command_() {
  // Don't broadcast a mode/setpoint command for an active mode without a valid
  // setpoint — incomplete info. The OFF command carries no setpoint and is
  // always allowed. A transient gap is harmless (the unit coasts); a gap longer
  // than the unit's ~7-min CS-mode timeout hands control back to its internal
  // controller. See project_failover_invariant.
  if (!test_mode_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF && isnan(get_active_ir_target_())) {
    return false;
  }

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so this mode frame is never jammed against a
  // preceding CS frame (CS→MODE-ON bracket / kickstart power-on). See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);

  uint8_t message[12] = {0};

  // Packet 1 header
  message[0] = 0xB2;
  message[1] = ~message[0];

  // Temperature code (dynamic setpoint → Gray code lookup).
  // F-mode encodes the user's precise °F target (1°F resolution, ranges
  // 60–86°F). C-mode encodes furrion_setpoint_c_ (the °C-rounded HA target,
  // ranges 16–30°C). Gear-CS math elsewhere always uses furrion_setpoint_c_
  // regardless of which path runs here — the only difference is what gets
  // shown on the unit's panel and which protocol the byte rides on.
  uint8_t temp_code;
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    temp_code = 0x0E;  // OFF special value (same for both protocols)
  } else if (use_fahrenheit_) {
    // target_c is guaranteed non-NaN — the gate at the top of this function
    // returns early for an active mode without a valid setpoint.
    float target_c = get_active_ir_target_();
    int target_f = (int)roundf(target_c * 1.8f + 32.0f);
    target_f = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, target_f));
    temp_code = TEMP_F_TABLE[target_f - FURRION_MIN_TEMP_F];
    last_tx_target_f_ = target_f;  // record the °F target actually put on the wire
  } else {
    int temp_c = std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, furrion_setpoint_c_));
    temp_code = TEMP_C_TABLE[temp_c - FURRION_MIN_TEMP_C];
  }
  // Record the °C anchor actually put on the wire (both protocols ride furrion_setpoint_c_ as
  // the anchor). This is the baseline update_furrion_setpoint_() diffs against for sp_changed.
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    last_tx_setpoint_c_ = furrion_setpoint_c_;
  }

  // Fan speed
  auto fan = get_effective_fan_mode_();
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    message[2] = 0x7B;  // FAN_OFF
    message[7] = 0x00;
  } else {
    switch (fan) {
      case climate::CLIMATE_FAN_LOW:
        message[2] = 0x9F;
        message[7] = 0x28;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        message[2] = 0x5F;
        message[7] = 0x3C;
        break;
      case climate::CLIMATE_FAN_HIGH:
        message[2] = 0x3F;
        message[7] = 0x64;
        break;
      case climate::CLIMATE_FAN_AUTO:
      default:
        message[2] = 0xBF;
        message[7] = 0x66;
        break;
    }
  }
  message[3] = ~message[2];

  // Temperature (upper nibble) + Mode (lower nibble)
  message[4] = (temp_code & 0x0F) << 4;
  switch (active_ir_mode_) {
    case climate::CLIMATE_MODE_HEAT:
      message[4] |= 0x0C;
      break;
    case climate::CLIMATE_MODE_COOL:
    case climate::CLIMATE_MODE_OFF:
    default:
      message[4] |= 0x00;
      break;
  }
  message[5] = ~message[4];

  // Packet 2 (only if not OFF)
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    message[6] = 0xD5;
    // message[7] already set (fan code2)
    if (temp_code & 0x20) message[8] |= 0x20;  // FRAC flag
    if (temp_code & 0x10) message[9] |= 0x10;  // NEG flag
    if (use_fahrenheit_) message[9] |= 0x01;   // FAH flag — F-mode display
    message[10] = 0x00;
    message[11] = 0;
    for (int i = 6; i <= 10; i++) message[11] += message[i];
  }

  // === Transmission 1: Mode command (B2 + D5) ===
  this->encode_(data, &message[0], 6, 1);
  bool send_packet2 = (active_ir_mode_ != climate::CLIMATE_MODE_OFF);
  if (send_packet2) {
    this->encode_(data, &message[6], 6, 0);
  }
  transmit.perform();

  // === Transmission 2: Swing (B9) — separate transmit call for clean state ===
  // Suppressed while the vane positioner is MOVING or a manual step is in flight: those
  // own the physical swing during their window, and a swing frame here (SWING_OFF, since
  // swing_mode stays OFF) would stop the vane early. Each sends its own stop frame.
  if (vent_phase_ != VentPhase::MOVING && !vane_step_active_) {
    auto swing_tx = this->transmitter_->transmit();
    auto *swing_data = swing_tx.get_data();
    swing_data->space(IR_PACKET_SPACE);
    if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL) {
      static const uint8_t SWING_ON[] = {0xB9, 0x46, 0xF5, 0x0A, 0x04, 0xFB};
      this->encode_(swing_data, SWING_ON, 6, 1);
    } else {
      static const uint8_t SWING_OFF[] = {0xB9, 0x46, 0xF5, 0x0A, 0x05, 0xFA};
      this->encode_(swing_data, SWING_OFF, 6, 1);
    }
    swing_data->space(IR_PACKET_SPACE);
    swing_tx.perform();
  }

  ESP_LOGD(TAG, "IR mode=%d fan=%d swing=%d",
           (int)active_ir_mode_, (int)fan, (int)this->swing_mode);

  // Track the fan actually put on the wire (v2): every mode frame funnels through here (including
  // transmit_mode_with_cs_), so this is the single point that keeps last_tx_fan_ current — which
  // maybe_apply_gear_fan_() diffs against to decide whether a per-gear fan change needs a new frame.
  last_tx_fan_ = (active_ir_mode_ == climate::CLIMATE_MODE_OFF) ? -1 : (int) fan;

  // Arm the one-shot reinforcement (fired in loop() step 3d): re-send this frame once,
  // mode_resend_delay_ms_ later (0 = disabled). Any newer mode frame re-arms, so the
  // reinforcement always carries the LATEST state (it re-reads mode/setpoint/fan at fire
  // time). mode_resending_ keeps the reinforcement itself from re-arming — at most two
  // frames per change, never a chain.
  if (mode_resend_delay_ms_ > 0 && !mode_resending_ && !test_mode_) {
    mode_resend_pending_ = true;
    mode_resend_armed_at_ = millis();
  }
  return true;
}

void FurrionChillCube::transmit_cs_update_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) return;
  // Don't broadcast a CS demand on incomplete info: the gear-CS signal needs
  // both a valid inside temperature and a valid setpoint. On a transient gap
  // the unit just coasts; on a gap longer than its ~7-min CS-mode timeout it
  // hands control back to its internal sensor. See project_failover_invariant.
  // In bench test mode the CS stream is operator-driven, independent of the room sensor —
  // bypass the failover sensor gates so a sensor dropout can't stall the test CS stream.
  if (!test_mode_) {
    if (isnan(inside_temp_c_)) return;
    if (isnan(get_active_ir_target_())) return;
  }

  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so this CS frame never jams the frame before/after
  // it in a CS→MODE-ON→CS bracket. See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);

  uint8_t message[6] = {0};
  message[0] = 0xBA;
  message[1] = ~message[0];  // 0x45

  // CS temperature — transmitted raw. Empirically the Furrion accepts
  // out-of-nominal-range values and treats them as stronger signals in
  // the same direction (tested on similar Midea units up to 55-95°F).
  // Gear 0 uses setpoint±5 which can land at CS=11 (setpoint 16°C cool)
  // or CS=35 (setpoint 30°C heat); both are transmitted unmodified.
  message[2] = (uint8_t)current_cs_;

  // Feature flags: always CS_ENABLED + CS_DATA. The value-bearing frame is the
  // only CS packet the unit acts on — a CS_DATA-clear frame is inert — and it
  // implicitly keeps the unit in Comfort Sense mode. No separate "enable" frame.
  message[2] |= 0xC0;  // CS_ENABLED + CS_DATA
  message[3] = ~message[2];

  // Mode footer
  switch (active_ir_mode_) {
    case climate::CLIMATE_MODE_HEAT:
      message[4] = 0x7E;
      break;
    case climate::CLIMATE_MODE_COOL:
      message[4] = 0x72;
      break;
    default:
      message[4] = 0x7A;  // AUTO footer
      break;
  }
  message[5] = ~message[4];

  this->encode_(data, message, 6, 1);
  transmit.perform();
}

// Bracket a mode/setpoint command with real CS_DATA frames so the unit is never
// left holding a stale CS against a new setpoint. On a running setpoint change
// the pre-CS makes the change atomic; on OFF→ON the pre-CS is transmitted but
// ignored by the unit (a CS before mode-on has no effect) — the post-CS is the
// one that matters there. transmit_cs_update_() no-ops only while active_ir_mode_
// is OFF.
void FurrionChillCube::transmit_mode_with_cs_() {
  // Deliberately NOT gated on boot_ready_: the OFF→ON mode command must still go
  // out on the first gear-controller pass, and boot_ready_ is not set until the
  // end of that pass. transmit_cs_update_() has its own isnan(inside_temp_c_)
  // gate against a boot-time CS leak, so a boot_ready_ gate here would be both
  // unnecessary and harmful — it would silently drop the first mode-on.
  if (failsafe_active_) return;  // no IR during failsafe — unit runs on its own sensor
  transmit_cs_update_();
  transmit_mode_command_();
  transmit_cs_update_();
  last_cs_heartbeat_ = millis();  // a CS just went out — restart the 30s heartbeat clock
}

void FurrionChillCube::transmit_raw_6byte_(const uint8_t *msg) {
  auto transmit = this->transmitter_->transmit();
  auto *data = transmit.get_data();
  // Inter-message guard idle so a raw command (swing/turbo/display) is never jammed
  // against an adjacent CS/mode frame. See IR_INTER_MSG_GAP.
  data->space(IR_INTER_MSG_GAP);
  this->encode_(data, msg, 6, 1);
  transmit.perform();
}

// ============================================================
// Button IR Commands
// ============================================================

void FurrionChillCube::send_display_toggle() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x09, 0xF6};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_turbo_on() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x01, 0xFE};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_turbo_off() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x02, 0xFD};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_swing_on() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x04, 0xFB};
  this->transmit_raw_6byte_(CMD);
}
void FurrionChillCube::send_swing_off() {
  static const uint8_t CMD[] = {0xB9, 0x46, 0xF5, 0x0A, 0x05, 0xFA};
  this->transmit_raw_6byte_(CMD);
}

void FurrionChillCube::send_probe_6byte(uint8_t code) {
  const uint8_t cmd[] = {0xB9, 0x46, 0xF5, 0x0A, code, (uint8_t) ~code};
  ESP_LOGI(TAG, "Probe TX: B9 46 F5 0A %02X %02X", cmd[4], cmd[5]);
  this->transmit_raw_6byte_(cmd);
}

void FurrionChillCube::resync_ir_state() {
  if (failsafe_active_) {
    ESP_LOGI(TAG, "Resync: skipped (failsafe active — unit runs on its own sensor)");
    return;
  }
  ESP_LOGI(TAG, "Resync: re-asserting CS/mode/CS bracket");
  this->transmit_mode_with_cs_();
}

void FurrionChillCube::send_vane_step() {
  // Step is a manual-positioning tool: meaningless while the unit is off (the vane
  // re-homes on power-on anyway, wiping any step) and while swing oscillation owns
  // the vane — a pulse there is a no-op SWING_ON followed by a SWING_OFF that stops
  // the user's oscillation while swing_mode (and the HA switch) still shows ON.
  // Same active-mode convention as send_swing_state_().
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    ESP_LOGI(TAG, "Vane: step ignored (unit off)");
    return;
  }
  if (this->swing_mode != climate::CLIMATE_SWING_OFF) {
    ESP_LOGI(TAG, "Vane: step ignored (swing active)");
    return;
  }
  // One uniform manual nudge: SWING_ON now, SWING_OFF after vane_step_duration_ms_
  // (completed non-blocking in loop()). Abort any in-progress auto-positioning so a
  // manual step takes over and the positioner's later SWING_OFF can't clip this pulse.
  abort_vent_positioning_();
  send_swing_on();
  vane_step_active_ = true;
  vane_step_start_ = millis();  // self-clock on millis() — pressed outside the loop-now context
  ESP_LOGI(TAG, "Vane: step ON (%lums)", (unsigned long) vane_step_duration_ms_);
}

// ============================================================
// Component Lifecycle
// ============================================================

void FurrionChillCube::setup() {
  // Build the modulation ladder trips from the configured spacing/hysteresis before any gear
  // pass can read them (defaults reproduce the shipped symmetric-1°F ladder), then derive each
  // mode's cold-start floor from its OFF (from_gear:-1) quirks.
  build_ladders_();
  compute_cold_start_floors_();

  // Restore mode, targets, fan, swing from flash
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
    ESP_LOGI(TAG, "Restored state: mode=%d temp=%.1f lo=%.1f hi=%.1f fan=%d swing=%d",
             (int)this->mode, this->target_temperature,
             this->target_temperature_low, this->target_temperature_high,
             (int)this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO),
             (int)this->swing_mode);
  }

  // Restore active IR mode from flash (survives reboot)
  mode_pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() ^ 0x4D4F4445);
  // Gear + adaptive bias, saved by save_gear_pref_() at the end of every gear pass.
  gear_pref_ = global_preferences->make_preference<GearPrefData>(this->get_object_id_hash() ^ 0x47454152);
  uint8_t saved_mode = 0;
  if (mode_pref_.load(&saved_mode) && (saved_mode == 1 || saved_mode == 2)) {
    bool is_heat = (saved_mode == 1);
    // The saved gear is the unit's true state only on a WARM reset (ESP-only reboot:
    // the Furrion kept power and is still at the last commanded gear CS) — resume
    // there directly instead of re-climbing from idle, and resume the adaptive bias
    // so the cool ladder doesn't immediately collapse the restored gear (bias 0 at
    // equilibrium diff≈0 reads as "no demand"). On a cold boot, with no saved gear,
    // or an inconsistent pair (gear -1 under an active saved mode — crash between
    // the two pref saves), fall back to gear 0: the pre-change behavior.
    int g = 0;
    GearPrefData saved_gear{};
    if (is_warm_reset_() && gear_pref_.load(&saved_gear)) {
      int max_gear = is_heat ? heat_max_gear_ : cool_max_gear_;  // the configured gear count for this
                         // mode; a saved gear above it (e.g. a stale gear from a different config)
                         // is out of range → falls through to gear 0 (cold-start path) rather than
                         // restoring an invalid gear whose CS would resolve to idle.
      if (saved_gear.gear >= 0 && saved_gear.gear <= max_gear) {
        g = saved_gear.gear;
        // Bias only when the adaptive controller is actually running: with adaptive
        // disabled the bias never integrates OR decays, so a restored value would sit
        // frozen across reboots and spring back stale (≤±2.0°C) whenever adaptive is
        // re-enabled — possibly a season later, against a different equilibrium.
        // The saved bias belongs to the ACTIVE mode at save time → restore it to that
        // mode's member (bias_h_ for heat, bias_c_ for cool).
        if (adaptive_enable_ && !isnan(saved_gear.bias)) {
          float b = std::max(-ADAPT_BIAS_C_MAX, std::min(ADAPT_BIAS_C_MAX, saved_gear.bias));
          if (is_heat) bias_h_ = b; else bias_c_ = b;
        }
      }
    }
    // Restore active_ir_mode_ to match the gear state — otherwise the first
    // controller run sees gear≥0 with active_ir_mode_=OFF and sends a spurious
    // MODE_ON IR command, waking the Furrion from idle.
    // Also sync furrion_setpoint_c_ and current_cs_ to the restored target +
    // gear CS, so update_furrion_setpoint_() and the CS check on first run
    // don't detect a bogus mismatch (default 22 vs. actual) and transmit a
    // spurious MODE_ON / CS frame the unit didn't ask for.
    if (is_heat) {
      heat_gear_ = g;
      active_ir_mode_ = climate::CLIMATE_MODE_HEAT;
      last_active_mode_ = MODE_HEAT;
    } else {
      cool_gear_ = g;
      active_ir_mode_ = climate::CLIMATE_MODE_COOL;
      last_active_mode_ = MODE_COOL;
    }
    furrion_setpoint_c_ = compute_setpoint_c_(is_heat);
    last_tx_setpoint_c_ = furrion_setpoint_c_;  // seed sp_changed baseline (avoid spurious 1st-pass tx)
    current_cs_ = compute_gear_cs_(is_heat, g);
    // Publish the restored CS so the diagnostic doesn't sit "unknown" until the
    // next CS change (no IR goes out — this is state-tracking only).
    if (cs_value_sensor_) cs_value_sensor_->publish_state(current_cs_);
    seed_last_tx_target_f_();   // F-protocol: avoid a bogus f_changed on first pass
    // Seed the fan baseline too (v2): active_ir_mode_ + gear + this->fan_mode are all restored now,
    // so get_effective_fan_mode_() is valid. Without this, last_tx_fan_ stays -1 and maybe_apply_
    // gear_fan_() would emit a spurious mode-on frame on the first post-warm-reboot pass.
    last_tx_fan_ = (int) get_effective_fan_mode_();
    boot_ready_ = true;          // restored state is valid — skip imm_off
    if (g == 0) {
      idle_since_ = millis();    // 10-min lockout before mode switch allowed
    }
    ESP_LOGI(TAG, "Restored prior mode: %s (gear=%d%s, skip kickstart, sp=%d°C cs=%d bias=%.2f)",
             is_heat ? "HEAT" : "COOL", g,
             g == 0 ? " → idle" : " (warm reset — resume gear)",
             furrion_setpoint_c_, current_cs_, bias_c_);
  }

  // Register temperature sensor callbacks
  if (inside_temp_sensor_) {
    inside_temp_sensor_->add_on_state_callback([this](float value) {
      uint32_t cb_now = millis();
      if (isnan(value)) {
        inside_temp_c_ = NAN;
        // Sensor unavailable → drop the drift history so we never compute a slope across the gap.
        drift_buf_head_ = 0;
        drift_buf_count_ = 0;
        room_drift_cpm_ = NAN;
        arm_ring_reset_();  // same rule for the displacement ring: never measure travel across a gap
      } else {
        inside_temp_c_ = inside_temp_fahrenheit_ ? (value - 32.0f) * (5.0f / 9.0f) : value;
        room_drift_cpm_ = update_room_drift_(cb_now);  // 3-min windowed slope (°C/min)
        arm_ring_record_(inside_temp_c_, cb_now);      // ~60-min displacement history (approach arming)
      }
      this->current_temperature = inside_temp_c_;
      this->last_temp_update_ = cb_now;
      this->temp_dirty_ = true;
      this->publish_state();
    });
  }

  // Vent-fan disturbance input: record each edge so the adaptive integral can freeze
  // across it (the burst is handled by feedforward, not by re-learning the equilibrium).
  if (vent_fan_sensor_) {
    // Leave vent_fan_changed_at_ at 0 until a REAL edge fires — the freeze window is gated on
    // (!= 0) so boot doesn't look like a fan edge.
    vent_fan_sensor_->add_on_state_callback([this](bool state) {
      this->vent_fan_changed_at_ = millis();
      // Re-run the controller promptly on a fan edge — but only when adaptive is enabled, so a
      // disabled build is strictly identical to main (no extra prompted passes).
      if (this->adaptive_enable_) this->temp_dirty_ = true;
    });
  }
  if (outside_temp_sensor_) {
    outside_temp_sensor_->add_on_state_callback([this](float value) {
      if (isnan(value)) {
        outside_temp_c_ = NAN;
      } else {
        outside_temp_c_ = outside_temp_fahrenheit_ ? (value - 32.0f) * (5.0f / 9.0f) : value;
      }
    });
  }

  // Initialize boot time and diagnostic sensors. Compressor output reflects the
  // restored gear (a warm reset can resume at gear N — 0% there would contradict
  // the gear sensor until the next gear change).
  boot_time_ = millis();
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(heat_gear_);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(cool_gear_);
  if (compressor_output_sensor_) {
    float pct = (heat_gear_ > 0) ? gear_output_pct(heat_gear_, heat_max_gear_)
              : (cool_gear_ > 0) ? gear_output_pct(cool_gear_, cool_max_gear_)
                                 : 0.0f;
    compressor_output_sensor_->publish_state(pct);
  }
}

void FurrionChillCube::loop() {
  uint32_t now = millis();

  // Bench test harness: while test_mode_ is set the production controller is fully inert —
  // no gear pass, kickstart, maneuver, vane, or heartbeat. The unit is driven ONLY by the
  // test_* hooks (from the YAML sequencer). set_test_mode(false) re-anchors on the next pass.
  if (test_mode_) return;

  // 1b. Commit a settled setpoint change (debounce). A temp change arms setpoint_pending_
  // in control() but does NOT transmit; once the user stops stepping (SETPOINT_SETTLE_MS of
  // no further change) we commit here by setting user_changed_, so the gear controller runs
  // once on the FINAL value and emits a single clean transmit.
  // Self-clock on millis() (NOT loop's cached `now`): setpoint_pending_since_ is armed in
  // control(), outside the loop-`now` context, so per reference_furrion_millis_now_footgun a
  // timer armed from a callback must compare against its own clock to avoid any underflow.
  if (setpoint_pending_ && (millis() - setpoint_pending_since_) >= SETPOINT_SETTLE_MS) {
    setpoint_pending_ = false;
    user_changed_ = true;
  }

  // 2. Run gear controller if triggered. Not blocked by an active override (clamped kickstart or
  // transition maneuver) — those suppress the gear pass's CS writes but the pass still runs so the
  // gear re-evaluates (a maneuver restores whatever gear the pass settled on when it ends).
  bool should_run = false;
  if (temp_dirty_) should_run = true;
  if (last_gear_run_ == 0 || (now - last_gear_run_) >= GEAR_INTERVAL_MS) should_run = true;
  if (user_changed_) should_run = true;

  if (should_run) {
    temp_dirty_ = false;  // only consume when gear controller actually runs
    run_gear_controller_();
    last_gear_run_ = now;
  }

  // Re-sample the clock after the gear pass. run_gear_controller_() reads its OWN
  // millis() and may spend hundreds of ms in blocking IR transmits; the timers it
  // arms (kickstart/keepalive) and last_cs_heartbeat_ (stamped at the END of a mode
  // bracket, after those transmits) are therefore NEWER than the `now` captured at
  // loop entry. Comparing steps 3-5 below against that stale `now` underflows:
  // a redundant CS fires after every bracket, and a kickstart armed this pass can
  // collapse instantly. Re-reading keeps the comparator >= any gear-pass stamp.
  // See reference_furrion_millis_now_footgun.
  now = millis();

  // 3. Advance the active IR override (after the gear pass so it can read the latest gear).
  // One unified maneuver engine now handles both the OFF→gear clamped start and running-unit quirks.
  if (maneuver_phase_ != ManeuverPhase::IDLE) {
    advance_maneuver_(now);
  }

  // 3b. Advance timed vane positioning (independent state machine; raw swing IR).
  // Aborted via set_active_ir_mode_(OFF) / user swing toggle, so it only runs while
  // the unit is on; the failsafe guard is belt-and-suspenders. It self-clocks on
  // millis() (does NOT take loop's cached `now`) — see advance_vent_positioning_.
  if (vent_positioning_active_() && !failsafe_active_) {
    advance_vent_positioning_();
  }

  // 3c. Complete a manual vane-step pulse: SWING_OFF after vane_step_duration_ms_.
  // Self-clocked on millis() (vane_step_start_ was set on the button press); runs to
  // completion even across a failsafe so the swing is never left stuck ON.
  if (vane_step_active_ && (millis() - vane_step_start_) >= vane_step_duration_ms_) {
    send_swing_off();
    vane_step_active_ = false;
    ESP_LOGI(TAG, "Vane: step OFF");
  }

  // 3d. Fire the one-shot mode-frame reinforcement armed by transmit_mode_command_(). Mode/fan
  // frames are fire-and-forget with no ack and no heartbeat (unlike CS): one missed frame on a
  // fan-only gear shift (g3↔g4, same CS) leaves fan AND compressor at the old operating point with
  // nothing to correct it (2026-07-27 15:36 overcool). Self-clocked on millis(), NOT loop's cached
  // `now` — the arming transmit can happen AFTER the step-2/3 re-sample (e.g. end_maneuver_'s
  // release resend), see reference_furrion_millis_now_footgun. Covers OFF frames too (a missed OFF
  // is the worst miss: the unit keeps running). Dropped, not deferred, under failsafe/boot — it is
  // reinforcement only, and a stale late resend is worse than none. DEFERRED (pending kept) while
  // an uncommitted setpoint is in flight — BOTH the debounce (setpoint_pending_) AND the converted
  // commit (user_changed_, which a NaN-room grace hold preserves across passes while CS frames
  // can't transmit): firing then would put the uncommitted target on the wire and stamp
  // last_tx_setpoint_c_/last_tx_target_f_, making the eventual commit see "unchanged" and skip its
  // deliberate CS→MODE→CS bracket (up to the full grace, ~5 min, of new-setpoint-vs-stale-CS).
  // Also deferred through a maneuver's 500ms PRE_CS lead so a large-delay config can't inject a
  // frame into the CS→mode-on choreography. NOT deferred during HOLD: a mid-HOLD fire re-reads
  // via_fan and usefully reinforces the maneuver's own frames. Accepted leak: during an OFF-start
  // kickstart whose quirk leaves via_fan unset, a fire can carry a user fan change that control()
  // deliberately withholds mid-wake — all shipped OFF-entry quirks pin via_fan, so unreachable in
  // practice. The commit's own transmit re-arms with a fresh stamp; if a deferred window ends with
  // no transmit, the resend fires then with committed state.
  uint32_t resend_elapsed = millis() - mode_resend_armed_at_;
  if (mode_resend_pending_ && !setpoint_pending_ && !user_changed_ &&
      maneuver_phase_ != ManeuverPhase::PRE_CS && resend_elapsed >= mode_resend_delay_ms_) {
    mode_resend_pending_ = false;
    if (boot_ready_ && !failsafe_active_) {
      mode_resending_ = true;
      bool sent = transmit_mode_command_();
      mode_resending_ = false;
      if (sent) {
        ESP_LOGI(TAG, "Mode frame reinforced (+%lums actual) mode=%d fan=%d",
                 (unsigned long) resend_elapsed, (int) active_ir_mode_, last_tx_fan_);
      }
    }
  }

  // 4. CS heartbeat every cs_transmit_interval_ms_ (default 10s; current_cs_ is the override CS
  // during a clamp/maneuver — correct). The shorter interval replaces the old keep-alive pulse:
  // re-asserting the gear's CS this often sustains the compressor at low gears and crosses the
  // anti-short-cycle lockout on a restart, so no separate pulse machinery is needed. During an
  // active override the clamp/maneuver re-assert already stamps last_cs_heartbeat_, so this stays
  // quiet then.
  if (boot_ready_ && !failsafe_active_ &&
      active_ir_mode_ != climate::CLIMATE_MODE_OFF &&
      (now - last_cs_heartbeat_) >= cs_transmit_interval_ms_) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
}

climate::ClimateTraits FurrionChillCube::traits() {
  auto traits = climate::ClimateTraits();
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE |
                           climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE |
                           climate::CLIMATE_SUPPORTS_ACTION);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  traits.set_visual_min_temperature(4.4f);    // 40°F
  traits.set_visual_max_temperature(30.0f);   // 86°F
  traits.set_visual_current_temperature_step(0.1f);
  traits.set_visual_target_temperature_step(1.0f);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
  return traits;
}

void FurrionChillCube::control(const climate::ClimateCall &call) {
  // Track whether each field *actually* changed. user_changed_ is set only for
  // real, mode-relevant changes — not for redundant HA re-syncs after reconnect
  // (which send the same values we already have) and not for tweaks to the
  // inactive endpoint (low in cool mode, high in heat mode).
  bool mode_changed = false;
  bool temp_changed = false;
  bool fan_changed = false;

  if (call.get_mode().has_value()) {
    auto new_mode = *call.get_mode();

    // Ensure two-point values valid for first boot (BEFORE sync to avoid NaN copy)
    if (isnan(this->target_temperature_low)) this->target_temperature_low = 20.0f;
    if (isnan(this->target_temperature_high)) this->target_temperature_high = 25.0f;

    // Sync target_temperature for HA single-slider display in HEAT/COOL modes
    if (new_mode == climate::CLIMATE_MODE_HEAT) {
      this->target_temperature = this->target_temperature_low;
    } else if (new_mode == climate::CLIMATE_MODE_COOL) {
      this->target_temperature = this->target_temperature_high;
    }

    mode_changed = (new_mode != this->mode);
    this->mode = new_mode;

    if (mode_changed) {
      // Abort an active IR override (clamped kickstart OR transition maneuver) only if the new
      // mode is incompatible with the active IR mode. Clear state directly (don't call end_*_
      // which would re-send the old mode).
      if (kickstart_active_()) {
        // The maneuver's mode is authoritative even during PRE_CS (active_ir_mode_ still OFF then).
        bool kick_is_heat = maneuver_is_heat_;
        bool kick_is_cool = !maneuver_is_heat_;
        bool compatible = false;
        if (kick_is_cool) {
          compatible = (new_mode == climate::CLIMATE_MODE_COOL || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
        } else if (kick_is_heat) {
          compatible = (new_mode == climate::CLIMATE_MODE_HEAT || new_mode == climate::CLIMATE_MODE_HEAT_COOL);
        }
        if (!compatible) {
          uint32_t now = millis();
          ESP_LOGI(TAG, "IR override aborted — mode %d incompatible with IR mode %d",
                   (int)new_mode, (int)active_ir_mode_);
          // Clear override state, force to OFF. All mode changes (including
          // COOL↔HEAT and to OFF) now go through -1 with 1-min off_since_ lockout.
          maneuver_phase_ = ManeuverPhase::IDLE;
          heat_gear_ = -1;
          cool_gear_ = -1;
          off_since_ = now;  // start 1-min off lockout
          if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
          if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
          if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
            set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
            transmit_mode_command_();
          }
        }
      }
    }
  }

  // Target temperature changes. Only flag as user-relevant when:
  //   (a) the value actually changed (idempotent re-sync from HA is a no-op), AND
  //   (b) the endpoint drives the active mode's gear math:
  //         target_temperature_low  → heat target (HEAT, HEAT_COOL)
  //         target_temperature_high → cool target (COOL, HEAT_COOL)
  //         target_temperature      → active target in HEAT or COOL single modes
  bool is_heat_mode = (this->mode == climate::CLIMATE_MODE_HEAT ||
                       this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool is_cool_mode = (this->mode == climate::CLIMATE_MODE_COOL ||
                       this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool is_single_active_mode = (this->mode == climate::CLIMATE_MODE_HEAT ||
                                this->mode == climate::CLIMATE_MODE_COOL);

  if (call.get_target_temperature().has_value()) {
    float v = *call.get_target_temperature();
    bool changed = isnan(this->target_temperature) || v != this->target_temperature;
    if (changed && is_single_active_mode) temp_changed = true;
    this->target_temperature = v;
  }
  if (call.get_target_temperature_low().has_value()) {
    float v = *call.get_target_temperature_low();
    bool changed = isnan(this->target_temperature_low) || v != this->target_temperature_low;
    if (changed && is_heat_mode) temp_changed = true;
    this->target_temperature_low = v;
  }
  if (call.get_target_temperature_high().has_value()) {
    float v = *call.get_target_temperature_high();
    bool changed = isnan(this->target_temperature_high) || v != this->target_temperature_high;
    if (changed && is_cool_mode) temp_changed = true;
    this->target_temperature_high = v;
  }
  // Sync target_temperature when low/high change in single modes.
  // Without this, get_heat/cool_target_() returns a stale value in HEAT/COOL modes.
  if (call.get_target_temperature_low().has_value() &&
      this->mode == climate::CLIMATE_MODE_HEAT) {
    this->target_temperature = this->target_temperature_low;
  }
  if (call.get_target_temperature_high().has_value() &&
      this->mode == climate::CLIMATE_MODE_COOL) {
    this->target_temperature = this->target_temperature_high;
  }

  // Fan mode change — idempotent: same-value sync is a no-op (no IR, no user flag)
  if (call.get_fan_mode().has_value()) {
    auto new_fan = *call.get_fan_mode();
    auto cur_fan = this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
    fan_changed = (new_fan != cur_fan);
    this->fan_mode = new_fan;
    // Deferred while a setpoint change is uncommitted: transmitting here would stamp
    // last_tx_setpoint_c_/last_tx_target_f_ with the uncommitted target (if a mid-debounce
    // gear pass already re-anchored it), making the flushed commit below see "unchanged" and
    // skip its CS→MODE→CS bracket. The flush's immediate gear pass delivers the fan instead
    // (commit bracket when sp/f changed, else the maybe_apply_gear_fan_ last_tx_fan_ diff).
    // Both flags: setpoint_pending_ is the normal debounce; user_changed_ is a settled-but-
    // unconsumed commit held across passes by a NaN-room grace hold — and this fan door is the
    // one transmit path live while user_changed_ is set, so it must gate on it too (mirrors the
    // step-3d reinforcement fire). The flush re-sets user_changed_, so the deferred fan is
    // delivered by the next gear pass to run — on grace-clear that pass consumes user_changed_
    // and maybe_apply_gear_fan_ transmits it via the last_tx_fan_ diff (or the commit bracket if
    // the setpoint also changed). Delayed at most until grace clears, never lost.
    if (fan_changed && active_ir_mode_ != climate::CLIMATE_MODE_OFF && !kickstart_active_() &&
        !setpoint_pending_ && !user_changed_) {
      bool sent = transmit_mode_command_();
      if (sent) ESP_LOGI(TAG, "User fan change → %d, mode command sent", (int)new_fan);
    }
  }

  // Swing mode change — standalone swing frame, works during kickstart
  // Does NOT set user_changed_: vent direction is cosmetic and must never
  // trigger gear recalculation, timer resets, or immediate-off logic.
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = *call.get_swing_mode();
    // User took manual vane control — exit any in-progress timed homing cleanly and
    // immediately. abort is a pure state reset (no IR); send_swing_state_() below puts
    // the vane in the user's requested state, so there is no competing frame.
    abort_vent_positioning_();
    // Also cancel an in-flight manual step pulse: the user's swing frame supersedes
    // it, and the pulse's deferred SWING_OFF (loop 3c) would otherwise stop a
    // just-enabled oscillation ~vane_step_duration later while HA shows it ON.
    vane_step_active_ = false;
    send_swing_state_();
    ESP_LOGI(TAG, "User swing change → %d", (int)*call.get_swing_mode());
  }

  // Flag gear recalculation for real, gear-relevant changes. Swing is cosmetic;
  // same-value re-syncs from HA are filtered above.
  //
  // Mode/fan changes are discrete, single-shot actions → act immediately, and flush any
  // pending setpoint (the immediate gear run commits the latest target too). A pure temp
  // change is DEBOUNCED instead: arm setpoint_pending_ and let loop() commit it after the
  // user stops stepping (SETPOINT_SETTLE_MS). target_temperature is already updated above,
  // so HA's card shows the new value instantly — only the IR transmit is deferred.
  if (mode_changed || fan_changed) {
    this->user_changed_ = true;
    this->setpoint_pending_ = false;  // flush: the immediate run handles the current target
  } else if (temp_changed) {
    this->setpoint_pending_ = true;
    this->setpoint_pending_since_ = millis();
  }
  this->publish_state();
}

// ============================================================
// Helpers
// ============================================================

float FurrionChillCube::get_heat_target_() {
  return this->target_temperature_low;
}

float FurrionChillCube::get_cool_target_() {
  return this->target_temperature_high;
}

// Target for whichever mode the unit's IR state is in. Keyed on active_ir_mode_
// (distinct from get_heat/cool_target_, which are keyed on the user's climate
// mode). Returns NAN when OFF — an OFF command carries no setpoint.
float FurrionChillCube::get_active_ir_target_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) return get_heat_target_();
  if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) return get_cool_target_();
  return NAN;
}

// Seed last_tx_target_f_ to the °F byte a real transmit would put on the wire for
// the currently active target — same math as transmit_mode_command_()'s F path.
// Called from setup()'s restore branches: without it last_tx_target_f_ stays 0, and
// update_furrion_setpoint_()'s f_changed check (target_f != 0, always true in the
// default use_fahrenheit_ build) fires a spurious MODE-ON+CS bracket on the first
// gear pass after every reboot — the exact un-commanded wake the restore setpoint/CS
// sync exists to prevent. NaN target (no restored climate state) → leave 0; the
// isnan gates downstream already suppress the transmit in that case.
void FurrionChillCube::seed_last_tx_target_f_() {
  float t = get_active_ir_target_();
  if (isnan(t)) return;
  int tf = (int) roundf(t * 1.8f + 32.0f);
  last_tx_target_f_ = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, tf));
}

// Returns true iff the given heat gear's hysteresis band covers `diff`.
// Used on user_input events to avoid collapsing a stable hunting gear when
// the user change doesn't actually move the room out of the current gear's band.
bool FurrionChillCube::gear_in_band_heat_(int gear, float diff) {
  // Gear N stays between its own transition trips. Heat: diff negative when cold, so the upshift
  // trip (heat_up_[N] = −N·S) is the LOWER bound and the downshift trip is the UPPER bound. Pins:
  // gear 0 sits in [start, idle]; gear 1's downshift is the pinned stop. Generalized over N gears;
  // reproduces the old case 0/1/2/3 for heat_max_gear_ == 3.
  int M = heat_max_gear_;
  if (gear < 0 || gear > M) return false;
  if (gear == 0) return diff >= heat_start_ && diff <= heat_idle_;
  float up = heat_up_[gear];                                   // upshift gear→gear+1 (lower bound)
  float dn = (gear == 1) ? heat_stop_ : heat_dn_[gear - 1];    // downshift gear→gear-1 (upper bound)
  if (gear == M) return diff <= dn;                            // top heat gear: no lower bound
  return diff >= up && diff <= dn;
}

bool FurrionChillCube::gear_in_band_cool_(int gear, float diff) {
  // Gear N stays between its own transition trips. Cool: diff positive when hot, so the downshift
  // trip is the LOWER bound and the upshift trip (cool_up_[N] = N·S) the UPPER bound. Pins: gear 0
  // sits in [idle, start]; gear 1's downshift is the pinned stop. Generalized over N gears;
  // reproduces the old case 0/1/2/3 for cool_max_gear_ == 3.
  int M = cool_max_gear_;
  if (gear < 0 || gear > M) return false;
  if (gear == 0) return diff >= cool_idle_ && diff <= cool_start_;
  float dn = (gear == 1) ? cool_stop_ : cool_dn_[gear - 1];    // downshift gear→gear-1 (lower bound)
  if (gear == M) return diff >= dn;                            // top cool gear: no upper bound
  return diff >= dn && diff <= cool_up_[gear];                 // upshift gear→gear+1 (upper bound)
}

bool FurrionChillCube::vent_fan_on_() {
  return vent_fan_sensor_ != nullptr && vent_fan_sensor_->state;
}

// Push the current inside temp into the drift ring and return the 3-min trailing-window slope
// (°C/min), or NAN when no baseline sample yet sits in the [MIN, MAX] age band (warmup / just after
// a sensor-unavailable reset). The gate treats NAN as warming (legacy-permissive). Baseline = the
// buffered sample whose age is nearest DRIFT_WINDOW_MS, chosen from the [MIN, MAX] band; the wide
// MAX (6 min) means a quiet plateau or a reporting gap still yields a correct ~0 slope instead of a
// frozen stale value. See DRIFT_WINDOW_MS above.
float FurrionChillCube::update_room_drift_(uint32_t now) {
  // Record this sample into the ring.
  drift_buf_at_[drift_buf_head_] = now;
  drift_buf_temp_[drift_buf_head_] = inside_temp_c_;
  drift_buf_head_ = (uint8_t)((drift_buf_head_ + 1) % DRIFT_BUF_N);
  if (drift_buf_count_ < DRIFT_BUF_N) drift_buf_count_++;

  // Pick the baseline: valid entries occupy indices [0, count_) whether or not the ring has wrapped
  // (pre-wrap they were written there in order; once full, count_ == N covers every slot).
  float base_temp = NAN;
  uint32_t base_at = 0;
  uint32_t best_err = 0xFFFFFFFFu;
  for (uint8_t i = 0; i < drift_buf_count_; i++) {
    uint32_t age = now - drift_buf_at_[i];           // monotonic within a boot; just-pushed = 0 → skipped
    if (age < DRIFT_BASELINE_MIN_MS || age > DRIFT_BASELINE_MAX_MS) continue;
    uint32_t err = (age > DRIFT_WINDOW_MS) ? (age - DRIFT_WINDOW_MS) : (DRIFT_WINDOW_MS - age);
    if (err < best_err) { best_err = err; base_temp = drift_buf_temp_[i]; base_at = drift_buf_at_[i]; }
  }
  if (isnan(base_temp)) return NAN;                  // no baseline in band yet
  float dt_min = (now - base_at) / 60000.0f;
  return (inside_temp_c_ - base_temp) / dt_min;
}

// Phase 2 adaptive (cool) — standard name: INPUT-DISTURBANCE ESTIMATOR via integral action
// (a crude disturbance observer with conditional-integration anti-windup); its output added to
// the raw error forms the DEMAND SIGNAL the stage sequencer quantizes.
// Advance the integral bias (bias_c_) with anti-windup, then return
// the effective diff the cool ladder's ACTIVE-gear thresholds select on
// (real_diff + bias_c_ + vent-fan feedforward). Returns real_diff unchanged when adaptive is
// disabled. The caller uses this ONLY for switch cases 1-5 — re-engage/idle decisions stay on
// real diff. Must be called once per cool pass (even at gear 0) so the integral advances/decays.
// See PHASE2_ADAPTIVE_DESIGN.md §2-3.
float FurrionChillCube::adaptive_cool_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear) {
  if (!adaptive_enable_) {
    adaptive_last_advance_ = now;  // keep dt fresh so a later enable doesn't see a huge gap
    cool_eff_up_diff_ = real_diff; // no bias → upshift path is the static ladder (bit-identical)
    return real_diff;
  }

  // dt since last advance, clamped (first pass, stalls, mode gaps, millis wrap-after-reboot)
  float dt_min = 0.0f;
  if (adaptive_last_advance_ != 0) {
    dt_min = (now - adaptive_last_advance_) / 60000.0f;
    if (dt_min < 0.0f) dt_min = 0.0f;
    if (dt_min > ADAPT_DT_CAP_MIN) dt_min = ADAPT_DT_CAP_MIN;
  }
  adaptive_last_advance_ = now;

  // Error with deadband (+ = room too warm = need more cooling). REAL diff drives the integral.
  float e = real_diff;
  if (e > -ADAPT_DEADBAND_C && e < ADAPT_DEADBAND_C) e = 0.0f;

  // Freeze the 3-min window after a vent-fan state publish. The (sensor != null && != 0) guard
  // fully suppresses the case with NO fan configured (callback never fires → stays 0). With a
  // fan configured, the binary_sensor's initial-state publish does set this at boot, so the
  // window can be active for the first ~3 min of uptime — harmless, since bias_c_ starts at 0
  // and there is nothing to freeze.
  bool fan_edge_freeze = (vent_fan_sensor_ != nullptr) && (vent_fan_changed_at_ != 0) &&
                         ((now - vent_fan_changed_at_) < FAN_EDGE_FREEZE_MS);
  bool idle = (cool_gear_ <= 0);  // compressor off/idle — error not controllable

  // Conditional integration anti-windup: freeze POSITIVE accumulation only when the gear cannot
  // rise right now — at gear 5, or while an upshift is hold-blocked (can_upshift_to). NEGATIVE
  // accumulation is never rail-blocked because gear 0/idle is always reachable (downshifts are
  // not hold-gated). This also lets a stale positive bias unwind at gear 1 (e<0 is not frozen),
  // fixing the prior sat_low trap, and stops windup behind a held upshift from cascading gears.
  bool upshift_held = (cool_gear_ < cool_max_gear_) && (time_in_gear < HOLD_MS[cool_gear_ + 1]);
  // Rate gate: the room is "warming" (gear-raising allowed) only when dT/dt clears the threshold AND
  // a recent sample backs the reading. room_drift_cpm_ is recomputed only on a sample, so a plateau
  // that goes quiet freezes it at its last climbing value; trusting a POSITIVE reading only while
  // fresh stops a wound-up bias from grabbing a top gear off frozen climb data (2026-07-02). Unknown
  // drift (NaN — warmup / a buffer rebuilding after a gap) stays legacy-permissive.
  bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
  bool warming = isnan(room_drift_cpm_) ||
                 (room_drift_cpm_ > ADAPT_UPSHIFT_DRIFT_MIN_CPM && drift_fresh);
  // Freeze positive accumulation only when the gear physically can't rise — at max gear or while an
  // upshift is hold-blocked. The `!warming` clause was REMOVED (iter-1 #4, 2026-07-20): it froze
  // accumulation whenever the room wasn't actively rising, which during a steady above-SP hunt gated
  // out ~7 of every 8 minutes of integration → the integral ran at ~1/8 speed and lagged the diurnal
  // load by HOURS (room sat +0.5-0.75°F above SP all afternoon, then overcooled all evening). The
  // upshift itself is still protected against grabbing a top gear off stale climb data by the
  // `warming ? eff : fminf(...)` gate on cool_eff_up_diff_ below — so a wound bias can accumulate but
  // still can't force an upshift unless the room is genuinely warming. Accepted trade: a mild
  // wind-then-step as the room ticks warm, bounded by HOLD_MS (one gear/pass). Tuning this empirically.
  // NOTE (fix-setpoint-transition-integral, 2026-08-02): a "pulldown windup window" that exempted
  // this max-gear freeze during user SP drops was implemented and CUT in the same session's bug
  // check: at pulldown-scale errors (e ≈ 3 °C) KI integrates at ~0.2 °C/min — a clamp-pegger, not
  // a trim — pinning max gear until the room passes BELOW the new SP, and its arrival-close
  // (real_diff < deadband) is unreachable on exactly the hot nights that motivated it (Fri
  // 2026-07-31 bottomed at +0.17 °C). The SP-drop bias PRELOAD (see run_cool_mode_) is the whole
  // fix: it supplies the bias this freeze was starving, and the freeze then correctly prevents
  // the double-count. Do not re-add a windup exemption without a rate cap + non-arrival close.
  bool block_up = (e > 0.0f) && (cool_gear_ >= cool_max_gear_ || upshift_held);
  // Raise-side bias freeze maintenance (Stephen 2026-08-14, see the member note): released at
  // the CROSSING (room back inside the deadband from below) — not at re-engagement — because the
  // post-clamp gear-1 stretch below SP is engineered sub-SP running, same rationale as the
  // approach-hold freeze (08-14 live: it ate 0.19 C of bias between handover and crossing). The
  // horizon release hands a long park back to the normal blind-idle decay. Both checks run
  // BEFORE the freeze is applied, so a released pass integrates/decays normally.
  if (raise_freeze_c_at_ != 0) {
    if (real_diff >= -ADAPT_DEADBAND_C) {
      ESP_LOGI(TAG, "Raise freeze (cool): crossing reached — bias %.2f preserved", bias_c_);
      raise_freeze_c_at_ = 0;
    } else if ((now - raise_freeze_c_at_) >= RAISE_FREEZE_MAX_MS) {
      ESP_LOGI(TAG, "Raise freeze (cool): horizon expired at bias %.2f — idle decay resumes", bias_c_);
      raise_freeze_c_at_ = 0;
    }
  }
  bool raise_frozen = (raise_freeze_c_at_ != 0);
  // approach_hold_cool_: gear 1 deliberately runs BELOW SP during an approach hold — e<0 there is
  // engineered, not overcooling, and integrating it would strip the retained bias the coming day
  // regime needs (~1.2 C in 20 min at these errors). Freeze both directions for the hold.
  bool freeze = kickstart_active_() || fan_edge_freeze || block_up || approach_hold_cool_ ||
                raise_frozen;

  if (idle) {
    // Idle is blind (no feedback) — slowly fade the stored bias on the load-change timescale (hours).
    // A natural descent already unwound it via e<0; this only meaningfully bites a long blind idle,
    // and near-freezes a short one so a mid-load SP-nudge/off keeps its still-valid bias. See ADAPT_DECAY_TAU_MIN.
    // A raise-freeze suspends even this fade — the raise interlude is measured in minutes and the
    // load is unchanged; the horizon release above restores the fade for genuine long parks.
    if (dt_min > 0.0f && !raise_frozen) bias_c_ *= expf(-dt_min / ADAPT_DECAY_TAU_MIN);
  } else if (!freeze && dt_min > 0.0f) {
    bias_c_ += ADAPT_KI * e * dt_min;
    if (bias_c_ > ADAPT_BIAS_C_MAX) bias_c_ = ADAPT_BIAS_C_MAX;
    if (bias_c_ < -ADAPT_BIAS_C_MAX) bias_c_ = -ADAPT_BIAS_C_MAX;
  }

  float ff_c = vent_fan_on_() ? (fan_feedforward_gears_ * gear_step_c_) : 0.0f;
  // A raise-frozen bias is STORED, not live (bug-check round 1, all three reviewers): if the
  // ladder saw it, the eff-based 0→1 re-engage would fire ~(bias − C_UP_01) BELOW the raised SP
  // and — with the e<0 unwind suspended by this very freeze — hunt 1↔0 there for the whole
  // horizon, actively defeating the raise. While frozen the ladder runs bias-BLIND (eff = real +
  // ff only); the crossing release above re-lives the bias on the same pass the room reaches the
  // band, so the ladder regains it exactly when demand returns.
  float live_bias = raise_frozen ? 0.0f : bias_c_;
  float eff = real_diff + live_bias + ff_c;
  // Upshift decisions see the learned bias only while warming. When not warming, fall back to the
  // unbiased diff (+ the fast fan feedforward, which is anticipatory and must still act). fminf
  // guarantees the gate can only ever SUPPRESS an upshift, never enable one: a stale NEGATIVE bias
  // (eff < real_diff) keeps eff, so the gate never makes an upshift easier than the static ladder.
  cool_eff_up_diff_ = warming ? eff : fminf(eff, real_diff + ff_c);
  return eff;
}

// Phase 2 adaptive (heat): the SIGN-MIRROR of adaptive_cool_eff_diff_. Heat "demand" is -real_diff
// (room below target = need heat), so the integral bias_h_ winds POSITIVE when persistently cold and
// eff = real_diff - bias_h_ (more negative → higher heat gear). Anti-windup freezes POSITIVE demand
// accumulation when the gear can't rise (gear 3 = max heat, an upshift is hold-blocked, OR the room
// isn't cooling per the rate gate). The rate gate's "raise allowed" condition is the room getting
// COLDER (drift < -threshold) — the mirror of cool's "warming". No fan feedforward on the heat side
// (the vent-fan FF is a cooling-load model; heat FF is a separate unmeasured question — omitted).
// ⚠️ WINTER-VALIDATE: this loop shifts the heat downshift thresholds via bias_h_. The pong-critical
// 1→0 STOP is deliberately evaluated on REAL diff in run_heat_mode_ so bias can't move it, but the
// centering + overshoot behavior is UNTESTED until a real heat cycle — verify before trusting.
float FurrionChillCube::adaptive_heat_eff_diff_(float real_diff, uint32_t now, uint32_t time_in_gear) {
  if (!adaptive_enable_) {
    heat_adaptive_last_advance_ = now;  // keep dt fresh so a later enable doesn't see a huge gap
    heat_eff_up_diff_ = real_diff;      // no bias → upshift path is the static ladder (bit-identical)
    return real_diff;
  }

  // dt since last advance, clamped (first pass, stalls, mode gaps, millis wrap-after-reboot)
  float dt_min = 0.0f;
  if (heat_adaptive_last_advance_ != 0) {
    dt_min = (now - heat_adaptive_last_advance_) / 60000.0f;
    if (dt_min < 0.0f) dt_min = 0.0f;
    if (dt_min > ADAPT_DT_CAP_MIN) dt_min = ADAPT_DT_CAP_MIN;
  }
  heat_adaptive_last_advance_ = now;

  // Heat-demand error (+ = room too COLD = need more heat). Mirror of cool's e = real_diff.
  float e = -real_diff;
  if (e > -ADAPT_DEADBAND_C && e < ADAPT_DEADBAND_C) e = 0.0f;

  bool idle = (heat_gear_ <= 0);  // heat off/idle — error not controllable
  // Conditional-integration anti-windup: freeze POSITIVE (more-heat) accumulation when the gear
  // cannot rise — at gear 3 (max heat), while an upshift is hold-blocked, or while the rate gate
  // suppresses the upshift. NEGATIVE (less-heat) accumulation is never rail-blocked (gear 0/idle is
  // always reachable), so a stale positive bias can always unwind. Mirror of the cool anti-windup.
  bool upshift_held = (heat_gear_ < heat_max_gear_) && (time_in_gear < HOLD_MS[heat_gear_ + 1]);
  bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
  // "cooling" = room getting colder = raising the heat gear is warranted (mirror of cool's "warming").
  // A POSITIVE-magnitude drift reading is trusted only while fresh; NaN stays legacy-permissive.
  bool cooling = isnan(room_drift_cpm_) ||
                 (room_drift_cpm_ < -ADAPT_UPSHIFT_DRIFT_MIN_CPM && drift_fresh);
  // NOTE: the cut "pull-up windup window" (see the cool-side note in adaptive_cool_eff_diff_)
  // applied here too — same reasoning, sign-mirrored. The SP-raise preload replaces it.
  bool block_up = (e > 0.0f) && (heat_gear_ >= heat_max_gear_ || upshift_held || !cooling);
  // Raise-side bias freeze maintenance — SIGN-MIRROR of the cool block (armed by a heat SP DROP
  // that removes demand; released when the room falls back into the band from above, or at the
  // horizon). ⚠️ winter-unvalidated. e for heat is -real_diff, so "inside the band from the
  // satisfied side" is e >= -ADAPT_DEADBAND_C, i.e. real_diff <= ADAPT_DEADBAND_C.
  if (raise_freeze_h_at_ != 0) {
    if (real_diff <= ADAPT_DEADBAND_C) {
      ESP_LOGI(TAG, "Raise freeze (heat): crossing reached — bias %.2f preserved", bias_h_);
      raise_freeze_h_at_ = 0;
    } else if ((now - raise_freeze_h_at_) >= RAISE_FREEZE_MAX_MS) {
      ESP_LOGI(TAG, "Raise freeze (heat): horizon expired at bias %.2f — idle decay resumes", bias_h_);
      raise_freeze_h_at_ = 0;
    }
  }
  bool raise_frozen = (raise_freeze_h_at_ != 0);
  // approach_hold_heat_: mirror of the cool approach freeze (gear 1 deliberately above SP).
  bool freeze = kickstart_active_() || block_up || approach_hold_heat_ ||
                raise_frozen;  // no fan-edge freeze (no heat fan feedforward)

  if (idle) {
    // Idle is blind — slow memory-fade on the load-change timescale (hours); mirror of cool. See
    // ADAPT_DECAY_TAU_MIN. ⚠️ WINTER-UNVALIDATED: shares the cool τ; not independently tuned for heat.
    // A raise-freeze suspends the fade (mirror of cool; released by crossing or horizon above).
    if (dt_min > 0.0f && !raise_frozen) bias_h_ *= expf(-dt_min / ADAPT_DECAY_TAU_MIN);
  } else if (!freeze && dt_min > 0.0f) {
    bias_h_ += ADAPT_KI * e * dt_min;
    if (bias_h_ > ADAPT_BIAS_C_MAX) bias_h_ = ADAPT_BIAS_C_MAX;
    if (bias_h_ < -ADAPT_BIAS_C_MAX) bias_h_ = -ADAPT_BIAS_C_MAX;
  }

  // Raise-frozen bias is STORED, not live — bias-blind eff while frozen, mirror of cool (heat's
  // 0→1 re-engage is real-diff so the cool pin bug doesn't exist here, but the downshift/hold
  // legs DO select on eff and must not act on a stored bias). ⚠️ winter-unvalidated.
  float live_bias = raise_frozen ? 0.0f : bias_h_;
  float eff = real_diff - live_bias;  // bias_h_ > 0 (cold demand) → more negative → higher heat gear
  // Upshift decisions see the learned bias only while cooling. When not cooling, fall back to the
  // unbiased diff. fmaxf (mirror of cool's fminf) guarantees the gate can only SUPPRESS an upshift,
  // never enable one: a stale POSITIVE bias (eff < real_diff) is clamped up to real_diff.
  heat_eff_up_diff_ = cooling ? eff : fmaxf(eff, real_diff);
  return eff;
}

int FurrionChillCube::compute_setpoint_c_(bool is_heat) {
  float target_c = is_heat ? get_heat_target_() : get_cool_target_();
  if (isnan(target_c)) return 22;  // safe default
  int rounded = (int)roundf(target_c);
  return std::max(FURRION_MIN_TEMP_C, std::min(FURRION_MAX_TEMP_C, rounded));
}

void FurrionChillCube::update_furrion_setpoint_(bool is_heat) {
  // NaN target → hold the last good setpoint. A NaN target can briefly arrive
  // from an unavailable HA sensor; never let compute_setpoint_c_()'s safe
  // default silently clobber furrion_setpoint_c_.
  float target_c = is_heat ? get_heat_target_() : get_cool_target_();
  if (isnan(target_c)) return;

  int new_sp = compute_setpoint_c_(is_heat);
  int gear = is_heat ? heat_gear_ : cool_gear_;

  // Update the °C anchor IMMEDIATELY — even mid-debounce. furrion_setpoint_c_ is the anchor
  // compute_gear_cs_() reads; deferring it would let a gear pass during the settle window emit
  // a CS frame whose gear was chosen for the new target but whose CS value is still anchored to
  // the OLD setpoint (a transient new-gear/old-anchor inconsistency). Updating it here is cheap
  // and IR-free — only the setpoint-display TRANSMIT is debounced (below). Because the anchor
  // now tracks the live target, change detection diffs against last_tx_setpoint_c_ (the last °C
  // actually put on the wire), NOT furrion_setpoint_c_.
  if (new_sp != furrion_setpoint_c_) {
    ESP_LOGI(TAG, "Furrion setpoint %d°C → %d°C (%s)",
             furrion_setpoint_c_, new_sp, is_heat ? "heat" : "cool");
    furrion_setpoint_c_ = new_sp;
    // A setpoint change ABORTS an in-flight RUNNING maneuver (from_gear >= 0): the target moved, so a
    // stale dip/hold is no longer meaningful — clear it and let the re-anchor below (now unguarded)
    // re-evaluate the gear's CS against the new setpoint. The OFF→gear clamped start (from_gear == -1)
    // is NOT aborted — its compressor-wake stimulus must survive a setpoint change, and end_maneuver_
    // re-derives the gear CS at release (see the transmit note below).
    if (maneuver_phase_ != ManeuverPhase::IDLE && maneuver_from_gear_ >= 0) {
      maneuver_phase_ = ManeuverPhase::IDLE;
      maneuver_start_ = 0;
      ESP_LOGI(TAG, "Maneuver aborted — setpoint changed");
    }
    // Re-anchor current_cs_ to the new setpoint for the current gear so CS frames stay
    // consistent with it. Skip during a kickstart: current_cs_ is then the kickstart CS and the
    // kickstart state machine must keep owning it (end_kickstart_ re-derives the gear CS).
    if (gear >= 0 && !kickstart_active_()) {
      current_cs_ = compute_gear_cs_(is_heat, gear);
      if (cs_value_sensor_) cs_value_sensor_->publish_state(current_cs_);
    }
  }

  // Debounce: defer ONLY the IR transmit while a setpoint change is still settling. The anchor
  // above already tracks the live target; loop() clears setpoint_pending_ + sets user_changed_
  // when the settle window elapses, and this runs again to fire the single coalesced transmit.
  if (setpoint_pending_) return;

  // sp_changed = whole-°C change vs the last transmitted setpoint. f_changed = sub-°C (1°F)
  // change (e.g. 73→74°F both round to 23°C) so the F-protocol panel display stays in sync.
  bool sp_changed = (furrion_setpoint_c_ != last_tx_setpoint_c_);
  bool f_changed = false;
  if (use_fahrenheit_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    int target_f = (int) roundf(target_c * 1.8f + 32.0f);
    target_f = std::max(FURRION_MIN_TEMP_F, std::min(FURRION_MAX_TEMP_F, target_f));
    f_changed = (target_f != last_tx_target_f_);
  }
  // Retransmit so the unit gets the new setpoint / panel target — but only when
  // there is an active gear to anchor the CS to; otherwise the gear controller's
  // own HVAC-on path handles the transmit a few lines later. transmit_mode_with_
  // cs_() brackets the mode command with real CS frames so the new setpoint never
  // lands against a stale CS, and a CS always follows a mode change.
  if ((sp_changed || f_changed) && gear >= 0 &&
      active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    // Transmit even during a kickstart. The old `!kickstart_active_()` guard here silently
    // dropped setpoint changes for the whole 90s/5:30 wake window — a confirmed cause of
    // "the setpoint won't take". We do NOT re-anchor current_cs_: it is already the kickstart
    // CS, and transmit_mode_with_cs_() brackets the new MODE/setpoint frame with that SAME
    // kickstart CS — so the new setpoint reaches the panel while the compressor-wake stimulus
    // is preserved exactly. (Re-deriving the kickstart CS from the new setpoint could WEAKEN
    // it below the compressor's restart threshold and stall the wake — the failure the
    // kickstart exists to prevent. end_kickstart_ recomputes the correct gear CS for the new
    // setpoint at release.) furrion_setpoint_c_ was already updated above, so the C-mode temp
    // byte and the post-kickstart CS both pick up the new anchor.
    transmit_mode_with_cs_();
  }
}

climate::ClimateFanMode FurrionChillCube::fan_int_to_mode_(int f) {
  switch (f) {
    case 1:  return climate::CLIMATE_FAN_LOW;
    case 2:  return climate::CLIMATE_FAN_MEDIUM;
    case 3:  return climate::CLIMATE_FAN_HIGH;
    default: return climate::CLIMATE_FAN_AUTO;   // 0 or unrecognized
  }
}

climate::ClimateFanMode FurrionChillCube::get_effective_fan_mode_() {
  // 1. Bench-test operator override — exercise startup-clamp (fan=LOW) sequences directly.
  if (test_mode_ && test_fan_ >= 0) return fan_int_to_mode_(test_fan_);
  // 2. An active maneuver's via_fan (the OFF→gear clamp, the 3→2 fan-LOW clamp, …) overrides all.
  //    end_maneuver_ clears maneuver_phase_ BEFORE its restoring mode-resend, so the release frame
  //    correctly picks up the settled gear's fan below rather than the maneuver's via_fan.
  if (maneuver_phase_ != ManeuverPhase::IDLE && maneuver_via_fan_ >= 0)
    return fan_int_to_mode_(maneuver_via_fan_);
  // 3. The current gear's commanded fan, if set (controller-driven fan overrides the HA fan entity).
  bool is_heat = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT);
  int gear = is_heat ? heat_gear_ : cool_gear_;
  const int *fans = is_heat ? heat_gear_fan_ : cool_gear_fan_;
  if (gear >= 1 && gear < MAX_GEARS && fans[gear] >= 0) return fan_int_to_mode_(fans[gear]);
  // 4. Fall through to the HA fan-mode entity (default AUTO) — v1 behavior when no gear fan is set.
  return this->fan_mode.value_or(climate::CLIMATE_FAN_AUTO);
}

int FurrionChillCube::compute_gear_cs_(bool is_heat, int gear) {
  // CS for a gear = setpoint anchor + the gear's configured offset, run through the group
  // boundary clamp (gear_cs_with_clamp_) that keeps the ACTIVE ladder inside the Furrion's
  // 15-30 CS range at extreme setpoints. Gear 0 (idle) is the raw, unclamped "stop compressor"
  // signal (SP-5 cool / SP+5 heat) — at boundary setpoints it reaches CS=11 or CS=35, sent raw
  // on the assumption the Furrion treats out-of-range CS as a stronger nearest-in-range value
  // (verified on a Midea at 55-95°F; low-risk here given symmetric protocol behavior).
  const int *offs = is_heat ? heat_gear_offset_ : cool_gear_offset_;
  if (gear <= 0 || gear >= MAX_GEARS) {
    return furrion_setpoint_c_ + offs[0];   // gear 0 (idle): raw, unclamped
  }
  return gear_cs_with_clamp_(is_heat, offs[gear]);
}

// SP + offset, shifted so the whole ACTIVE-gear span (offsets of gears 1..max) stays inside
// [15,30]. The shift preserves inter-gear spacing; a floor keeps the lowest active gear above
// the raw idle CS so it can't collapse into a "stop while reporting cooling". Reproduces the old
// per-mode cool_cs_/heat clamp in the realistic setpoint band (only differs at absurd extremes
// ≥29°C where the unit is never run). Used for active-gear CS and quirk via-CS alike.
int FurrionChillCube::gear_cs_with_clamp_(bool is_heat, int offset) {
  const int *offs = is_heat ? heat_gear_offset_ : cool_gear_offset_;
  int max_gear = is_heat ? heat_max_gear_ : cool_max_gear_;
  int min_a = offs[1], max_a = offs[1];      // min/max offset across active gears 1..max_gear
  for (int g = 2; g <= max_gear && g < MAX_GEARS; g++) {
    if (offs[g] < min_a) min_a = offs[g];
    if (offs[g] > max_a) max_a = offs[g];
  }
  int lo = furrion_setpoint_c_ + min_a;
  int hi = furrion_setpoint_c_ + max_a;
  int shift = (lo < 15) ? (15 - lo) : (hi > 30) ? (30 - hi) : 0;
  // Keep the shift from dragging an active gear onto the raw idle CS (SP+offs[0]). Direction
  // depends on which side idle sits: cool idle is BELOW active gears (floor a downward shift so
  // the lowest active stays ≥1 above idle — mirrors old cool_cs_ "cap at -2"); heat idle is
  // ABOVE (ceil an upward shift so the highest active stays ≥1 below idle — never binds in range).
  if (offs[0] < min_a) {
    int floor = offs[0] - min_a + 1;     // negative
    if (shift < floor) shift = floor;
  } else if (offs[0] > max_a) {
    int ceil = offs[0] - max_a - 1;      // positive
    if (shift > ceil) shift = ceil;
  }
  return furrion_setpoint_c_ + offset + shift;
}

void FurrionChillCube::set_cs_value_(int cs, uint32_t now) {
  current_cs_ = cs;
  if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
  }
  if (cs_value_sensor_) cs_value_sensor_->publish_state(cs);
}

void FurrionChillCube::update_action_() {
  climate::ClimateAction action;
  if (this->mode == climate::CLIMATE_MODE_OFF) {
    action = climate::CLIMATE_ACTION_OFF;
  } else if (heat_gear_ >= 1) {
    action = climate::CLIMATE_ACTION_HEATING;
  } else if (cool_gear_ >= 1) {
    action = climate::CLIMATE_ACTION_COOLING;
  } else {
    action = climate::CLIMATE_ACTION_IDLE;
  }
  if (this->action != action) {
    this->action = action;
    this->publish_state();
  }
}

void FurrionChillCube::send_swing_state_() {
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) return;
  if (this->swing_mode == climate::CLIMATE_SWING_VERTICAL) {
    send_swing_on();
  } else {
    send_swing_off();
  }
}

// ============================================================
// Transition Maneuver (unified path-dependent quirk engine)
// ============================================================
// One engine runs every quirk, INCLUDING the OFF→gear clamped start (from_gear == -1) that v1 had as
// a separate ClampPhase machine. A from_gear == -1 quirk runs PRE_CS (set the via CS ~500ms before
// mode-on) → HOLD (mode on with via_fan); from_gear >= 0 enters HOLD directly on the running unit.
// See design_gear_engine_v2.

// Emit a mode frame if the CURRENT gear's commanded fan differs from what was last put on the wire.
// A pure gear change otherwise only re-sends CS (no fan), so a per-gear fan (e.g. gear 3 = med,
// gear 4 = high) needs this to reach the unit. No-op in v1-style configs (no gear sets a fan →
// get_effective_fan_mode_() == the HA fan, which the last mode frame already carried).
void FurrionChillCube::maybe_apply_gear_fan_(uint32_t now) {
  // setpoint_pending_ defers, not drops: a mid-debounce transmit would stamp last_tx_setpoint_c_/
  // last_tx_target_f_ with the uncommitted target and suppress the settle commit's CS→MODE→CS
  // bracket (unit holds new setpoint against stale CS for up to a heartbeat). This runs every
  // gear pass and self-retries on the last_tx_fan_ diff, so the fan goes out right after the
  // commit — or rides the commit's own bracket.
  if (!boot_ready_ || failsafe_active_ || active_ir_mode_ == climate::CLIMATE_MODE_OFF ||
      kickstart_active_() || setpoint_pending_)
    return;
  if ((int) get_effective_fan_mode_() != last_tx_fan_) {
    transmit_mode_command_();   // carries the new fan; updates last_tx_fan_
  }
}

// Arm a maneuver from a matched quirk. from_gear == -1 → OFF→gear start (PRE_CS lead); else the unit
// is running and we enter HOLD immediately.
void FurrionChillCube::start_maneuver_(const QuirkDef *q, uint32_t now) {
  bool is_heat = q->is_heat;
  int via_cs = gear_cs_with_clamp_(is_heat, q->via_offset);
  uint32_t dur = q->duration_ms > 0 ? q->duration_ms : quirk_duration_ms_;
  maneuver_is_heat_ = is_heat;
  maneuver_from_gear_ = q->from_gear;
  maneuver_to_gear_ = q->to_gear;
  maneuver_via_cs_ = via_cs;
  maneuver_via_fan_ = q->via_fan;
  maneuver_escape_up_ = q->escape_up;
  maneuver_duration_ms_ = dur;
  maneuver_last_tx_ = now;
  current_cs_ = via_cs;

  if (q->from_gear == -1) {
    // OFF→gear: pre-set the via CS ~500ms before mode-on (the unit ignores a CS before mode-on, but
    // this lead matches the v1 clamp's frame ordering). Mode-on happens when PRE_CS elapses.
    maneuver_phase_ = ManeuverPhase::PRE_CS;
    maneuver_phase_start_ = now;
    if (boot_ready_ && !failsafe_active_) {
      transmit_cs_update_();
      last_cs_heartbeat_ = now;
    }
    if (cs_value_sensor_) cs_value_sensor_->publish_state(via_cs);
    ESP_LOGI(TAG, "Maneuver PRE_CS: %s off->%d via_cs=%d fan=%d",
             is_heat ? "HEAT" : "COOL", (int) q->to_gear, via_cs, (int) q->via_fan);
  } else {
    enter_maneuver_hold_(now);   // running unit → HOLD immediately
  }
}

// Enter HOLD. For an OFF→gear start this turns the unit on with via_fan; for a running maneuver it
// asserts the via CS and, if via_fan changes the fan, re-sends the mode frame.
void FurrionChillCube::enter_maneuver_hold_(uint32_t now) {
  maneuver_phase_ = ManeuverPhase::HOLD;
  maneuver_start_ = now;
  maneuver_last_tx_ = now;
  current_cs_ = maneuver_via_cs_;
  if (maneuver_from_gear_ == -1) {
    // Mode ON with via_fan. maneuver_phase_ is HOLD, so get_effective_fan_mode_() returns via_fan
    // when transmit_mode_command_() builds the frame (v1 ordering: fan set BEFORE the frame).
    set_active_ir_mode_(maneuver_is_heat_ ? climate::CLIMATE_MODE_HEAT : climate::CLIMATE_MODE_COOL);
    transmit_mode_command_();   // carries via_fan; updates last_tx_fan_
    transmit_cs_update_();
    last_cs_heartbeat_ = now;
    ESP_LOGI(TAG, "Maneuver HOLD (off-start): %s cs=%d fan=%d hold=%lus",
             maneuver_is_heat_ ? "HEAT" : "COOL", maneuver_via_cs_, (int) maneuver_via_fan_,
             (unsigned long) (maneuver_duration_ms_ / 1000));
  } else {
    if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      // If via_fan changes the effective fan, send a mode frame first (get_effective returns via_fan).
      if (maneuver_via_fan_ >= 0 && (int) get_effective_fan_mode_() != last_tx_fan_) {
        transmit_mode_command_();
      }
      transmit_cs_update_();
      last_cs_heartbeat_ = now;
    }
    if (cs_value_sensor_) cs_value_sensor_->publish_state(maneuver_via_cs_);
    ESP_LOGI(TAG, "Maneuver HOLD: %s via_cs=%d fan=%d hold=%lus", maneuver_is_heat_ ? "HEAT" : "COOL",
             maneuver_via_cs_, (int) maneuver_via_fan_, (unsigned long) (maneuver_duration_ms_ / 1000));
  }
}

void FurrionChillCube::advance_maneuver_(uint32_t now) {
  if (maneuver_phase_ == ManeuverPhase::PRE_CS) {
    if ((now - maneuver_phase_start_) >= 500) enter_maneuver_hold_(now);
    return;
  }
  if (maneuver_phase_ != ManeuverPhase::HOLD) return;
  // Escape up: release early if the demanded gear rose above to_gear (the v1 clamp's drop-early rule,
  // drop_gear == to_gear + 1). The gear pass runs during the maneuver, so the demanded gear moves.
  if (maneuver_escape_up_) {
    int cur = maneuver_is_heat_ ? heat_gear_ : cool_gear_;
    if (cur > maneuver_to_gear_) {
      ESP_LOGI(TAG, "Maneuver: escape up — gear %d > to %d", cur, (int) maneuver_to_gear_);
      end_maneuver_(now);
      return;
    }
  }
  if ((now - maneuver_start_) >= maneuver_duration_ms_) {
    ESP_LOGI(TAG, "Maneuver: ended — %lus", (unsigned long) (maneuver_duration_ms_ / 1000));
    end_maneuver_(now);
    return;
  }
  // Re-assert the via CS so it stays continuously asserted across the compressor's lockout / any
  // dropped frame. Running quirks (from >= 0) re-assert densely at quirk_transmit_interval_ms_ (5s);
  // the OFF→gear clamped start (from == -1) uses cs_transmit_interval_ms_ (10s) to reproduce v1's
  // cadence EXACTLY — v1's ClampPhase never re-asserted CS itself, leaving it to the 10s loop()
  // heartbeat. (Keeps the bit-identical-defaults invariant on IR bus traffic.)
  uint32_t reassert_ms = (maneuver_from_gear_ == -1) ? cs_transmit_interval_ms_
                                                     : quirk_transmit_interval_ms_;
  if ((now - maneuver_last_tx_) >= reassert_ms) {
    maneuver_last_tx_ = now;
    if (boot_ready_ && !failsafe_active_ && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      transmit_cs_update_();
      last_cs_heartbeat_ = now;
    }
    ESP_LOGI(TAG, "Maneuver: reinforce via_cs=%d", maneuver_via_cs_);
  }
}

// Release the maneuver → restore the CS of whatever gear the controller now holds ("evaluate the
// correct gear when done"), then re-send the mode frame (refreshes the display AND restores the
// settled gear's fan after a via_fan clamp). maneuver_phase_ is cleared BEFORE the resend so
// get_effective_fan_mode_() returns the gear's fan, not the maneuver's via_fan. Also serves as the
// teardown for the run_*_mode_ OFF paths (mode already OFF → the resend + gear CS set both no-op).
void FurrionChillCube::end_maneuver_(uint32_t now) {
  bool is_heat = maneuver_is_heat_;
  maneuver_phase_ = ManeuverPhase::IDLE;
  maneuver_start_ = 0;
  int gear = is_heat ? heat_gear_ : cool_gear_;
  if (gear >= 0) {
    set_cs_value_(compute_gear_cs_(is_heat, gear), now);
  }
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    transmit_mode_command_();   // restores the settled gear's fan (via_fan no longer applies)
  }
  ESP_LOGI(TAG, "Maneuver released: cs=%d gear=%d", current_cs_, gear);
}

const FurrionChillCube::QuirkDef *FurrionChillCube::find_quirk_(bool is_heat, int from_gear, int to_gear) {
  for (int i = 0; i < quirk_count_; i++) {
    const QuirkDef &q = quirks_[i];
    if (q.is_heat == is_heat && q.from_gear == from_gear && q.to_gear == to_gear) {
      return &q;
    }
  }
  return nullptr;
}

void FurrionChillCube::add_quirk(bool is_heat, int from_gear, int to_gear, int via_offset,
                                 int via_fan, bool escape_up, uint32_t duration_ms) {
  if (quirk_count_ >= MAX_QUIRKS) return;
  quirks_[quirk_count_++] = QuirkDef{is_heat, (int8_t)from_gear, (int8_t)to_gear,
                                     (int8_t)via_offset, (int8_t)via_fan, escape_up, duration_ms};
}

void FurrionChillCube::set_gear_offset_(bool is_heat, int gear, int cs_offset, int fan) {
  if (gear < 0 || gear >= MAX_GEARS) return;
  if (is_heat) {
    heat_gear_offset_[gear] = cs_offset;
    heat_gear_fan_[gear] = fan;
    if (gear > heat_max_gear_) heat_max_gear_ = gear;
  } else {
    cool_gear_offset_[gear] = cs_offset;
    cool_gear_fan_[gear] = fan;
    if (gear > cool_max_gear_) cool_max_gear_ = gear;
  }
}

// Build the modulation up/down trips from spacing S + hysteresis h. Boundary n (gear n↔n+1)
// upshift trip = ±n·S; downshift trip = ±(n·S − h). Sign is + for cool (positive=hot),
// − for heat (negative=cold). start/stop/idle pins are stored as-is (not on the grid).
void FurrionChillCube::build_ladders_() {
  for (int n = 1; n < MAX_GEARS; n++) {
    // COOL: offset the modulation grid by the 0↔1 start pin so gear 1 gets a FULL-spacing runway
    // from its entry (cool_start_) to its upshift (cool_up_[1] = start + S), matching every other
    // rung. Without the offset gear 1's runway is only (S − start) ≈ 0.20°C — the narrowest rung and
    // the first to collapse under a residual adaptive bias (the overnight 0↔1→2 popping diagnosed
    // 2026-07-20). Trade: shifts the whole cool ladder top up by cool_start_ (~0.35°C), so MAX needs
    // a touch more demand — acceptable once the bias tracks (iter-1 #1). HEAT keeps the un-offset grid
    // (winter-unvalidated; do the sign-mirror deliberately before heating season).
    cool_up_[n] = cool_start_ + n * cool_spacing_;
    cool_dn_[n] = cool_start_ + n * cool_spacing_ - cool_hyst_;
    heat_up_[n] = -(n * heat_spacing_);
    heat_dn_[n] = -(n * heat_spacing_ - heat_hyst_);
  }
}

// Derive the cold-start floor per mode = the lowest to_gear among that mode's from_gear:-1 (OFF)
// quirks (default 1 = no floor). The compressor can't cold-start below the lowest gear you gave an
// OFF-entry clamp for; from OFF the selection is max(ladder_demand, floor). See design_gear_engine_v2.
void FurrionChillCube::compute_cold_start_floors_() {
  cool_cold_start_floor_ = 1;
  heat_cold_start_floor_ = 1;
  int cool_min = 0, heat_min = 0;   // 0 = none seen yet
  for (int i = 0; i < quirk_count_; i++) {
    const QuirkDef &q = quirks_[i];
    if (q.from_gear != -1 || q.to_gear < 1) continue;
    if (q.is_heat) { if (heat_min == 0 || q.to_gear < heat_min) heat_min = q.to_gear; }
    else           { if (cool_min == 0 || q.to_gear < cool_min) cool_min = q.to_gear; }
  }
  if (cool_min > 0) cool_cold_start_floor_ = cool_min;
  if (heat_min > 0) heat_cold_start_floor_ = heat_min;
}

// ============================================================
// Timed Vane Positioning
// ============================================================

// Called on an OFF→active transition (from set_active_ir_mode_). Starts the timed
// homing only when the mode's (delay, interval) are BOTH configured AND the user has
// the vane switch OFF (swing_mode == OFF). When swing is ON the user wants oscillation,
// so there is no fixed position to seek and we leave the vane alone.
void FurrionChillCube::maybe_start_vent_positioning_(bool is_heat) {
  uint32_t delay_ms = is_heat ? heat_vent_move_delay_ms_ : cool_vent_move_delay_ms_;
  uint32_t interval_ms = is_heat ? heat_vent_interval_ms_ : cool_vent_interval_ms_;
  if (delay_ms == 0 || interval_ms == 0) return;                     // feature unset for this mode
  if (this->swing_mode != climate::CLIMATE_SWING_OFF) return;        // user wants oscillation
  vent_active_delay_ms_ = delay_ms;
  vent_active_interval_ms_ = interval_ms;
  vent_phase_ = VentPhase::WAIT_DELAY;
  vent_phase_start_ = millis();
  ESP_LOGI(TAG, "Vane: %s positioning armed — wait %lus then move %lus",
           is_heat ? "HEAT" : "COOL",
           (unsigned long)(delay_ms / 1000), (unsigned long)(interval_ms / 1000));
}

// Non-blocking state machine. Sends ONLY raw swing IR and never touches this->swing_mode,
// so the HA swing switch stays "off" the whole time (the homing is invisible to the user).
void FurrionChillCube::advance_vent_positioning_() {
  // Self-clock on millis(): vent_phase_start_ is set (in maybe_start_/below) to a millis()
  // value that is newer than loop()'s cached `now`, so using the cached `now` here would
  // underflow `elapsed` and fire the move instantly. See the header note.
  uint32_t now = millis();
  uint32_t elapsed = now - vent_phase_start_;
  switch (vent_phase_) {
    case VentPhase::WAIT_DELAY:
      if (elapsed >= vent_active_delay_ms_) {
        send_swing_on();                       // raw SWING_ON — start the vane moving
        vent_phase_ = VentPhase::MOVING;
        vent_phase_start_ = now;
        ESP_LOGI(TAG, "Vane: move ON (hold %lus)", (unsigned long)(vent_active_interval_ms_ / 1000));
      }
      break;
    case VentPhase::MOVING:
      if (elapsed >= vent_active_interval_ms_) {
        send_swing_off();                      // raw SWING_OFF — stop at the target position
        vent_phase_ = VentPhase::IDLE;
        ESP_LOGI(TAG, "Vane: move OFF — positioned");
      }
      break;
    default:
      vent_phase_ = VentPhase::IDLE;
      break;
  }
}

// Pure state reset (no IR). Every abort trigger — unit OFF (the OFF command's own swing
// frame stops the vane) or a user swing toggle (send_swing_state_() sets the requested
// state) — already emits the correct swing frame, so emitting one here would only fight it.
void FurrionChillCube::abort_vent_positioning_() {
  if (vent_phase_ != VentPhase::IDLE) {
    ESP_LOGI(TAG, "Vane: positioning aborted (phase=%d)", (int)vent_phase_);
    vent_phase_ = VentPhase::IDLE;
  }
}

// Force a real OFF + off-dwell on a heat↔cool transition. Turns the unit OFF, stamps
// off_since_ so the fresh-start off_long_enough gate then holds the new mode off for
// mode_switch_off_ms_, and tears down any IR override (kickstart/maneuver). The OFF→ON that
// follows the dwell re-homes the vane. set_active_ir_mode_(OFF) also aborts any vane sequence.
void FurrionChillCube::force_off_for_mode_switch_(uint32_t now) {
  set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
  transmit_mode_command_();   // OFF on the wire; its swing frame parks the vane
  heat_gear_ = -1;
  cool_gear_ = -1;
  off_since_ = now;
  bias_c_ = 0.0f;
  bias_h_ = 0.0f;
  // Mode switch ends both control episodes: drop SP-transition baselines with the biases
  // (a target change observed across the OFF dwell must not preload onto a zeroed bias),
  // any raise freeze (nothing left to freeze), and any approach hold (the forced OFF
  // supersedes an early engagement).
  last_committed_cool_target_c_ = NAN;
  last_committed_heat_target_c_ = NAN;
  raise_freeze_c_at_ = 0;
  raise_freeze_h_at_ = 0;
  approach_hold_cool_ = false;
  approach_hold_heat_ = false;
  approach_hold_from_off_ = false;
  approach_entry_drift_cpm_ = NAN;
  approach_entry_drift_at_ = 0;
  approach_abort_at_ = 0;   // stale retry cooldowns don't survive a mode switch
  maneuver_phase_ = ManeuverPhase::IDLE;
  if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
  update_action_();
}

// ============================================================
// Gear Controller
// ============================================================

// Hold time in the current gear. The 999999999 sentinel ("no gear change yet
// this boot session") reads as effectively-infinite so any can_upshift_to()
// check passes on the first computation.
uint32_t FurrionChillCube::time_in_gear_(uint32_t now) {
  return (last_gear_change_ == 0) ? 999999999 : (now - last_gear_change_);
}

void FurrionChillCube::run_gear_controller_() {
  float room = inside_temp_c_;
  uint32_t now = millis();

  // Failsafe scenarios + NaN-room grace (also clears a stale failsafe). Returns
  // true if a failsafe/grace path was taken — stop this pass.
  if (check_failsafe_(now, room)) return;

  // Fix: seed idle_since_ if gear is 0 but timer was never set (post-boot recovery)
  if ((heat_gear_ == 0 || cool_gear_ == 0) && idle_since_ == 0) {
    idle_since_ = now;
  }

  bool user_input = user_changed_;
  if (user_input) user_changed_ = false;
  // Test-exit re-anchor: land the gear on the bias-justified value (eff_diff pick), not a real-diff
  // drop that discards the wound-up integral. Latched/cleared with user_input; preserved across a
  // NaN-target hold below, exactly like user_input.
  bool from_test = resume_from_test_;
  if (from_test) resume_from_test_ = false;

  // Captured before dispatch so the periodic state log reports the hold time the
  // gear logic actually saw (a gear change this pass resets last_gear_change_).
  uint32_t time_in_gear = time_in_gear_(now);

  update_outside_lockout_();

  bool do_heat = false, do_cool = false;
  arbitrate_mode_(room, do_heat, do_cool);

  // Dispatch to the active mode. run_heat/cool_mode_ return true if they took an
  // early (NaN-target) hold-return — propagate it so this pass stops cleanly.
  float gear_diff = NAN;  // active diff, for debug publishing
  if (do_heat) {
    if (run_heat_mode_(room, now, user_input, from_test, gear_diff)) return;
  } else if (do_cool) {
    if (run_cool_mode_(room, now, user_input, from_test, gear_diff)) return;
  } else {
    run_idle_mode_(now);
  }

  // Phase 2: each mode's adaptive bias only exists while that mode is active. Clear the inactive
  // mode's bias every pass so a mode switch (or a return from full-off) starts fresh rather than
  // inheriting a stale equilibrium. (Within-mode gear-0 idle is handled by the decay in
  // adaptive_*_eff_diff_, which still runs because do_cool/do_heat stays true at gear 0.)
  if (!do_cool) { bias_c_ = 0.0f; last_committed_cool_target_c_ = NAN; approach_hold_cool_ = false; raise_freeze_c_at_ = 0; }
  if (!do_heat) { bias_h_ = 0.0f; last_committed_heat_target_c_ = NAN; approach_hold_heat_ = false; raise_freeze_h_at_ = 0; }

  // Boot gate: first successful gear computation enables IR
  if (!boot_ready_) {
    boot_ready_ = true;
    ESP_LOGI(TAG, "Boot ready — first gear computation complete, IR enabled");
  }

  // Persist gear + bias for the warm-reboot restore (no-op unless one changed)
  save_gear_pref_();

  // Debug sensor publishing
  publish_debug_state_(gear_diff);

  // Periodic state log
  ESP_LOGD(TAG, "state: heat=%d cool=%d room=%.2f cs=%d hold=%lus idle=%lum mode=%d biasC=%.2f biasH=%.2f drift=%.3f",
           heat_gear_, cool_gear_, room, current_cs_,
           time_in_gear / 1000,
           idle_since_ > 0 ? (now - idle_since_) / 60000 : 0,
           (int)last_active_mode_, bias_c_, bias_h_, room_drift_cpm_);
}

// Failsafe detection + NaN-room grace. Returns true if the caller should stop
// this pass (a failsafe or NaN-grace path was taken). On the normal path it
// also clears a stale failsafe_active_ flag.
bool FurrionChillCube::check_failsafe_(uint32_t now, float room) {
  // === Failsafe scenario 1: Boot, HA never connects (5 min) ===
  bool never_got_update = (last_temp_update_ == 0 && (now - boot_time_) > 300000);

  // === Failsafe scenario 2: HA API disconnected (15 min) ===
#ifdef USE_API
  bool api_connected = (api::global_api_server != nullptr &&
                        api::global_api_server->is_connected());
  if (api_connected) {
    ha_disconnect_time_ = 0;
  } else if (ha_disconnect_time_ == 0 && last_temp_update_ > 0) {
    ha_disconnect_time_ = now;
    ESP_LOGW(TAG, "HA API disconnected — 15-min failsafe timer started");
  }
  bool ha_disconnected = (ha_disconnect_time_ > 0 &&
                          (now - ha_disconnect_time_) > 900000);
#else
  bool ha_disconnected = false;
#endif

  // === Failsafe scenario 3: Room temp NaN (5 min grace) ===
  if (isnan(room)) {
    if (temp_nan_since_ == 0) {
      temp_nan_since_ = now;
      ESP_LOGW(TAG, "Room temp NaN — 5-min grace period started");
    }
  } else {
    temp_nan_since_ = 0;
  }
  bool temp_unavailable = (temp_nan_since_ > 0 && (now - temp_nan_since_) > 300000);

  // === Failsafe trigger ===
  if (never_got_update || ha_disconnected || temp_unavailable) {
    ESP_LOGW(TAG, "FAILSAFE — CS stopped, unit will revert to internal sensor (boot=%d ha_dc=%d nan=%d)",
             never_got_update, ha_disconnected, temp_unavailable);
    heat_gear_ = -1;
    cool_gear_ = -1;
    bias_c_ = 0.0f;   // drop the adaptive equilibrium on failsafe — re-engage learns fresh
    bias_h_ = 0.0f;
    last_committed_cool_target_c_ = NAN;  // SP-transition baselines die with the biases
    last_committed_heat_target_c_ = NAN;
    raise_freeze_c_at_ = 0;               // raise freezes die with them (zeroed bias, nothing to hold)
    raise_freeze_h_at_ = 0;
    approach_hold_cool_ = false;          // approach holds die with them (failsafe = hands off)
    approach_hold_heat_ = false;
    approach_entry_drift_cpm_ = NAN;
    approach_entry_drift_at_ = 0;
    approach_hold_from_off_ = false;
    approach_abort_at_ = 0;
    arm_ring_reset_();  // unit ran itself during failsafe — its travel is machine-made, don't
                        // let the post-recovery warm-back arm approach off it (NaN path already
                        // resets via the callback; this covers the HA-disconnect/boot triggers)
    idle_since_ = 0;
    last_active_mode_ = MODE_NONE;
    last_mode_event_at_ = 0;
    off_since_ = 0;
    heater_locked_out_ = false;
    setpoint_pending_ = false;  // drop any in-flight debounce — no deferred commit after failsafe
    // Drop a pending mode-frame reinforcement at failsafe ENTRY, not just at fire time. The
    // fire-time !failsafe_active_ gate misses the enter-and-exit-within-the-window race: a
    // resend armed pre-failsafe would otherwise fire right after a quick failsafe clear with
    // active_ir_mode_ == OFF and transmit the very OFF frame this block deliberately withholds
    // (shutting down a unit that is running on its internal sensor).
    mode_resend_pending_ = false;
    maneuver_phase_ = ManeuverPhase::IDLE;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // Sync active_ir_mode_ (and mode_pref_) to OFF to match gear state.
    // No IR is transmitted — set_active_ir_mode_() only updates state + saves flash.
    // Without this, mode_pref_ stays at HEAT/COOL and a reboot during failsafe
    // causes setup() to restore gear=0, producing a spurious MODE_ON IR command
    // on the next controller run (6A compressor spike).
    if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
      set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    }
    // Stop all IR transmission — failsafe_active_ gates CS heartbeat.
    // Unit's setpoint is already near the desired target (dynamic setpoint).
    // After ~7 min with no CS, unit reverts to its own internal sensor.
    failsafe_active_ = true;
    boot_ready_ = true;
    update_action_();
    return true;
  }

  // NaN grace: hold current state
  if (isnan(room)) {
    ESP_LOGD(TAG, "Room temp NaN — holding (%lus left)",
             (300000 - (now - temp_nan_since_)) / 1000);
    publish_debug_state_(NAN);
    return true;
  }

  // Clear failsafe if normal operation
  if (failsafe_active_) {
    failsafe_active_ = false;
    ESP_LOGI(TAG, "Failsafe cleared — normal operation resumed");
  }
  return false;
}

// Outside-temperature lockout for heating (1°F hysteresis; fail-open if the
// outside sensor is unavailable).
void FurrionChillCube::update_outside_lockout_() {
  if (!isnan(outside_temp_c_)) {
    float lockout_low = outside_lockout_temp_c_ - 0.556f;  // 1°F hysteresis
    if (outside_temp_c_ < lockout_low && !heater_locked_out_) {
      heater_locked_out_ = true;
      ESP_LOGW(TAG, "Outside %.1f°C < lockout — heating locked out", outside_temp_c_);
    } else if (outside_temp_c_ >= outside_lockout_temp_c_ && heater_locked_out_) {
      heater_locked_out_ = false;
      ESP_LOGI(TAG, "Outside %.1f°C >= lockout — heating re-enabled", outside_temp_c_);
    }
  } else if (heater_locked_out_) {
    // Outside sensor unavailable — fail-open to allow heating
    heater_locked_out_ = false;
    ESP_LOGW(TAG, "Outside temp unavailable — heating lockout cleared (fail-open)");
  }
}

// Picks the active mode for this pass. At most one of do_heat / do_cool is true.
void FurrionChillCube::arbitrate_mode_(float room, bool &do_heat, bool &do_cool) {
  // Determine active modes from user's climate mode
  bool heater_on = !heater_locked_out_ &&
                   (this->mode == climate::CLIMATE_MODE_HEAT ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);
  bool cooler_on = (this->mode == climate::CLIMATE_MODE_COOL ||
                    this->mode == climate::CLIMATE_MODE_HEAT_COOL);

  // When both modes enabled, decide based on room temp vs targets
  do_heat = heater_on;
  do_cool = cooler_on;
  if (heater_on && cooler_on) {
    float h_target = get_heat_target_();
    float c_target = get_cool_target_();
    // Guard: if targets are inverted (low >= high), swap to prevent stuck mode
    if (!isnan(h_target) && !isnan(c_target) && h_target > c_target) {
      float tmp = h_target; h_target = c_target; c_target = tmp;
    }

    // Mode change protection now lives in the 0→-1 gate (idle/event/temp lockouts)
    // and -1→active gate (1-min off lockout). Here we just pick the active mode
    // based on current gear state and temperature.
    if (heat_gear_ >= 0) {
      do_cool = false;  // heat already engaged — stay in heat
    } else if (cool_gear_ >= 0) {
      do_heat = false;  // cool already engaged — stay in cool
    } else {
      // Both gears -1 (fully off). Pick based on temperature.
      if (room <= h_target) { do_cool = false; }
      else if (room >= c_target) { do_heat = false; }
      else {
        // Room in deadband — neither mode should engage. Stay off.
        do_heat = false;
        do_cool = false;
      }
    }
  }
}

// ── Displacement arming ring (see APPROACH_ARM_RISE_C) ──────────────────────────────────────
// Recorded from the temp-sensor callback (self-clocks on the callback's millis(), like the drift
// ring — armed outside the gear pass, per the millis-vs-now convention). Read from the gear pass
// with its cached `now`: a sample recorded after that cache was taken underflows to a huge age and
// is skipped — harmless, the live side of the subtraction is inside_temp_c_ itself.
void FurrionChillCube::arm_ring_record_(float temp_c, uint32_t now) {
  if (arm_ring_last_sample_ != 0 && (now - arm_ring_last_sample_) < APPROACH_ARM_SAMPLE_MS) return;
  arm_ring_at_[arm_ring_head_] = now;
  arm_ring_temp_[arm_ring_head_] = temp_c;
  arm_ring_head_ = (uint8_t)((arm_ring_head_ + 1) % ARM_RING_N);
  if (arm_ring_count_ < ARM_RING_N) arm_ring_count_++;
  arm_ring_last_sample_ = now;
}

void FurrionChillCube::arm_ring_reset_() {
  arm_ring_head_ = 0;
  arm_ring_count_ = 0;
  arm_ring_last_sample_ = 0;
}

// Rise of the current room temp above the trailing-window trough (°C). NAN with no valid history
// (boot / post-sensor-gap) — every caller compares with >=, and NAN comparisons are false, so the
// gate FAILS CLOSED (approach stays unarmed until real history exists). A partial ring is fine: a
// shorter window only makes the gate stricter, and noise cannot fake displacement at any length.
// The age filter also retires stale samples if recording ever stalls (test_mode, sensor quiet).
float FurrionChillCube::arm_rise_c_(uint32_t now) {
  if (isnan(inside_temp_c_)) return NAN;
  float trough = NAN;
  for (uint8_t i = 0; i < arm_ring_count_; i++) {
    if (now - arm_ring_at_[i] > APPROACH_ARM_WINDOW_MS) continue;
    if (isnan(trough) || arm_ring_temp_[i] < trough) trough = arm_ring_temp_[i];
  }
  if (isnan(trough)) return NAN;
  return inside_temp_c_ - trough;
}

// Heat mirror: fall of the current room temp below the trailing-window peak. ⚠️ winter-unvalidated.
float FurrionChillCube::arm_fall_c_(uint32_t now) {
  if (isnan(inside_temp_c_)) return NAN;
  float peak = NAN;
  for (uint8_t i = 0; i < arm_ring_count_; i++) {
    if (now - arm_ring_at_[i] > APPROACH_ARM_WINDOW_MS) continue;
    if (isnan(peak) || arm_ring_temp_[i] > peak) peak = arm_ring_temp_[i];
  }
  if (isnan(peak)) return NAN;
  return peak - inside_temp_c_;
}

// Approach-side predictive re-engagement — standard name: OPTIMUM START (optimal start/stop
// recovery, classic building-automation technique: measure the drift rate, predict time-to-
// setpoint, start the equipment early enough to arrive at equilibrium).
// Fires only from off/idle via the callers below; all gear machinery (off-dwell lockout,
// OFF-entry clamps, HOLD_MS) still applies to the resulting entry. NaN drift (warmup) and NaN
// diff can't reach here true: NaN comparisons are false.
// lead_ms is the entry-state lead (OFF-entry: approach_off_lead_ms_(), ~clamp duration + latency;
// idle: approach_lead_ms_, the comfort lead) — see set_approach_lead_off_ms in the header.
bool FurrionChillCube::approach_predict_cool_(float diff, uint32_t now, uint32_t lead_ms) {
  if (approach_lead_ms_ == 0) return false;  // master enable, regardless of lead_ms
  if (approach_abort_at_ != 0 && (now - approach_abort_at_) < APPROACH_RETRY_MS) return false;
  // Displacement arming gate (incident 2026-08-05): the room must have genuinely traveled before
  // approach is eligible at all — see APPROACH_ARM_RISE_C. NAN rise (no history) fails closed.
  if (!(arm_rise_c_(now) >= APPROACH_ARM_RISE_C)) return false;
  // Fire only from meaningfully BELOW the setpoint: the release boundary lives at the ladder's
  // 1→0 rail, so firing inside the deadband would engage and release in back-to-back passes
  // (a wasted start). Bug-check round-1 finding.
  if (!(diff < -ADAPT_DEADBAND_C)) return false;
  bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
  if (!drift_fresh || !(room_drift_cpm_ > APPROACH_DRIFT_MIN_CPM)) return false;
  float minutes_to_cross = (-diff) / room_drift_cpm_;
  return minutes_to_cross * 60000.0f <= (float) lead_ms;
}

// Sign-mirror: room ABOVE the heat setpoint, drifting DOWN toward it. ⚠️ winter-unvalidated.
bool FurrionChillCube::approach_predict_heat_(float diff, uint32_t now, uint32_t lead_ms) {
  if (approach_lead_ms_ == 0) return false;  // master enable, regardless of lead_ms
  if (approach_abort_at_ != 0 && (now - approach_abort_at_) < APPROACH_RETRY_MS) return false;
  // Displacement arming gate — mirror of the cool gate (fall below the windowed peak).
  if (!(arm_fall_c_(now) >= APPROACH_ARM_RISE_C)) return false;
  if (!(diff > ADAPT_DEADBAND_C)) return false;  // mirror of the cool deadband fire-gate
  bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
  if (!drift_fresh || !(room_drift_cpm_ < -APPROACH_DRIFT_MIN_CPM)) return false;
  float minutes_to_cross = diff / (-room_drift_cpm_);
  return minutes_to_cross * 60000.0f <= (float) lead_ms;
}

// Crossing preload — one-shot bias floor at approach-hold → ladder handover (see the setter note
// in the header). The cheap special case of the review-mpc-alignment P2 disturbance observer:
// free-rise drift ∝ present load / thermal mass, and equilibrium bias is the same load in ladder
// units, so needed_bias ≈ k_d × drift with the mass cancelling (k_d ≈ 20 min, ±15% across four
// regimes; the 0.75 derate is folded into crossing_preload_kd_). The drift is the ENGAGEMENT
// snapshot (approach_entry_drift_cpm_) — the trailing 3-min slope of the compressor-free rise
// just before the maneuver, the cleanest available load signal (Stephen 2026-08-14; the prior
// handover-pass sample read ~0 whenever the clamp blast arrested the rise — total suppression,
// floor no-op'd in the first live payoff case). approach_predict_* validated the drift fresh and
// demand-directed at the moment the snapshot was taken; the cap bounds any door-transient spike.
// The hold freezes the integral, so bias here is exactly the retained value the hold entered with.
// Returns the bias delta applied (0 on no-op) so the call site can patch its cached eff_diff —
// eff was computed at the top of the pass, before the hold released, and the handover pass is
// the "single decision point"; without the patch the decision runs on the pre-floor bias and a
// spurious 1→0 stop there costs a compressor cycle + the 0→1 dwell gate (bug-check 2026-08-10).
float FurrionChillCube::apply_crossing_preload_(bool is_heat, uint32_t now) {
  float snap = approach_entry_drift_cpm_;
  uint32_t snap_at = approach_entry_drift_at_;
  approach_entry_drift_cpm_ = NAN;  // one-shot: consumed at this handover regardless of outcome
  approach_entry_drift_at_ = 0;
  if (!adaptive_enable_ || crossing_preload_kd_ <= 0.0f) return 0.0f;
  if (isnan(snap) || snap_at == 0) return 0.0f;
  // Age bound (bug-check round 1): an idle-fired hold can legally run to its 3×-lead hard cap
  // (~45 min at the 15-min lead) before the band release — a snapshot that old is a stale load
  // estimate. 2× lead passes every normal hold (OFF-entry rides ~305 s; idle holds typically
  // release ≈1 lead in) and no-ops only marathon holds, where the integral is the safer authority.
  if (now - snap_at > 2UL * approach_lead_ms_) return 0.0f;
  // Demand-direction drift: cool = room rising toward the cool SP; heat = falling toward heat SP.
  float toward = is_heat ? -snap : snap;
  if (!(toward > 0.0f)) return 0.0f;
  if (toward > CROSSING_PRELOAD_DRIFT_CAP_CPM) toward = CROSSING_PRELOAD_DRIFT_CAP_CPM;
  float floor_b = crossing_preload_kd_ * toward;
  if (floor_b > ADAPT_BIAS_C_MAX) floor_b = ADAPT_BIAS_C_MAX;
  float &bias = is_heat ? bias_h_ : bias_c_;
  if (floor_b <= bias) return 0.0f;
  float delta = floor_b - bias;
  ESP_LOGI(TAG, "Crossing preload (%s): entry drift %.3f C/min -> bias %.2f -> %.2f",
           is_heat ? "heat" : "cool", toward, bias, floor_b);
  bias = floor_b;
  return delta;
}

// HEATING mode pass. Returns true if it took an early (NaN-target) hold-return.
bool FurrionChillCube::run_heat_mode_(float room, uint32_t now, bool user_input, bool from_test,
                                     float &gear_diff) {
  // The cool-mode adaptive bias is meaningless in heat; clear it here (not just in the
  // post-dispatch !do_cool clear) so it's cleared even if this pass early-returns on a NaN
  // heat target — guaranteeing a later heat→cool switch always starts from bias_c_ = 0.
  bias_c_ = 0.0f;
  last_committed_cool_target_c_ = NAN;  // cool SP-transition state dies with its bias
  raise_freeze_c_at_ = 0;
  approach_hold_cool_ = false;

  uint32_t time_in_gear = time_in_gear_(now);
  auto can_upshift_to = [&](int target_gear) -> bool {
    return user_input || (time_in_gear >= HOLD_MS[target_gear]);
  };

  // Cool→Heat MUST route through a real OFF held for the off-dwell — same rationale as
  // run_cool_mode_ (compressor safety + vane OFF→ON anchor). Catches a DIRECT switch while
  // the unit is still actively COOLing; the natural HEAT_COOL handoff already turned it OFF.
  if (active_ir_mode_ == climate::CLIMATE_MODE_COOL) {
    ESP_LOGI(TAG, "Cool→Heat: force OFF + %lus dwell before heating",
             (unsigned long)(mode_switch_off_ms_ / 1000));
    force_off_for_mode_switch_(now);
    return true;  // hold; heat re-engages from -1 after the dwell (OFF→ON homes the vane)
  }
  // Defensive: clear a stale cool gear (unit already OFF via the natural path).
  if (cool_gear_ != -1) {
    cool_gear_ = -1;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  }

  // Pinned ladder trips (start/stop/idle). Grid trips (gear n↔n+1) are read directly from
  // heat_up_[]/heat_dn_[] (built from spacing in build_ladders_()) by the generalized selection below.
  const float H_UP_01 = heat_start_, H_DN_10 = heat_stop_, H_IDLE = heat_idle_;

  update_furrion_setpoint_(true);
  float target = get_heat_target_();
  if (isnan(target)) {
    ESP_LOGW(TAG, "Heat target NaN — holding gear/CS");
    user_changed_ = user_input;  // don't consume a user event during a NaN hold
    resume_from_test_ = from_test;  // ditto: preserve the test-exit re-pick across the hold
    publish_debug_state_(NAN);
    return true;
  }
  float diff = room - target;
  gear_diff = diff;  // debug always reports REAL (unbiased) diff

  // Setpoint-transition detector — SIGN-MIRROR of the cool one (see run_cool_mode_ for the full
  // rationale + accepted edges). Heat demand rises on a setpoint RAISE: preload bias_h_ with a
  // fraction of the raise, gated on real heating demand (room below the NEW target by more than
  // the deadband). A demand-removing DROP freezes the bias until the crossing (see below).
  // ⚠️ WINTER-UNVALIDATED like the rest of the heat adaptive path — structurally symmetric only.
  if (!setpoint_pending_) {
    if (isnan(last_committed_heat_target_c_)) {
      last_committed_heat_target_c_ = target;
    } else if (target != last_committed_heat_target_c_) {
      float delta_c = target - last_committed_heat_target_c_;
      last_committed_heat_target_c_ = target;
      if (delta_c > 0.0f && adaptive_enable_ && sp_preload_factor_ > 0.0f &&
          diff < -ADAPT_DEADBAND_C) {
        float preload = sp_preload_factor_ * delta_c;
        bias_h_ += preload;
        if (bias_h_ > ADAPT_BIAS_C_MAX) bias_h_ = ADAPT_BIAS_C_MAX;
        if (bias_h_ < -ADAPT_BIAS_C_MAX) bias_h_ = -ADAPT_BIAS_C_MAX;
        ESP_LOGI(TAG, "Heat SP raise %.2fC: bias preloaded +%.2f -> %.2fC", delta_c, preload, bias_h_);
      } else if (delta_c < 0.0f && adaptive_enable_ && diff > ADAPT_DEADBAND_C) {
        // Demand-REMOVING heat SP drop (room now above the new target): freeze the retained heat
        // bias until the room falls back to the band — sign-mirror of the cool raise freeze
        // (Stephen 2026-08-14). First-arm time kept on re-drops (see the cool site).
        // ⚠️ winter-unvalidated.
        if (raise_freeze_h_at_ == 0) raise_freeze_h_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Heat SP drop %.2fC: bias %.2f frozen until crossing (cap %lu min)",
                 -delta_c, bias_h_, (unsigned long)(RAISE_FREEZE_MAX_MS / 60000UL));
      }
    }
  }

  // Phase 2 adaptive (heat): advance the integral and get the effective diff for the active-gear
  // switch cases. Mirror of the cool call. Re-engage/idle/mode-switch AND the pong-critical 1→0
  // STOP decision stay on real diff. Called every heat pass so the integral advances/decays.
  float eff_diff = adaptive_heat_eff_diff_(diff, now, time_in_gear);
  // Upshift comparisons use the rate-gated diff (bias removed while the room isn't cooling).
  float up_diff = heat_eff_up_diff_;
  int gear = heat_gear_;
  int new_gear = gear;

  bool approach_engaged_this_pass = false;  // set by the two engagement sites below

  // Approach-hold maintenance — sign-mirror of the cool block (see run_cool_mode_ for the full
  // rationale; ⚠️ winter-unvalidated). Release = the heat ladder's own gear-1 band.
  if (approach_hold_heat_ && approach_hold_from_off_) {
    // Atomic clamp commitment — sign-mirror of the cool block (see run_cool_mode_ for the full
    // rationale; ⚠️ winter-unvalidated): an OFF-fired heat approach rides its full OFF-entry clamp,
    // then releases without a retry cooldown on the next pass as the single decision point.
    if (!kickstart_active_()) {
      approach_hold_heat_ = false;
      approach_hold_from_off_ = false;
      ESP_LOGI(TAG, "Approach (heat): OFF-entry clamp complete — handing off to ladder");
      // Handover preload from the engagement drift snapshot — mirror of the cool site (same-pass
      // eff patch; heat eff = real − bias_h_, so a raised bias_h_ LOWERS eff_diff). Bias-blind
      // guard mirrors cool. ⚠️ winter-unvalidated.
      {
        float pd = apply_crossing_preload_(true, now);
        if (raise_freeze_h_at_ == 0) eff_diff -= pd;
      }
    }
  } else if (approach_hold_heat_) {
    bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
    float mins = (room_drift_cpm_ < 0.0f) ? diff / (-room_drift_cpm_) : 1e9f;
    if (diff <= H_DN_10) {
      // Release at the heat ladder's gear-1 hold boundary. Deliberately REAL-diff (H_DN_10 is the
      // pong-pinned real-diff rail) so the release keeps the heat 1→0 semantics intact.
      approach_hold_heat_ = false;
      ESP_LOGI(TAG, "Approach (heat): reached ladder band — hold released, ladder takes over");
      // Handover preload from the engagement drift snapshot — mirror of the cool band-release
      // site, incl. the bias-blind guard. ⚠️ winter-unvalidated.
      {
        float pd = apply_crossing_preload_(true, now);
        if (raise_freeze_h_at_ == 0) eff_diff -= pd;
      }
    } else if (!drift_fresh || !(room_drift_cpm_ < 0.0f) ||
               mins * 60000.0f > 2.0f * (float) approach_lead_ms_ ||
               (now - approach_started_at_) / 3 >= approach_lead_ms_) {
      approach_hold_heat_ = false;
      approach_abort_at_ = (now != 0) ? now : 1;  // 0 is the "no cooldown" sentinel
      ESP_LOGI(TAG, "Approach (heat): aborted (drift died/receded or hold cap) — standing down");
    }
  }

  // Generalized N-gear selection (sign-mirror of cool: diff negative = cold = higher heat gear).
  // M = highest configured heat gear. Grid trips come from heat_up_[]/heat_dn_[]; the 0↔1 boundary +
  // idle are the pinned H_UP_01/H_DN_10/H_IDLE.
  int M = heat_max_gear_;
  auto entry_thresh = [&](int g) -> float { return (g <= 1) ? H_UP_01 : heat_up_[g - 1]; };
  // Highest gear whose from-below (colder) entry threshold `d` clears (0 = none).
  auto pick_from_below = [&](float d) -> int {
    for (int g = M; g >= 1; g--) if (d < entry_thresh(g)) return g;
    return 0;
  };

  if (gear == -1 || user_input) {
    // Fresh start from -1 requires the 1-min off lockout (hardware wind-down); no bypass.
    bool off_long_enough = (off_since_ == 0) || (now - off_since_ >= mode_switch_off_ms_);
    if (gear == -1 && !off_long_enough) {
      new_gear = -1;  // still in 1-min wind-down period
    } else if (user_input && gear >= 0 && gear_in_band_heat_(gear, diff)) {
      // User event but the current gear is still valid for the diff — preserve hunting state.
      new_gear = gear;
    } else {
      // From -1: floor at the derived cold-start floor (heat default 1 → no-op); never gear 0.
      // Test-exit (from_test): pick on eff_diff (sign-mirror of cool) so the wound bias lands the
      // load-justified gear, not a real-diff drop. Genuine user events keep the REAL-diff pick.
      int picked = pick_from_below(from_test ? eff_diff : diff);
      if (picked >= 1) {
        new_gear = picked;
        if (gear == -1 && new_gear < heat_cold_start_floor_) new_gear = heat_cold_start_floor_;
      } else if (gear == -1 && approach_predict_heat_(diff, now, approach_off_lead_ms_())) {
        // Approach-side early engagement from OFF — through the normal heat OFF→1 clamp (mirror
        // of cool: kickstart quirks are REQUIRED for all OFF entries; see the cool comment).
        // OFF-entry lead is shared with cool (the heat OFF→1 clamp is also ~305s).
        new_gear = heat_cold_start_floor_;
        approach_hold_heat_ = true;
        approach_hold_from_off_ = true;   // atomic clamp commitment — see hold maintenance
        approach_engaged_this_pass = true;
        approach_started_at_ = now;
        // Snapshot the free-fall drift for the handover preload (mirror of cool; the clamp will
        // suppress the live value from here). ⚠️ winter-unvalidated.
        approach_entry_drift_cpm_ = room_drift_cpm_;
        approach_entry_drift_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Approach (heat): crossing predicted within lead — gear-%d engagement via clamp",
                 new_gear);
      } else if (gear == -1) {
        new_gear = -1;                               // stays off
      } else if (user_input && diff > H_IDLE) {
        new_gear = -1;                               // user tap past setpoint → off
      } else {
        new_gear = 0;
      }
    }
  } else {
    if (gear == 0) {
      // First compute post-restore: jump straight to the correct gear (skip the HOLD_MS ladder).
      if (last_gear_change_ == 0) {
        new_gear = pick_from_below(diff);
      } else if (can_upshift_to(1) && diff < H_UP_01) {
        new_gear = 1;
      } else if (can_upshift_to(1) && approach_predict_heat_(diff, now, approach_lead_ms_)) {
        // Approach-side early engagement from idle (mirror of cool; idle quirk not bypassed).
        new_gear = 1;
        approach_hold_heat_ = true;
        approach_hold_from_off_ = false;  // idle-fired hold — normal 4-exit maintenance
        approach_engaged_this_pass = true;
        approach_started_at_ = now;
        // Snapshot the free-fall drift for the handover preload (mirror of cool idle site).
        approach_entry_drift_cpm_ = room_drift_cpm_;
        approach_entry_drift_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Approach (heat): crossing predicted within lead — early gear-1 from idle");
      }
      // 0→-1 gate (natural path only — user_input handled above). A predicted approach suppresses
      // the natural full-off (mirror of cool); imm_off is NOT suppressible.
      bool imm_off = !boot_ready_;
      bool idle_enough = (idle_since_ > 0) && (now - idle_since_ >= mode_switch_idle_ms_);
      bool event_ok = (last_mode_event_at_ == 0) || (now - last_mode_event_at_ >= mode_switch_event_ms_);
      bool past_setpoint = diff > mode_switch_temp_offset_c_;
      // Natural-off gate — sign-mirror of the cool block (see run_cool_mode_ + incident
      // 2026-08-05 + NATURAL_OFF_BIAS_EPS_C): heat full-off waits for the heat integral to be
      // unwound (no drift leg — Stephen 2026-08-06) — EXCEPT through the HEAT_COOL handoff door
      // (room risen into cool's engagement territory; this mirror bites in shoulder-season
      // mornings: sun lifts the room while bias_h_ is still wound and heat idle would otherwise
      // lock cool out). ⚠️ winter-unvalidated, structurally symmetric only.
      bool handoff_demand = (this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                            (room >= get_cool_target_() - mode_switch_temp_offset_c_);
      // Frozen heat bias counts as unwound — mirror of the cool gate. ⚠️ winter-unvalidated.
      bool bias_unwound = bias_h_ <= NATURAL_OFF_BIAS_EPS_C || raise_freeze_h_at_ != 0;
      bool natural_off = idle_enough && event_ok && past_setpoint &&
                         (handoff_demand || bias_unwound) &&
                         !approach_predict_heat_(diff, now, approach_lead_ms_);
      if ((imm_off || natural_off) && diff > H_IDLE) new_gear = -1;
    } else {
      // Active gears 1..M: upshift on the rate-gated up_diff (colder crosses heat_up_[gear]). The
      // 1→0 STOP is pong-critical and evaluated on REAL diff (bias_h_ must not move it); gears 2+
      // downshift on eff_diff. Downshift trip = heat_stop_ (gear 1) else heat_dn_[gear-1].
      if (gear < M && can_upshift_to(gear + 1) && up_diff < heat_up_[gear] &&
          !(approach_hold_heat_ &&
            (gear == 1 || (approach_hold_from_off_ && gear == heat_cold_start_floor_)))) {
        // Upshift suppressed during a hold — sign-mirror of the cool pin (see run_cool_mode_).
        new_gear = gear + 1;
      } else {
        float dn = (gear == 1) ? H_DN_10 : heat_dn_[gear - 1];
        float dcmp = (gear == 1) ? diff : eff_diff;   // 1→0 STOP on REAL diff (pong-critical)
        // The approach hold pins gear 1 above SP; the hold-maintenance block owns its exits.
        // (The pong-critical 1→0 STOP stays real-diff for every NON-held pass.)
        if (dcmp > dn &&
            !(approach_hold_heat_ &&
              (gear == 1 || (approach_hold_from_off_ && gear == heat_cold_start_floor_))))
          new_gear = gear - 1;
      }
    }
  }

  // Track idle_since
  if (new_gear == 0 && gear != 0) {
    idle_since_ = now;
  } else if (new_gear != 0) {
    idle_since_ = 0;
  }

  // Track off_since_ for the 1-min off lockout
  if (new_gear == -1 && gear != -1) {
    off_since_ = now;
  }

  // Publish gear and compressor on change
  if (new_gear != gear) {
    heat_gear_ = new_gear;
    last_gear_change_ = now;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(new_gear);
    if (new_gear >= 1) last_active_mode_ = MODE_HEAT;
    if (compressor_output_sensor_)
      compressor_output_sensor_->publish_state(gear_output_pct(new_gear, heat_max_gear_));
    ESP_LOGI(TAG, "HEAT %d -> %d (room=%.2f target=%.2f diff=%.2f)",
             gear, new_gear, room, target, diff);
  }

  // CS + maneuver (quirk) logic — unified. A matching quirk (including the OFF→gear-1 clamped start,
  // from_gear == -1) runs the maneuver engine; otherwise set the gear CS directly. OFF→2+ has no
  // quirk by default → direct start. Any OFF→gear (gear == -1) is a fresh-start → stamp last_mode_event.
  // Selection-exit cleanup — mirror of the cool block (see run_cool_mode_).
  if (approach_hold_heat_ && !approach_engaged_this_pass && new_gear != 1 &&
      !(approach_hold_from_off_ && new_gear == heat_cold_start_floor_)) {
    // (mirror of the cool cleanup: floor exemption gated on the atomic flag)
    approach_hold_heat_ = false;
    approach_hold_from_off_ = false;
    ESP_LOGI(TAG, "Approach (heat): selection moved gear to %d — hold released", new_gear);
  }

  if (new_gear >= 0) {
    int cs = compute_gear_cs_(true, new_gear);
    if (!kickstart_active_()) {
      if (gear == -1) last_mode_event_at_ = now;
      const QuirkDef *q = find_quirk_(true, gear, new_gear);
      if (q != nullptr) {
        start_maneuver_(q, now);
      } else if (current_cs_ != cs) {
        set_cs_value_(cs, now);
      }
    }
  }

  // HVAC on/off
  if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_HEAT && !kickstart_active_()) {
    set_active_ir_mode_(climate::CLIMATE_MODE_HEAT);
    transmit_mode_with_cs_();
  }
  if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    if (kickstart_active_()) end_maneuver_(now);
  }

  // Apply a per-gear commanded fan (heat gears carry none by default → no-op).
  maybe_apply_gear_fan_(now);

  update_action_();
  return false;
}

// COOLING mode pass. Returns true if it took an early (NaN-target) hold-return.
bool FurrionChillCube::run_cool_mode_(float room, uint32_t now, bool user_input, bool from_test,
                                     float &gear_diff) {
  uint32_t time_in_gear = time_in_gear_(now);
  auto can_upshift_to = [&](int target_gear) -> bool {
    return user_input || (time_in_gear >= HOLD_MS[target_gear]);
  };

  // Heat→Cool MUST route through a real OFF held for the off-dwell: compressor safety
  // AND the vane's known OFF→ON re-home anchor. This catches a DIRECT switch — the unit
  // still actively HEATing when COOL is selected (user flip, or sync_mode handoff). The
  // natural HEAT_COOL drift already turns the unit OFF via the heat 0→-1 path, so by the
  // time it reaches here active_ir_mode_ is already OFF and this is skipped.
  if (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) {
    ESP_LOGI(TAG, "Heat→Cool: force OFF + %lus dwell before cooling",
             (unsigned long)(mode_switch_off_ms_ / 1000));
    force_off_for_mode_switch_(now);
    return true;  // hold; cool re-engages from -1 after the dwell (OFF→ON homes the vane)
  }
  // Defensive: clear a stale heat gear (unit already OFF via the natural path).
  if (heat_gear_ != -1) {
    heat_gear_ = -1;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  }
  // Cool pass: heat is not the active mode → drop its integral so a later cool→heat switch starts
  // from bias_h_ = 0 (mirror of the bias_c_ = 0 at the top of run_heat_mode_).
  bias_h_ = 0.0f;
  last_committed_heat_target_c_ = NAN;  // heat SP-transition state dies with its bias
  raise_freeze_h_at_ = 0;
  approach_hold_heat_ = false;

  // Pinned ladder trips (start/stop/idle). The grid trips (gear n↔n+1) are read directly from
  // cool_up_[]/cool_dn_[] (built from spacing in build_ladders_()) by the generalized selection below.
  const float C_UP_01 = cool_start_, C_DN_10 = cool_stop_, C_IDLE = cool_idle_;

  update_furrion_setpoint_(false);
  float target = get_cool_target_();
  if (isnan(target)) {
    ESP_LOGW(TAG, "Cool target NaN — holding gear/CS");
    user_changed_ = user_input;  // don't consume a user event during a NaN hold
    resume_from_test_ = from_test;  // ditto: preserve the test-exit re-pick across the hold
    publish_debug_state_(NAN);
    return true;
  }
  float diff = room - target;
  gear_diff = diff;  // debug always reports REAL (unbiased) diff

  // Setpoint-transition detector — standard name: REFERENCE FEEDFORWARD (two-degree-of-freedom
  // control / setpoint weighting; Åström & Hägglund). Compare the
  // COMMITTED target against the last committed one; skip while a debounce is in flight so a
  // mid-scroll intermediate never preloads (the settled pass then sees the full delta once).
  // A NAN baseline (boot restore, mode re-entry after the !do_cool clear, failsafe, test session)
  // records only — preload requires a real prior committed target within this control episode.
  // On a user SP DROP with real cooling demand (room above the NEW target by more than the
  // deadband): pre-load the integral with a fraction of the drop. Trip data 2026-07-30..08-02:
  // all 3 nights needed Δbias ≈ +0.3 °C per °C of drop; the factor is set BELOW that so
  // under-prediction — which the integral tops up at the ordinary rate — is the failure mode,
  // never overshoot. The demand gate stops a drop-while-already-satisfied (room at/below the new
  // SP) from banking bias that would spuriously trip the eff_diff-based 0→1 re-engage.
  // On a RAISE: retain the bias — and if the raise REMOVES demand (room now below the new target
  // beyond the deadband → the unit will idle down / go OFF and park below SP), FREEZE it until
  // the crossing (Stephen 2026-08-14; see raise_freeze_c_at_). "Retain untouched" alone (the
  // 08-10 design: Sat AM outperformed Fri AM because the bias survived the raise) only held while
  // the controller stayed engaged: a ≥~3°F raise routes through the past-SP OFF door, and the
  // 08-14 live raise (64→69) drained 1.51→1.09 through blind-idle decay + the post-clamp e<0
  // stretch before re-entry — the load hadn't changed, and the drained bias cost a ~1h integral
  // re-climb + 1.2°F overshoot. A demand-KEEPING raise (room still above the new target) arms
  // nothing — normal integration continues uninterrupted.
  // Accepted (bounded) edge: separately-committed drop→raise→drop re-preloads each drop (raises
  // never unload), so an oops-toggle can double a kick — clamped at ±ADAPT_BIAS_C_MAX and unwound
  // by the never-frozen e<0 path (a PRELOADING drop mid-freeze also releases the freeze on the
  // same pass — its demand gate diff > +deadband implies the crossing test; only a sub-deadband
  // drop leaves the freeze armed against the new target). NOTE: in HEAT_COOL, a both-gears-off
  // deadband idle wipes this baseline each pass (!do_cool clear), so a drop issued across such a
  // gap records-only; drops during an engaged cool episode preload normally, as in pure COOL.
  if (!setpoint_pending_) {
    if (isnan(last_committed_cool_target_c_)) {
      last_committed_cool_target_c_ = target;
    } else if (target != last_committed_cool_target_c_) {
      float delta_c = target - last_committed_cool_target_c_;
      last_committed_cool_target_c_ = target;
      if (delta_c < 0.0f && adaptive_enable_ && sp_preload_factor_ > 0.0f &&
          diff > ADAPT_DEADBAND_C) {
        float preload = sp_preload_factor_ * (-delta_c);
        bias_c_ += preload;
        if (bias_c_ > ADAPT_BIAS_C_MAX) bias_c_ = ADAPT_BIAS_C_MAX;
        if (bias_c_ < -ADAPT_BIAS_C_MAX) bias_c_ = -ADAPT_BIAS_C_MAX;
        ESP_LOGI(TAG, "Cool SP drop %.2fC: bias preloaded +%.2f -> %.2fC", -delta_c, preload, bias_c_);
      } else if (delta_c > 0.0f && adaptive_enable_ && diff < -ADAPT_DEADBAND_C) {
        // Keep the FIRST arm time on a re-raise mid-freeze (bug-check round 1): the horizon
        // bounds the age of the frozen bias MEASUREMENT, which repeated raises don't refresh —
        // an unconditional restamp would let 1°F nudges extend the freeze indefinitely.
        if (raise_freeze_c_at_ == 0) raise_freeze_c_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Cool SP raise %.2fC: bias %.2f frozen until crossing (cap %lu min)",
                 delta_c, bias_c_, (unsigned long)(RAISE_FREEZE_MAX_MS / 60000UL));
      }
    }
  }

  // Phase 2 adaptive: advance the integral and get the effective diff used ONLY for the
  // active-gear switch cases (1-5). Re-engage/idle/mode-switch decisions stay on real diff.
  // Called every cool pass so the integral advances (or decays while idle) consistently.
  float eff_diff = adaptive_cool_eff_diff_(diff, now, time_in_gear);
  // Upshift comparisons use the rate-gated diff (bias removed while the room isn't warming);
  // downshifts and re-engage/idle decisions keep using eff_diff / real diff respectively.
  float up_diff = cool_eff_up_diff_;
  int gear = cool_gear_;
  int new_gear = gear;

  bool approach_engaged_this_pass = false;  // set by the two engagement sites below

  // Approach-hold maintenance (early gear-1 engagement below SP; see approach_predict_cool_).
  // Re-validated every pass from LIVE signals — no event wiring: (a) crossing (room within the
  // deadband of SP, from below) completes the mission and hands the ladder over; (b) drift dying
  // or going stale aborts with a retry cooldown (transient gust in storage → one quiet stand-down);
  // (c) the recomputed crossing receding past 2× lead aborts (SP moved away / cloud edge);
  // (d) a 3× lead hard cap bounds a mispredicted hold. After an abort the normal selection
  // downshifts 1→0→-1 naturally. The adaptive integral is FROZEN while a hold is active (see
  // adaptive_cool_eff_diff_) — gear 1 deliberately running below SP would otherwise read as
  // overcooling and rapidly unwind the retained bias the coming day regime needs.
  if (approach_hold_cool_ && approach_hold_from_off_) {
    // Atomic clamp commitment (Stephen 2026-08-06): an OFF-fired approach IS its OFF-entry clamp —
    // once committed, no maintenance exits. A mid-clamp release would unfreeze the integral and
    // unpin the ladder while the maneuver still holds the via-CS blast; with a wound bias, eff can
    // sit deep inside an active band at release, stacking a genuine gear-2+ start onto the clamp
    // (the gear-3/4-grade burst this rule exists to prevent). The hold lives exactly as long as
    // the maneuver; when the clamp ends, THIS pass becomes the single decision point — the ladder
    // takes whatever gear eff calls for, or drops to idle. No retry cooldown: a completed clamp is
    // not an abort, and the idle-side approach must be immediately eligible ("back to idle, ready
    // for an idle→1 firing"). Machine supersedes (user tap / mode switch / failsafe / test) still
    // clear the hold through their own paths, unchanged. If the engagement found no matching
    // OFF-entry quirk (kickstart never started), this releases on the next pass — plain start.
    if (!kickstart_active_()) {
      approach_hold_cool_ = false;
      approach_hold_from_off_ = false;
      ESP_LOGI(TAG, "Approach (cool): OFF-entry clamp complete — handing off to ladder");
      // Handover preload from the engagement drift snapshot (2026-08-14 — the former MIN_HOLD
      // gate guarded against sampling engage-time flux at the handover pass; the snapshot IS the
      // engage-time signal now, deliberately, so the gate is gone). eff_diff is patched with the
      // delta so THIS pass — the single decision point — selects on the floored bias
      // (downshift/hold legs; up_diff untouched: upshifts wait a pass, dwell-gated anyway).
      // While a raise freeze is armed, eff is bias-BLIND — the delta updates only the STORED
      // bias (live at the crossing release) and must not leak into this pass's blind eff.
      {
        float pd = apply_crossing_preload_(false, now);
        if (raise_freeze_c_at_ == 0) eff_diff += pd;
      }
    }
  } else if (approach_hold_cool_) {
    bool drift_fresh = (last_temp_update_ != 0) && (now - last_temp_update_ <= DRIFT_STALE_MS);
    float mins = (room_drift_cpm_ > 0.0f) ? (-diff) / room_drift_cpm_ : 1e9f;
    if (eff_diff >= C_DN_10) {
      // Release at the ladder's OWN gear-1 hold boundary (eff-based, like the cool 1→0 rail):
      // the handover lands exactly where the ladder would keep gear 1 running — no release-pass
      // downshift, no extra compressor cycle, and no minimum-bias floor for a seamless handoff
      // (bug-check round-1: releasing at the raw deadband dropped 1→0 whenever bias < ~0.3).
      approach_hold_cool_ = false;
      ESP_LOGI(TAG, "Approach (cool): reached ladder band — hold released, ladder takes over");
      // Handover preload from the engagement drift snapshot (2026-08-14). eff_diff patched for
      // the same-pass decision; the release condition already fired on the old value. Bias-blind
      // guard mirrors the atomic site (frozen bias = stored, not live).
      {
        float pd = apply_crossing_preload_(false, now);
        if (raise_freeze_c_at_ == 0) eff_diff += pd;
      }
    } else if (!drift_fresh || !(room_drift_cpm_ > 0.0f) ||
               mins * 60000.0f > 2.0f * (float) approach_lead_ms_ ||
               (now - approach_started_at_) / 3 >= approach_lead_ms_) {
      approach_hold_cool_ = false;
      approach_abort_at_ = (now != 0) ? now : 1;  // 0 is the "no cooldown" sentinel
      ESP_LOGI(TAG, "Approach (cool): aborted (drift died/receded or hold cap) — standing down");
    }
  }

  // Generalized N-gear selection — standard name: STAGE SEQUENCER (staging differentials +
  // minimum dwell). (design_gear_engine_v2.) M = highest configured gear. Grid trips
  // come from cool_up_[]/cool_dn_[]; the 0↔1 boundary + idle are the pinned C_UP_01/C_DN_10/C_IDLE.
  int M = cool_max_gear_;
  auto entry_thresh = [&](int g) -> float { return (g <= 1) ? C_UP_01 : cool_up_[g - 1]; };
  auto dn_thresh    = [&](int g) -> float { return (g <= 1) ? C_DN_10 : cool_dn_[g - 1]; };
  // Highest gear whose from-below entry threshold `d` clears (0 = none).
  auto pick_from_below = [&](float d) -> int {
    for (int g = M; g >= 1; g--) if (d > entry_thresh(g)) return g;
    return 0;
  };

  if (gear == -1 || user_input) {
    // Fresh start from -1 requires the 1-min off lockout (hardware wind-down); no bypass.
    bool off_long_enough = (off_since_ == 0) || (now - off_since_ >= mode_switch_off_ms_);
    if (gear == -1 && !off_long_enough) {
      new_gear = -1;  // still in 1-min wind-down period
    } else if (user_input && gear >= 0 && gear_in_band_cool_(gear, diff)) {
      // User event but the current gear is still valid for the diff — preserve hunting state.
      new_gear = gear;
    } else {
      // From -1: floor at the derived cold-start floor (compressor can't cold-start below it) and
      // never gear 0 (only reachable by downshift from 1). Running (user_input, gear>=0): no floor.
      // Test-exit (from_test): pick on eff_diff so the wound-up bias lands the load-justified gear
      // instead of a real-diff drop (real diff can be ~0 while the load needs a top gear). Genuine
      // user events keep the REAL-diff pick (fresh target — the reverted Round-1/2 bias-aware pick).
      int picked = pick_from_below(from_test ? eff_diff : diff);
      if (picked >= 1) {
        new_gear = picked;
        if (gear == -1 && new_gear < cool_cold_start_floor_) new_gear = cool_cold_start_floor_;
      } else if (gear == -1 && approach_predict_cool_(diff, now, approach_off_lead_ms_())) {
        // Approach-side early engagement from OFF — through the NORMAL OFF-entry clamp, like every
        // other cold start. HARD RULE (Stephen, 2026-08-03, after a live failure): kickstart quirks
        // are REQUIRED for all OFF→gear changes INCLUDING approach entries. The 2026-07-07 CS
        // characterization is unambiguous: from-OFF compressor start needs CS ≥ SP+1 (from-idle
        // SP+0), and any start-capable CS saturates output upward — there is NO gentle cold start
        // on this hardware. A brief "direct entry" variant (mode-on at gear-1's SP−1 CS) shipped
        // 2026-08-03 and simply never started the compressor. The clamp's ~5-min burst below SP is
        // accepted BECAUSE it is timed to land at the crossing: this site fires on the OFF-entry
        // lead (~clamp duration + latency, see approach_off_lead_ms_), so the blast completes right
        // as the room reaches SP — not minutes-of-margin early (2026-08-06 12:54: the shared 15-min
        // lead put the blast at 66.4°F on a 69°F SP and arrested the climb occupant-noticeably).
        // The off-dwell lockout was enforced above.
        new_gear = cool_cold_start_floor_;
        approach_hold_cool_ = true;
        approach_hold_from_off_ = true;   // atomic clamp commitment — see hold maintenance
        approach_engaged_this_pass = true;
        approach_started_at_ = now;
        // Snapshot the free-rise drift for the handover preload — approach_predict_cool_ just
        // validated it fresh and positive; the clamp will suppress the live value from here.
        approach_entry_drift_cpm_ = room_drift_cpm_;
        approach_entry_drift_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Approach (cool): crossing predicted within lead — gear-%d engagement via clamp",
                 new_gear);
      } else if (gear == -1) {
        new_gear = -1;                               // stays off
      } else if (user_input && diff < C_IDLE) {
        new_gear = -1;                               // user tap past setpoint → off
      } else {
        new_gear = 0;
      }
    }
    // NOTE: this fresh-start/user block deliberately uses REAL diff, NOT eff_diff — the adaptive
    // bias governs only the steady-state active-gear cases below (keeps the path bit-identical to
    // the non-adaptive ladder; Round-1/2 attempts to make it bias-aware regressed, reverted Round-3).
  } else {
    if (gear == 0) {
      // First compute post-restore: jump straight to the correct gear (skip the HOLD_MS ladder).
      if (last_gear_change_ == 0) {
        new_gear = pick_from_below(diff);
      } else if (can_upshift_to(1) && eff_diff > C_UP_01) {
        // iter-1 #2 (2026-07-20): re-engage 0→1 on eff_diff (real + bias), not real diff, so the
        // integral shifts the WHOLE ladder together — the 1→2 upshift already trips on eff_diff, so
        // matching the 0→1 entry to it stops the bias from inverting gear 1's narrow band overnight.
        // Made safe by iter-1 #4 (fast unwind → bias won't sit wound while the room is at SP). The
        // 0→-1 off-decision below deliberately stays on REAL diff (failover / don't cool below SP on
        // a stale bias). Summer/cool-only; revisit for heat↔cool mode hunting before fall.
        new_gear = 1;
      } else if (can_upshift_to(1) && approach_predict_cool_(diff, now, approach_lead_ms_)) {
        // Approach-side early engagement from idle. The idle→1 quick-kick quirk (CS=SP+0) is
        // harmless here (CS above the below-SP room → fan only), so it is NOT bypassed.
        new_gear = 1;
        approach_hold_cool_ = true;
        approach_hold_from_off_ = false;  // idle-fired hold — normal 4-exit maintenance
        approach_engaged_this_pass = true;
        approach_started_at_ = now;
        // Snapshot the free-rise drift for the handover preload (gear 0 is fan-only, so the
        // trailing slope here is compressor-free too).
        approach_entry_drift_cpm_ = room_drift_cpm_;
        approach_entry_drift_at_ = (now != 0) ? now : 1;
        ESP_LOGI(TAG, "Approach (cool): crossing predicted within lead — early gear-1 from idle");
      }
      // 0→-1 gate (natural path only — user_input handled above). A predicted approach also
      // suppresses the natural drop to full-off (going -1 only to restart within the lead time
      // wastes an off/on cycle); imm_off (!boot_ready_) is deliberately NOT suppressible.
      bool imm_off = !boot_ready_;
      bool idle_enough = (idle_since_ > 0) && (now - idle_since_ >= mode_switch_idle_ms_);
      bool event_ok = (last_mode_event_at_ == 0) || (now - last_mode_event_at_ >= mode_switch_event_ms_);
      bool past_setpoint = diff < -mode_switch_temp_offset_c_;
      // Natural-off gate (incident 2026-08-05, see NATURAL_OFF_BIAS_EPS_C): full-off waits for
      // the integral to be unwound — a wound bias marks the sub-SP room as self-inflicted
      // integral lag; hold at idle while the τ=180 idle decay fades it. No drift leg (Stephen
      // 2026-08-06): a parked or even recovering room below SP−offset with an unwound bias goes
      // properly OFF. A negative bias (over-satisfied) also counts as unwound. imm_off and the
      // user-tap-past-SP door above are deliberately NOT gated.
      // HEAT_COOL handoff door (bug-check 2026-08-06 CRITICAL): 0→−1 is the ONLY road to the
      // other mode (arbitrate_mode_ pins do_heat while cool_gear_ >= 0), so when the room has
      // fallen into heat's engagement territory the bias/drift legs MUST NOT stand in the way —
      // else cool idles for hours (bias decay) or forever (room plateaus sub-floor) while the
      // room goes arbitrarily cold with heat locked out. NaN heat target compares false → door
      // stays closed in degenerate states.
      bool handoff_demand = (this->mode == climate::CLIMATE_MODE_HEAT_COOL) &&
                            (room <= get_heat_target_() + mode_switch_temp_offset_c_);
      // A raise-FROZEN bias counts as unwound (bug-check round 1): the freeze marks the sub-SP
      // room as user-inflicted (the raise), the bias is stored-not-live, and cooling below a
      // raised SP is pointless — go properly OFF; the OFF-entry approach handles re-entry.
      bool bias_unwound = bias_c_ <= NATURAL_OFF_BIAS_EPS_C || raise_freeze_c_at_ != 0;
      bool natural_off = idle_enough && event_ok && past_setpoint &&
                         (handoff_demand || bias_unwound) &&
                         !approach_predict_cool_(diff, now, approach_lead_ms_);
      if ((imm_off || natural_off) && diff < C_IDLE) new_gear = -1;
    } else {
      // Active gears 1..M select on eff_diff (= diff + adaptive bias + fan feedforward); upshifts
      // use the rate-gated up_diff. eff_diff == up_diff == diff when adaptive is off → bit-identical
      // to the static ladder. Upshift trip = cool_up_[gear]; downshift trip = dn_thresh(gear).
      if (gear < M && can_upshift_to(gear + 1) && up_diff > cool_up_[gear] &&
          !(approach_hold_cool_ &&
            (gear == 1 || (approach_hold_from_off_ && gear == cool_cold_start_floor_)))) {
        // Upshift suppressed during an approach hold: the room is still below SP there, and
        // up_diff carries the retained (frozen) bias — letting it climb would run gear 2+
        // actively cooling below setpoint off exactly the bias the freeze protects
        // (bug-check round-1 BUG). The hold pins gear 1 in BOTH directions; the maintenance
        // block owns every exit.
        new_gear = gear + 1;
      } else if (eff_diff < dn_thresh(gear) &&
                 !(approach_hold_cool_ &&
                   (gear == 1 || (approach_hold_from_off_ && gear == cool_cold_start_floor_)))) {
        new_gear = gear - 1;
      }
    }
  }

  // Track idle_since
  if (new_gear == 0 && gear != 0) {
    idle_since_ = now;
  } else if (new_gear != 0) {
    idle_since_ = 0;
  }

  // Track off_since_ for the 1-min off lockout
  if (new_gear == -1 && gear != -1) {
    off_since_ = now;
  }

  // Publish gear and compressor on change
  if (new_gear != gear) {
    cool_gear_ = new_gear;
    last_gear_change_ = now;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(new_gear);
    if (new_gear >= 1) last_active_mode_ = MODE_COOL;
    if (compressor_output_sensor_)
      compressor_output_sensor_->publish_state(gear_output_pct(new_gear, cool_max_gear_));
    ESP_LOGI(TAG, "COOL %d -> %d (room=%.2f target=%.2f diff=%.2f)",
             gear, new_gear, room, target, diff);
  }

  // CS + maneuver (quirk) logic — unified. A matching quirk (including an OFF→gear clamped start,
  // from_gear == -1) runs the maneuver engine; otherwise set the gear CS directly. Default cool
  // quirks: OFF→MED clamp (SP+1, fan=LOW), idle→LOW (SP+0 hold), MAX→MED dip / 3→2 fan-clamp.
  // Any OFF→gear (gear == -1) is a fresh-start event → stamp last_mode_event_at_.
  // Selection-exit cleanup (bug-check round-1): if the pass moved the gear anywhere other than
  // the held gear (user re-pick to -1/0, etc.), the hold no longer describes reality — release it
  // plainly (no retry cooldown: a user action superseded it, and the deadband fire-gate prevents
  // an instant spurious re-fire). The engagement pass itself is exempt.
  if (approach_hold_cool_ && !approach_engaged_this_pass && new_gear != 1 &&
      !(approach_hold_from_off_ && new_gear == cool_cold_start_floor_)) {
    // (floor exemption gated on the atomic flag: an OFF-fired hold's entry gear is the cold-start
    // floor (may be >1) and must not self-release; an IDLE-fired hold moved to the floor gear by a
    // user re-pick must still release exactly as before the split — bug-check 2026-08-06.)
    approach_hold_cool_ = false;
    approach_hold_from_off_ = false;
    ESP_LOGI(TAG, "Approach (cool): selection moved gear to %d — hold released", new_gear);
  }

  if (new_gear >= 0) {
    int cs = compute_gear_cs_(false, new_gear);
    if (!kickstart_active_()) {
      if (gear == -1) last_mode_event_at_ = now;
      const QuirkDef *q = find_quirk_(false, gear, new_gear);
      if (q != nullptr) {
        start_maneuver_(q, now);
      } else if (current_cs_ != cs) {
        set_cs_value_(cs, now);
      }
    }
  }

  // HVAC on/off
  if (new_gear >= 0 && active_ir_mode_ != climate::CLIMATE_MODE_COOL && !kickstart_active_()) {
    set_active_ir_mode_(climate::CLIMATE_MODE_COOL);
    transmit_mode_with_cs_();
  }
  if (new_gear == -1 && active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    if (kickstart_active_()) end_maneuver_(now);
  }

  // Apply a per-gear commanded fan (e.g. gear 3 = med → gear 4 = high, same CS): emit a mode frame
  // if the settled gear's fan differs from what's on the wire. No-op during a maneuver / when OFF /
  // in v1-style configs (no gear sets a fan). Runs after the mode-on above so a fresh start's fan
  // is already correct (this then no-ops).
  maybe_apply_gear_fan_(now);

  update_action_();
  return false;
}

// NEITHER-active pass: ensure the unit is OFF and both gears are -1.
void FurrionChillCube::run_idle_mode_(uint32_t now) {
  // Ensure HVAC is OFF
  if (active_ir_mode_ != climate::CLIMATE_MODE_OFF) {
    set_active_ir_mode_(climate::CLIMATE_MODE_OFF);
    transmit_mode_command_();
    maneuver_phase_ = ManeuverPhase::IDLE;
    if (compressor_output_sensor_) compressor_output_sensor_->publish_state(0.0f);
    // Start the absolute 60s off-lockout. This is the user-OFF (or both-off) path that
    // turns the unit off WITHOUT going through run_heat/cool_mode_'s -1 transition, so it
    // is the one active→OFF route that didn't stamp off_since_. Without this, heat→OFF→cool
    // (or any mode→OFF→mode) re-engaged immediately. The off_long_enough gate in the
    // fresh-start paths then holds ANY re-engage for mode_switch_off_ms_. Reboot is exempt:
    // setup() restores to gear 0 in the saved mode and never reaches here.
    off_since_ = now;
    ESP_LOGI(TAG, "NEITHER ACTIVE — HVAC OFF (60s lockout armed)");
  }

  // Enforce neutral CS (at setpoint — unit is OFF so doesn't matter much)
  if (current_cs_ != furrion_setpoint_c_) {
    set_cs_value_(furrion_setpoint_c_, now);
  }

  // Force gears to -1
  if (heat_gear_ != -1) {
    ESP_LOGI(TAG, "HEAT %d -> -1 (neither active)", heat_gear_);
    heat_gear_ = -1;
    if (heat_gear_sensor_) heat_gear_sensor_->publish_state(-1);
  }
  if (cool_gear_ != -1) {
    ESP_LOGI(TAG, "COOL %d -> -1 (neither active)", cool_gear_);
    cool_gear_ = -1;
    if (cool_gear_sensor_) cool_gear_sensor_->publish_state(-1);
  }
  idle_since_ = 0;

  update_action_();
}

// ============================================================
// Debug State Publishing
// ============================================================

void FurrionChillCube::publish_debug_state_(float diff) {
  // Skip if no debug sensors registered
  if (!debug_active_ir_mode_sensor_) return;

  uint32_t now = millis();

  // Helper: only publish if value changed (reduces HA state churn)
  auto pub = [](sensor::Sensor *s, float val) {
    if (s && (isnan(val) != isnan(s->state) || (!isnan(val) && val != s->state))) {
      s->publish_state(val);
    }
  };

  // Active IR mode: 0=OFF, 1=HEAT, 2=COOL
  float ir_mode = (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ? 1.0f :
                  (active_ir_mode_ == climate::CLIMATE_MODE_COOL) ? 2.0f : 0.0f;
  pub(debug_active_ir_mode_sensor_, ir_mode);
  pub(debug_last_active_mode_sensor_, (float)last_active_mode_);

  // 0 idle · 1 PRE_CS · 2 OFF-start clamp HOLD · 10 running-unit maneuver HOLD
  float phase = 0.0f;
  if (maneuver_phase_ == ManeuverPhase::PRE_CS) phase = 1.0f;
  else if (maneuver_phase_ == ManeuverPhase::HOLD) phase = (maneuver_from_gear_ == -1) ? 2.0f : 10.0f;
  pub(debug_kick_phase_sensor_, phase);

  pub(debug_gear_diff_sensor_, diff);

  float tig = (last_gear_change_ == 0) ? -1.0f : (float)((now - last_gear_change_) / 1000);
  pub(debug_time_in_gear_sensor_, tig);

  float idle = (idle_since_ == 0) ? -1.0f : (float)((now - idle_since_) / 1000);
  pub(debug_idle_duration_sensor_, idle);

  // Debug "mode_switch_cooldown" sensor now reflects off_since_ 1-min lockout
  float cd = 0.0f;
  if (off_since_ > 0) {
    uint32_t elapsed = now - off_since_;
    cd = (elapsed < mode_switch_off_ms_) ? (float)((mode_switch_off_ms_ - elapsed) / 1000) : 0.0f;
  }
  pub(debug_mode_switch_cooldown_sensor_, cd);

  float clamp = 0.0f;
  if (maneuver_phase_ == ManeuverPhase::HOLD && maneuver_via_fan_ >= 0 && maneuver_start_ > 0) {
    uint32_t elapsed = now - maneuver_start_;
    clamp = (elapsed < maneuver_duration_ms_) ? (float)((maneuver_duration_ms_ - elapsed) / 1000) : 0.0f;
  }
  pub(debug_fan_clamp_remaining_sensor_, clamp);

  pub(debug_heater_locked_out_sensor_, heater_locked_out_ ? 1.0f : 0.0f);
  pub(debug_failsafe_active_sensor_, failsafe_active_ ? 1.0f : 0.0f);
  pub(debug_boot_ready_sensor_, boot_ready_ ? 1.0f : 0.0f);

  // Phase 2 adaptive observability
  // Publish the ACTIVE mode's adaptive bias (the inactive mode's is held at 0). During heat this
  // shows bias_h_; during cool, bias_c_ — so the one debug sensor tracks whichever loop is live.
  pub(debug_adaptive_bias_c_sensor_,
      (active_ir_mode_ == climate::CLIMATE_MODE_HEAT) ? bias_h_ : bias_c_);
  pub(debug_room_drift_sensor_, room_drift_cpm_);
  pub(debug_fan_feedforward_sensor_,
      (adaptive_enable_ && vent_fan_on_()) ? (float)fan_feedforward_gears_ : 0.0f);

  // Effective (last-transmitted) fan, normalized to the config `fan:` convention:
  // -1 off / nothing transmitted · 0 auto · 1 low · 2 med · 3 high. Sourced from last_tx_fan_
  // (the exact ClimateFanMode enum put on the last mode frame), so it reflects the CONTROLLER-driven
  // fan — including a per-gear fan (g3=med/g4=high) or a maneuver via_fan clamp — not the HA fan entity.
  float eff_fan;
  switch (last_tx_fan_) {
    case climate::CLIMATE_FAN_AUTO:   eff_fan = 0.0f; break;
    case climate::CLIMATE_FAN_LOW:    eff_fan = 1.0f; break;
    case climate::CLIMATE_FAN_MEDIUM: eff_fan = 2.0f; break;
    case climate::CLIMATE_FAN_HIGH:   eff_fan = 3.0f; break;
    default:                          eff_fan = -1.0f; break;   // -1 (OFF) or an unmapped mode
  }
  pub(debug_effective_fan_sensor_, eff_fan);
}

// ============================================================
// Diagnostics
// ============================================================

// ============================================================
// Bench Test Harness Hooks (active only when test_mode_ is set)
// ============================================================

void FurrionChillCube::set_test_mode(bool t) {
  if (test_mode_ && !t) {
    // Resuming production: clear the fan override and force a gear pass next loop so the unit's
    // setpoint/CS re-anchor to the real HA target (failover restored — project_failover_invariant).
    test_fan_ = -1;
    user_changed_ = true;
    resume_from_test_ = true;   // land on the bias-justified gear (eff_diff pick), not a real-diff drop
    last_gear_run_ = 0;
    // Test sequences move the room themselves (overcool steps) — that travel is machine-made, not
    // free, so it must not arm the approach displacement gate on exit. Same rule as the sensor-NaN
    // reset: never measure travel across a discontinuity. First fresh sample re-seeds the trough.
    arm_ring_reset_();
    ESP_LOGI(TAG, "TEST mode OFF — resuming production controller (will re-anchor next pass)");
  } else if (!test_mode_ && t) {
    mode_resend_pending_ = false;  // drop a pending reinforcement — stale late resends never cross a test session
    // test_frame rewrites target_temperature_high/low; a surviving baseline would read the
    // test-vs-real difference as a user SP change on exit and spuriously preload. Reset both
    // (first post-test pass records a fresh baseline). Bench note: the post-test pass records the
    // TEST setpoint, so the user's later restore of the real SP reads as a genuine drop and (with
    // demand) preloads onto the surviving §9d bias — bounded by the clamp, bench-only, accepted.
    last_committed_cool_target_c_ = NAN;
    last_committed_heat_target_c_ = NAN;
    raise_freeze_c_at_ = 0;   // raise freezes are SP-transition state — die with the baselines
    raise_freeze_h_at_ = 0;
    approach_hold_cool_ = false;
    approach_hold_heat_ = false;
    approach_hold_from_off_ = false;
    approach_entry_drift_cpm_ = NAN;
    approach_entry_drift_at_ = 0;
    ESP_LOGI(TAG, "TEST mode ON — production controller suspended");
  }
  test_mode_ = t;
}

// Send one full test frame. mode 0=OFF, 1=COOL, 2=HEAT; fan 0=AUTO,1=LOW,2=MED,3=HIGH (-1=AUTO).
void FurrionChillCube::test_frame(int mode, int setpoint_c, int cs, int fan) {
  test_mode_ = true;          // a test frame always implies test mode
  mode_resend_pending_ = false;  // bypasses set_test_mode() — drop a pending production reinforcement here too
  last_committed_cool_target_c_ = NAN;  // bypasses set_test_mode() — reset SP-transition state here too
  last_committed_heat_target_c_ = NAN;
  raise_freeze_c_at_ = 0;
  raise_freeze_h_at_ = 0;
  approach_hold_cool_ = false;
  approach_hold_heat_ = false;
  approach_hold_from_off_ = false;
  approach_entry_drift_cpm_ = NAN;
  approach_entry_drift_at_ = 0;
  failsafe_active_ = false;
  boot_ready_ = true;
  test_fan_ = fan;
  if (mode == 0) {
    active_ir_mode_ = climate::CLIMATE_MODE_OFF;
  } else if (mode == 2) {
    active_ir_mode_ = climate::CLIMATE_MODE_HEAT;
    furrion_setpoint_c_ = setpoint_c;
    this->target_temperature_low = (float) setpoint_c;   // keep the HA card consistent
  } else {
    active_ir_mode_ = climate::CLIMATE_MODE_COOL;
    furrion_setpoint_c_ = setpoint_c;
    this->target_temperature_high = (float) setpoint_c;
  }
  current_cs_ = cs;
  if (active_ir_mode_ == climate::CLIMATE_MODE_OFF) {
    transmit_mode_command_();
  } else {
    transmit_mode_with_cs_();   // CS → MODE/setpoint → CS bracket
  }
  last_cs_heartbeat_ = millis();
  if (cs_value_sensor_) cs_value_sensor_->publish_state(current_cs_);
  ESP_LOGI(TAG, "TEST frame: mode=%d sp=%dC cs=%d fan=%d", mode, setpoint_c, cs, fan);
}

// Keep-alive tick: re-assert the current CS (no-op when the held mode is OFF).
void FurrionChillCube::test_resend_cs() {
  transmit_cs_update_();
  last_cs_heartbeat_ = millis();
}

void FurrionChillCube::test_off() {
  test_mode_ = true;
  mode_resend_pending_ = false;  // bypasses set_test_mode() — drop a pending production reinforcement here too
  active_ir_mode_ = climate::CLIMATE_MODE_OFF;
  transmit_mode_command_();
  ESP_LOGI(TAG, "TEST: unit OFF");
}

void FurrionChillCube::dump_config() {
  ESP_LOGCONFIG(TAG, "Furrion Chill Cube:");
  ESP_LOGCONFIG(TAG, "  Outside Lockout: %.1f°F (%.1f°C)",
                outside_lockout_temp_c_ * 9.0f / 5.0f + 32.0f, outside_lockout_temp_c_);
  ESP_LOGCONFIG(TAG, "  Inside Temp Unit: %s", inside_temp_fahrenheit_ ? "°F" : "°C");
  if (outside_temp_sensor_) {
    ESP_LOGCONFIG(TAG, "  Outside Temp Unit: %s", outside_temp_fahrenheit_ ? "°F" : "°C");
  }
  const char *mode_str = (heat_gear_ >= 0) ? "HEAT (restored from prior session)" :
                         (cool_gear_ >= 0) ? "COOL (restored from prior session)" :
                         "none (fresh boot)";
  ESP_LOGCONFIG(TAG, "  Prior Mode: %s", mode_str);
  ESP_LOGCONFIG(TAG, "  Mode-switch off-dwell: %lus", (unsigned long)(mode_switch_off_ms_ / 1000));
  ESP_LOGCONFIG(TAG, "  CS transmit interval: %lus (quirk %lus)",
                (unsigned long)(cs_transmit_interval_ms_ / 1000),
                (unsigned long)(quirk_transmit_interval_ms_ / 1000));
  ESP_LOGCONFIG(TAG, "  Mode resend delay: %.1fs%s", mode_resend_delay_ms_ / 1000.0f,
                mode_resend_delay_ms_ == 0 ? " (disabled)" : "");
  ESP_LOGCONFIG(TAG, "  SP preload factor: %.2f%s", sp_preload_factor_,
                sp_preload_factor_ == 0.0f ? " (off)" : "");
  ESP_LOGCONFIG(TAG, "  Approach lead: %lus%s", (unsigned long)(approach_lead_ms_ / 1000),
                approach_lead_ms_ == 0 ? " (off)" : "");
  ESP_LOGCONFIG(TAG, "  Quirks: %d (default hold %lus)", quirk_count_,
                (unsigned long)(quirk_duration_ms_ / 1000));
  ESP_LOGCONFIG(TAG, "  Cool gears: %d (cold-start floor %d); Heat gears: %d (floor %d)",
                cool_max_gear_ + 1, cool_cold_start_floor_, heat_max_gear_ + 1, heat_cold_start_floor_);
  for (int g = 0; g <= cool_max_gear_ && g < MAX_GEARS; g++)
    ESP_LOGCONFIG(TAG, "    cool gear %d: cs_off=%d fan=%d", g, cool_gear_offset_[g], cool_gear_fan_[g]);
  if (heat_vent_move_delay_ms_ && heat_vent_interval_ms_) {
    ESP_LOGCONFIG(TAG, "  Vane HEAT positioning: wait %lus, move %lus",
                  (unsigned long)(heat_vent_move_delay_ms_ / 1000),
                  (unsigned long)(heat_vent_interval_ms_ / 1000));
  }
  if (cool_vent_move_delay_ms_ && cool_vent_interval_ms_) {
    ESP_LOGCONFIG(TAG, "  Vane COOL positioning: wait %lus, move %lus",
                  (unsigned long)(cool_vent_move_delay_ms_ / 1000),
                  (unsigned long)(cool_vent_interval_ms_ / 1000));
  }
}

}  // namespace furrion_chill_cube
}  // namespace esphome
