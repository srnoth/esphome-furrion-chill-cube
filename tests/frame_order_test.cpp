// Standalone host test — mirrors apply_gear_frames_() (frame ORDER on a within-setpoint gear change,
// design-frame-ordering-2026-09-05) and script_gear_pick_() (gear-script mode hardware guards).
// Build: g++ -std=c++17 -o tests/frame_order_test tests/frame_order_test.cpp && ./tests/frame_order_test
#include <cstdio>
#include <cstdint>
#include <string>
#include <vector>

static int fails = 0;
static void check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) fails++;
}

// ---- mirror of apply_gear_frames_ ----
enum Fan { AUTO = 0, LOW = 1, MED = 2, HIGH = 3 };
struct Wire {
  int cs;            // current_cs_
  int last_tx_fan;   // last_tx_fan_ (-1 = OFF)
  bool can_tx;       // boot_ready && !failsafe && mode != OFF
  bool sp_pending;   // setpoint_pending_
};
static std::string apply_gear_frames(Wire &w, int new_cs, int new_fan) {
  std::string out;
  bool cs_changed = (new_cs != w.cs);
  w.cs = new_cs;
  bool fan_changed = w.can_tx && !w.sp_pending && (new_fan != w.last_tx_fan);
  if (!w.can_tx) return out;
  auto CS = [&] { out += out.empty() ? "CS" : ",CS"; };
  auto MAIN = [&] { out += out.empty() ? "MAIN" : ",MAIN"; w.last_tx_fan = new_fan; };
  if (!fan_changed) {
    if (cs_changed) CS();
  } else if (!cs_changed) {
    MAIN();
  } else if (new_fan == AUTO) {
    CS(); MAIN(); CS();
  } else {
    MAIN(); CS();
  }
  return out;
}

// ---- mirror of script_gear_pick_ ----
struct Ctl {
  int max_g = 4, floor = 1;
  uint32_t off_since = 0, off_ms = 60000;
};
static int script_pick(const Ctl &c, int script_gear, int gear, uint32_t now) {
  int want = script_gear;
  if (want > c.max_g) want = c.max_g;
  if (gear == -1 && want >= 0) {
    bool off_long_enough = (c.off_since == 0) || (now - c.off_since >= c.off_ms);
    if (!off_long_enough) return -1;
    if (want == 0) return -1;           // OFF never enters idle (refused, logged once)
    if (want < c.floor) want = c.floor;
  }
  return want;
}

int main() {
  printf("apply_gear_frames_ — one ordering rule for bare changes, quirk entries, quirk exits\n");
  {
    // Cool tables exercised: the 09-03 set (g1 SP-1 auto, g2 SP+0 auto, g3 SP+3 med, g4 SP+3 high, via
    // low/SP+3) AND the 09-05 live set (g2 = low/SP+3 fixed: 1->2 bare = the "quirk 1->2 entry" row below,
    // 2->1 bare = the "quirk exit low/SP+3 -> auto" sandwich row, 2->3 = low->med CS same = MAIN only).
    Wire w{19, AUTO, true, false};
    check(apply_gear_frames(w, 20, AUTO) == "CS", "bare 1->2 (fan same): CS only");
    w = {20, AUTO, true, false};
    check(apply_gear_frames(w, 23, MED) == "MAIN,CS", "bare 2->3 (auto->med, CS up): MAIN then CS (was CS then MAIN)");
    w = {23, MED, true, false};
    check(apply_gear_frames(w, 23, HIGH) == "MAIN", "bare 3->4 (CS same): MAIN only");
    w = {23, HIGH, true, false};
    check(apply_gear_frames(w, 23, MED) == "MAIN", "bare 4->3 (CS same): MAIN only");
    w = {20, AUTO, true, false};
    check(apply_gear_frames(w, 19, AUTO) == "CS", "bare 2->1 (fan same): CS only");
    // quirk entries (maneuver HOLD set → effective fan = via_fan)
    w = {19, AUTO, true, false};
    check(apply_gear_frames(w, 23, LOW) == "MAIN,CS", "quirk 1->2 entry (auto->low, CS -1->+3): MAIN then CS");
    w = {23, MED, true, false};
    check(apply_gear_frames(w, 23, LOW) == "MAIN", "quirk 3->2 entry (med->low, CS same): MAIN only");
    w = {23, HIGH, true, false};
    check(apply_gear_frames(w, 23, LOW) == "MAIN", "quirk 4->2 entry (high->low, CS same): MAIN only");
    w = {15, AUTO, true, false};
    check(apply_gear_frames(w, 20, AUTO) == "CS", "quirk idle->1 entry (via_fan unset = gear fan auto): CS only");
    // quirk exits (maneuver cleared → effective fan = gear fan)
    w = {23, LOW, true, false};
    check(apply_gear_frames(w, 20, AUTO) == "CS,MAIN,CS", "quirk exit low/SP+3 -> g2 auto/SP+0: CS, MAIN, CS (sandwich)");
    w = {20, AUTO, true, false};
    check(apply_gear_frames(w, 19, AUTO) == "CS", "quirk idle->1 exit (fan same): CS only (no extra MAIN)");
    w = {23, LOW, true, false};
    check(apply_gear_frames(w, 23, MED) == "MAIN", "escape-up exit low/SP+3 -> g3 med/SP+3: MAIN only");
    w = {23, LOW, true, false};
    check(apply_gear_frames(w, 23, HIGH) == "MAIN", "escape-up exit low/SP+3 -> g4 high/SP+3: MAIN only");
    // nothing changed → nothing sent
    w = {20, AUTO, true, false};
    check(apply_gear_frames(w, 20, AUTO) == "", "no change: no frames");
    // gates
    w = {20, AUTO, false, false};
    check(apply_gear_frames(w, 23, MED) == "" && w.cs == 23, "mode OFF/boot/failsafe: no frames, CS state still set");
    w = {20, AUTO, true, true};
    check(apply_gear_frames(w, 23, MED) == "CS", "setpoint debouncing: MAIN deferred (maybe_apply_gear_fan_ retries), CS only");
    // FAN_ONLY left behind by a fan-only bench step: can_tx must be false (mirrors transmit_cs_update_'s guard) —
    // otherwise the first production pass emits a spurious FAN_ONLY Main (bug-check R1). Modelled as can_tx=false.
    w = {20, HIGH, false, false};
    check(apply_gear_frames(w, 23, MED) == "" && w.cs == 23, "FAN_ONLY residue: no frames (HVAC-on bracket owns the pass)");
    // symmetry: every running entry to g2 is the same wire sequence from 1, 3 and 4 after the MAIN
    Wire a{19, AUTO, true, false}, b{23, MED, true, false}, c{23, HIGH, true, false};
    apply_gear_frames(a, 23, LOW); apply_gear_frames(b, 23, LOW); apply_gear_frames(c, 23, LOW);
    check(a.cs == b.cs && b.cs == c.cs && a.last_tx_fan == LOW && b.last_tx_fan == LOW && c.last_tx_fan == LOW,
          "1/3/4 -> g2 all land at low/SP+3 with the fan on the wire before or with the CS");
  }

  printf("script_gear_pick_ — hardware guards in force under a gear script\n");
  {
    Ctl c;
    check(script_pick(c, 2, 1, 100000) == 2, "running g1, script 2 -> 2 (no dwell gate)");
    check(script_pick(c, 4, 1, 100000) == 4, "running g1, script 4 -> 4 (multi-rung allowed)");
    check(script_pick(c, 0, 3, 100000) == 0, "running g3, script 0 -> idle");
    check(script_pick(c, -1, 2, 100000) == -1, "running g2, script -1 -> OFF (natural-off gates bypassed)");
    check(script_pick(c, 7, 2, 100000) == 4, "script above max clamps to max gear");
    c.off_since = 100000;
    check(script_pick(c, 1, -1, 130000) == -1, "OFF 30 s ago, script 1 -> stays OFF (1-min wind-down)");
    check(script_pick(c, 1, -1, 160000) == 1, "OFF 60 s ago, script 1 -> 1 (the OFF->1 quirk row will run)");
    check(script_pick(c, 0, -1, 160000) == -1, "OFF, script 0 -> refused, stays OFF (OFF never enters idle)");
    check(script_pick(c, 3, -1, 160000) == 3, "OFF, script 3 -> 3 (bare OFF->3 entry, as production)");
    c.floor = 2;
    check(script_pick(c, 1, -1, 160000) == 2, "OFF with cold-start floor 2, script 1 -> 2 (floor is a minimum)");
    c.floor = 1;
    c.off_since = 0;
    check(script_pick(c, 1, -1, 5000) == 1, "OFF with no off_since stamp (boot): allowed");
  }

  printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL PASS", fails);
  return fails ? 1 : 0;
}
