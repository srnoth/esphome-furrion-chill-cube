# Phase 2 — Adaptive Equilibrium-Gear Controller (cool mode)

**Branch:** `phase2-adaptive` · **Status:** design → implementation · **Scope:** COOL mode only (heat unchanged)

This is the spec the iterative bug-check reviews against. It defines exactly what the
adaptive layer does, the invariants it must preserve, and the anti-windup rules.

---

## 1. Problem it solves

The static ladder maps **temperature error → gear** with a *fixed* center. But the gear
that *sustains* a given load (the equilibrium gear) depends on the load, which varies
enormously and is **not observable from outside air temp** (bodies, the CO₂-triggered
fresh-air fan dumping conditioned air, solar gain on the shell). At a fixed ladder center,
the zero-error gear is ~2 regardless of load, so on any non-mild day the ladder under- or
over-cools at center and has to *swing* to compensate — the "cycling up and down through
gears instead of finding the ideal gear to sustain at" complaint. A proportional ladder
holding a high load also runs with a **steady-state offset** (room sits persistently hot,
because the ladder only reaches high gears when error is positive).

The fix is classic **PI control**: add a slow integral term that drives the steady-state
offset to zero by floating the ladder's operating point to wherever today's load sits. The
existing ladder becomes the fast inner (proportional) loop; the integral is the slow outer
loop. No outside-temp model — the room's own response is the universal load sensor.

## 2. Mechanism — dynamic error-bias (`bias_c`)

A single new control state, `bias_c` (°C), shifts the error the **cool gear ladder** sees:

```
eff_diff = diff + bias_c          // diff = room - setpoint (°C, + = too warm)
```

`eff_diff` (not `diff`) feeds the gear up/down threshold comparisons in `run_cool_mode_`.
A positive `bias_c` makes the ladder behave as if the room were warmer → it centers on a
higher (more-cooling) gear. The integral grows `bias_c` while the room runs hot and shrinks
it while the room runs cold, so at equilibrium the room sits **on** setpoint and the inner
ladder does a gentle ±1-gear cycle around the *correct* center — on a mild day or a
five-people-plus-fan day alike.

### Integral law (runs once per cool control pass)
```
dt_min = clamp((now - adaptive_last_advance_)/60000, 0, ADAPT_DT_CAP_MIN)
e = diff                                   // REAL error drives the integral
if |e| < ADAPT_DEADBAND_C: e = 0           // noise/offset deadband
if allow_integrate:
    bias_c += ADAPT_KI * e * dt_min
    bias_c = clamp(bias_c, -ADAPT_BIAS_C_MAX, +ADAPT_BIAS_C_MAX)
else if idle_decay:
    bias_c *= exp(-dt_min / ADAPT_DECAY_TAU_MIN)   // forget stale equilibrium while off
adaptive_last_advance_ = now
```

### Where `eff_diff` is used vs REAL `diff`
- **`eff_diff`** → the active-cooling switch cases 1–5 only (every up *and* down comparison,
  including 1→0), AND the `user_input` preserve-state check `gear_in_band_cool_(gear, eff_diff)`
  (consistent with how the gear was selected). This is the only behavioral change to gear
  selection.
- **REAL `diff`** → (a) the fresh-start/`-1` and case-0 *re-engage* selection — a restart from
  off/idle keys on genuine error, not a possibly-stale bias (which has decayed during idle
  anyway), keeping cold-start conservative and the restart machinery's gear-2 minimum intact;
  (b) the case-0 `past_setpoint` mode-switch-eligibility gate; (c) the `gear_diff` debug value;
  (d) everything outside cool gear selection. Rationale: the adaptive cool bias must never
  influence heat/cool arbitration, the "room is genuinely satisfied" decision, or a cold restart.

### Applying nothing else
`bias_c` changes *only* which gear the ladder picks. The resulting `cool_gear_` flows through
the **unchanged** CS computation, kickstart selection, keepalive, publish, and IR transmit
paths. Going up a gear while already running is always safe (more cooling, no restart
concern); the restart/kickstart machinery already keys on the final `cool_gear_` transition.

## 3. Anti-windup (the load-bearing safety logic)

**Conditional integration.** Accumulation is frozen when ANY of:
1. `!adaptive_enable_` — master switch off (behaves exactly like current main).
2. `kickstart_active_()` — CS is overridden; gear isn't reflecting steady control.
3. `cool_gear_ <= 0` — compressor idle/off; error isn't controllable. (Takes the
   `idle_decay` branch instead — bias_c relaxes toward 0 so a long idle forgets a stale load.)
4. **`block_up`** — POSITIVE accumulation (`e > 0`) is frozen when the gear cannot rise right
   now: at `cool_gear_ >= 5`, OR while an upshift is hold-blocked
   (`time_in_gear < HOLD_MS[cool_gear_+1]`). NEGATIVE accumulation is *never* rail-blocked,
   because gear 0/idle is always reachable (downshifts aren't hold-gated). This is the key
   correctness property: it stops windup behind a held upshift from cascading gears, AND lets a
   stale *positive* bias unwind at gear 1 (a prior naive `gear<=1 && e<0` freeze trapped it,
   causing indefinite overcooling — fixed Round-1).
5. Within `FAN_EDGE_FREEZE_MS` of a *real* vent-fan edge (`vent_fan_sensor_ != null &&
   vent_fan_changed_at_ != 0`) — the transient is handled by feedforward, not the integral.
   The `!= 0` guard prevents a spurious freeze in the first 3 min of every boot.

Failsafe/NaN never reach this code (`run_gear_controller_` returns early). `bias_c` starts
at 0 on boot (converges within ~20–40 min; not persisted in v1) and is cleared whenever
cooling isn't the active mode (including at the top of `run_heat_mode_`, so a NaN-target heat
early-return can't leave a stale cool bias).

## 4. Vent-fan feedforward (optional, observed-disturbance only)

The fresh-air fan is a *fast, large, intermittent* disturbance the slow loop would lag. If a
`vent_fan` binary_sensor is configured, while it reads ON we add a fixed feedforward shift:
```
ff_c = vent_fan_on_ ? (fan_feedforward_gears_ * GEAR_STEP_C) : 0
eff_diff = diff + bias_c + ff_c
```
and freeze the integral for `FAN_EDGE_FREEZE_MS` after each fan edge so the burst doesn't
pollute the learned equilibrium. The integral still tracks any *sustained* residual beyond
what the feedforward covers (e.g. a near-continuous fan in a full camper). This is the one
feedforward worth doing — it's a **directly observed disturbance**, not a load guess from
ambient conditions. Default `fan_feedforward_gears = 1`; needs field tuning.

## 5. Failover invariant (unchanged, must be preserved)

The adaptive layer only selects gear/CS; it transmits nothing new and adds no IR path. On
any failure (ESP death, WiFi/IR loss, NaN, HA disconnect) the existing failsafe still stops
all IR and the unit reverts to its own sensor + the real setpoint (`furrion_setpoint_c_`,
still tracking the user's true target). `bias_c` is internal-only and vanishes on reboot.
Fixed-anchor setpoints remain forbidden. See `project_failover_invariant`.

## 6. New config (climate.py)

| Key | Type | Default | Meaning |
|---|---|---|---|
| `adaptive_enable` | bool | **false** | Master switch (safe, explicit opt-in). False → identical to main. The camper test config sets it `true`. |
| `vent_fan` | binary_sensor id | — | Optional observed-disturbance input (expected to be a *hysteretic* CO₂-threshold sensor — see §9). |
| `fan_feedforward_gears` | int 0–3 | 1 | Gear-equivalents of feedforward while fan on (1 ≈ +0.85 °C, a firm shove). |

Trip-safety / revert paths (defence in depth):
- **Code:** camper YAML pins `source: github://srnoth/esphome-furrion-chill-cube` with no
  `ref:` (= `main`, the stable ladder). Testing Phase 2 = add `ref: phase2-adaptive`;
  reverting = drop the ref and reflash.
- **Runtime:** `adaptive_enable: false` collapses to the exact current ladder with no
  reflash-of-logic needed.

## 7. New debug sensors (debug: true)

| Sensor | Unit | Meaning |
|---|---|---|
| `debug_adaptive_bias_c` | °C | current `bias_c` |
| `debug_room_drift` | °C/min | EMA of dT/dt (observability + future rate-gated upshift) |
| `debug_fan_feedforward` | gears | current `ff_c` expressed in gear-equivalents |

`debug_gear_diff` continues to report **real** diff (unbiased) so traces stay interpretable.

## 8. Constants (tunable; start conservative)

| Const | Value | Note |
|---|---|---|
| `ADAPT_KI` | 0.06 | bias_c (°C) per (°C-error·min). ~1 gear of bias per ~25 min at 1 °F error. **Primary tuning knob.** |
| `ADAPT_BIAS_C_MAX` | 2.0 °C | authority clamp (~2–3 gears); runaway backstop |
| `ADAPT_DEADBAND_C` | 0.15 °C | don't integrate noise/tiny offset |
| `ADAPT_DECAY_TAU_MIN` | 30 min | idle forgetting time-constant |
| `ADAPT_DT_CAP_MIN` | 5 min | clamp dt across stalls/reboots |
| `GEAR_STEP_C` | 0.85 °C | one gear ≈ this much eff_diff (≈ cool C_UP spacing) |
| `FAN_EDGE_FREEZE_MS` | 180000 | integral freeze window after a fan edge |
| `ADAPT_DRIFT_ALPHA` | 0.3 | EMA smoothing for room drift |

## 8b. Known limitations (v1)
- **Vent-fan input must be hysteretic.** `ff_c` tracks the live fan state, and downshifts are
  not hold-gated, so a fan that *chatters* on/off rapidly would ratchet the gear down with IR
  churn (safe — downshift-while-running is CS-only, no compressor restart). A real CO₂-threshold
  fan has natural on/off hysteresis and runs minutes at a time, so this is a non-issue in
  practice; if a noisy relay is ever used, debounce the binary_sensor upstream.

## 9. Out of scope for v1 (future)
- Heat-mode adaptive (heat hunts fine; root-cause-first).
- `bias_c` persistence across reboot (warm-start).
- Rate-gated top-gear upshift (block 3→4 unless still warming) — `debug_room_drift` is the
  groundwork; revisit if recovery overshoot persists.
- Live compressor-CT feedback (needs the dedicated 6-channel channel; would give fast
  restart-loaf detection). Designed to slot in as an *enhancement that degrades to
  temperature-only*, never a dependency.

## 10. Test plan
- **Unit:** `eff_diff` shifts gear selection as specified; anti-windup freezes in each
  listed condition; idle decay relaxes bias_c; fan FF + edge-freeze; real-diff still used
  for the mode-switch gate and debug.
- **Closed-loop sim** (`simulation_test.cpp` extension): mild / hot / hot+fan regimes with
  drift-rate plant; assert adaptive centers on the sustaining gear, holds room near
  setpoint (smaller offset than static), and reduces swing/changes vs the static ladder —
  without winding up during idle or behind the restart lockout.
- **Regression:** with `adaptive_enable=false`, behavior is bit-identical to main.
