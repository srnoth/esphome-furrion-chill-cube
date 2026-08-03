# Architecture — furrion_chill_cube

## One-sentence identity

A **supervisory cascade controller** for a variable-speed heat pump (Furrion Chill Cube) whose
only actuator interface is the unit's **one-way IR remote protocol** — standard staged-equipment
control logic wrapped around a deliberately bespoke actuator layer.

## The standard half (control logic)

The control logic is the classic industrial decomposition for continuous control of a
staged/quantized actuator:

```
                 ┌────────────────────────────────────────────────────────┐
 room sensor ──▶ │ ESTIMATION                                             │
 (via HA)        │  • rate estimate (room_drift_cpm_, 3-min window)       │
                 │  • input-disturbance estimate (bias_c_/bias_h_ —       │
                 │    integral action w/ conditional-integration          │
                 │    anti-windup; idle memory-fade)                      │
                 └──────────────┬─────────────────────────────────────────┘
                                │  demand signal: eff_diff = error + disturbance est.
                                │  (+ measured-disturbance feedforward: vent fan)
                                ▼
                 ┌────────────────────────────────────────────────────────┐
                 │ STAGE SEQUENCER (the "gear ladder")                    │
                 │  • staging differentials (auto-built rungs)            │
                 │  • minimum dwell times (HOLD_MS)                       │
                 │  • equipment-protection interlocks (off-dwell,         │
                 │    idle/event lockouts, low-ambient heat lockout)      │
                 │  Transition aids (event-scoped, standard names):       │
                 │  • reference feedforward on setpoint changes           │
                 │    (sp_preload — 2-DOF control)                        │
                 │  • optimum start (approach_lead — drift-predictive     │
                 │    early engagement)                                   │
                 └──────────────┬─────────────────────────────────────────┘
                                │  gear (stage) command
                                ▼
                 ┌────────────────────────────────────────────────────────┐
                 │ ACTUATOR COMMAND SHAPING  ◀── the bespoke half         │
                 │  • CS detent map per gear (coaxes the unit's internal  │
                 │    modulation; g3/g4 share a detent — fan speed is the │
                 │    compressor lever)                                   │
                 │  • kickstart clamps / transition quirks (path-         │
                 │    dependent maneuver engine)                          │
                 │  • IR frame transmit + one-shot mode-frame             │
                 │    reinforcement + CS heartbeat (no ack channel)       │
                 └────────────────────────────────────────────────────────┘
```

Design invariant (rare in commercial gear, non-negotiable here): **fail-operational degraded
mode** — on any failure (sensor loss, HA disconnect, controller death) the unit must revert
transparently to its own internal thermostat at the user's real setpoint. The supervisory layer
never lies to the unit about the setpoint (no fixed-anchor or offset-spoofing tricks).

## Why not MPC?

A 4-level quantized actuator with minimum-dwell constraints makes true MPC a **hybrid
(mixed-integer) MPC** problem — a research topic, not a standard deployment. The demand-signal +
stage-sequencer split above IS the standard industrial answer for this actuator class. See
`context/camper-hvac/review-mpc-alignment.md` (project repo) for the full analysis and the
planned evolution: system identification (RC plant model) → unified disturbance observer
(absorbs the event-scoped transition aids) → optional shadow-mode enumerated MPC.

## Map of the code

- `components/furrion_chill_cube/furrion_chill_cube.h` — Rosetta table (top of file), config
  surface, state.
- `furrion_chill_cube.cpp` — estimation (`adaptive_*_eff_diff_`, `update_room_drift_`),
  sequencer (`run_cool_mode_`/`run_heat_mode_` selection blocks), transition aids (setpoint
  detectors, `approach_predict_*` + hold blocks), actuator shaping (maneuver engine, transmit
  paths), protection (failsafe, lockouts).
- `climate.py` — YAML schema; every behavior knob is config, defaults reproduce legacy behavior
  bit-identically.
- `tests/` — ⚠️ stale mirrors, not authoritative; real validation is on-device (test harness in
  the production YAML).
