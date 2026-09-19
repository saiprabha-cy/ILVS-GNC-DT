# ILVS-GNC-DT: Launch Vehicle Ascent & Insertion GNC Digital Twin

A MATLAB/GNU Octave → Simulink → Embedded Coder digital twin of powered
ascent flight: liftoff through atmospheric flight, closed-loop gravity-turn
pitch guidance, discrete-event staging, and orbital insertion targeting —
with the pitch guidance/control law cross-compiled and cross-validated
through Embedded Coder the same way the sibling repos below cross-validate
their own algorithms.

**Status:** Fully built and confirmed working end-to-end by a real run in
MATLAB R2025a, including successful Embedded Coder code generation and a
passing MATLAB-vs-generated-C cross-validation. See
`docs/02_results_analysis.md` for every number and `docs/04_verification_
note.md` for the three real bugs a first execution found and how each was
fixed.

Related work: [CubeSat-ADCS-Sim](https://github.com/saiprabha-cy/CubeSat-ADCS-Sim)
(attitude control), [kalman-attitude-estimation](https://github.com/saiprabha-cy/kalman-attitude-estimation)
(attitude estimation), [Mission-Operations-Communication-Platform-MOCP](https://github.com/saiprabha-cy/Mission-Operations-Communication-Platform-MOCP)
(comms/ground segment), [aerospace-telemetry-flight-computer-platform](https://github.com/saiprabha-cy/aerospace-telemetry-flight-computer-platform)
(embedded flight computer), [planetary-rover-navigation](https://github.com/saiprabha-cy/planetary-rover-navigation)
(ground mobility). Those five cover a spacecraft that points, talks, or
drives; this one covers the ~200 seconds between "engines light" and "a
payload is on its way to orbit" that none of them touch. See
`docs/01_design_decisions.md` for the full overlap analysis behind that
scoping decision.

---

## Why this project, and why it's scoped this narrowly

Every real launch vehicle solves the same problem this project solves: fly
a thrust- and mass-changing vehicle through a drag-bearing atmosphere,
close a real-time guidance loop that has to reject disturbances, survive a
discrete staging event, and hit a target insertion window — all under
constraints (fixed-step, deterministic execution) that matter because the
guidance law has to run on embedded flight hardware, not just in a MATLAB
script. This project builds and cross-validates that problem from first
principles, and is honest in its own documentation about exactly what has
and hasn't been executed before being handed to you (see
`docs/04_verification_note.md`) — verification and execution are different
claims, and this project doesn't blur them.

## What's actually new here (vs. the sibling repos)

1. **3-DOF translational flight dynamics** through a rotating-frame-free,
   drag-bearing atmosphere — a different dynamics regime than the purely
   orbital or purely attitude problems the sibling repos solve.
2. **Time-varying mass and thrust** (propellant depletion, staging) —
   nothing else in the portfolio models a system whose own mass properties
   change during the simulation.
3. **Closed-loop ascent guidance**, cross-compiled through Embedded Coder
   the same way `kalman-attitude-estimation` cross-compiles its MEKF,
   applied here to a genuinely different domain (ascent guidance, not
   attitude estimation).
4. **Staging as a discrete-event state machine** driving a continuous
   simulation — a different kind of software problem than anything else in
   the portfolio.
5. **Insertion targeting** — a real GNC problem (hit a target
   altitude/velocity/flight-path-angle window at engine cutoff), reported
   honestly even when the vehicle sizing doesn't reach full orbital energy
   (see `docs/03_simplifications.md`).

## Repository structure

```
ILVS-GNC-DT/
├── config/                vehicle, Earth, and simulation-timing parameters
├── src/
│   ├── environment/       atmosphere density, gravity
│   ├── dynamics/          7-state ascent EOM + isolated ballistic check
│   ├── guidance/          open-loop pitch program reference
│   ├── control/           discrete PID (explicit state, Embedded-Coder-ready)
│   ├── staging/           discrete-event staging state machine
│   ├── integration/       generic fixed-step RK4
│   ├── telemetry/         thin, MOCP-compatible packet + CRC-16
│   ├── visualization/     engineering plots, 3D trajectory, animation
│   └── utilities/         path setup
├── simulink/               Simulink-API model-build + Embedded Coder config
├── embedded/               hand-written C PID reference (cross-validation)
├── sim/                    run scripts, one per validation stage
├── tests/                  unit tests
├── docs/                   design decisions, results (template), simplifications,
│                           and a transparent verification-methodology note
├── results/figures/        generated plots (populated when you run the sims)
├── results/animations/     generated animation GIF
└── data/telemetry/         generated telemetry log (.mat)
```

## Confirmed results

| Check | Result |
|---|---|
| Environment validation | PASS — density and gravity match closed-form expectations exactly |
| Ballistic energy conservation | PASS — 4.541e-15 relative error over 1000s / 20,000 RK4 steps |
| Full ascent run (liftoff → MECO) | PASS — stage sep at t=88.40s, MECO at t=255.80s, RMS pitch tracking error 0.058 deg |
| Simulink model build | PASS — `pitch_control_loop.slx` builds cleanly |
| Embedded Coder code generation | PASS — generated, compiled, and linked to a standalone executable |
| MATLAB vs. hand-written C PID | PASS — 8.004e-11 max difference over a 2000-sample realistic sequence |

Full numbers, the complete build log, and three real Simulink/Embedded-Coder
issues found and fixed along the way (persistent-variable sample time,
continuous-time codegen support) are documented in
`docs/02_results_analysis.md` and `docs/04_verification_note.md` — kept as
an honest record of real iteration, not smoothed over.

## Running this project

Requires MATLAB (Simulink + Embedded Coder for the `simulink/` scripts
only) or GNU Octave (for everything under `config/`, `src/`, `sim/`,
`tests/` — the core algorithms are written to be Octave-compatible
throughout).

```matlab
% From the project root:
cd sim
main_environment_check              % fast: atmosphere/gravity sanity checks
main_ballistic_conservation_check   % fast: EOM energy-conservation proof
main_ascent_full_run                % full liftoff-to-MECO run, saves plots+animation
main_embedded_crossvalidation       % MATLAB vs. hand-written C PID, needs a C compiler

cd ../tests
run_all_tests                       % all unit tests
```

For the Simulink/Embedded Coder path (requires those licenses):

```matlab
cd simulink
build_pitch_control_model           % programmatically builds pitch_control_loop.slx
configure_embedded_coder            % configures ERT target, runs rtwbuild
```

**Next steps, if you want to extend this further:** the remaining open
item is the true three-way cross-validation — MATLAB vs. hand-written C
vs. the *actual* Embedded-Coder-generated `pitch_control_loop.c` (current
cross-validation compares against the hand-written C only). See the final
section of `docs/02_results_analysis.md` for two concrete ways to close
that gap.

## Key engineering decisions worth knowing before you dig in

- The control loop updates its internal state once per integration step,
  not once per RK4 sub-stage — a deliberate zero-order-hold choice, not an
  accident. See `docs/01_design_decisions.md`.
- The vertical-rise phase at liftoff is handled as an explicit special
  case (not an epsilon-hack) to avoid a real 1/V singularity in the
  flight-path-angle equation.
- Telemetry packets use the same CRC-16 polynomial and packet philosophy
  as `MOCP`, deliberately, so this project's output is ground-station-
  compatible rather than a second competing protocol.
- The hand-written C in `embedded/` is not a substitute for running
  Embedded Coder — it's an independent second implementation, already
  compiled and verified, that gives you something to cross-check the real
  generated code against once you have it. See `embedded/README.md`.

## Simplifications

See `docs/03_simplifications.md` for the full list: non-rotating Earth, a
simplified exponential atmosphere, planar (not 6-DOF) dynamics, constant
per-stage thrust and drag coefficients, a fixed rather than targeting-
computed pitch program, and an honest statement that this vehicle's sizing
is not expected to reach full orbital insertion energy.

## Author

SAIPRABHA C Y
Related work: [CubeSat-ADCS-Sim](https://github.com/saiprabha-cy/CubeSat-ADCS-Sim), [kalman-attitude-estimation](https://github.com/saiprabha-cy/kalman-attitude-estimation), [Mission-Operations-Communication-Platform-MOCP](https://github.com/saiprabha-cy/Mission-Operations-Communication-Platform-MOCP), [aerospace-telemetry-flight-computer-platform](https://github.com/saiprabha-cy/aerospace-telemetry-flight-computer-platform), [planetary-rover-navigation](https://github.com/saiprabha-cy/planetary-rover-navigation)