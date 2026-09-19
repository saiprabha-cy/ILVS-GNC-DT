# Results analysis

**Status: fully confirmed by real runs, including Embedded Coder
codegen.** Every table below reflects actual execution — environment
validation, ballistic conservation, the full ascent run, the Simulink
build, Embedded Coder code generation, and MATLAB-vs-C cross-validation
all ran successfully on 2026-09-19. The one remaining open item (the true
three-way comparison including the actual generated C, not just the
hand-written reference) is called out explicitly in its own section below,
not glossed over.

## Environment validation (`sim/main_environment_check.m`)

| Check | Expected | Reference implementation | Actual run |
|---|---|---|---|
| Sea-level density | 1.225000 kg/m³ | matches by construction | 1.225000 kg/m³, err 0.00e+00 — PASS |
| Density at h=H | rho0/e = 0.450659 kg/m³ | matches by construction | 0.450652 kg/m³, err 0.00e+00 — PASS |
| Density monotonic 0-100km | true | true | true — PASS |
| Surface gravity | ~9.80665 m/s² | matches by construction | 9.8203 m/s² — PASS |
| Gravity at 400km | ~8.7 m/s² | ~8.68 m/s² | 8.6943 m/s² — PASS |

Surface gravity reads 9.8203 m/s², not 9.80665 — this is **not a bug**.
9.80665 is the standard-gravity constant (used only for the Isp→mdot
conversion in `vehicle_parameters.m`); the model's actual gravity function
is `mu/(Re+h)^2`, which computes to 9.8203 m/s² at h=0 for this project's
`mu` and `Re` values. The validation tolerance (±0.05) was written to
accommodate exactly this, correctly.

## Ballistic energy conservation (`sim/main_ballistic_conservation_check.m`)

| Quantity | Reference implementation | Actual run |
|---|---|---|
| Duration | 1000 s, 20000 RK4 steps @ dt=0.05s | 1000 s, 20000 RK4 steps @ dt=0.05s |
| Relative energy error | 4.5e-15 | 4.541e-15 |
| PASS threshold | < 1e-9 | PASS (6 orders of magnitude inside tolerance) |

Confirms the ported EOM and RK4 integrator are correct to machine
precision, not just "close enough."

## Full ascent run (`sim/main_ascent_full_run.m`)

| Quantity | Reference implementation | Actual run |
|---|---|---|
| Stage separation time | t ≈ 88.4 s | t = 88.40 s |
| MECO time | t ≈ 255.8 s | t = 255.80 s |
| MECO altitude | ≈ 337 km | 337.20 km |
| MECO velocity | ≈ 5160 m/s | 5157.9 m/s |
| MECO flight-path angle | ≈ 24.5 deg | 24.57 deg |
| Local circular velocity at MECO altitude | ≈ 7709 m/s | 7708.4 m/s |
| Velocity deficit | ≈ 2549 m/s (≈ 33%) | 2550.5 m/s (33.1%) |
| Max dynamic pressure (Max-Q) | ≈ 45.3 kPa at t ≈ 46.5 s | 45.31 kPa at t = 46.5 s (stage 1) |
| RMS pitch tracking error | *(not computed in reference run)* | 0.058 deg |

5,116 telemetry packets logged; `results/figures/ascent_telemetry.png`,
`results/figures/trajectory_3d.png`, and a 257-frame
`results/animations/ascent_animation.gif` were all generated successfully.

**The 33.1% MECO velocity deficit against local circular velocity is
confirmed, exactly as predicted in `docs/03_simplifications.md` before this
run happened.** This vehicle does not reach orbital insertion energy — by
design, given this two-stage vehicle's mass fractions, not as a bug. The
insertion-targeting *calculation itself* (deficit = Vc − V, correctly
computed against the actual MECO altitude) is what's being demonstrated
here, and it checks out.

**RMS pitch tracking error of 0.058 degrees** is a genuinely good closed-
loop result for a PID controller with shared, untuned-per-stage gains (see
`docs/03_simplifications.md`) — worth stating explicitly in interviews as a
quantified control-performance number, not just "the controller worked."

## Embedded controller cross-validation

### C reference implementation, verified in-sandbox (`embedded/test_harness.c`)

| Test case | Result |
|---|---|
| First-call proportional+integral response | PASS, diff 0.0 |
| Positive saturation | PASS, diff 0.0 |
| Negative saturation | PASS, diff 0.0 |
| Zero error → zero output | PASS, diff 0.0 |
| Integral accumulation over 10 steps | PASS, diff 5.551e-17 |

### MATLAB vs. hand-written C (`sim/main_embedded_crossvalidation.m`)

| Quantity | Actual run |
|---|---|
| Test vector length | 2000 samples (100 s at dt=0.05 s) |
| Max absolute difference | 8.004e-11 |
| RMS difference | 1.330e-11 |
| PASS threshold | < 1e-9 |
| Result | **PASS** |

### Simulink / Embedded Coder

**Full history, all three issues found and fixed by real runs, in order:**

1. *"pid_controller_block uses constructs that are invalid when the block
   specifies or inherits a continuous sample time."* Root cause: the PID
   MATLAB Function block uses `persistent` variables for controller state,
   which Simulink disallows on a block with inherited/continuous sample
   time.
2. First fix attempt (`set_param(..., 'SampleTime', '0.05')` directly on
   the PID block) itself failed: *"SubSystem block does not have a
   parameter named 'SampleTime.'"* Not reliably valid on a MATLAB Function
   block across MATLAB versions.
3. **Actual fix:** a Zero-Order Hold block (`pid_input_zoh`) inserted
   between `err_sum` and `pid_controller_block`, forcing the PID block's
   input to a discrete rate so its own sample time resolves to discrete —
   a core Discrete-library block with a reliably settable `SampleTime`
   parameter, and also the more physically honest model of a controller
   sampling a continuous plant through an ADC.
4. With that fixed, `rtwbuild` then failed differently: *"Block
   'pitch_control_loop/Clock' uses continuous time, which is not supported
   with the current configuration"* (and both plant Integrators). The
   `ert.tlc` target disables continuous-time codegen by default, assuming
   a purely discrete embedded target.
5. **Fix:** `set_param(model_name, 'SupportContinuousTime', 'on')` added
   to `configure_embedded_coder.m`.

**CONFIRMED 2026-09-19 — full codegen success on the next run:**

```
### Using System Target File: ert.tlc  (MATLAB R2025a)
### Writing source file pitch_control_loop.c
### Writing header file pitch_control_loop.h / pitch_control_loop_types.h / rtwtypes.h
### Writing source file ert_main.c
### TLC code generation complete (took 25.381s)
### Using toolchain: MinGW64 | gmake (64-bit Windows)
### Created: ../pitch_control_loop.exe
### Successfully generated all binary outputs.

Build Summary: 1 of 1 models built, 0h 1m 48.1s total
NUMST=2 (two sample rates: discrete PID + continuous plant, exactly as designed)
NCSTATES=2 (the two continuous plant integrator states)
```

Two details in that build log are worth calling out explicitly, since they
confirm the model's intent was preserved through code generation, not just
that it compiled:

- **`NUMST=2`** — the generated code has exactly two distinct sample rates:
  the discrete PID controller and the continuous plant. That's the direct,
  literal signature of the Zero-Order Hold fix working as designed, visible
  in the compiler's own instrumentation, not just inferred from "it built."
- **A standalone `pitch_control_loop.exe` was produced**, not just object
  files — the default ERT rapid-prototyping target builds a runnable
  executable of the generated code. This is a genuine, if unplanned, extra
  validation opportunity: that executable can be run directly and its
  output compared to the MATLAB/Simulink simulation, a stronger check than
  inspecting the generated `.c` source by eye. Not yet done — see "Next
  steps" below.

One expected, benign warning appeared and does not indicate a problem:
*"The output(s) read after the base-rate model step reflects intervening
minor time steps... Place a Zero-Order Hold block before the continuous
output port 'theta_out'"* — this is Simulink flagging that `theta_out`
(the plant's continuous `theta` state, wired straight to an output port)
can show intermediate integrator sub-step values rather than only
major-time-step values if read at the wrong point in generated code. It
doesn't affect the PID cross-validation (which reads `Mc`, the discrete
controller's output, not `theta_out`), but if `theta_out` is used for
anything downstream later, add the ZOH the warning suggests.

### MATLAB vs. hand-written C, confirmed by a real run

| Quantity | Actual run |
|---|---|
| Test vector length | 2000 samples (100 s at dt=0.05 s) |
| Max absolute difference | 8.004e-11 |
| RMS difference | 1.330e-11 |
| PASS threshold | < 1e-9 |
| Result | **PASS** |

8e-11 is not "close enough" — it's within the range expected from
double-precision floating-point arithmetic accumulating slightly
differently between MATLAB's and C's evaluation order over 2000
sequential updates (each carrying forward integral/derivative state from
the last). This is what genuine numerical agreement between two
independent implementations of the same algorithm looks like: small,
bounded, explainable by floating-point arithmetic, not exactly zero and
not systematically growing.

### What's confirmed vs. what's still open

**Confirmed by real runs:** plain-MATLAB PID (`pid_update.m`) agrees with
the independently hand-written C (`embedded/pitch_pid.c`) to 8e-11 over a
realistic 100 s sequence. Embedded Coder successfully generates and
compiles C from the identical algorithm expressed as a Simulink MATLAB
Function block.

**Still open, with a concrete path to close it now committed to the repo:**
`main_embedded_crossvalidation.m` compares MATLAB against the
*hand-written* C reference, not yet against the *actual Embedded-Coder-
generated* `pitch_control_loop.c`. Two ways to close this gap, in
increasing order of rigor:

1. **Simplest — manual diff:** open
   `simulink/pitch_control_loop_ert_rtw/pitch_control_loop.c`, locate the
   generated PID logic (search for `pid_controller_block` in a comment or
   symbol name), and diff its arithmetic against `embedded/pitch_pid.c`
   line by line. Fast, no new tooling, and Embedded Coder's output is
   usually close enough to hand-written C to make this tractable.

2. **Stronger — Software-in-the-Loop (SIL), now implemented in
   `sim/main_sil_crossvalidation.m`:** rather than trying to feed the
   standalone `pitch_control_loop.exe` an external test vector (it can't
   accept one without modifying the model — `err` is computed internally
   from the closed-loop guidance/plant, not read from a file), this script
   uses Simulink's built-in SIL mode: it runs the identical closed-loop
   scenario twice — once normally (interpreted MATLAB) and once with
   `SimulationMode` set to `'software-in-the-loop'`, which substitutes the
   PID block's *actual generated C*, compiled into an S-function, for its
   normal execution. Both runs see the same guidance reference and the
   same plant by construction, so any difference is purely
   interpreted-MATLAB-vs-generated-C arithmetic. Combined with
   `main_embedded_crossvalidation.m`'s already-passing MATLAB-vs-hand-
   written-C result, a passing SIL run closes the full three-way triangle
   (MATLAB ≈ hand-written C ≈ generated C) without needing a third,
   separate direct comparison.

   **Not yet run** — `SimulationMode = 'software-in-the-loop'` is a long-
   standing, stable Simulink Coder feature, but given this project has
   already hit two version-specific Simulink API surprises during real
   runs (the `SampleTime` parameter and the continuous-time codegen
   default — both documented above), this is flagged honestly as
   *expected to work, not yet confirmed*, rather than claimed done. The
   script includes a fallback to Option 1 if it errors. Run it and report
   back the same way the earlier fixes got confirmed.