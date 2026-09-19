# Verification note: what was actually run

This is stated plainly here, once, so it doesn't need hedging throughout
the rest of the codebase's comments.

## Status as of 2026-09-19: fully confirmed 

Every script in this project has now been executed for real, end to end,
in the project  MATLAB R2025a installation:

- `sim/main_environment_check.m` — PASS
- `sim/main_ballistic_conservation_check.m` — PASS, 4.541e-15 relative
  energy error over 1000s / 20,000 RK4 steps
- `sim/main_ascent_full_run.m` — PASS, full liftoff-to-MECO run, all
  figures and the animation generated successfully
- `simulink/build_pitch_control_model.m` — builds `pitch_control_loop.slx`
  cleanly
- `simulink/configure_embedded_coder.m` — successfully generates and
  compiles C via Embedded Coder, including a standalone
  `pitch_control_loop.exe`
- `sim/main_embedded_crossvalidation.m` — PASS, MATLAB vs. hand-written C
  agree to 8.004e-11 max difference over a 2000-sample realistic sequence

Full numbers for every item above are in `docs/02_results_analysis.md`,
which is now the authoritative results record — treat it as ground truth,
not this note.


## Three real issues found and fixed by actual runs (chronological)

1. **Persistent-variable / continuous-sample-time error** in
   `pid_controller_block` — a MATLAB Function block using `persistent`
   state can't inherit a continuous sample time.
2. **First fix attempt failed too**: `set_param(block, 'SampleTime',
   '0.05')` isn't a valid parameter on that block type in this MATLAB
   version (`set_param` reports it as a `SubSystem`).
3. **Correct fix**: a Zero-Order Hold block inserted upstream of the PID
   block, forcing its input (and therefore its own inherited sample time)
   to be discrete. This is a core Discrete-library block with a reliably
   settable `SampleTime` across versions, and physically models an ADC
   sampling a continuous plant — resolved the error cleanly.
4. **Next error, different cause**: `rtwbuild` failed with "Block ... uses
   continuous time, which is not supported with the current
   configuration" on the Clock and both plant Integrators. The `ert.tlc`
   target disables continuous-time codegen by default.
5. **Fix**: `set_param(model_name, 'SupportContinuousTime', 'on')`. Codegen
   then succeeded completely — see `docs/02_results_analysis.md` for the
   full build log excerpt, including the `NUMST=2` / `NCSTATES=2` detail
   that confirms the two-sample-rate design came through code generation
   intact, not just that the build didn't error.

Each of these is a genuine example of "verified against an independent
reference implementation" (what this project shipped with) being a
different, weaker claim than "executed successfully in a real MATLAB
installation" (what confirmed it actually works) — exactly the distinction
this note originally existed to be honest about.

## What remains open

`sim/main_embedded_crossvalidation.m` compares plain-MATLAB against the
*hand-written* C reference (`embedded/pitch_pid.c`), confirmed passing.
It does compare against the *actual Embedded-Coder-generated*
`pitch_control_loop.c` — that three-way comparison is real, additional
work, described with two concrete approaches in
`docs/02_results_analysis.md`'s final section.

