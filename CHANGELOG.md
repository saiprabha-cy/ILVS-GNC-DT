# Changelog

# Changelog

## v1.0.3 — Embedded Coder continuous-time codegen fix

- `build_pitch_control_model` confirmed working on a real run (the v1.0.2
  Zero-Order Hold fix resolved the persistent-variable error cleanly).
- `configure_embedded_coder` then failed during `rtwbuild` with "Block ...
  uses continuous time, which is not supported with the current
  configuration" on the Clock and both plant Integrator blocks. The
  `ert.tlc` target disables continuous-time codegen by default; added
  `set_param(model_name, 'SupportContinuousTime', 'on')`.
- `docs/02_results_analysis.md` updated with the full three-issue account
  (persistent-variable error → SampleTime parameter didn't exist → ZOH fix
  → continuous-time codegen error → SupportContinuousTime fix).

## v1.0.2 — Simulink sample-time fix, corrected approach

- The v1.0.1 fix for the persistent-variable/continuous-sample-time error
  (`set_param(..., 'SampleTime', '0.05')` directly on the PID MATLAB
  Function block) itself failed on a real run: *"SubSystem block does not
  have a parameter named 'SampleTime.'"* That parameter isn't reliably
  valid on a MATLAB Function block across MATLAB versions.
- Replaced with a version-robust fix: a Zero-Order Hold block
  (`pid_input_zoh`) inserted between `err_sum` and `pid_controller_block`,
  forcing the PID block's input to a discrete rate so its own inherited
  sample time resolves to discrete. Zero-Order Hold is a core Discrete-
  library block with a reliably settable `SampleTime` parameter in every
  MATLAB version.
- `docs/02_results_analysis.md` updated with the full two-attempt account.

## v1.0.1 — First real-run fix

- Confirmed by an actual MATLAB run: environment validation, ballistic
  energy conservation (4.541e-15 relative error), and the full ascent run
  (stage separation, MECO, Max-Q, RMS pitch tracking error) all matched
  the pre-delivery reference implementation to the reported precision.
- Fixed a real bug found by that run: `pid_controller_block` in
  `simulink/build_pitch_control_model.m` used `persistent` state while
  inheriting a continuous sample time, which Simulink disallows. Now sets
  an explicit discrete `SampleTime` of 0.05s on that block — the
  physically correct fix (a real embedded controller samples at a fixed
  rate), not a workaround.
- `docs/02_results_analysis.md` and `docs/04_verification_note.md` updated
  with the actual run's numbers, replacing the pre-run template/reference
  values.

## v1.0.0 — Initial delivery (rescoped ascent GNC)

- Rescoped from the original 9-phase full-mission-lifecycle plan to
  liftoff-through-MECO powered ascent, after an overlap review against
  four existing sibling repositories plus the rover-navigation project.
- Environment models (atmosphere, gravity) with validation script.
- 7-state ascent EOM (translation + pitch attitude), isolated ballistic
  energy-conservation check.
- Open-loop pitch program + closed-loop PID attitude control (explicit
  state, Embedded-Coder-ready calling convention).
- Discrete-event staging state machine (stage separation, MECO).
- Thin, MOCP-compatible telemetry packet format with CRC-16.
- 2D engineering plots, 3D trajectory plot, ascent+staging animation (GIF).
- Simulink API build script for the closed-loop pitch guidance/control
  subsystem, and an Embedded Coder configuration/codegen script.
- Hand-written C reference implementation of the PID controller,
  compiled and verified (all test vectors passing to machine precision)
  independently of Embedded Coder.
- Unit tests: atmosphere, gravity/energy-conservation, PID, staging,
  telemetry CRC.
- Full transparency note (docs/04_verification_note.md) on what was and
  wasn't executed before delivery, since MATLAB/Octave/Simulink were not
  available in the build sandbox.
