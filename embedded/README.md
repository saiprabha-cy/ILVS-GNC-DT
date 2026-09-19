# embedded/

Hand-written, portable ANSI C89 reference implementation of the pitch-attitude
PID controller (`pitch_pid.c` / `pitch_pid.h`), independent of and
cross-validated against `src/control/pid_update.m`.

## Why this exists

Program F's actual goal (see the project README and
`docs/01_design_decisions.md`) is generating this controller through
**Embedded Coder** and proving the generated C matches the MATLAB source.
That requires a MATLAB/Simulink/Embedded Coder license, which was not
available in the sandbox this project was built and packaged in (see
`docs/04_verification_note.md` for the full explanation). Rather than ship
an untested `simulink/build_pitch_control_model.m` and stop there, this
folder adds a second, independent, hand-written C implementation of the
exact same algorithm — one that **was** compiled and verified in that
sandbox — so there are two independently-arrived-at implementations
agreeing with each other before Embedded Coder ever enters the picture.

## What's actually verified, right now, without MATLAB

```
$ cd embedded
$ gcc -std=c89 -Wall -Wextra -o test_harness test_harness.c pitch_pid.c -lm
$ ./test_harness
```

All 6 test vectors (matching `tests/test_pid.m` exactly) pass to machine
precision (max diff ~5.6e-17). This was run and confirmed before these files
were included in the project.

```
$ gcc -std=c89 -O2 -o vector_runner vector_runner.c pitch_pid.c -lm
$ ./vector_runner <input.csv> <output.csv> <kp> <ki> <kd> <out_limit>
```

Also compiled and run against a 2000-sample (100 s) realistic error sequence
using this project's actual PID gains (`config/vehicle_parameters.m`
`vp.pid`); output is bounded, finite, and free of NaN/Inf across the full
run.

## What you still need to do (requires MATLAB + Simulink + Embedded Coder)

1. Run `simulink/build_pitch_control_model.m` to construct
   `pitch_control_loop.slx`.
2. Run `simulink/configure_embedded_coder.m` to generate C via Embedded
   Coder. This produces a THIRD independent implementation.
3. Run `sim/main_embedded_crossvalidation.m` to numerically compare
   plain-MATLAB vs. this hand-written C over an identical test vector (it
   already does this half of the comparison). Extend it to also read the
   Embedded-Coder-generated step function's output over the same vector —
   the exact generated function signature depends on your MATLAB version,
   so this wiring is left for you to complete following the same
   CSV-interchange pattern `main_embedded_crossvalidation.m` already uses.
4. Report the three-way comparison (MATLAB / hand-written C / generated C)
   in `docs/02_results_analysis.md`, the same "numeric bound, not just it
   compiled" standard used throughout this project and its sibling repos.

## Files

| File | Purpose |
|---|---|
| `pitch_pid.h` / `pitch_pid.c` | The controller, explicit state, no globals |
| `test_harness.c` | Six unit-test-style vectors, matches `tests/test_pid.m` |
| `vector_runner.c` | CSV-in/CSV-out batch runner, used by `sim/main_embedded_crossvalidation.m` |