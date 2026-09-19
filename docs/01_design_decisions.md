# Design decisions

## Scope: powered ascent only, not the full mission

This project was rescoped from a 9-phase full-mission-lifecycle plan down to
liftoff-through-MECO after reviewing it against five sibling repositories
(`CubeSat-ADCS-Sim`, `kalman-attitude-estimation`,
`Mission-Operations-Communication-Platform-MOCP`,
`aerospace-telemetry-flight-computer-platform`,
`planetary-rover-navigation`) that already cover satellite attitude control/
estimation, ground-segment comms, embedded flight-computer telemetry, and
ground mobility. None of them model powered atmospheric flight, so that's
this project's entire scope. See the project-level rescoping discussion for
the full overlap analysis.

## Two separate, minimal dynamics regimes, not one big model

Translational (7-state ascent) and rotational (single-axis pitch attitude)
dynamics are coupled only through the angle of attack
(`alpha = theta - gamma`), not fully merged into one state-space model. This
keeps each piece independently testable: `ballistic_derivatives.m` validates
pure translational physics via energy conservation without any attitude
loop involved, and the Simulink pitch-control model
(`simulink/build_pitch_control_model.m`) validates the attitude loop without
needing the full nonlinear translational EOM in block-diagram form.

## PID controller state: explicit struct in/out, not persistent variables

`src/control/pid_update.m` takes and returns an explicit state struct rather
than using MATLAB's `persistent` keyword. This is deliberate, for two
reasons documented in the function's own header: it makes the controller
directly unit-testable without hidden state to reset between test cases,
and it matches the calling convention Embedded Coder expects for a function
with state preserved across calls. The Simulink MATLAB Function block
version (`pid_controller_block_source()` in `simulink/build_pitch_control_
model.m`) DOES use `persistent` variables instead -- that's the idiomatic
Simulink/Embedded-Coder pattern for block state, and deliberately different
in kind from the plain-MATLAB version rather than a weaker copy of it,
mirroring the same principle used for the complementary filter vs. EKF
choice in the rover-navigation project.

## Control loop runs once per integration step, not once per RK4 sub-stage

`ascent_derivatives.m` is called four times per RK4 step (k1..k4), each at
a different intermediate time/state, and each internally computes a PID
output using the *same* controller state -- the state itself is only
actually advanced once, after the full RK4 step completes, in
`sim/main_ascent_full_run.m`. This is a deliberate zero-order-hold pattern:
a real discrete embedded controller samples its input once per control
cycle and holds its output constant across that cycle, and using the same
frozen `pid_state` across all four RK4 sub-evaluations is the correct way
to represent that inside a continuous-time integrator, rather than letting
the controller's integral/derivative terms update four times per physical
control cycle (which would make the closed-loop behavior depend on the
integrator's internal sub-stepping, not just on `dt`).

## Vertical-rise phase handled as a special case, not a smooth transition

The flight-path-angle rate equation divides by `V`, which is undefined at
liftoff. Rather than adding an epsilon to avoid the singularity (which
would introduce a tuning parameter with no physical meaning),
`ascent_derivatives.m` explicitly holds `gamma` constant at 90 degrees
during a short vertical-rise phase (`V < vertical_rise_speed` and
`t < vertical_rise_tmax`, both in `config/simulation_settings.m`), matching
how real vehicles actually fly a brief vertical segment before pitchover.
This is a physically-motivated modeling choice, not a numerical hack.

## Staging is a discrete-event check, not folded into the continuous EOM

`src/staging/staging_check.m` is called once per integration step, before
the derivative/integration call, and can trigger an instantaneous mass/
property discontinuity. This mirrors the Stateflow-chart structure the
logic would take in a full Simulink port (not yet built -- see
`docs/03_simplifications.md`), and keeps the discrete "is a stage event
happening" question separate from the continuous "what are the forces right
now" question the dynamics function answers.

## Telemetry: thin, MOCP-compatible packet, not a new protocol

`src/telemetry/pack_telemetry_packet.m` uses the same CRC-16/CCITT-FALSE
polynomial and initial value as `MOCP`'s existing packet layer, so a
telemetry stream from this project is decodable by that repo's ground
station without a second, competing packet definition -- see the
project-level portfolio narrative in the README.

## Embedded Coder target: generic portable C, not a specific board

`simulink/configure_embedded_coder.m` targets `ert.tlc` with no hardware
board selected. This proves the guidance/control algorithm cross-compiles
correctly (the actual Program F goal), without the added complexity and
sandbox-unavailability of a real board support package. Targeting an actual
STM32 board (matching `aerospace-telemetry-flight-computer-platform`'s
target) is a documented extension once this generic-C path is validated.

## A hand-written C reference exists alongside the Embedded-Coder path

`embedded/pitch_pid.c` is not a substitute for actually running Embedded
Coder -- it's a second, independent implementation of the same algorithm,
written by hand and compiled/verified in the sandbox this project was built
in (see `docs/04_verification_note.md`), so there is a working, verified
cross-check available immediately, and a second point of comparison once
Embedded Coder's real output is available too.