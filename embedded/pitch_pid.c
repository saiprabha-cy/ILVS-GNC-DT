/*
 * pitch_pid.c
 *
 * See pitch_pid.h for the cross-validation rationale. This is a
 * line-for-line translation of src/control/pid_update.m's arithmetic;
 * any behavioral difference between this file and the MATLAB source is
 * a bug in one of the two, which is exactly what
 * sim/main_embedded_crossvalidation.m and tests/test_pitch_pid_c.c
 * (via test_harness.c) are for.
 */

#include "pitch_pid.h"

void pitch_pid_init(PitchPidState *state)
{
    state->integral = 0.0;
    state->prev_err = 0.0;
    state->first_call = 1;
}

double pitch_pid_update(double err, double dt, const PitchPidGains *gains, PitchPidState *state)
{
    double integral;
    double deriv;
    double u_unsat;
    double u;

    integral = state->integral + err * dt;

    if (state->first_call) {
        deriv = 0.0;
    } else {
        deriv = (err - state->prev_err) / dt;
    }

    u_unsat = gains->kp * err + gains->ki * integral + gains->kd * deriv;

    if (u_unsat > gains->out_limit) {
        u = gains->out_limit;
    } else if (u_unsat < -gains->out_limit) {
        u = -gains->out_limit;
    } else {
        u = u_unsat;
    }

    state->integral = integral;
    state->prev_err = err;
    state->first_call = 0;

    return u;
}
