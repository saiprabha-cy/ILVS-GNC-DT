/*
 * test_harness.c
 *
 * Standalone C test harness for pitch_pid.c, using the SAME test
 * vectors as tests/test_pid.m so the two can be checked against each
 * other by inspection (and, once you have Embedded Coder available,
 * against the generated code too -- see
 * sim/main_embedded_crossvalidation.m).
 *
 * Build (from the embedded/ directory):
 *     gcc -std=c89 -Wall -Wextra -o test_harness test_harness.c pitch_pid.c -lm
 * Run:
 *     ./test_harness
 *
 * Exits 0 and prints PASS for every case on success; exits 1 and
 * prints FAIL with the mismatch on any failure.
 */

#include <stdio.h>
#include <math.h>
#include "pitch_pid.h"

static int all_ok = 1;

static void check_close(const char *label, double actual, double expected, double tol)
{
    double diff = fabs(actual - expected);
    if (diff > tol) {
        printf("FAIL  %-45s actual=%.10f expected=%.10f diff=%.3e\n", label, actual, expected, diff);
        all_ok = 0;
    } else {
        printf("PASS  %-45s actual=%.10f (expected %.10f, diff=%.3e)\n", label, actual, expected, diff);
    }
}

static void check_true(const char *label, int cond)
{
    if (!cond) {
        printf("FAIL  %s\n", label);
        all_ok = 0;
    } else {
        printf("PASS  %s\n", label);
    }
}

int main(void)
{
    PitchPidGains gains;
    PitchPidState st;
    double u, expected;
    int i;

    gains.kp = 2.0; gains.ki = 0.5; gains.kd = 0.1; gains.out_limit = 100.0;

    printf("pitch_pid.c cross-validation against tests/test_pid.m vectors\n");
    printf("----------------------------------------------------------------\n");

    /* Case 1: first-call proportional+integral response (deriv=0) */
    pitch_pid_init(&st);
    u = pitch_pid_update(1.0, 0.1, &gains, &st);
    expected = gains.kp * 1.0 + gains.ki * (1.0 * 0.1) + gains.kd * 0.0;
    check_close("first-call output", u, expected, 1e-9);
    check_true("first flag clears after one call", st.first_call == 0);

    /* Case 2: positive saturation */
    pitch_pid_init(&st);
    u = pitch_pid_update(1000.0, 0.1, &gains, &st);
    check_close("positive saturation", u, gains.out_limit, 1e-12);

    /* Case 3: negative saturation */
    pitch_pid_init(&st);
    u = pitch_pid_update(-1000.0, 0.1, &gains, &st);
    check_close("negative saturation", u, -gains.out_limit, 1e-12);

    /* Case 4: zero error, zero initial state -> zero output */
    pitch_pid_init(&st);
    u = pitch_pid_update(0.0, 0.1, &gains, &st);
    check_close("zero error zero output", u, 0.0, 1e-12);

    /* Case 5: integral accumulation over 10 calls with constant error */
    pitch_pid_init(&st);
    for (i = 0; i < 10; i++) {
        pitch_pid_update(0.5, 0.1, &gains, &st);
    }
    expected = 0.5 * 0.1 * 10;
    check_close("integral accumulation over 10 steps", st.integral, expected, 1e-9);

    printf("\n%s\n", all_ok ? "ALL PASS" : "SOME FAILED");
    return all_ok ? 0 : 1;
}
