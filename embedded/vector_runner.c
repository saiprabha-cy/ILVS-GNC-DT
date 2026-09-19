/*
 * vector_runner.c
 *
 * Reads a CSV of (err, dt) pairs from stdin-equivalent input file,
 * runs them through pitch_pid_update() as one continuous control run
 * (state carried across rows, matching a real control loop), and
 * writes a CSV of outputs. Used by
 * sim/main_embedded_crossvalidation.m to cross-validate this C
 * implementation against src/control/pid_update.m over an identical,
 * realistic error sequence -- not just the small hand-picked cases in
 * test_harness.c.
 *
 * Gains are passed as command-line arguments so MATLAB and C are
 * guaranteed to use identical values (no risk of the constants
 * silently drifting apart in two hardcoded copies).
 *
 * Build:
 *     gcc -std=c89 -Wall -Wextra -o vector_runner vector_runner.c pitch_pid.c -lm
 * Run:
 *     ./vector_runner input.csv output.csv kp ki kd out_limit
 *
 * input.csv format (no header): err,dt  one pair per line
 * output.csv format (no header): u   one value per line
 */

#include <stdio.h>
#include <stdlib.h>
#include "pitch_pid.h"

int main(int argc, char *argv[])
{
    FILE *fin, *fout;
    PitchPidGains gains;
    PitchPidState state;
    double err, dt, u;
    int n = 0;

    if (argc != 7) {
        fprintf(stderr, "Usage: %s input.csv output.csv kp ki kd out_limit\n", argv[0]);
        return 2;
    }

    fin = fopen(argv[1], "r");
    if (!fin) {
        fprintf(stderr, "Could not open input file %s\n", argv[1]);
        return 2;
    }
    fout = fopen(argv[2], "w");
    if (!fout) {
        fprintf(stderr, "Could not open output file %s\n", argv[2]);
        fclose(fin);
        return 2;
    }

    gains.kp = atof(argv[3]);
    gains.ki = atof(argv[4]);
    gains.kd = atof(argv[5]);
    gains.out_limit = atof(argv[6]);

    pitch_pid_init(&state);

    while (fscanf(fin, "%lf,%lf", &err, &dt) == 2) {
        u = pitch_pid_update(err, dt, &gains, &state);
        fprintf(fout, "%.15e\n", u);
        n++;
    }

    fclose(fin);
    fclose(fout);

    fprintf(stderr, "vector_runner: processed %d rows\n", n);
    return 0;
}
