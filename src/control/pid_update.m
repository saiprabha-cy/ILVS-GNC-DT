function [u, pid_state_out] = pid_update(err, dt, gains, pid_state_in)
% PID_UPDATE  Discrete PID step with explicit state in/out.
%
%   [u, pid_state_out] = PID_UPDATE(err, dt, gains, pid_state_in)
%
%   Deliberately written with explicit state passed in and returned,
%   rather than using persistent/global variables, for two reasons:
%     1. It is directly testable in isolation (tests/test_pid.m) without
%        needing to reset hidden state between test cases.
%     2. This is exactly the calling convention Embedded Coder expects
%        for a step function with state that must be preserved between
%        calls on an embedded target -- see embedded/pitch_pid.c, which
%        is the hand-written C port of this exact function, used for
%        the cross-validation in sim/main_embedded_crossvalidation.m
%        (Program F).
%
%   Inputs:
%     err            - tracking error (commanded - actual), radians
%     dt             - time step, s
%     gains          - struct with fields kp, ki, kd, out_limit
%     pid_state_in   - struct with fields integral, prev_err, first
%                       (first = true on the very first call, so the
%                       derivative term is not computed from an
%                       undefined previous error)
%
%   Outputs:
%     u              - controller output, saturated to +/- out_limit
%     pid_state_out  - updated state struct, feed into the next call

    integral = pid_state_in.integral + err * dt;

    if pid_state_in.first
        deriv = 0.0;
    else
        deriv = (err - pid_state_in.prev_err) / dt;
    end

    u_unsat = gains.kp * err + gains.ki * integral + gains.kd * deriv;
    u = max(min(u_unsat, gains.out_limit), -gains.out_limit);

    pid_state_out.integral = integral;
    pid_state_out.prev_err = err;
    pid_state_out.first    = false;

end
