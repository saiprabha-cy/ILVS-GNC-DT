function test_pid()
% TEST_PID  Unit tests for src/control/pid_update.m and pid_init.m.
%
%   Also serves as the source of the test vectors compared against the
%   hand-written C port in embedded/pitch_pid.c -- see
%   sim/main_embedded_crossvalidation.m.

    gains.kp = 2.0; gains.ki = 0.5; gains.kd = 0.1; gains.out_limit = 100.0;

    % Proportional-only response on the first call (integral=0, deriv=0
    % because `first` is true)
    st = pid_init();
    [u, st] = pid_update(1.0, 0.1, gains, st);
    expected_first = gains.kp * 1.0; % + ki*integral(=0.05) -> not zero, recompute below
    expected_first = gains.kp*1.0 + gains.ki*(1.0*0.1) + gains.kd*0.0;
    assert(abs(u - expected_first) < 1e-9, 'first-call PID output mismatch');
    assert(st.first == false, 'first flag should clear after one call');

    % Output saturates at out_limit for a large error
    st2 = pid_init();
    [u2, ~] = pid_update(1000.0, 0.1, gains, st2);
    assert(u2 == gains.out_limit, 'PID output should saturate at out_limit');

    st3 = pid_init();
    [u3, ~] = pid_update(-1000.0, 0.1, gains, st3);
    assert(u3 == -gains.out_limit, 'PID output should saturate at -out_limit');

    % Zero error, zero initial state -> zero output
    st4 = pid_init();
    [u4, st4] = pid_update(0.0, 0.1, gains, st4);
    assert(abs(u4) < 1e-12, 'zero error should give zero output on first call');

    % Integral term accumulates over repeated calls with constant error
    st5 = pid_init();
    dt = 0.1;
    err = 0.5;
    for i = 1:10
        [~, st5] = pid_update(err, dt, gains, st5);
    end
    expected_integral = err * dt * 10;
    assert(abs(st5.integral - expected_integral) < 1e-9, 'integral accumulation mismatch');

    fprintf('test_pid: all assertions passed\n');

end
