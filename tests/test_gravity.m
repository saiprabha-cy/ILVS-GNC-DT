function test_gravity()
% TEST_GRAVITY  Unit tests for src/environment/gravity_accel.m and the
% energy-conservation invariant of src/dynamics/ballistic_derivatives.m.

    ep = earth_parameters();

    % Surface gravity should be close to standard 9.80665 m/s^2
    g0 = gravity_accel(0, ep);
    assert(abs(g0 - 9.80665) < 0.02, 'surface gravity should be ~9.80665 m/s^2');

    % Gravity strictly decreases with altitude
    g_low = gravity_accel(1000, ep);
    g_high = gravity_accel(400000, ep);
    assert(g_high < g_low, 'gravity should decrease with altitude');

    % Inverse-square scaling sanity check: g(2*Re) should be g(0)/4-ish
    % relative to Re (using Re above ground, i.e. h = Re -> distance = 2Re)
    g_at_Re = gravity_accel(ep.Re, ep);
    ratio = g0 / g_at_Re;
    assert(abs(ratio - 4.0) < 0.01, 'gravity should follow inverse-square law');

    % Energy conservation over a short ballistic arc (fast version of the
    % full check in sim/main_ballistic_conservation_check.m, kept here so
    % `tests/run_all_tests.m` catches an EOM regression without needing
    % the full 1000 s integration)
    state = [200000.0; 0.0; 6000.0; deg2rad(20.0)];
    E0 = 0.5*state(3)^2 - ep.mu/(ep.Re + state(1));
    deriv_fn = @(s) ballistic_derivatives(s, ep);
    dt = 0.05;
    for i = 1:2000  % 100 s
        state = rk4_step(deriv_fn, state, dt);
    end
    Ef = 0.5*state(3)^2 - ep.mu/(ep.Re + state(1));
    rel_err = abs((Ef - E0)/E0);
    assert(rel_err < 1e-9, sprintf('energy should be conserved, got rel_err=%.3e', rel_err));

    fprintf('test_gravity: all assertions passed\n');

end
