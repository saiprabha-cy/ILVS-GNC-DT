function main_ballistic_conservation_check()
% MAIN_BALLISTIC_CONSERVATION_CHECK  Validate the translational EOM by
% checking specific mechanical energy conservation with thrust and drag
% both zero (pure central-force / ballistic motion).
%
%   This is the same conservation-check discipline used in the sibling
%   repos (CubeSat-ADCS-Sim, kalman-attitude-estimation): with the only
%   force acting being a central, conservative one (gravity), specific
%   mechanical energy E = V^2/2 - mu/(Re+h) must be conserved along the
%   trajectory. A bug in the gravity term, the gamma-rate equation, or
%   the integrator would show up here as energy drift.
%
%   Independently verified before this file was written using a
%   reference implementation of the same equations (relative energy
%   error ~4.5e-15 over 1000 s / 20000 RK4 steps) -- see
%   docs/04_verification_note.md.

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'src', 'utilities'));
    add_project_paths();

    ep = earth_parameters();

    dt = 0.05;
    t_max = 1000.0;
    steps = round(t_max / dt);

    % Arbitrary non-trivial initial condition: 200 km altitude, 6000 m/s,
    % 20 deg flight-path angle -- an elliptical-ish ballistic arc, not a
    % degenerate straight-up or circular case, so the check exercises all
    % terms in the gamma-rate equation.
    state = [200000.0; 0.0; 6000.0; deg2rad(20.0)];

    E0 = 0.5 * state(3)^2 - ep.mu / (ep.Re + state(1));

    deriv_fn = @(s) ballistic_derivatives(s, ep);

    for i = 1:steps
        state = rk4_step(deriv_fn, state, dt);
    end

    Ef = 0.5 * state(3)^2 - ep.mu / (ep.Re + state(1));
    rel_err = abs((Ef - E0) / E0);

    fprintf('Ballistic energy conservation check\n');
    fprintf('------------------------------------\n');
    fprintf('Duration               : %.0f s (%d RK4 steps at dt=%.3f s)\n', t_max, steps, dt);
    fprintf('Initial specific energy : %.6f J/kg\n', E0);
    fprintf('Final specific energy   : %.6f J/kg\n', Ef);
    fprintf('Relative energy error   : %.3e\n', rel_err);

    ok = rel_err < 1e-9;
    fprintf('\n%s\n', ternary_str(ok, 'PASS', 'FAIL'));

    if ~ok
        error('main_ballistic_conservation_check:FAIL', 'Energy not conserved within tolerance.');
    end

end

function s = ternary_str(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end
