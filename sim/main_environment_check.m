function main_environment_check()
% MAIN_ENVIRONMENT_CHECK  Validate atmosphere and gravity models against
% known reference values, printing a numeric PASS/FAIL summary.

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'src', 'utilities'));
    add_project_paths();

    ep = earth_parameters();

    fprintf('Environment model validation\n');
    fprintf('-----------------------------\n');

    % --- Atmosphere: sea-level density should equal rho0 exactly ---
    rho_sl = atmosphere_density(0, ep);
    err_sl = abs(rho_sl - ep.rho0);
    fprintf('Density at sea level      : %.6f kg/m^3 (expected %.6f, err %.2e)\n', ...
            rho_sl, ep.rho0, err_sl);

    % --- Atmosphere: density at one scale height should be rho0/e ---
    rho_H = atmosphere_density(ep.H, ep);
    expected_H = ep.rho0 / exp(1);
    err_H = abs(rho_H - expected_H);
    fprintf('Density at h = H (%.0f m)  : %.6f kg/m^3 (expected %.6f, err %.2e)\n', ...
            ep.H, rho_H, expected_H, err_H);

    % --- Atmosphere: monotonically decreasing with altitude ---
    h_test = linspace(0, 100000, 50);
    rho_test = arrayfun(@(h) atmosphere_density(h, ep), h_test);
    monotonic = all(diff(rho_test) <= 0);
    fprintf('Density monotonically decreasing 0-100km : %d\n', monotonic);

    % --- Gravity: at h=0, should equal mu/Re^2 (surface gravity ~9.8) ---
    g_surface = gravity_accel(0, ep);
    fprintf('Surface gravity            : %.4f m/s^2 (expected ~9.80 m/s^2)\n', g_surface);
    g_surface_ok = abs(g_surface - 9.80) < 0.05;

    % --- Gravity: decreasing with altitude ---
    g_400km = gravity_accel(400000, ep); % ISS-ish altitude
    fprintf('Gravity at 400 km altitude : %.4f m/s^2 (expected ~8.7 m/s^2)\n', g_400km);
    g_400_ok = abs(g_400km - 8.7) < 0.2;

    ok = (err_sl < 1e-9) && (err_H < 1e-6) && monotonic && g_surface_ok && g_400_ok;

    fprintf('\n%s\n', ternary_str(ok, 'PASS', 'FAIL'));

    if ~ok
        error('main_environment_check:FAIL', 'Environment validation failed.');
    end

end

function s = ternary_str(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end
