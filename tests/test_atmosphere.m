function test_atmosphere()
% TEST_ATMOSPHERE  Unit tests for src/environment/atmosphere_density.m

    ep = earth_parameters();

    % Sea-level density equals rho0 exactly
    assert(abs(atmosphere_density(0, ep) - ep.rho0) < 1e-12, ...
           'sea-level density should equal rho0');

    % Density at h=0 (negative clamp) should equal density at h=0
    assert(abs(atmosphere_density(-500, ep) - atmosphere_density(0, ep)) < 1e-12, ...
           'negative altitude should clamp to sea level');

    % Density at one scale height equals rho0/e
    expected = ep.rho0 / exp(1);
    assert(abs(atmosphere_density(ep.H, ep) - expected) < 1e-9, ...
           'density at h=H should equal rho0/e');

    % Monotonically decreasing
    h = linspace(0, 200000, 100);
    rho = arrayfun(@(hh) atmosphere_density(hh, ep), h);
    assert(all(diff(rho) <= 0), 'density should be monotonically non-increasing with altitude');

    % Strictly positive everywhere tested
    assert(all(rho > 0), 'density should be strictly positive');

    fprintf('test_atmosphere: all assertions passed\n');

end
