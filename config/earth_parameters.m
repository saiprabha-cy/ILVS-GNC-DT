function ep = earth_parameters()
% EARTH_PARAMETERS  Central-body and atmosphere constants.
%
% Non-rotating, spherical Earth with inverse-square gravity and a
% simplified exponential atmosphere. Rotating-Earth and the full US
% Standard Atmosphere 1976 table are documented extensions, not
% implemented here -- see docs/03_simplifications.md.

    ep.mu   = 3.986004418e14;  % m^3/s^2, Earth gravitational parameter
    ep.Re   = 6371000.0;       % m, mean Earth radius
    ep.g0   = 9.80665;         % m/s^2, standard gravity (Isp reference only)

    ep.rho0 = 1.225;           % kg/m^3, sea-level atmospheric density
    ep.H    = 8500.0;          % m, atmospheric scale height (exponential model)

end
