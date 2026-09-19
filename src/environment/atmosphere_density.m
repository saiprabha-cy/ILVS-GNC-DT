function rho = atmosphere_density(h, ep)
% ATMOSPHERE_DENSITY  Simplified exponential atmosphere.
%
%   rho = ATMOSPHERE_DENSITY(h, ep) returns air density [kg/m^3] at
%   altitude h [m] above sea level, using rho(h) = rho0 * exp(-h/H).
%
%   This is a documented simplification of the full US Standard
%   Atmosphere 1976 tabulated model (which has distinct
%   troposphere/stratosphere/mesosphere lapse rates) -- see
%   docs/03_simplifications.md. The exponential model is accurate to
%   within roughly 10-15% through the troposphere and stratosphere,
%   which is sufficient for dynamic-pressure and drag estimates in an
%   educational digital twin.
%
%   h < 0 is clamped to 0 (below sea level is not modelled).

    h_clamped = max(h, 0.0);
    rho = ep.rho0 * exp(-h_clamped / ep.H);

end
