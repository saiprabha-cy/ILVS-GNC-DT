function g = gravity_accel(h, ep)
% GRAVITY_ACCEL  Inverse-square gravitational acceleration.
%
%   g = GRAVITY_ACCEL(h, ep) returns gravitational acceleration [m/s^2]
%   at altitude h [m] above the mean Earth radius, using
%   g(h) = mu / (Re + h)^2.
%
%   Non-rotating, spherically-symmetric Earth (no J2 oblateness term).
%   J2 is a documented extension, not implemented here -- see
%   docs/03_simplifications.md. This is also the invariant used by
%   tests/test_gravity.m and sim/main_ballistic_conservation_check.m:
%   with thrust and drag set to zero, specific mechanical energy
%   E = V^2/2 - mu/(Re+h) must be conserved along any trajectory using
%   this gravity model, because it is a central (conservative) force.

    g = ep.mu ./ (ep.Re + h).^2;

end
