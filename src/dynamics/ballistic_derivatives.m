function dstate = ballistic_derivatives(state, ep)
% BALLISTIC_DERIVATIVES  Pure central-force (no thrust, no drag) motion.
%
%   dstate = BALLISTIC_DERIVATIVES(state, ep)
%
%   State: [h; x; V; gamma]. This is the same translational physics as
%   ascent_derivatives.m with thrust, drag, and normal force all set to
%   zero, extracted as its own small function so the conservation
%   validation in sim/main_ballistic_conservation_check.m and
%   tests/test_gravity.m tests exactly this physics in isolation,
%   without needing to thread zeroed-out stage/PID parameters through
%   the full 7-state function.
%
%   With no external forces but gravity (a central, conservative
%   force), specific mechanical energy
%       E = V^2/2 - mu/(Re+h)
%   must be conserved along the trajectory. See
%   docs/04_verification_note.md for the independent numerical check
%   (relative error ~4.5e-15 over 1000 s) performed before this file
%   was written.

    h     = state(1);
    V     = state(3);
    gamma = state(4);

    g = gravity_accel(h, ep);

    dV     = -g * sin(gamma);
    dgamma = (1.0 / V) * ( -g * cos(gamma) + (V^2 * cos(gamma)) / (ep.Re + h) );
    dh     = V * sin(gamma);
    dx     = V * cos(gamma);

    dstate = [dh; dx; dV; dgamma];

end
