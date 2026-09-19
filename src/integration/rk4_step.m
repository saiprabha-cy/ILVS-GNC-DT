function state_next = rk4_step(deriv_fn, state, dt)
% RK4_STEP  One classical 4th-order Runge-Kutta integration step.
%
%   state_next = RK4_STEP(deriv_fn, state, dt)
%
%   deriv_fn must be a function handle taking a single state vector
%   argument and returning dstate/dt (only). For ascent_derivatives.m,
%   which also returns a diagnostics struct and needs (t, state, ...)
%   arguments, wrap it in an anonymous function at the call site (see
%   sim/main_ascent_full_run.m) -- keeping this integrator generic and
%   dependency-free is what makes it reusable for both the full ascent
%   state and the isolated ballistic conservation check.
%
%   A fixed step is used throughout this project rather than a
%   variable-step solver (ode45 etc.) because Program F ports the
%   guidance/control loop through Embedded Coder, which requires
%   fixed-step execution to generate deterministic embedded C -- using
%   a fixed-step integrator everywhere, not just in the ported
%   subsystem, keeps the MATLAB reference and the eventual Simulink
%   model directly comparable.

    k1 = deriv_fn(state);
    k2 = deriv_fn(state + 0.5 * dt * k1);
    k3 = deriv_fn(state + 0.5 * dt * k2);
    k4 = deriv_fn(state + dt * k3);

    state_next = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);

end
