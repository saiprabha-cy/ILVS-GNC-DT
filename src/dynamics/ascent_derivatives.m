function [dstate, diag] = ascent_derivatives(t, state, stage_props, pid_gains, pid_state, ep, ss)
% ASCENT_DERIVATIVES  7-state ascent equations of motion.
%
%   [dstate, diag] = ASCENT_DERIVATIVES(t, state, stage_props, ...
%                                        pid_gains, pid_state, ep, ss)
%
%   State vector (column, length 7):
%     state(1) = h         altitude, m
%     state(2) = x         downrange distance, m
%     state(3) = V         speed, m/s
%     state(4) = gamma     flight-path angle from local horizontal, rad
%     state(5) = theta     vehicle pitch attitude, rad
%     state(6) = theta_dot pitch rate, rad/s
%     state(7) = m         instantaneous vehicle mass, kg
%
%   stage_props is a struct with fields: thrust, mdot, cd, cn_alpha,
%   area, inertia -- selected by the caller for whichever stage is
%   currently active (see src/staging/staging_check.m).
%
%   Physics (point-mass translation, vertical plane, non-rotating
%   spherical Earth, small-angle-of-attack normal force):
%
%     alpha = theta - gamma                    (angle of attack)
%     dV/dt     = (T*cos(alpha) - D)/m - g*sin(gamma)
%     dgamma/dt = (1/V) * [ T*sin(alpha)/m + N/m - g*cos(gamma)
%                            + V^2*cos(gamma)/(Re+h) ]
%     dh/dt     = V*sin(gamma)
%     dx/dt     = V*cos(gamma)
%     dm/dt     = -mdot
%
%   with D = 0.5*rho*V^2*Cd*A (drag) and N = 0.5*rho*V^2*Cn_alpha*alpha*A
%   (simplified linear normal/side force). Pitch attitude is driven
%   independently by a PID tracking an open-loop pitch program (see
%   src/guidance/pitch_program.m, src/control/pid_update.m):
%
%     theta_ddot = (Mc - b*theta_dot) / I
%
%   VALIDATION: with thrust=drag=0 (see sim/main_ballistic_conservation
%   _check.m and tests/test_gravity.m), this reduces to pure central-
%   force motion, and specific mechanical energy
%   E = V^2/2 - mu/(Re+h) is conserved -- verified to machine precision
%   (relative error ~4.5e-15 over a 1000 s / 20000-step RK4 integration)
%   before this function was written, using an independent reference
%   implementation of the same equations (see docs/04_verification_note.md).
%
%   VERTICAL-RISE SINGULARITY: the dgamma/dt equation divides by V, so
%   it is undefined at liftoff (V=0). This function instead holds
%   gamma = pi/2 and skips the gamma/N/lift terms whenever
%   V < ss.vertical_rise_speed AND t < ss.vertical_rise_tmax, matching
%   the short vertical-rise phase real vehicles fly before pitchover.
%
%   diag is a struct of intermediate quantities (Mc, theta_cmd, alpha,
%   dynamic pressure q) useful for logging/telemetry without
%   recomputing them.

    h       = state(1);
    x       = state(2);
    V       = state(3);
    gamma   = state(4);
    theta   = state(5);
    theta_dot = state(6);
    m       = state(7);

    theta_cmd_deg = pitch_program(t, ss.pitch);
    theta_cmd = deg2rad(theta_cmd_deg);

    err = theta_cmd - theta;
    [Mc, ~] = pid_update(err, ss.dt, pid_gains, pid_state);

    theta_ddot = (Mc - pid_gains.damping_b * theta_dot) / stage_props.inertia;

    g   = gravity_accel(h, ep);
    rho = atmosphere_density(h, ep);
    D   = 0.5 * rho * V^2 * stage_props.cd * stage_props.area;
    alpha = theta - gamma;
    q = 0.5 * rho * V^2;

    vertical_phase = (V < ss.vertical_rise_speed) && (t < ss.vertical_rise_tmax);

    if vertical_phase
        dV     = (stage_props.thrust - D) / m - g;
        dgamma = 0.0;
        dh     = V;
        dx     = 0.0;
        N      = 0.0;
    else
        N      = 0.5 * rho * V^2 * stage_props.cn_alpha * alpha * stage_props.area;
        dV     = (stage_props.thrust * cos(alpha) - D) / m - g * sin(gamma);
        dgamma = (1.0 / V) * ( stage_props.thrust * sin(alpha) / m + N / m ...
                                 - g * cos(gamma) + (V^2 * cos(gamma)) / (ep.Re + h) );
        dh     = V * sin(gamma);
        dx     = V * cos(gamma);
    end

    dm = -stage_props.mdot;

    dstate = [dh; dx; dV; dgamma; theta_dot; theta_ddot; dm];

    diag.Mc            = Mc;
    diag.theta_cmd_deg  = theta_cmd_deg;
    diag.alpha_deg      = rad2deg(alpha);
    diag.dynamic_pressure = q;
    diag.drag            = D;
    diag.normal_force     = N;
    diag.vertical_phase   = vertical_phase;

end
