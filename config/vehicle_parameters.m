function vp = vehicle_parameters()
% VEHICLE_PARAMETERS  Two-stage launch vehicle configuration.
%
% Returns a struct VP with stage-1, stage-2, and payload properties.
% These are representative, publicly-plausible values for an
% educational small-sat launcher, NOT any real vehicle's data.
%
% Mass, thrust, and specific impulse determine stage burn time via
% mdot = thrust / (Isp * g0); this is computed here once so every
% caller (dynamics, staging, tests) uses a single consistent value.

    g0 = 9.80665; % standard gravity, m/s^2, for Isp -> mdot conversion only

    % ---- Stage 1 ----
    vp.s1.dry_mass   = 1500.0;    % kg, structure + engine, no propellant
    vp.s1.prop_mass  = 8500.0;    % kg
    vp.s1.thrust     = 250000.0;  % N, assumed constant (documented simplification)
    vp.s1.isp        = 265.0;     % s
    vp.s1.cd         = 0.40;      % drag coefficient, constant (simplification)
    vp.s1.cn_alpha   = 3.5;       % 1/rad, linear normal-force slope
    vp.s1.area       = 1.0;       % m^2, reference area
    vp.s1.inertia    = 120000.0;  % kg*m^2, pitch-axis moment of inertia (lumped estimate)

    vp.s1.mdot       = vp.s1.thrust / (vp.s1.isp * g0);
    vp.s1.burn_time  = vp.s1.prop_mass / vp.s1.mdot;

    % ---- Stage 2 ----
    vp.s2.dry_mass   = 700.0;
    vp.s2.prop_mass  = 3200.0;
    vp.s2.thrust     = 60000.0;
    vp.s2.isp        = 320.0;
    vp.s2.cd         = 0.30;
    vp.s2.cn_alpha   = 2.5;
    vp.s2.area       = 0.7;
    vp.s2.inertia    = 9000.0;

    vp.s2.mdot       = vp.s2.thrust / (vp.s2.isp * g0);
    vp.s2.burn_time  = vp.s2.prop_mass / vp.s2.mdot;

    % ---- Payload ----
    vp.payload_mass  = 150.0;     % kg

    vp.liftoff_mass = vp.s1.dry_mass + vp.s1.prop_mass + ...
                       vp.s2.dry_mass + vp.s2.prop_mass + vp.payload_mass;

    % ---- Pitch attitude control (PID) gains ----
    % Shared by both stages for simplicity; re-tuning per stage
    % (different inertia) is listed as a documented extension in
    % docs/03_simplifications.md.
    vp.pid.kp        = 800000.0;
    vp.pid.ki        = 5000.0;
    vp.pid.kd        = 250000.0;
    vp.pid.out_limit = 2.0e6;     % N*m-equivalent commanded moment saturation
    vp.pid.damping_b = 50000.0;   % simple rotational rate damping term

end
