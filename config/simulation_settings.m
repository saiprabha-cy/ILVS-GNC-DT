function ss = simulation_settings()
% SIMULATION_SETTINGS  Fixed-step integration and pitch-program timing.
%
% A fixed step is used throughout (not a variable-step solver) because
% Program F ports the guidance/control loop through Embedded Coder,
% which requires a fixed-step, discrete or fixed-step-continuous
% configuration to generate deterministic embedded C.

    ss.dt        = 0.05;   % s, RK4 integration step
    ss.t_max     = 400.0;  % s, safety cap on simulation duration

    % Vertical-rise phase: gamma is held at 90 deg until V exceeds this
    % speed (avoids the 1/V singularity in the flight-path-angle rate
    % equation at liftoff, and matches how real vehicles fly a short
    % vertical segment before pitchover).
    ss.vertical_rise_speed = 5.0;   % m/s
    ss.vertical_rise_tmax  = 12.0;  % s, hard cap independent of speed

    % Pitch program (open-loop guidance reference), see
    % src/guidance/pitch_program.m for the schedule this defines.
    ss.pitch.t_vertical     = 10.0;  % s, hold vertical until this time
    ss.pitch.t_kick_end     = 15.0;  % s, end of the initial pitch-over kick
    ss.pitch.kick_deg       = 3.0;   % deg, size of the initial kick
    ss.pitch.t_pitch_end    = 260.0; % s, end of the gravity-turn pitch ramp
    ss.pitch.theta_final_deg = 6.0;  % deg, target pitch approaching MECO

end
