function theta_cmd_deg = pitch_program(t, pp)
% PITCH_PROGRAM  Open-loop commanded pitch attitude vs. time.
%
%   theta_cmd_deg = PITCH_PROGRAM(t, pp) returns the commanded vehicle
%   pitch angle [deg, from local horizontal] at time t [s]. pp is the
%   ss.pitch struct from config/simulation_settings.m.
%
%   This is the guidance REFERENCE the closed-loop attitude PID
%   (src/control/pid_update.m) tracks -- it does not itself close any
%   loop. Three phases, matching how early gravity-turn guidance is
%   actually flown:
%     1. Vertical rise   (t < t_vertical)            : 90 deg (straight up)
%     2. Pitch-over kick  (t_vertical..t_kick_end)     : small linear ramp,
%        which initiates the gravity turn by introducing a small angle
%        of attack for gravity to act on
%     3. Gravity-turn ramp (t_kick_end..t_pitch_end)   : linear decay
%        toward theta_final_deg, approximating the natural gravity-turn
%        pitch-down, held constant after t_pitch_end
%
%   A real ascent guidance system computes this profile from a
%   targeting algorithm (e.g. optimized for a target insertion state);
%   a fixed, precomputed schedule is a documented simplification here
%   -- see docs/03_simplifications.md.

    if t < pp.t_vertical
        theta_cmd_deg = 90.0;
        return;
    end

    if t < pp.t_kick_end
        frac = (t - pp.t_vertical) / (pp.t_kick_end - pp.t_vertical);
        theta_cmd_deg = 90.0 - pp.kick_deg * frac;
        return;
    end

    theta_kick = 90.0 - pp.kick_deg;

    if t < pp.t_pitch_end
        frac = (t - pp.t_kick_end) / (pp.t_pitch_end - pp.t_kick_end);
        theta_cmd_deg = theta_kick + (pp.theta_final_deg - theta_kick) * frac;
        return;
    end

    theta_cmd_deg = pp.theta_final_deg;

end
