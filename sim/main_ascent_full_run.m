function result = main_ascent_full_run()
% MAIN_ASCENT_FULL_RUN  Liftoff-to-MECO ascent simulation.
%
%   result = MAIN_ASCENT_FULL_RUN()
%
%   Integrates the 7-state ascent EOM (src/dynamics/ascent_derivatives.m)
%   from liftoff through stage separation to main-engine cutoff (MECO),
%   using the staging state machine (src/staging/staging_check.m) and a
%   closed-loop pitch-attitude PID (src/control/pid_update.m) tracking
%   the open-loop pitch program (src/guidance/pitch_program.m).
%
%   Prints a numeric PASS/FAIL summary, saves telemetry to
%   data/telemetry/, saves engineering plots and a 3D trajectory plot to
%   results/figures/, and saves an ascent animation to
%   results/animations/.
%
%   PASS criteria: MECO is reached (stage-2 propellant depletes) before
%   t_max, altitude and velocity are monotonically sane (no NaN/Inf),
%   and Max-Q occurs during stage-1 flight as expected. This project
%   does NOT assert that MECO velocity reaches local circular orbital
%   velocity -- see the printed velocity-deficit line and
%   docs/03_simplifications.md for why an educational two-stage vehicle
%   sized this way is not expected to reach full orbital energy, and
%   docs/02_results_analysis.md for how that's reported rather than
%   hidden once you've run this and have real numbers.

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'src', 'utilities'));
    add_project_paths();

    ep = earth_parameters();
    vp = vehicle_parameters();
    ss = simulation_settings();

    state = [0.0; 0.0; 0.0; pi/2; pi/2; 0.0; vp.liftoff_mass];
    stage = 1;
    t = 0.0;
    prop_consumed = 0.0;
    pid_state = pid_init();

    steps = round(ss.t_max / ss.dt);

    log.t = zeros(steps,1); log.h = zeros(steps,1); log.x = zeros(steps,1);
    log.V = zeros(steps,1); log.gamma_deg = zeros(steps,1);
    log.theta_deg = zeros(steps,1); log.theta_cmd_deg = zeros(steps,1);
    log.m = zeros(steps,1); log.q = zeros(steps,1); log.alpha_deg = zeros(steps,1);
    log.stage = zeros(steps,1);

    telemetry = {};
    seq = 0;

    stage_sep_time = []; stage_sep_idx = [];
    meco_time = []; meco_idx = []; meco_state = [];
    max_q = 0.0; max_q_time = 0.0; max_q_stage = 1;

    n_logged = 0;

    for i = 1:steps
        [stage_next, sp, event] = staging_check(stage, state, vp, prop_consumed);

        if strcmp(event, 'stage_separation')
            state(7) = state(7) - vp.s1.dry_mass;  % jettison stage-1 structure
            stage = stage_next;
            prop_consumed = 0.0;
            pid_state = pid_init();  % fresh integrator for the new stage
            stage_sep_time = t;
            stage_sep_idx = n_logged + 1;
        elseif strcmp(event, 'meco')
            meco_time = t;
            meco_idx = n_logged;
            meco_state = state;
            break;
        end

        deriv_fn = @(s) ascent_step_wrapper(t, s, sp, vp.pid, pid_state, ep, ss);
        [~, diag] = ascent_derivatives(t, state, sp, vp.pid, pid_state, ep, ss);

        n_logged = n_logged + 1;
        log.t(n_logged) = t; log.h(n_logged) = state(1); log.x(n_logged) = state(2);
        log.V(n_logged) = state(3); log.gamma_deg(n_logged) = rad2deg(state(4));
        log.theta_deg(n_logged) = rad2deg(state(5)); log.theta_cmd_deg(n_logged) = diag.theta_cmd_deg;
        log.m(n_logged) = state(7); log.q(n_logged) = diag.dynamic_pressure;
        log.alpha_deg(n_logged) = diag.alpha_deg; log.stage(n_logged) = stage;

        if diag.dynamic_pressure > max_q
            max_q = diag.dynamic_pressure;
            max_q_time = t;
            max_q_stage = stage;
        end

        seq = seq + 1;
        telemetry{end+1} = pack_telemetry_packet(t, state, diag, stage, seq); %#ok<AGROW>

        [~, pid_state] = pid_update(deg2rad(diag.theta_cmd_deg) - state(5), ss.dt, vp.pid, pid_state);
        state = rk4_step(deriv_fn, state, ss.dt);
        prop_consumed = prop_consumed + sp.mdot * ss.dt;

        t = t + ss.dt;
    end

    % trim log arrays to actual length
    fn = fieldnames(log);
    for k = 1:length(fn)
        log.(fn{k}) = log.(fn{k})(1:n_logged);
    end

    fprintf('Full ascent run: liftoff through MECO\n');
    fprintf('----------------------------------------\n');

    if isempty(meco_state)
        fprintf('MECO NOT reached within t_max = %.0f s\n', ss.t_max);
        ok = false;
    else
        h = meco_state(1); V = meco_state(3); gamma = meco_state(4);
        Vc = sqrt(ep.mu / (ep.Re + h));
        deficit = Vc - V;

        fprintf('Stage separation at     : t = %.2f s\n', stage_sep_time);
        fprintf('MECO at                 : t = %.2f s\n', meco_time);
        fprintf('MECO altitude           : %.2f km\n', h/1000);
        fprintf('MECO velocity           : %.1f m/s\n', V);
        fprintf('MECO flight-path angle  : %.2f deg\n', rad2deg(gamma));
        fprintf('Local circular velocity : %.1f m/s\n', Vc);
        fprintf('Velocity deficit        : %.1f m/s (%.1f%% of circular velocity)\n', ...
                deficit, deficit/Vc*100);
        fprintf('Max dynamic pressure    : %.2f kPa at t = %.1f s (stage %d)\n', ...
                max_q/1000, max_q_time, max_q_stage);

        pitch_err = log.theta_cmd_deg - log.theta_deg;
        rms_pitch_err = sqrt(mean(pitch_err.^2));
        fprintf('RMS pitch tracking error: %.3f deg\n', rms_pitch_err);

        sane = all(isfinite(log.h)) && all(isfinite(log.V)) && all(log.h >= -1);
        maxq_during_stage1 = (max_q_stage == 1);

        ok = sane && maxq_during_stage1 && (rms_pitch_err < 2.0);
    end

    fprintf('\n%s\n', ternary_str(ok, 'PASS', 'FAIL'));

    % --- Save telemetry, plots, animation ---
    data_dir = fullfile(this_dir, '..', 'data', 'telemetry');
    fig_dir  = fullfile(this_dir, '..', 'results', 'figures');
    anim_dir = fullfile(this_dir, '..', 'results', 'animations');

    if ~exist(data_dir, 'dir'); mkdir(data_dir); end
    save(fullfile(data_dir, 'ascent_telemetry.mat'), 'telemetry');
    fprintf('\nSaved %d telemetry packets to %s\n', length(telemetry), ...
            fullfile(data_dir, 'ascent_telemetry.mat'));

    plot_ascent_telemetry(log, meco_time, stage_sep_time, max_q_time, fig_dir);
    plot_trajectory_3d(log, stage_sep_idx, meco_idx, fig_dir);

    frame_stride = max(1, round(1.0 / ss.dt)); % ~1 animation frame per sim-second
    animate_ascent(log, stage_sep_idx, meco_idx, anim_dir, frame_stride);

    result.log = log;
    result.telemetry = telemetry;
    result.stage_sep_time = stage_sep_time;
    result.meco_time = meco_time;
    result.meco_state = meco_state;
    result.max_q = max_q;
    result.max_q_time = max_q_time;
    result.ok = ok;

    if ~ok
        error('main_ascent_full_run:FAIL', 'Ascent run did not meet PASS criteria.');
    end

end


function dstate = ascent_step_wrapper(t, s, sp, pid_gains, pid_state, ep, ss)
% Adapts ascent_derivatives' (dstate, diag) signature to the single-
% output form rk4_step.m expects.
    [dstate, ~] = ascent_derivatives(t, s, sp, pid_gains, pid_state, ep, ss);
end

function s = ternary_str(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end
