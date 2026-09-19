function plot_ascent_telemetry(log, meco_time, stage_sep_time, maxq_time, out_dir)
% PLOT_ASCENT_TELEMETRY  Multi-panel ascent engineering plots.
%
%   PLOT_ASCENT_TELEMETRY(log, meco_time, stage_sep_time, maxq_time, out_dir)
%
%   log is the struct-of-arrays produced by sim/main_ascent_full_run.m
%   (fields: t, h, x, V, gamma_deg, theta_deg, theta_cmd_deg, m, q,
%   alpha_deg). Saves a 2x3 panel figure to
%   <out_dir>/ascent_telemetry.png: altitude, velocity, dynamic
%   pressure (Max-Q marked), pitch (commanded vs actual), mass, and
%   flight-path angle, each vs. time, with stage separation and MECO
%   marked as vertical lines on every panel.
%
%   Octave-compatible: uses only base plotting calls (plot, subplot,
%   xline is avoided in favor of plotting a manual vertical line, since
%   xline's Octave support has historically lagged MATLAB's).

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    fig = figure('Visible', 'off', 'Position', [100 100 1400 800]);

    subplot(2,3,1);
    plot(log.t, log.h/1000, 'b-', 'LineWidth', 1.3);
    hold on; grid on;
    add_event_lines(log.t, [log.h/1000], stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Altitude (km)'); title('Altitude vs Time');

    subplot(2,3,2);
    plot(log.t, log.V, 'b-', 'LineWidth', 1.3);
    hold on; grid on;
    add_event_lines(log.t, log.V, stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Velocity (m/s)'); title('Velocity vs Time');

    subplot(2,3,3);
    plot(log.t, log.q/1000, 'r-', 'LineWidth', 1.3);
    hold on; grid on;
    add_event_lines(log.t, log.q/1000, stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Dynamic Pressure (kPa)'); title('Dynamic Pressure (Max-Q marked)');

    subplot(2,3,4);
    plot(log.t, log.theta_cmd_deg, 'k--', 'LineWidth', 1.0); hold on;
    plot(log.t, log.theta_deg, 'b-', 'LineWidth', 1.3);
    grid on;
    add_event_lines(log.t, log.theta_deg, stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Pitch (deg)'); title('Pitch: Commanded vs Actual');
    legend('Commanded', 'Actual', 'Location', 'best');

    subplot(2,3,5);
    plot(log.t, log.m, 'b-', 'LineWidth', 1.3);
    hold on; grid on;
    add_event_lines(log.t, log.m, stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Mass (kg)'); title('Vehicle Mass vs Time');

    subplot(2,3,6);
    plot(log.t, log.gamma_deg, 'b-', 'LineWidth', 1.3);
    hold on; grid on;
    add_event_lines(log.t, log.gamma_deg, stage_sep_time, meco_time, maxq_time);
    xlabel('Time (s)'); ylabel('Flight-Path Angle (deg)'); title('Flight-Path Angle vs Time');

    % sgtitle is not available in all Octave versions; degrade gracefully.
    try
        sgtitle('ILVS-GNC-DT: Ascent Telemetry (green=stage sep, red=MECO, magenta=Max-Q)');
    catch
        % no-op: per-panel titles already convey the same information
    end

    out_path = fullfile(out_dir, 'ascent_telemetry.png');
    print(fig, out_path, '-dpng', '-r130');
    close(fig);
    fprintf('Saved %s\n', out_path);

end


function add_event_lines(t, y, stage_sep_time, meco_time, maxq_time)
% Manual vertical event lines (Octave-compatible alternative to xline).
    yl = [min(y) max(y)];
    if ~isempty(stage_sep_time)
        plot([stage_sep_time stage_sep_time], yl, 'g--', 'LineWidth', 0.8);
    end
    if ~isempty(meco_time)
        plot([meco_time meco_time], yl, 'r--', 'LineWidth', 0.8);
    end
    if ~isempty(maxq_time)
        plot([maxq_time maxq_time], yl, 'm:', 'LineWidth', 0.8);
    end
end
