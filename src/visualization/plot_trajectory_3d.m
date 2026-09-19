function plot_trajectory_3d(log, stage_sep_idx, meco_idx, out_dir)
% PLOT_TRAJECTORY_3D  Static 3D ascent trajectory (downrange-x, downrange-y, altitude).
%
%   PLOT_TRAJECTORY_3D(log, stage_sep_idx, meco_idx, out_dir)
%
%   The ascent dynamics are planar (vertical-plane, 2D translation), so
%   the "3D" plot renders the trajectory in a plane embedded in 3D
%   space (downrange along X, altitude along Z, Y held at zero) --
%   this is stated plainly rather than implying a true out-of-plane
%   trajectory the underlying 2-DOF model doesn't actually produce. A
%   6-DOF extension with real cross-range motion is listed in
%   docs/03_simplifications.md as future work.

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    fig = figure('Visible', 'off', 'Position', [100 100 1000 700]);

    x_km = log.x / 1000;
    y_km = zeros(size(x_km));
    h_km = log.h / 1000;

    plot3(x_km, y_km, h_km, 'b-', 'LineWidth', 1.8);
    hold on; grid on;

    plot3(x_km(1), y_km(1), h_km(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);

    if ~isempty(stage_sep_idx) && stage_sep_idx <= length(x_km)
        plot3(x_km(stage_sep_idx), y_km(stage_sep_idx), h_km(stage_sep_idx), ...
              'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
    end

    if ~isempty(meco_idx) && meco_idx <= length(x_km)
        plot3(x_km(meco_idx), y_km(meco_idx), h_km(meco_idx), ...
              'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    end

    xlabel('Downrange (km)'); ylabel('Cross-range (km, unmodeled)'); zlabel('Altitude (km)');
    title('ILVS-GNC-DT: 3D Ascent Trajectory (planar model)');
    legend('Trajectory', 'Liftoff', 'Stage separation', 'MECO', 'Location', 'best');
    view(35, 20);

    out_path = fullfile(out_dir, 'trajectory_3d.png');
    print(fig, out_path, '-dpng', '-r130');
    close(fig);
    fprintf('Saved %s\n', out_path);

end
