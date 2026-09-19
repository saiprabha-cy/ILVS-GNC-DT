function animate_ascent(log, stage_sep_idx, meco_idx, out_dir, frame_stride)
% ANIMATE_ASCENT  Ascent + stage-separation animation, saved as an animated GIF.
%
%   ANIMATE_ASCENT(log, stage_sep_idx, meco_idx, out_dir, frame_stride)
%
%   Builds a simple vehicle glyph (a patch: a triangle nose + rectangle
%   body) whose position and orientation are updated each frame with
%   HGTRANSFORM, matching the pitch angle from the simulation log. At
%   stage_sep_idx, the body is drawn shrinking (representing the
%   jettisoned stage-1 structure) to make the separation event visually
%   obvious rather than just a marker on a plot.
%
%   frame_stride subsamples the log (e.g. 20 -> one animation frame
%   per 20 simulation steps = 1 s of sim time at dt=0.05) so the
%   ~5000-30000-step full-resolution log doesn't produce a multi-
%   thousand-frame GIF.
%
%   Uses only base MATLAB/Octave graphics (patch, hgtransform, makehgtform,
%   imwrite with DelayTime) -- no Simulink 3D Animation dependency, so
%   this runs even without that toolbox license. If Simulink 3D
%   Animation IS available, it is a natural upgrade path for a more
%   polished visualization -- see docs/01_design_decisions.md.

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end

    idx = 1:frame_stride:length(log.t);
    if isempty(idx) || idx(end) ~= length(log.t)
        idx = [idx, length(log.t)];
    end

    fig = figure('Visible', 'off', 'Position', [100 100 900 700]);
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'on');
    xlabel(ax, 'Downrange (km)'); ylabel(ax, 'Altitude (km)');
    title(ax, 'ILVS-GNC-DT: Ascent Animation');

    x_km = log.x / 1000;
    h_km = log.h / 1000;
    xlim(ax, [min(x_km)-5, max(x_km)+5]);
    ylim(ax, [0, max(h_km)*1.1 + 1]);

    plot(ax, x_km, h_km, 'b:', 'LineWidth', 0.6); % ghost trajectory for context

    vehicle_length_km = max(h_km) * 0.015; % visually scaled, not to physical scale
    body = patch(ax, [-1 1 1 -1]*vehicle_length_km*0.15, [-1 -1 1 1]*vehicle_length_km, ...
                 'w', 'EdgeColor', 'k', 'LineWidth', 1.2);
    nose = patch(ax, [-1 0 1]*vehicle_length_km*0.15, [1 1.6 1]*vehicle_length_km, ...
                 [0.8 0.1 0.1], 'EdgeColor', 'k');
    tgroup = hgtransform('Parent', ax);
    set(body, 'Parent', tgroup);
    set(nose, 'Parent', tgroup);

    out_gif = fullfile(out_dir, 'ascent_animation.gif');
    first_frame = true;
    separated_shown = false;

    for k = 1:length(idx)
        i = idx(k);
        px = x_km(i);
        py = h_km(i);
        pitch_rad = log.theta_deg(i) * pi / 180 - pi/2; % 0 deg pitch (vertical) -> glyph pointing up

        if ~isempty(stage_sep_idx) && i >= stage_sep_idx && ~separated_shown
            % Shrink the glyph once, at the frame nearest separation, to
            % visually represent the jettisoned stage-1 structure.
            set(body, 'XData', get(body,'XData')*0.6, 'YData', get(body,'YData')*0.6);
            separated_shown = true;
        end

        T = makehgtform('translate', [px, py, 0]) * makehgtform('zrotate', -pitch_rad);
        set(tgroup, 'Matrix', T);

        title(ax, sprintf('ILVS-GNC-DT Ascent  t = %.1f s  |  alt = %.1f km  |  V = %.0f m/s', ...
                           log.t(i), py, log.V(i)));

        drawnow;
        frame = getframe(fig);
        [imind, cm] = rgb2ind(frame.cdata, 256);
        if first_frame
            imwrite(imind, cm, out_gif, 'gif', 'Loopcount', inf, 'DelayTime', 0.08);
            first_frame = false;
        else
            imwrite(imind, cm, out_gif, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
        end
    end

    close(fig);
    fprintf('Saved %s (%d frames)\n', out_gif, length(idx));

end
