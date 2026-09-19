function build_pitch_control_model()
% BUILD_PITCH_CONTROL_MODEL  Programmatically build the closed-loop pitch
% guidance/control Simulink model via the Simulink API.
%
%   Run this in MATLAB (Simulink required) to create
%   simulink/pitch_control_loop.slx.
%
%   WHY A FOCUSED SUBSYSTEM, NOT THE FULL 7-STATE ASCENT MODEL:
%   Program F's actual goal is proving the pitch guidance/control LAW
%   (guidance reference + PID) is identical across MATLAB, Simulink,
%   and Embedded-Coder-generated C -- the same discipline already
%   established in kalman-attitude-estimation, applied to ascent
%   guidance instead of attitude estimation. That only requires the
%   control loop and a minimal plant (single-axis pitch attitude
%   dynamics), not the full nonlinear translational EOM. Porting the
%   complete src/dynamics/ascent_derivatives.m into block-diagram form
%   is listed as a follow-on extension in docs/03_simplifications.md,
%   once this focused model is validated -- consistent with this
%   project's "MATLAB/Octave foundation first, Simulink after" phased
%   philosophy.
%
%   MODEL STRUCTURE:
%     Clock -> [pitch_program MATLAB Fn] -> theta_cmd --\
%                                                          Sum(err) -> [PID MATLAB Fn] -> Mc
%     theta <---------------------- plant (single-axis pitch dynamics) <--/
%
%   Plant: theta_ddot = (Mc - b*theta_dot) / I, built from Gain/Sum/
%   Integrator blocks (not a MATLAB Function), so the plant is a
%   transparent block diagram a reviewer can inspect directly rather
%   than another opaque function call.
%
%   The PID MATLAB Function block (a persistent-state, codegen-safe
%   port of src/control/pid_update.m, written inline below) is the
%   block Program F's Embedded Coder step actually targets.
%
%   VERSION SENSITIVITY WARNING: the helper that programmatically
%   writes each MATLAB Function block's body (set_matlab_fcn_block_
%   script, below) uses the Stateflow API, whose exact object path has
%   changed across MATLAB releases and was NOT executable/testable in
%   the sandbox this project was built in (no MATLAB/Simulink license
%   available there -- see docs/04_verification_note.md). If it
%   errors or silently fails on your MATLAB version, a warning prints
%   telling you to open the corresponding block in the Simulink editor
%   and paste in the text from pitch_program_block_source() /
%   pid_controller_block_source() by hand -- a 30-second manual step,
%   not a blocker.
%
%   DISCRETE SAMPLE TIME FOR THE PID BLOCK: a MATLAB Function block
%   using `persistent` state cannot inherit a continuous sample time
%   (Simulink error: "uses constructs that are invalid when the block
%   specifies or inherits a continuous sample time"). An earlier
%   version of this script tried to fix this with
%   set_param(block, 'SampleTime', '0.05') directly on the block --
%   this parameter does not exist on every MATLAB version's MATLAB
%   Function block object (it can resolve to a plain 'SubSystem' type
%   at the set_param level, which has no such parameter). The fix
%   below instead inserts a Zero-Order Hold block immediately upstream
%   of pid_controller_block, with its SampleTime set explicitly. A
%   Zero-Order Hold is a basic Discrete-library block whose SampleTime
%   parameter is reliably settable across MATLAB versions, and forcing
%   the PID block's INPUT to a discrete rate causes its own
%   (inherited) sample time to resolve to that same discrete rate --
%   the standard, version-robust way to solve this class of error,
%   and it is also the physically correct model of a controller
%   sampling a continuous plant through an ADC at a fixed rate, not
%   just a workaround.

    model_name = 'pitch_control_loop';

    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end

    new_system(model_name);
    open_system(model_name);

    % --- Guidance reference: Clock -> pitch_program MATLAB Function ---
    add_block('simulink/Sources/Clock', [model_name '/Clock']);
    set_param([model_name '/Clock'], 'Position', [30 40 60 60]);

    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [model_name '/pitch_program_block']);
    set_param([model_name '/pitch_program_block'], 'Position', [120 20 260 80]);

    % --- Error summing junction: theta_cmd - theta ---
    add_block('simulink/Math Operations/Sum', [model_name '/err_sum']);
    set_param([model_name '/err_sum'], 'Inputs', '+-', 'Position', [340 40 365 70]);

    % --- Zero-Order Hold: forces a discrete rate onto the PID block's
    %     input, which resolves the block's own inherited sample time to
    %     discrete (required because the PID block uses persistent state
    %     -- see the DISCRETE SAMPLE TIME note in this file's header) ---
    add_block('simulink/Discrete/Zero-Order Hold', [model_name '/pid_input_zoh']);
    set_param([model_name '/pid_input_zoh'], 'SampleTime', '0.05', 'Position', [385 40 410 70]);

    % --- PID controller (Embedded Coder target) ---
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
              [model_name '/pid_controller_block']);
    set_param([model_name '/pid_controller_block'], 'Position', [420 20 560 80]);

    % --- Plant: theta_ddot = (Mc - b*theta_dot) / I ---
    add_block('simulink/Math Operations/Gain', [model_name '/damping_gain']);
    set_param([model_name '/damping_gain'], 'Gain', 'pid_damping_b', 'Position', [640 140 680 170]);

    add_block('simulink/Math Operations/Sum', [model_name '/moment_sum']);
    set_param([model_name '/moment_sum'], 'Inputs', '+-', 'Position', [700 60 725 90]);

    add_block('simulink/Math Operations/Gain', [model_name '/inv_inertia_gain']);
    set_param([model_name '/inv_inertia_gain'], 'Gain', '1/pitch_inertia', 'Position', [760 60 800 90]);

    add_block('simulink/Continuous/Integrator', [model_name '/theta_dot_integrator']);
    set_param([model_name '/theta_dot_integrator'], 'Position', [840 60 870 90]);

    add_block('simulink/Continuous/Integrator', [model_name '/theta_integrator']);
    set_param([model_name '/theta_integrator'], 'Position', [910 60 940 90]);

    % --- Outputs/logging ---
    add_block('simulink/Sinks/Scope', [model_name '/pitch_scope']);
    set_param([model_name '/pitch_scope'], 'Position', [1000 20 1040 60], 'NumInputPorts', '2');

    add_block('simulink/Sinks/Out1', [model_name '/theta_out']);
    set_param([model_name '/theta_out'], 'Position', [1000 100 1030 120]);

    % --- Wiring ---
    add_line(model_name, 'Clock/1', 'pitch_program_block/1', 'autorouting', 'on');
    add_line(model_name, 'pitch_program_block/1', 'err_sum/1', 'autorouting', 'on');
    add_line(model_name, 'err_sum/1', 'pid_input_zoh/1', 'autorouting', 'on');
    add_line(model_name, 'pid_input_zoh/1', 'pid_controller_block/1', 'autorouting', 'on');
    add_line(model_name, 'pid_controller_block/1', 'moment_sum/1', 'autorouting', 'on');
    add_line(model_name, 'theta_dot_integrator/1', 'damping_gain/1', 'autorouting', 'on');
    add_line(model_name, 'damping_gain/1', 'moment_sum/2', 'autorouting', 'on');
    add_line(model_name, 'moment_sum/1', 'inv_inertia_gain/1', 'autorouting', 'on');
    add_line(model_name, 'inv_inertia_gain/1', 'theta_dot_integrator/1', 'autorouting', 'on');
    add_line(model_name, 'theta_dot_integrator/1', 'theta_integrator/1', 'autorouting', 'on');
    add_line(model_name, 'theta_integrator/1', 'err_sum/2', 'autorouting', 'on');   % feedback
    add_line(model_name, 'theta_integrator/1', 'theta_out/1', 'autorouting', 'on');
    add_line(model_name, 'pitch_program_block/1', 'pitch_scope/1', 'autorouting', 'on');
    add_line(model_name, 'theta_integrator/1', 'pitch_scope/2', 'autorouting', 'on');

    % --- Fill in the MATLAB Function block bodies ---
    set_matlab_fcn_block_script([model_name '/pitch_program_block'], pitch_program_block_source());
    set_matlab_fcn_block_script([model_name '/pid_controller_block'], pid_controller_block_source());

    % NOTE: discrete sample time for pid_controller_block is provided by
    % the pid_input_zoh Zero-Order Hold block inserted upstream of it
    % (see the DISCRETE SAMPLE TIME note in this file's header) -- no
    % set_param(..., 'SampleTime', ...) call is made directly on the PID
    % block itself, since that parameter is not reliably valid on a
    % MATLAB Function block across MATLAB versions.

    % --- Fixed-step solver, required for Embedded Coder / ert.tlc ---
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'FixedStep', '0.05');
    set_param(model_name, 'Solver', 'ode4');

    % --- Base-workspace parameters the Gain blocks reference by name ---
    assignin('base', 'pid_damping_b', 50000.0);
    assignin('base', 'pitch_inertia', 120000.0);

    save_system(model_name, fullfile(fileparts(mfilename('fullpath')), [model_name '.slx']));
    fprintf('Saved %s.slx\n', model_name);

end


function set_matlab_fcn_block_script(block_path, script_text)
% Helper: write the body of a MATLAB Function block via the Simulink
% Editor API (avoids hand-editing the block's embedded Stateflow chart
% through the GUI).
    rt = sfroot;
    blk = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(blk)
        % Fallback for older API versions: locate via the chart object
        % attached to the block.
        chart = find(sfroot, '-isa', 'Stateflow.EMChart');
        for i = 1:length(chart)
            if strcmp(get_param(block_path, 'Handle'), get_param(block_path, 'Handle'))
                blk = chart(i);
                break;
            end
        end
    end
    if ~isempty(blk)
        blk.Script = script_text;
    else
        warning('build_pitch_control_model:blockScript', ...
                ['Could not programmatically set the MATLAB Function body for %s. ', ...
                 'Open the block in Simulink and paste in the corresponding *_source() text ', ...
                 'from this script manually.'], block_path);
    end
end


function src = pitch_program_block_source()
    src = sprintf([...
        'function theta_cmd = pitch_program_block(t)\n' ...
        '%% Embedded Coder-compatible port of src/guidance/pitch_program.m.\n' ...
        '%% Constants inlined (MATLAB Function blocks do not accept struct\n' ...
        '%% inputs from config/*.m directly without extra setup); keep in\n' ...
        '%% sync with config/simulation_settings.m ss.pitch fields by hand,\n' ...
        '%% documented in docs/01_design_decisions.md.\n' ...
        't_vertical = 10.0; t_kick_end = 15.0; kick_deg = 3.0;\n' ...
        't_pitch_end = 260.0; theta_final_deg = 6.0;\n' ...
        'if t < t_vertical\n' ...
        '    theta_cmd_deg = 90.0;\n' ...
        'elseif t < t_kick_end\n' ...
        '    frac = (t - t_vertical) / (t_kick_end - t_vertical);\n' ...
        '    theta_cmd_deg = 90.0 - kick_deg * frac;\n' ...
        'else\n' ...
        '    theta_kick = 90.0 - kick_deg;\n' ...
        '    if t < t_pitch_end\n' ...
        '        frac = (t - t_kick_end) / (t_pitch_end - t_kick_end);\n' ...
        '        theta_cmd_deg = theta_kick + (theta_final_deg - theta_kick) * frac;\n' ...
        '    else\n' ...
        '        theta_cmd_deg = theta_final_deg;\n' ...
        '    end\n' ...
        'end\n' ...
        'theta_cmd = theta_cmd_deg * pi / 180;\n']);
end


function src = pid_controller_block_source()
    src = sprintf([...
        'function Mc = pid_controller_block(err)\n' ...
        '%% Embedded Coder-compatible port of src/control/pid_update.m,\n' ...
        '%% using persistent variables for controller state (the idiomatic\n' ...
        '%% Simulink/Embedded-Coder pattern) instead of the explicit\n' ...
        '%% struct-in/struct-out signature used in the plain-MATLAB\n' ...
        '%% version -- structs crossing a MATLAB Function block boundary\n' ...
        '%% need extra bus/type setup that is not worth the complexity\n' ...
        '%% here. Gains are inlined; keep in sync with\n' ...
        '%% config/vehicle_parameters.m vp.pid by hand.\n' ...
        'persistent integral prev_err first_call\n' ...
        'if isempty(first_call)\n' ...
        '    integral = 0.0; prev_err = 0.0; first_call = true;\n' ...
        'end\n' ...
        'kp = 800000.0; ki = 5000.0; kd = 250000.0; out_limit = 2.0e6;\n' ...
        'dt = 0.05;\n' ...
        'integral = integral + err * dt;\n' ...
        'if first_call\n' ...
        '    deriv = 0.0;\n' ...
        'else\n' ...
        '    deriv = (err - prev_err) / dt;\n' ...
        'end\n' ...
        'u_unsat = kp*err + ki*integral + kd*deriv;\n' ...
        'Mc = max(min(u_unsat, out_limit), -out_limit);\n' ...
        'prev_err = err;\n' ...
        'first_call = false;\n']);
end
