function configure_embedded_coder()
% CONFIGURE_EMBEDDED_CODER  Configure and run Embedded Coder code
% generation on the pitch_control_loop model.
%
%   Run AFTER build_pitch_control_model.m (or with pitch_control_loop.slx
%   already open). Requires Simulink Coder and Embedded Coder licenses.
%
%   This targets portable ANSI/ISO C89/C90 with no specific hardware
%   board selected -- the same "generate and cross-validate" pattern
%   used in kalman-attitude-estimation, NOT a claim of building for a
%   real flight computer. Selecting an actual embedded target (e.g. an
%   STM32 board support package, matching the target already used in
%   aerospace-telemetry-flight-computer-platform) is a documented next
%   step once this generic-C path is validated -- see
%   docs/03_simplifications.md.
%
%   After this runs, generated C sits in
%   simulink/pitch_control_loop_ert_rtw/ (or similar, exact folder name
%   is MATLAB-version-dependent -- printed at the end of this
%   function). Compare it against embedded/pitch_pid.c (the
%   independently hand-written reference implementation) using
%   sim/main_embedded_crossvalidation.m.
%
%   KNOWN GOTCHA, already handled below: the ert.tlc target disables
%   continuous-time code generation by default (it assumes a purely
%   discrete embedded target), which fails codegen with "Block ... uses
%   continuous time, which is not supported with the current
%   configuration" for every continuous block (Clock, the plant's
%   Integrators) if this model has any continuous states -- which it
%   does, since only the PID controller is discrete (via the
%   pid_input_zoh Zero-Order Hold in build_pitch_control_model.m), not
%   the plant. SupportContinuousTime is set 'on' below to fix this.

    model_name = 'pitch_control_loop';

    if ~bdIsLoaded(model_name)
        model_path = fullfile(fileparts(mfilename('fullpath')), [model_name '.slx']);
        if ~exist(model_path, 'file')
            error('configure_embedded_coder:modelNotFound', ...
                  ['%s.slx not found. Run build_pitch_control_model.m first.'], model_name);
        end
        load_system(model_path);
    end

    % --- Target: generic portable C, Embedded Coder ---
    set_param(model_name, 'SystemTargetFile', 'ert.tlc');
    set_param(model_name, 'TemplateMakefile', 'ert_default_tmf');
    set_param(model_name, 'GenerateMakefile', 'on');
    set_param(model_name, 'TargetLang', 'C');

    % --- Fixed-step solver: mandatory for ERT/Embedded Coder ---
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'FixedStep', '0.05');
    set_param(model_name, 'Solver', 'ode4');

    % --- Continuous-time support: REQUIRED, and not on by default ---
    % The ert.tlc target assumes a purely discrete embedded target
    % unless told otherwise, and by default disables support for
    % continuous states/blocks to produce leaner code for that
    % assumption. This model's plant (theta_dot_integrator,
    % theta_integrator) is genuinely continuous -- only the PID
    % controller is discrete, sampled via the pid_input_zoh Zero-Order
    % Hold (see simulink/build_pitch_control_model.m) -- so continuous-
    % time support must be explicitly enabled, or codegen fails with
    % "Block ... uses continuous time, which is not supported with the
    % current configuration" for every continuous block in the model
    % (Clock, both Integrators). This is exactly the ODE4 fixed-step
    % continuous solver already selected above; SupportContinuousTime
    % tells the CODE GENERATOR (not the simulation solver, which was
    % already working) to actually emit the corresponding continuous
    % integration code.
    set_param(model_name, 'SupportContinuousTime', 'on');

    % --- Code generation report + traceability, for the kind of review
    %     a real avionics team would actually do (Program F deliverable:
    %     "document the code-generation report") ---
    set_param(model_name, 'GenerateReport', 'on');
    set_param(model_name, 'LaunchReport', 'off');
    set_param(model_name, 'GenerateTraceInfo', 'on');
    set_param(model_name, 'GenerateTraceReport', 'on');

    % --- Code style/metrics settings a reviewer would look for ---
    set_param(model_name, 'ERTCustomFileBanners', 'on');
    set_param(model_name, 'GenerateComments', 'on');
    set_param(model_name, 'ShowEliminatedStatement', 'off');
    set_param(model_name, 'IncludeMdlTerminateFcn', 'on');

    save_system(model_name);

    fprintf('Embedded Coder configuration applied to %s.\n', model_name);
    fprintf('Running code generation (rtwbuild)...\n');

    rtwbuild(model_name);

    build_dir = fullfile(pwd, [model_name '_ert_rtw']);
    fprintf('\nCode generation complete.\n');
    fprintf('Generated C source directory: %s\n', build_dir);
    fprintf('Code generation report: %s\n', ...
            fullfile(build_dir, 'html', [model_name '_codegen_rpt.html']));
    fprintf(['\nNext: run sim/main_embedded_crossvalidation.m to compare the generated\n' ...
             'C output against embedded/pitch_pid.c (the hand-written reference) and\n' ...
             'against the plain-MATLAB src/control/pid_update.m, over the same test\n' ...
             'vectors, reporting a numeric max error -- not just "it compiled".\n']);

end
