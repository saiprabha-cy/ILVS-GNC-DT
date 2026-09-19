function main_embedded_crossvalidation()
% MAIN_EMBEDDED_CROSSVALIDATION  Cross-validate src/control/pid_update.m
% against embedded/pitch_pid.c over an identical, realistic error
% sequence, via CSV file interchange (no MEX compilation required).
%
%   This is Program F's core validation step for the hand-written C
%   reference. It compiles embedded/vector_runner.c with your system's
%   C compiler (gcc/clang/MSVC via `mex -setup` toolchain, or any `cc`
%   on the PATH), runs both implementations over the same inputs, and
%   reports the maximum absolute difference -- not just "outputs look
%   similar".
%
%   IMPORTANT: this validates the plain-MATLAB implementation against
%   the HAND-WRITTEN C reference (embedded/pitch_pid.c), which was
%   independently compiled and verified (all 6 test_harness.c vectors
%   passing to machine precision) in the sandbox this project was
%   built in, since Embedded Coder itself was not available there --
%   see docs/04_verification_note.md. Once you run
%   simulink/configure_embedded_coder.m yourself, extend the
%   comparison in this script to the ACTUAL generated C too (the
%   generated step function's signature will depend on your MATLAB
%   version's Embedded Coder output; wire it in following the same
%   CSV-interchange pattern, or write a small MEX wrapper if you
%   prefer an in-process call).

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'src', 'utilities'));
    add_project_paths();

    vp = vehicle_parameters();
    embedded_dir = fullfile(this_dir, '..', 'embedded');

    % --- 1. Build a realistic error sequence: simulate what the pitch
    %        tracking error might look like during the pitch-over kick
    %        and early gravity turn (a step then a decaying oscillation
    %        is a reasonable stand-in without running the full plant). ---
    dt = 0.05;
    n = 2000; % 100 s
    t = (0:n-1)' * dt;
    err = deg2rad(3.0) * exp(-t/20) .* cos(2*pi*t/8) + deg2rad(0.5) * sin(2*pi*t/50);

    % --- 2. Run through plain MATLAB implementation ---
    pid_state = pid_init();
    u_matlab = zeros(n, 1);
    for i = 1:n
        [u_matlab(i), pid_state] = pid_update(err(i), dt, vp.pid, pid_state);
    end

    % --- 3. Write inputs to CSV, compile + run the C reference ---
    input_csv = fullfile(embedded_dir, 'crossval_input.csv');
    output_csv = fullfile(embedded_dir, 'crossval_output.csv');

    fid = fopen(input_csv, 'w');
    for i = 1:n
        fprintf(fid, '%.15e,%.15e\n', err(i), dt);
    end
    fclose(fid);

    runner_exe = fullfile(embedded_dir, 'vector_runner');
    if ispc
        runner_exe = [runner_exe '.exe'];
    end

    compile_cmd = sprintf('gcc -std=c89 -O2 -o "%s" "%s" "%s" -lm', ...
                           runner_exe, ...
                           fullfile(embedded_dir, 'vector_runner.c'), ...
                           fullfile(embedded_dir, 'pitch_pid.c'));
    fprintf('Compiling C reference: %s\n', compile_cmd);
    compile_status = system(compile_cmd);
    if compile_status ~= 0
        error('main_embedded_crossvalidation:compileFailed', ...
              'C compilation failed (status %d). Ensure gcc (or another C compiler on PATH) is available.', ...
              compile_status);
    end

    run_cmd = sprintf('"%s" "%s" "%s" %.15g %.15g %.15g %.15g', ...
                       runner_exe, input_csv, output_csv, ...
                       vp.pid.kp, vp.pid.ki, vp.pid.kd, vp.pid.out_limit);
    fprintf('Running C reference: %s\n', run_cmd);
    run_status = system(run_cmd);
    if run_status ~= 0
        error('main_embedded_crossvalidation:runFailed', 'C reference run failed (status %d).', run_status);
    end

    % --- 4. Read C output, compare ---
    u_c = dlmread(output_csv); %#ok<DLMRD> % dlmread is Octave/older-MATLAB compatible

    assert(length(u_c) == n, 'C output row count (%d) does not match input row count (%d)', length(u_c), n);

    abs_diff = abs(u_matlab - u_c);
    max_diff = max(abs_diff);
    rms_diff = sqrt(mean(abs_diff.^2));

    fprintf('\nCross-validation: MATLAB vs. hand-written C reference\n');
    fprintf('---------------------------------------------------------\n');
    fprintf('Test vector length      : %d samples (%.0f s at dt=%.2f s)\n', n, n*dt, dt);
    fprintf('Max absolute difference : %.3e\n', max_diff);
    fprintf('RMS difference          : %.3e\n', rms_diff);

    ok = max_diff < 1e-9;
    fprintf('\n%s\n', ternary_str(ok, 'PASS', 'FAIL'));

    % Clean up generated interchange files and binary (not build artifacts
    % worth committing to the repo)
    delete(input_csv);
    delete(output_csv);
    if exist(runner_exe, 'file')
        delete(runner_exe);
    end

    if ~ok
        error('main_embedded_crossvalidation:FAIL', 'MATLAB and C implementations disagree beyond tolerance.');
    end

end

function s = ternary_str(cond, a, b)
    if cond
        s = a;
    else
        s = b;
    end
end
