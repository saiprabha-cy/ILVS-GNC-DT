function run_all_tests()
% RUN_ALL_TESTS  Run every unit test file in tests/ and report a summary.
%
%   Octave-compatible replacement for a formal test framework (MATLAB's
%   `runtests` requires a toolbox not guaranteed to be present, and the
%   project targets GNU Octave compatibility throughout) -- each
%   test_*.m function raises via `assert` on failure, which this
%   collector catches with try/catch so one failing test doesn't stop
%   the rest from running.

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(this_dir, '..', 'src', 'utilities'));
    add_project_paths();

    test_fns = {
        @test_atmosphere, ...
        @test_gravity, ...
        @test_pid, ...
        @test_staging, ...
        @test_telemetry_crc ...
    };
    test_names = {
        'test_atmosphere', 'test_gravity', 'test_pid', 'test_staging', 'test_telemetry_crc'
    };

    n_pass = 0;
    n_total = length(test_fns);

    fprintf('Running %d test files\n', n_total);
    fprintf('----------------------\n');

    for i = 1:n_total
        try
            test_fns{i}();
            fprintf('PASS  %s\n', test_names{i});
            n_pass = n_pass + 1;
        catch err
            fprintf('FAIL  %s: %s\n', test_names{i}, err.message);
        end
    end

    fprintf('\n%d/%d test files passed\n', n_pass, n_total);

    if n_pass < n_total
        error('run_all_tests:FAIL', '%d test file(s) failed.', n_total - n_pass);
    end

end
