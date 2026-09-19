function pid_state = pid_init()
% PID_INIT  Zero-initialized PID controller state.
%
%   Returns a struct matching what src/control/pid_update.m expects as
%   pid_state_in. Call once before the first pid_update() call, and
%   again whenever the controller should be reset (this project resets
%   it at stage separation -- see docs/01_design_decisions.md for why).

    pid_state.integral = 0.0;
    pid_state.prev_err = 0.0;
    pid_state.first    = true;

end
