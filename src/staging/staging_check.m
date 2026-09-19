function [stage_next, stage_props, event] = staging_check(stage_current, state, vp, prop_consumed)
% STAGING_CHECK  Discrete-event staging state machine.
%
%   [stage_next, stage_props, event] = STAGING_CHECK(stage_current, ...
%                                          state, vp, prop_consumed)
%
%   A real staging event is a discontinuity (mass, thrust, aerodynamic
%   and inertia properties all change instantaneously), not something
%   a continuous integrator should smooth over -- so this is written as
%   an explicit state machine checked once per integration step, not
%   folded into the continuous dynamics. This mirrors the
%   Stateflow-chart structure this logic ports to in Program H
%   (simulink/build_ascent_model.m adds an equivalent MATLAB Function
%   block implementing the same transition logic).
%
%   Inputs:
%     stage_current   - 1 or 2
%     state           - current 7-state vector (only state(7)=mass used
%                        directly; prop_consumed is tracked by the
%                        caller since jettisoning stage-1 dry mass at
%                        separation makes "propellant consumed" NOT
%                        simply "initial mass minus current mass" once
%                        stage 2 is active)
%     vp              - vehicle_parameters() struct
%     prop_consumed   - propellant mass burned in the CURRENT stage, kg
%
%   Outputs:
%     stage_next   - 1 or 2 (possibly advanced)
%     stage_props  - struct with thrust, mdot, cd, cn_alpha, area,
%                    inertia for stage_next, and a `separated` flag
%                    (true only on the exact step separation occurs)
%     event        - 'none', 'stage_separation', or 'meco'
%                    (meco = main engine cutoff, stage-2 propellant
%                    depleted; caller stops integration on this event)

    stage_next = stage_current;
    event = 'none';
    stage_props = struct();
    stage_props.separated = false;

    if stage_current == 1
        if prop_consumed >= vp.s1.prop_mass
            stage_next = 2;
            event = 'stage_separation';
            stage_props.separated = true;
        else
            stage_props.thrust   = vp.s1.thrust;
            stage_props.mdot     = vp.s1.mdot;
            stage_props.cd       = vp.s1.cd;
            stage_props.cn_alpha = vp.s1.cn_alpha;
            stage_props.area     = vp.s1.area;
            stage_props.inertia  = vp.s1.inertia;
            return;
        end
    end

    % stage_next == 2 here, either just separated or already stage 2
    if prop_consumed >= vp.s2.prop_mass && stage_current == 2
        event = 'meco';
    end

    stage_props.thrust   = vp.s2.thrust;
    stage_props.mdot     = vp.s2.mdot;
    stage_props.cd       = vp.s2.cd;
    stage_props.cn_alpha = vp.s2.cn_alpha;
    stage_props.area     = vp.s2.area;
    stage_props.inertia  = vp.s2.inertia;

end
