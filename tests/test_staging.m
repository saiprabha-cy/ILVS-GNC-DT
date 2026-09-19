function test_staging()
% TEST_STAGING  Unit tests for src/staging/staging_check.m.

    vp = vehicle_parameters();
    dummy_state = [10000; 1000; 500; deg2rad(45); deg2rad(45); 0; vp.liftoff_mass];

    % Stage 1, propellant not yet depleted -> stays stage 1, no event
    [stage_next, sp, event] = staging_check(1, dummy_state, vp, vp.s1.prop_mass * 0.5);
    assert(stage_next == 1, 'should remain stage 1 before propellant depletion');
    assert(strcmp(event, 'none'), 'should report no event before depletion');
    assert(sp.thrust == vp.s1.thrust, 'should report stage-1 thrust');
    assert(sp.separated == false, 'separated flag should be false');

    % Stage 1, propellant exactly depleted -> separation event
    [stage_next, sp, event] = staging_check(1, dummy_state, vp, vp.s1.prop_mass);
    assert(stage_next == 2, 'should advance to stage 2 at full depletion');
    assert(strcmp(event, 'stage_separation'), 'should report stage_separation event');
    assert(sp.separated == true, 'separated flag should be true on the event step');

    % Stage 2, propellant not yet depleted -> stays stage 2, no event
    [stage_next, sp, event] = staging_check(2, dummy_state, vp, vp.s2.prop_mass * 0.5);
    assert(stage_next == 2, 'should remain stage 2 before propellant depletion');
    assert(strcmp(event, 'none'), 'should report no event before depletion');
    assert(sp.thrust == vp.s2.thrust, 'should report stage-2 thrust');

    % Stage 2, propellant depleted -> MECO event
    [stage_next, sp, event] = staging_check(2, dummy_state, vp, vp.s2.prop_mass);
    assert(stage_next == 2, 'stage should remain 2 at MECO (no stage 3)');
    assert(strcmp(event, 'meco'), 'should report meco event');

    fprintf('test_staging: all assertions passed\n');

end
