function test_budget_recycled_formation_repair_update_schedule_v205()
% Focused mixed-operator schedule contract for supported/missing labels.

legacy = normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
    'label_kla', 0.5, 3, true);
assert(isequal(legacy.modes, {'label-kla', 'label-kla', 'label-kla'}));
assert(isequal(legacy.sourceWeights, [0.5, 0.5, 0.5]));
assert(all(legacy.labelKlaMask) && ~any(legacy.hardReplacementMask));
assert(~legacy.mixedUpdateModes);

mixed = normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
    {'label-kla', 'hard_replacement', 'label-kla'}, ...
    [0.5, 1.0, 0.25], 3, true);
assert(isequal(mixed.modes, ...
    {'label-kla', 'hard-replacement', 'label-kla'}));
assert(isequal(mixed.hardReplacementMask, [false, true, false]));
assert(isequal(mixed.labelKlaMask, [true, false, true]));
assert(mixed.mixedUpdateModes);

assertThrows(@() ...
    normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
        {'label-kla', 'hard-replacement'}, [0.5, 0.5], 2, true));
assertThrows(@() ...
    normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
        'label-kla', 0.5, 2, false));
assertThrows(@() ...
    normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
        {'label-kla'}, 0.5, 2, true));

fprintf([ ...
    'test_budget_recycled_formation_repair_update_schedule_v205 ', ...
    'passed.\n']);
end

function assertThrows(callback)
threw = false;
try
    callback();
catch
    threw = true;
end
assert(threw);
end
