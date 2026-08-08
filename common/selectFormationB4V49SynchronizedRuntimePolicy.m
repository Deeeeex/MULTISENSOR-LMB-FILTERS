function [adjacency, details] = ...
    selectFormationB4V49SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V49SYNCHRONIZEDRUNTIMEPOLICY Frozen V49 wrapper.

[adjacency, details] = buildFormationB4V49FixedRuntimeArm( ...
    context, 'v49-feasible-cycle-sync-b4-e20');
end
