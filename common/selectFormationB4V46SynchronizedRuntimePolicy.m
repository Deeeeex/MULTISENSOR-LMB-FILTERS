function [adjacency, details] = ...
    selectFormationB4V46SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V46SYNCHRONIZEDRUNTIMEPOLICY Frozen repaired sync arm.

[adjacency, details] = buildFormationB4V46FixedRuntimeArm( ...
    context, 'v46-repaired-sync-all-b4-e20-mc');
end
