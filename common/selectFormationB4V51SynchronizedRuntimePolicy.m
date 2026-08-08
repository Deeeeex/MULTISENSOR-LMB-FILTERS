function [adjacency, details] = ...
    selectFormationB4V51SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V51SYNCHRONIZEDRUNTIMEPOLICY V51 wrapper.

[adjacency, details] = buildFormationB4V51FixedRuntimeArm( ...
    context, 'v51-retention-gated-sync-b4-e20');
end
