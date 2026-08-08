function [adjacency, details] = ...
    selectFormationB4V50SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V50SYNCHRONIZEDRUNTIMEPOLICY V50 wrapper.

[adjacency, details] = buildFormationB4V50FixedRuntimeArm( ...
    context, 'v50-posterior-aware-cycle-sync-b4-e20');
end
