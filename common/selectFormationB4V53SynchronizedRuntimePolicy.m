function [adjacency, details] = ...
    selectFormationB4V53SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V53SYNCHRONIZEDRUNTIMEPOLICY V53 wrapper.

[adjacency, details] = buildFormationB4V53FixedRuntimeArm( ...
    context, 'v53-exact-selective-cross-pulse-b4-e20');
end
