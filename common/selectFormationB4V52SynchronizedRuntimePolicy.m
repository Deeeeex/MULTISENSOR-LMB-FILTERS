function [adjacency, details] = ...
    selectFormationB4V52SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V52SYNCHRONIZEDRUNTIMEPOLICY V52 wrapper.

[adjacency, details] = buildFormationB4V52FixedRuntimeArm( ...
    context, 'v52-counterfactual-pulse-timing-b4-e20');
end
