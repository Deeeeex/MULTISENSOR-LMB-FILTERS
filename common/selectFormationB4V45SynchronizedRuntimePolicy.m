function [adjacency, details] = ...
    selectFormationB4V45SynchronizedRuntimePolicy(context)
% SELECTFORMATIONB4V45SYNCHRONIZEDRUNTIMEPOLICY Frozen synchronized B=4 arm.

% The public V45 wrapper intentionally accepts no policy options.  It uses
% the mass-matched all-residual V44 schedule at the absolute current phase.

[adjacency, details] = buildFormationB4V45FixedRuntimeArm( ...
    context, 'v44-sync-all-b4-e20-mc');
end
