function [adjacency, details] = ...
    selectFormationB4V45FormationStaggeredRuntimePolicy(context)
% SELECTFORMATIONB4V45FORMATIONSTAGGEREDRUNTIMEPOLICY Frozen staggered B=4 arm.

% The public V45 wrapper intentionally accepts no policy options.  It uses
% the mass-matched formation-staggered V44 schedule at the absolute phase.

[adjacency, details] = buildFormationB4V45FixedRuntimeArm( ...
    context, 'v44-formation-all-b4-e20-mc');
end
