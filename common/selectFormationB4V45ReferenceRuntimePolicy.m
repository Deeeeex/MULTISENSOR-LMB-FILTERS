function [adjacency, details] = ...
    selectFormationB4V45ReferenceRuntimePolicy(context)
% SELECTFORMATIONB4V45REFERENCERUNTIMEPOLICY Frozen V43 reference arm.

% The public V45 wrapper intentionally accepts no policy options.  Its
% weights and arm identity are frozen before paired tracking is opened.

[adjacency, details] = buildFormationB4V45FixedRuntimeArm( ...
    context, 'v43-reference-a70-e05');
end
