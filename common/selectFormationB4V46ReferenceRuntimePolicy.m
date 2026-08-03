function [adjacency, details] = ...
    selectFormationB4V46ReferenceRuntimePolicy(context)
% SELECTFORMATIONB4V46REFERENCERUNTIMEPOLICY Frozen repaired reference arm.

[adjacency, details] = buildFormationB4V46FixedRuntimeArm( ...
    context, 'v46-repaired-reference-a70-e05');
end
