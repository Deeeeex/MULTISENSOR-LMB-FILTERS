function [adjacency, details] = ...
    selectBackbonePreservingResidualControl(context, options)
% SELECTBACKBONEPRESERVINGRESIDUALCONTROL Static matched residual control.

if nargin < 2 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, ...
    'dominantWeight', 0.70);
residualWeight = getField(options, ...
    'residualWeight', 0.05);
[dominantAdjacency, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
[residualAdjacency, residualDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-balanced-cycle', ...
        struct('sourceWeight', residualWeight, 'phase', 1));
[adjacency, fusionWeights, routeDetails] = ...
    buildBackbonePreservingResidualRoute( ...
        context, dominantAdjacency, residualAdjacency, ...
        dominantWeight, residualWeight);

details = routeDetails;
details.mode = ...
    'backbone-preserving-residual-static';
details.objective = NaN;
details.candidateIndex = NaN;
details.selectionSeconds = 0;
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = NaN;
details.fusionWeightMatrix = fusionWeights;
details.baselineAdjacency = adjacency;
details.dominantAdjacency = dominantAdjacency;
details.residualAdjacency = residualAdjacency;
details.dominantPolicyDetails = dominantDetails;
details.residualPolicyDetails = residualDetails;
details.backboneMode = ...
    'fixed-index-plus-balanced-cycle-residual';
details.sourceWeight = residualWeight;
details.overrideFraction = 0;
details.crossFormationMessageCount = 0;
details.posteriorUsed = false;
details.posteriorPayloadMetadataUsed = false;
details.truthUsed = false;
details.currentLinkReliabilityUsed = false;
details.topologyInfeasible = false;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
