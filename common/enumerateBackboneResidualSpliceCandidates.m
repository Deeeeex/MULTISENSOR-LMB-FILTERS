function [receiverIndices, senderIndices, details] = ...
    enumerateBackboneResidualSpliceCandidates(context, options)
% ENUMERATEBACKBONERESIDUALSPLICECANDIDATES Exact residual action set.
%
% A candidate replaces one low-weight balanced-cycle sender while leaving
% the high-weight fixed-index sender untouched. Candidates that would
% collapse the two attempted messages onto the same sender are removed.
% The resulting receiver/sender ordering is shared by the oracle teacher,
% the truth-free feature extractor, and the learned policy.

if nargin < 2 || isempty(options)
    options = struct();
end
dominantWeight = getField(options, 'dominantWeight', 0.70);
residualWeight = getField(options, 'residualWeight', 0.05);
[dominantAdjacency, dominantDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-index-star', ...
        struct('sourceWeight', dominantWeight));
[residualBaselineAdjacency, residualBaselineDetails] = ...
    selectRegisteredDirectedRoutingPolicy( ...
        context, 'fixed-balanced-cycle', ...
        struct('sourceWeight', residualWeight, 'phase', 1));
dominantSources = reshape( ...
    dominantDetails.selectedSourcesByReceiver, 1, []);
residualBaselineSources = reshape( ...
    residualBaselineDetails.selectedSourcesByReceiver, 1, []);
nodeCount = numel(dominantSources);
if ~isfield(context, 'model') || ...
        ~isfield(context.model, 'dynamicTopologyScenario') || ...
        ~isfield(context.model.dynamicTopologyScenario, 'config') || ...
        ~isfield(context.model.dynamicTopologyScenario.config, ...
            'sensorGroupIds')
    error('Residual splice candidates require sensorGroupIds.');
end
groupIds = reshape(context.model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
physical = logical(context.physicalAdjacency);
if numel(groupIds) ~= nodeCount || ...
        ~isequal(size(physical), [nodeCount, nodeCount])
    error('Residual splice candidate graph dimensions are invalid.');
end
physical(1:nodeCount+1:end) = false;

[receiverIndices, senderIndices] = find(physical);
crossMask = groupIds(receiverIndices) ~= ...
    groupIds(senderIndices);
receiverIndices = receiverIndices(crossMask);
senderIndices = senderIndices(crossMask);
baselineSourcesForReceiver = reshape( ...
    residualBaselineSources(receiverIndices), [], 1);
dominantSourcesForReceiver = reshape( ...
    dominantSources(receiverIndices), [], 1);
messageCountPreserving = ...
    baselineSourcesForReceiver ~= ...
        dominantSourcesForReceiver & ...
    senderIndices ~= dominantSourcesForReceiver;
receiverIndices = reshape( ...
    receiverIndices(messageCountPreserving), [], 1);
senderIndices = reshape( ...
    senderIndices(messageCountPreserving), [], 1);
if isempty(receiverIndices)
    error('ResidualCycle:Infeasible', ...
        'No message-count-preserving cross routes are available.');
end

candidateKeys = receiverIndices + ...
    nodeCount * (senderIndices - 1);
if numel(unique(candidateKeys)) ~= numel(candidateKeys)
    error('Residual splice candidates contain duplicate directed edges.');
end
details = struct();
details.contractVersion = ...
    'backbone-residual-splice-candidate-set-v1';
details.nodeCount = nodeCount;
details.groupIds = groupIds;
details.physicalAdjacency = physical;
details.dominantWeight = dominantWeight;
details.residualWeight = residualWeight;
details.dominantAdjacency = dominantAdjacency;
details.dominantSourcesByReceiver = dominantSources;
details.dominantPolicyDetails = dominantDetails;
details.residualBaselineAdjacency = ...
    residualBaselineAdjacency;
details.residualBaselineSourcesByReceiver = ...
    residualBaselineSources;
details.residualBaselinePolicyDetails = ...
    residualBaselineDetails;
details.receiverIndices = receiverIndices;
details.senderIndices = senderIndices;
details.candidateEdgeKeys = candidateKeys;
details.messageCountPreserving = true;
details.truthUsed = false;
details.futureOutcomeUsed = false;
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
