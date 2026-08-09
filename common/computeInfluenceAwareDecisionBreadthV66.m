function metrics = computeInfluenceAwareDecisionBreadthV66( ...
        control, context, groupIds, options)
% COMPUTEINFLUENCEAWAREDECISIONBREADTHV66 Current-only impact breadth.
%
% V65 measures how much receiver-supported existence mass a formation
% intervention rescues.  That amplitude alone does not say how many
% receiver decisions change or whether a change can influence the rest of
% the network.  V66 therefore exposes receiver-level threshold crossings
% and propagates each affected receiver through the current reference KLA
% matrix over a finite horizon.  It reads no truth, future measurement, or
% future link outcome.

if nargin < 4 || isempty(options)
    options = struct();
end
positiveSupportThreshold = getField( ...
    options, 'positiveSupportThreshold', 0.20);
horizonSteps = getField(options, 'horizonSteps', 3);
groupIds = reshape(groupIds, 1, []);
if ~isstruct(control) || ~isstruct(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        numel(context.localPosteriorBySensor) ~= numel(groupIds) || ...
        ~isfield(control, 'groups') || ...
        ~isfield(control, 'singleActionScores') || ...
        ~isfield(control, 'singleActionAvailableMask') || ...
        ~isfield(control, 'referenceFusionWeights') || ...
        ~isscalar(positiveSupportThreshold) || ...
        ~isfinite(positiveSupportThreshold) || ...
        positiveSupportThreshold < 0 || positiveSupportThreshold > 1 || ...
        ~isscalar(horizonSteps) || ~isfinite(horizonSteps) || ...
        horizonSteps < 1 || horizonSteps ~= round(horizonSteps)
    error('InfluenceAwareBreadthV66:InvalidInput', ...
        'The current control state or V66 options are invalid.');
end

nodeCount = numel(groupIds);
weights = control.referenceFusionWeights;
if ~isequal(size(weights), [nodeCount, nodeCount]) || ...
        any(~isfinite(weights(:))) || any(weights(:) < -1e-12) || ...
        any(abs(sum(weights, 2) - 1) > 1e-10)
    error('InfluenceAwareBreadthV66:InvalidReferenceWeights', ...
        'The reference fusion matrix is invalid.');
end
groups = reshape(control.groups, 1, []);
formationCount = numel(groups);
available = reshape(control.singleActionAvailableMask, 1, []);
if numel(available) ~= formationCount || ...
        numel(control.singleActionScores) ~= formationCount
    error('InfluenceAwareBreadthV66:FormationMismatch', ...
        'The formation counterfactuals do not align.');
end

rescueBySensorFormation = nan(nodeCount, formationCount);
usefulLossBySensorFormation = nan(nodeCount, formationCount);
upwardBySensorFormation = nan(nodeCount, formationCount);
downwardBySensorFormation = nan(nodeCount, formationCount);
crossingSupportBySensorFormation = nan(nodeCount, formationCount);
crossingMarginBySensorFormation = nan(nodeCount, formationCount);
for formationIdx = 1:formationCount
    if ~available(formationIdx)
        continue;
    end
    receivers = find(groupIds == groups(formationIdx));
    rescueBySensorFormation(:, formationIdx) = 0;
    usefulLossBySensorFormation(:, formationIdx) = 0;
    upwardBySensorFormation(:, formationIdx) = 0;
    downwardBySensorFormation(:, formationIdx) = 0;
    crossingSupportBySensorFormation(:, formationIdx) = 0;
    crossingMarginBySensorFormation(:, formationIdx) = 0;
    details = control.singleActionScores{formationIdx}.retentionDetails;
    for receiverIdx = reshape(receivers, 1, [])
        receiver = receiverDiagnostics( ...
            details, context.localPosteriorBySensor, receiverIdx, ...
            groupIds, weights, positiveSupportThreshold);
        rescueBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.rescueMass;
        usefulLossBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.usefulLossMass;
        upwardBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.upwardCrossings;
        downwardBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.downwardCrossings;
        crossingSupportBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.upwardCrossingSupportMass;
        crossingMarginBySensorFormation(receiverIdx, formationIdx) = ...
            receiver.upwardCrossingRobustMargin;
    end
end

[influenceByDepth, reachCountByDepth] = ...
    finiteHorizonInfluence(weights, horizonSteps);
formationReceiverCount = zeros(1, formationCount);
formationAffectedReceiverCount = nan(1, formationCount);
formationUpwardCrossings = nan(1, formationCount);
formationWeightedDecisionExposure = nan(1, formationCount);
formationWeightedSupportExposure = nan(1, formationCount);
formationWeightedRobustMarginExposure = nan(1, formationCount);
formationReachableDecisionExposure = nan(1, formationCount);
formationEffectiveRescueReceiverCount = nan(1, formationCount);
for formationIdx = 1:formationCount
    receivers = find(groupIds == groups(formationIdx));
    formationReceiverCount(formationIdx) = numel(receivers);
    if ~available(formationIdx)
        continue;
    end
    crossing = upwardBySensorFormation(:, formationIdx);
    rescue = rescueBySensorFormation(:, formationIdx);
    formationAffectedReceiverCount(formationIdx) = nnz(crossing > 0);
    formationUpwardCrossings(formationIdx) = sum(crossing);
    formationWeightedDecisionExposure(formationIdx) = sum( ...
        crossing .* sum(influenceByDepth, 1)');
    supportMass = crossingSupportBySensorFormation(:, formationIdx);
    robustMargin = crossingMarginBySensorFormation(:, formationIdx);
    formationWeightedSupportExposure(formationIdx) = sum( ...
        supportMass .* sum(influenceByDepth, 1)');
    formationWeightedRobustMarginExposure(formationIdx) = sum( ...
        robustMargin .* sum(influenceByDepth, 1)');
    formationReachableDecisionExposure(formationIdx) = sum( ...
        crossing .* sum(reachCountByDepth, 1)');
    formationEffectiveRescueReceiverCount(formationIdx) = ...
        effectiveCount(rescue);
end

metrics = struct();
metrics.contractVersion = ...
    'influence-aware-decision-breadth-v66-v1';
metrics.positiveSupportThreshold = positiveSupportThreshold;
metrics.horizonSteps = horizonSteps;
metrics.groups = groups;
metrics.groupIds = groupIds;
metrics.singleActionAvailableMask = available;
metrics.receiverSupportedRescueMassBySensorFormation = ...
    rescueBySensorFormation;
metrics.crossSupportedUsefulLossMassBySensorFormation = ...
    usefulLossBySensorFormation;
metrics.receiverSupportedUpwardCrossingsBySensorFormation = ...
    upwardBySensorFormation;
metrics.crossSupportedDownwardCrossingsBySensorFormation = ...
    downwardBySensorFormation;
metrics.upwardCrossingSupportMassBySensorFormation = ...
    crossingSupportBySensorFormation;
metrics.upwardCrossingRobustMarginBySensorFormation = ...
    crossingMarginBySensorFormation;
metrics.referenceInfluenceMassByDepth = influenceByDepth;
metrics.referenceReachableReceiverCountByDepth = reachCountByDepth;
metrics.formationReceiverCount = formationReceiverCount;
metrics.formationAffectedReceiverCount = ...
    formationAffectedReceiverCount;
metrics.formationAffectedReceiverFraction = ...
    formationAffectedReceiverCount ./ max(formationReceiverCount, 1);
metrics.formationUpwardCrossings = formationUpwardCrossings;
metrics.formationWeightedDecisionExposure = ...
    formationWeightedDecisionExposure;
metrics.formationWeightedSupportExposure = ...
    formationWeightedSupportExposure;
metrics.formationWeightedRobustMarginExposure = ...
    formationWeightedRobustMarginExposure;
metrics.formationNormalizedDecisionExposure = ...
    formationWeightedDecisionExposure / (nodeCount * horizonSteps);
metrics.formationNormalizedSupportExposure = ...
    formationWeightedSupportExposure / (nodeCount * horizonSteps);
metrics.formationNormalizedRobustMarginExposure = ...
    formationWeightedRobustMarginExposure / ...
        (nodeCount * horizonSteps);
metrics.formationReachableDecisionExposure = ...
    formationReachableDecisionExposure;
metrics.formationEffectiveRescueReceiverCount = ...
    formationEffectiveRescueReceiverCount;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureLinkOutcomesUsed = false;
end

function value = receiverDiagnostics( ...
        details, posteriors, receiverIdx, groupIds, weights, ...
        supportThreshold)
labels = details.labelUniverseByReceiver{receiverIdx};
reference = reshape( ...
    details.expectedReferenceExistenceByReceiver{receiverIdx}, 1, []);
candidate = reshape( ...
    details.expectedCandidateExistenceByReceiver{receiverIdx}, 1, []);
if size(labels, 1) ~= 2 || ...
        size(labels, 2) ~= numel(reference) || ...
        numel(reference) ~= numel(candidate)
    error('InfluenceAwareBreadthV66:LabelMismatch', ...
        'The receiver label universe does not align.');
end
receiverSupport = currentAssociationMass( ...
    posteriors{receiverIdx}, labels);
crossSenders = find(weights(receiverIdx, :) > 1e-12 & ...
    groupIds ~= groupIds(receiverIdx));
crossSupport = zeros(1, size(labels, 2));
for senderIdx = reshape(crossSenders, 1, [])
    crossSupport = max(crossSupport, currentAssociationMass( ...
        posteriors{senderIdx}, labels));
end
threshold = details.decisionExistenceThreshold;
upwardMask = receiverSupport >= supportThreshold - 1e-12 & ...
    reference < threshold & candidate >= threshold;
downwardMask = crossSupport >= supportThreshold - 1e-12 & ...
    reference >= threshold & candidate < threshold;
robustMargin = min(threshold - reference, candidate - threshold);
robustMargin = max(robustMargin, 0);

value = struct();
value.rescueMass = sum(max(candidate - reference, 0) .* receiverSupport);
value.usefulLossMass = ...
    sum(max(reference - candidate, 0) .* crossSupport);
value.upwardCrossings = nnz(upwardMask);
value.downwardCrossings = nnz(downwardMask);
value.upwardCrossingSupportMass = sum(receiverSupport(upwardMask));
value.upwardCrossingRobustMargin = ...
    sum(receiverSupport(upwardMask) .* robustMargin(upwardMask));
end

function [influence, reachCount] = finiteHorizonInfluence(weights, horizon)
nodeCount = size(weights, 1);
influence = zeros(horizon, nodeCount);
reachCount = zeros(horizon, nodeCount);
power = eye(nodeCount);
for depth = 1:horizon
    influence(depth, :) = sum(power, 1);
    reachCount(depth, :) = sum(power > 1e-12, 1);
    power = weights * power;
end
end

function value = effectiveCount(values)
values = max(reshape(values, 1, []), 0);
values = values(isfinite(values));
if isempty(values) || sum(values .^ 2) <= eps
    value = 0;
else
    value = sum(values) ^ 2 / sum(values .^ 2);
end
end

function values = currentAssociationMass(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents > 0 && ...
                objects(objectIdx).birthTime == labels(1, labelIdx) && ...
                objects(objectIdx).birthLocation == labels(2, labelIdx)
            values(labelIdx) = clamp01(getField( ...
                objects(objectIdx), 'detectionAssociationMass', 0));
            break;
        end
    end
end
end

function value = clamp01(value)
if ~isscalar(value) || ~isfinite(value)
    value = 0;
end
value = min(max(value, 0), 1);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
