function metrics = computeNetworkAdditiveFormationRiskV65( ...
        control, context, groupIds, positiveSupportThreshold)
% COMPUTENETWORKADDITIVEFORMATIONRISKV65 Additive current-only risk mass.
%
% For each receiver formation, compare the registered reference fusion with
% the one-formation counterfactual that withholds complete cross-formation
% input.  Local rescue is weighted by receiver measurement association;
% possible useful-information loss is weighted by the strongest active
% cross-formation sender association for the same label.  A single network
% reference-existence denominator makes the resulting masses additive across
% formations and comparable across M24 and X36.

if nargin < 4 || isempty(positiveSupportThreshold)
    positiveSupportThreshold = 0.20;
end
groupIds = reshape(groupIds, 1, []);
if ~isstruct(control) || ~isstruct(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        numel(context.localPosteriorBySensor) ~= numel(groupIds) || ...
        ~isfield(control, 'groups') || ...
        ~isfield(control, 'singleActionScores') || ...
        ~isfield(control, 'singleActionAvailableMask') || ...
        ~isfield(control, 'singleActionSafetyMask') || ...
        ~isfield(control, 'referenceFusionWeights') || ...
        ~isfield(control, 'referenceScore') || ...
        ~isscalar(positiveSupportThreshold) || ...
        ~isfinite(positiveSupportThreshold) || ...
        positiveSupportThreshold < 0 || positiveSupportThreshold > 1
    error('NetworkAdditiveRiskV65:InvalidInput', ...
        'The current control state or support threshold is invalid.');
end

nodeCount = numel(groupIds);
weights = control.referenceFusionWeights;
if ~isequal(size(weights), [nodeCount, nodeCount]) || ...
        any(~isfinite(weights(:))) || any(weights(:) < -1e-12) || ...
        any(abs(sum(weights, 2) - 1) > 1e-10)
    error('NetworkAdditiveRiskV65:InvalidReferenceWeights', ...
        'Reference fusion weights are invalid.');
end
groups = reshape(control.groups, 1, []);
formationCount = numel(groups);
available = reshape(control.singleActionAvailableMask, 1, []);
safe = reshape(control.singleActionSafetyMask, 1, []);
if numel(available) ~= formationCount || numel(safe) ~= formationCount || ...
        numel(control.singleActionScores) ~= formationCount
    error('NetworkAdditiveRiskV65:FormationMismatch', ...
        'Formation-level counterfactuals do not align.');
end

referenceDetails = control.referenceScore.retentionDetails;
networkReferenceMass = sum(reshape( ...
    referenceDetails.expectedReferenceCardinality, 1, []));
if ~isscalar(networkReferenceMass) || ~isfinite(networkReferenceMass) || ...
        networkReferenceMass <= eps
    error('NetworkAdditiveRiskV65:InvalidReferenceMass', ...
        'The network reference existence mass is invalid.');
end

rescueMass = nan(1, formationCount);
usefulLossMass = nan(1, formationCount);
referenceMass = nan(1, formationCount);
upwardCrossings = nan(1, formationCount);
downwardCrossings = nan(1, formationCount);
for formationIdx = 1:formationCount
    if ~available(formationIdx)
        continue;
    end
    receivers = find(groupIds == groups(formationIdx));
    details = control.singleActionScores{formationIdx}.retentionDetails;
    [rescueMass(formationIdx), usefulLossMass(formationIdx), ...
        referenceMass(formationIdx), upwardCrossings(formationIdx), ...
        downwardCrossings(formationIdx)] = aggregateFormationMass( ...
            details, context.localPosteriorBySensor, receivers, ...
            groupIds, weights, positiveSupportThreshold);
end

rescueFraction = rescueMass / networkReferenceMass;
usefulLossFraction = usefulLossMass / networkReferenceMass;
availableFinite = available & isfinite(rescueFraction) & ...
    isfinite(usefulLossFraction);

metrics = struct();
metrics.contractVersion = 'network-additive-formation-risk-v65-v1';
metrics.positiveSupportThreshold = positiveSupportThreshold;
metrics.groups = groups;
metrics.singleActionAvailableMask = availableFinite;
metrics.singleActionSafetyMask = safe & availableFinite;
metrics.networkReferenceExistenceMass = networkReferenceMass;
metrics.referenceExistenceMassByFormation = referenceMass;
metrics.receiverSupportedRescueMassByFormation = rescueMass;
metrics.crossSupportedUsefulLossMassByFormation = usefulLossMass;
metrics.networkRescueFractionByFormation = rescueFraction;
metrics.networkUsefulLossFractionByFormation = usefulLossFraction;
metrics.receiverSupportedUpwardCrossingsByFormation = upwardCrossings;
metrics.crossSupportedDownwardCrossingsByFormation = downwardCrossings;
metrics.totalNetworkRescueFraction = sum(rescueFraction(availableFinite));
metrics.totalNetworkUsefulLossFraction = ...
    sum(usefulLossFraction(availableFinite));
metrics.commonNetworkDenominatorUsed = true;
metrics.crossSupportUsesMaximumActiveSenderAssociation = true;
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureOutcomesUsed = false;
end

function [rescueMass, usefulLossMass, referenceMass, ...
        upwardCrossings, downwardCrossings] = aggregateFormationMass( ...
            details, posteriors, receivers, groupIds, weights, ...
            supportThreshold)
rescueMass = 0;
usefulLossMass = 0;
referenceMass = 0;
upwardCrossings = 0;
downwardCrossings = 0;
decisionThreshold = details.decisionExistenceThreshold;
for receiverIdx = reshape(receivers, 1, [])
    labels = details.labelUniverseByReceiver{receiverIdx};
    reference = reshape( ...
        details.expectedReferenceExistenceByReceiver{receiverIdx}, 1, []);
    candidate = reshape( ...
        details.expectedCandidateExistenceByReceiver{receiverIdx}, 1, []);
    if size(labels, 1) ~= 2 || ...
            size(labels, 2) ~= numel(reference) || ...
            numel(reference) ~= numel(candidate)
        error('NetworkAdditiveRiskV65:LabelMismatch', ...
            'The counterfactual label universe does not align.');
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
    rescue = max(candidate - reference, 0);
    usefulLoss = max(reference - candidate, 0);
    rescueMass = rescueMass + sum(rescue .* receiverSupport);
    usefulLossMass = usefulLossMass + sum(usefulLoss .* crossSupport);
    referenceMass = referenceMass + sum(reference);
    upwardCrossings = upwardCrossings + nnz( ...
        receiverSupport >= supportThreshold - 1e-12 & ...
        reference < decisionThreshold & candidate >= decisionThreshold);
    downwardCrossings = downwardCrossings + nnz( ...
        crossSupport >= supportThreshold - 1e-12 & ...
        reference >= decisionThreshold & candidate < decisionThreshold);
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
