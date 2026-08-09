function metrics = computeObservationSupportedRoutingLeverage( ...
        control, context, groupIds, positiveSupportThreshold)
% COMPUTEOBSERVATIONSUPPORTEDROUTINGLEVERAGE Align rescued labels with data.
%
% V58 counts every increase in expected label existence produced by a
% one-formation suspension.  This diagnostic asks which part of that rescue
% belongs to receiver labels with current local measurement-association
% support.  It uses only current posterior metadata and the already computed
% reference/counterfactual outcome distributions.

if nargin < 4 || isempty(positiveSupportThreshold)
    positiveSupportThreshold = 0.20;
end
groupIds = reshape(groupIds, 1, []);
if ~isstruct(control) || ~isstruct(context) || ...
        ~isfield(context, 'localPosteriorBySensor') || ...
        numel(context.localPosteriorBySensor) ~= numel(groupIds) || ...
        ~isscalar(positiveSupportThreshold) || ...
        ~isfinite(positiveSupportThreshold) || ...
        positiveSupportThreshold < 0 || positiveSupportThreshold > 1
    error('ObservationSupportedLeverage:InvalidInput', ...
        'The control state or positive-support threshold is invalid.');
end

formationCount = control.formationCount;
supportedMass = nan(1, formationCount);
weightedMass = nan(1, formationCount);
rawMass = nan(1, formationCount);
referenceMass = nan(1, formationCount);
weightedReferenceMass = nan(1, formationCount);
supportedFractionOfReference = nan(1, formationCount);
weightedFractionOfReference = nan(1, formationCount);
weightedProportionalRescue = nan(1, formationCount);
supportedShareOfRescue = nan(1, formationCount);
supportedCrossings = nan(1, formationCount);

for formationIdx = 1:formationCount
    if ~control.singleActionAvailableMask(formationIdx)
        continue;
    end
    receivers = find(groupIds == control.groups(formationIdx));
    details = control.singleActionScores{formationIdx}.retentionDetails;
    [raw, supported, weighted, reference, weightedReference, crossings] = ...
        aggregateSupportedRescue( ...
            details, context.localPosteriorBySensor, receivers, ...
            positiveSupportThreshold);
    rawMass(formationIdx) = raw;
    supportedMass(formationIdx) = supported;
    weightedMass(formationIdx) = weighted;
    referenceMass(formationIdx) = reference;
    weightedReferenceMass(formationIdx) = weightedReference;
    supportedFractionOfReference(formationIdx) = ...
        supported / max(reference, 1);
    weightedFractionOfReference(formationIdx) = ...
        weighted / max(reference, 1);
    if weightedReference > eps
        weightedProportionalRescue(formationIdx) = ...
            weighted / weightedReference;
    else
        weightedProportionalRescue(formationIdx) = 0;
    end
    if raw > eps
        supportedShareOfRescue(formationIdx) = supported / raw;
    else
        supportedShareOfRescue(formationIdx) = 0;
    end
    supportedCrossings(formationIdx) = crossings;
end

metrics = struct();
metrics.contractVersion = ...
    'observation-supported-routing-leverage-v60-v1';
metrics.positiveSupportThreshold = positiveSupportThreshold;
metrics.singleRawRescuedExistenceMass = rawMass;
metrics.singlePositiveSupportedRescuedExistenceMass = supportedMass;
metrics.singleSupportWeightedRescuedExistenceMass = weightedMass;
metrics.singleReferenceExistenceMass = referenceMass;
metrics.singleSupportWeightedReferenceExistenceMass = ...
    weightedReferenceMass;
metrics.singlePositiveSupportedRescuedFractionOfReference = ...
    supportedFractionOfReference;
metrics.singleSupportWeightedRescuedFractionOfReference = ...
    weightedFractionOfReference;
metrics.singleSupportWeightedProportionalRescue = ...
    weightedProportionalRescue;
metrics.singlePositiveSupportedShareOfRescue = ...
    supportedShareOfRescue;
metrics.singlePositiveSupportedDecisionCrossings = supportedCrossings;
metrics.maximumPositiveSupportedRescuedFractionOfReference = ...
    finiteMaximum(supportedFractionOfReference);
metrics.maximumSupportWeightedRescuedFractionOfReference = ...
    finiteMaximum(weightedFractionOfReference);
metrics.maximumSupportWeightedProportionalRescue = ...
    finiteMaximum(weightedProportionalRescue);
metrics.maximumPositiveSupportedShareOfRescue = ...
    finiteMaximum(supportedShareOfRescue);
metrics.maximumPositiveSupportedDecisionCrossings = ...
    finiteMaximum(supportedCrossings);
metrics.truthUsed = false;
metrics.futureMeasurementsUsed = false;
metrics.futureLinkOutcomesUsed = false;
end

function [rawMass, supportedMass, weightedMass, referenceMass, ...
        weightedReferenceMass, crossingCount] = ...
    aggregateSupportedRescue( ...
        details, posteriors, receivers, supportThreshold)
rawMass = 0;
supportedMass = 0;
weightedMass = 0;
referenceMass = 0;
weightedReferenceMass = 0;
crossingCount = 0;
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
        error('ObservationSupportedLeverage:LabelMismatch', ...
            'The receiver label universe does not align with existence.');
    end
    associationMass = currentAssociationMass( ...
        posteriors{receiverIdx}, labels);
    positiveSupport = associationMass >= supportThreshold - 1e-12;
    gain = max(candidate - reference, 0);
    rawMass = rawMass + sum(gain);
    supportedMass = supportedMass + sum(gain(positiveSupport));
    weightedMass = weightedMass + sum(gain .* associationMass);
    referenceMass = referenceMass + sum(reference);
    weightedReferenceMass = weightedReferenceMass + ...
        sum(reference .* associationMass);
    crossingCount = crossingCount + nnz(positiveSupport & ...
        reference < decisionThreshold & candidate >= decisionThreshold);
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

function value = finiteMaximum(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
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
