function [signature, control] = computeFormationRoutingLeverageSignature( ...
        context, groupIds, options)
% COMPUTEFORMATIONROUTINGLEVERAGESIGNATURE Compact truth-free V58 state.
%
% The signature distinguishes generic posterior contrast from a causal
% question: what label-existence mass is restored when one formation stops
% consuming its registered cross-formation residual input for one round?
% It uses only the current local posteriors, current link probabilities, and
% past selected topology exposed by loadFormationH3ObservableState.

if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getTrackingAlignedRoutingLeverageV58Protocol();
groupIds = reshape(groupIds, 1, []);
eventMetrics = computeFormationH3ObservableEventScore( ...
    context, groupIds, struct( ...
        'tailFraction', protocol.eventScoreTailFraction, ...
        'tailWeight', protocol.eventScoreTailWeight));
control = buildFormationRetentionDebtControl(context, options);
debtProtocol = getFormationIsolateReconnectProbeProtocol();
if ~isequal(groupIds, control.groupIds) || ...
        abs(protocol.retentionDebtOnFraction - ...
            debtProtocol.retentionDebtOnFraction) > 1e-12 || ...
        eventMetrics.truthUsed || ...
        eventMetrics.futureMeasurementsUsed || ...
        eventMetrics.futureLinkOutcomesUsed || ...
        control.truthUsed || control.futureOutcomeUsed
    error('TrackingAlignedV58:InvalidLeverageInput', ...
        'The V58 routing-leverage signature crossed its causal boundary.');
end

debt = reshape(control.formationRetentionDebtFraction, 1, []);
threshold = reshape(control.debtThresholdByFormation, 1, []);
referenceCardinality = reshape( ...
    control.referenceFormationExpectedCardinality, 1, []);
positiveDebt = max(debt, 0);
thresholdExcess = max(debt - threshold, 0);
[maximumDebt, maximumDebtIndex] = max(debt);
if isempty(maximumDebtIndex)
    maximumDebtIndex = 0;
    maximumDebtFormationId = 0;
else
    maximumDebtFormationId = control.groups(maximumDebtIndex);
end
weightedDenominator = max(sum(referenceCardinality), 1);
weightedPositiveDebt = sum( ...
    positiveDebt .* referenceCardinality) / weightedDenominator;
tailCount = max(1, ceil( ...
    protocol.eventScoreTailFraction * control.formationCount));
sortedPositiveDebt = sort(positiveDebt, 'descend');
upperTailDebt = mean(sortedPositiveDebt(1:tailCount));

singleNetworkImprovement = nan(1, control.formationCount);
singleRetentionRisk = nan(1, control.formationCount);
singleMinimumRetentionRatio = nan(1, control.formationCount);
singleDownwardCrossingCount = nan(1, control.formationCount);
singleRescuedExistenceMass = nan(1, control.formationCount);
singleRescuedExistenceFraction = nan(1, control.formationCount);
singleRescuedDecisionCrossingCount = nan(1, control.formationCount);
for formationIdx = 1:control.formationCount
    if ~control.singleActionAvailableMask(formationIdx)
        continue;
    end
    score = control.singleActionScores{formationIdx};
    singleNetworkImprovement(formationIdx) = relativeImprovement( ...
        control.referenceScore.networkRisk, score.networkRisk);
    singleRetentionRisk(formationIdx) = score.retentionRisk;
    singleMinimumRetentionRatio(formationIdx) = ...
        score.minimumSupportedLabelRetentionRatio;
    singleDownwardCrossingCount(formationIdx) = ...
        score.decisionThresholdCrossingCount;
    receivers = find(groupIds == control.groups(formationIdx));
    [rescuedMass, rescuedCrossings] = rescuedExistence( ...
        score.retentionDetails, receivers);
    singleRescuedExistenceMass(formationIdx) = rescuedMass;
    singleRescuedExistenceFraction(formationIdx) = rescuedMass / max( ...
        referenceCardinality(formationIdx), 1);
    singleRescuedDecisionCrossingCount(formationIdx) = ...
        rescuedCrossings;
end

selectedRetention = control.selectedScore.retentionDetails;
[selectedRescuedMass, selectedRescuedCrossings] = ...
    rescuedExistence(selectedRetention, 1:numel(groupIds));
selectedMessageSavingFraction = control.messageSavingCount / max( ...
    control.referenceMessageCount, 1);

signature = struct();
signature.contractVersion = ...
    'formation-routing-leverage-signature-v58-v1';
signature.protocolId = protocol.id;
signature.currentTime = context.currentTime;
signature.sensorCount = numel(groupIds);
signature.formationCount = control.formationCount;
signature.groupIds = groupIds;
signature.groups = control.groups;
signature.eventScore = eventMetrics.score;
signature.posteriorContrast = eventMetrics.posteriorContrast;
signature.robustSelectedCrossLinkStress = ...
    eventMetrics.robustSelectedCrossLinkStress;
signature.formationRetentionDebtFraction = debt;
signature.debtThresholdByFormation = threshold;
signature.positiveDebtFractionByFormation = positiveDebt;
signature.thresholdExcessByFormation = thresholdExcess;
signature.maximumRetentionDebtFraction = maximumDebt;
signature.maximumDebtFormationId = maximumDebtFormationId;
signature.meanPositiveRetentionDebtFraction = mean(positiveDebt);
signature.upperTailRetentionDebtFraction = upperTailDebt;
signature.referenceCardinalityWeightedPositiveDebtFraction = ...
    weightedPositiveDebt;
signature.maximumThresholdExcessFraction = max(thresholdExcess);
signature.formationFractionAboveThreshold = ...
    mean(debt >= threshold - 1e-12);
signature.referenceFormationExpectedCardinality = referenceCardinality;
signature.singleActionAvailableMask = ...
    control.singleActionAvailableMask;
signature.singleActionSafetyMask = control.singleActionSafetyMask;
signature.singleNetworkRiskImprovementFraction = ...
    singleNetworkImprovement;
signature.singleRetentionRisk = singleRetentionRisk;
signature.singleMinimumSupportedLabelRetentionRatio = ...
    singleMinimumRetentionRatio;
signature.singleDownwardDecisionCrossingCount = ...
    singleDownwardCrossingCount;
signature.singleRescuedExistenceMass = singleRescuedExistenceMass;
signature.singleRescuedExistenceFraction = ...
    singleRescuedExistenceFraction;
signature.singleRescuedDecisionCrossingCount = ...
    singleRescuedDecisionCrossingCount;
signature.maximumRescuedExistenceFraction = ...
    finiteMaximum(singleRescuedExistenceFraction);
signature.maximumRescuedDecisionCrossingCount = ...
    finiteMaximum(singleRescuedDecisionCrossingCount);
signature.requestedFormationIds = control.requestedFormationIds;
signature.selectedFormationIds = control.selectedFormationIds;
signature.referenceFallbackUsed = control.referenceFallbackUsed;
signature.referenceMessageCount = control.referenceMessageCount;
signature.selectedMessageCount = control.selectedMessageCount;
signature.messageSavingCount = control.messageSavingCount;
signature.messageSavingFraction = selectedMessageSavingFraction;
signature.selectedNetworkRisk = control.selectedScore.networkRisk;
signature.selectedRetentionRisk = control.selectedScore.retentionRisk;
signature.selectedMinimumSupportedLabelRetentionRatio = ...
    control.selectedScore.minimumSupportedLabelRetentionRatio;
signature.selectedDownwardDecisionCrossingCount = ...
    control.selectedScore.decisionThresholdCrossingCount;
signature.selectedRescuedExistenceMass = selectedRescuedMass;
signature.selectedRescuedDecisionCrossingCount = ...
    selectedRescuedCrossings;
signature.evaluatedRouteCount = control.evaluatedRouteCount;
signature.selectionUsesCurrentPosterior = true;
signature.selectionUsesCurrentLinkReliability = true;
signature.selectionUsesSelectedTopologyHistory = true;
signature.truthUsed = false;
signature.futureMeasurementsUsed = false;
signature.futureLinkOutcomesUsed = false;
signature.openedDevelopmentEvidenceOnly = true;
signature.validationClaimAllowed = false;
end

function [mass, crossingCount] = rescuedExistence(details, receivers)
mass = 0;
crossingCount = 0;
threshold = details.decisionExistenceThreshold;
for receiverIdx = reshape(receivers, 1, [])
    reference = reshape( ...
        details.expectedReferenceExistenceByReceiver{receiverIdx}, 1, []);
    candidate = reshape( ...
        details.expectedCandidateExistenceByReceiver{receiverIdx}, 1, []);
    if numel(reference) ~= numel(candidate)
        error('TrackingAlignedV58:InvalidExistenceComparison', ...
            'Reference and candidate label universes do not align.');
    end
    gain = max(candidate - reference, 0);
    mass = mass + sum(gain);
    crossingCount = crossingCount + nnz( ...
        reference < threshold & candidate >= threshold);
end
end

function value = relativeImprovement(reference, candidate)
value = (reference - candidate) / max(abs(reference), eps);
end

function value = finiteMaximum(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end
