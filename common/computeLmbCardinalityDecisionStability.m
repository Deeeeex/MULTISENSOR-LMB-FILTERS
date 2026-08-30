function [risk, details] = computeLmbCardinalityDecisionStability( ...
        referenceReceiverDistributions, ...
        candidateReceiverDistributions, groupIds)
% COMPUTELMBCARDINALITYDECISIONSTABILITY Exact marginal MAP certificate.
%
% Each receiver distribution enumerates the current link-delivery outcomes
% and the LMB existence probabilities produced by that outcome.  This
% function first forms the exact outcome-marginal cardinality PMF.  If the
% reference PMF has unique MAP margin Delta and the candidate perturbation
% has infinity norm delta, delta < Delta/2 certifies that the reference MAP
% cardinality is unchanged.  Failure to certify is a risk signal, not proof
% that either posterior is correct.

if ~iscell(referenceReceiverDistributions) || ...
        ~iscell(candidateReceiverDistributions) || ...
        isempty(referenceReceiverDistributions) || ...
        numel(referenceReceiverDistributions) ~= ...
            numel(candidateReceiverDistributions)
    error('CardinalityStability:InvalidInput', ...
        'Receiver outcome distributions are invalid.');
end
nodeCount = numel(referenceReceiverDistributions);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
if numel(groupIds) ~= nodeCount || isempty(groups) || ...
        any(~isfinite(groupIds)) || any(groupIds < 1) || ...
        any(groupIds ~= round(groupIds))
    error('CardinalityStability:InvalidInput', ...
        'Formation membership is invalid.');
end

referenceMapCardinality = zeros(1, nodeCount);
candidateMapCardinality = zeros(1, nodeCount);
referenceMapProbability = zeros(1, nodeCount);
candidateMapProbability = zeros(1, nodeCount);
referenceMapMargin = zeros(1, nodeCount);
pmfInfinityShift = zeros(1, nodeCount);
pmfTotalVariation = zeros(1, nodeCount);
stabilitySlack = zeros(1, nodeCount);
mapCardinalityChanged = false(1, nodeCount);
mapCardinalityCertified = false(1, nodeCount);
referenceExpectedCardinality = zeros(1, nodeCount);
candidateExpectedCardinality = zeros(1, nodeCount);
referencePmfByReceiver = cell(1, nodeCount);
candidatePmfByReceiver = cell(1, nodeCount);

for receiverIdx = 1:nodeCount
    referencePmf = marginalCardinalityPmf( ...
        referenceReceiverDistributions{receiverIdx});
    candidatePmf = marginalCardinalityPmf( ...
        candidateReceiverDistributions{receiverIdx});
    width = max(numel(referencePmf), numel(candidatePmf));
    referencePmf(end + 1:width) = 0;
    candidatePmf(end + 1:width) = 0;
    [referenceMapProbability(receiverIdx), referenceIndex] = ...
        max(referencePmf);
    [candidateMapProbability(receiverIdx), candidateIndex] = ...
        max(candidatePmf);
    sortedReference = sort(referencePmf, 'descend');
    runnerUp = 0;
    if numel(sortedReference) > 1
        runnerUp = sortedReference(2);
    end
    referenceMapMargin(receiverIdx) = ...
        referenceMapProbability(receiverIdx) - runnerUp;
    pmfInfinityShift(receiverIdx) = ...
        max(abs(referencePmf - candidatePmf));
    pmfTotalVariation(receiverIdx) = ...
        0.5 * sum(abs(referencePmf - candidatePmf));
    stabilitySlack(receiverIdx) = referenceMapMargin(receiverIdx) - ...
        2 * pmfInfinityShift(receiverIdx);
    referenceMapCardinality(receiverIdx) = referenceIndex - 1;
    candidateMapCardinality(receiverIdx) = candidateIndex - 1;
    mapCardinalityChanged(receiverIdx) = referenceIndex ~= candidateIndex;
    mapCardinalityCertified(receiverIdx) = ...
        stabilitySlack(receiverIdx) > 1e-12;
    support = 0:(width - 1);
    referenceExpectedCardinality(receiverIdx) = ...
        sum(support .* referencePmf);
    candidateExpectedCardinality(receiverIdx) = ...
        sum(support .* candidatePmf);
    referencePmfByReceiver{receiverIdx} = referencePmf;
    candidatePmfByReceiver{receiverIdx} = candidatePmf;
end

formationCount = numel(groups);
formationMapChangeCount = zeros(1, formationCount);
formationUncertifiedCount = zeros(1, formationCount);
formationMinimumStabilitySlack = zeros(1, formationCount);
formationMaximumPmfInfinityShift = zeros(1, formationCount);
formationMeanCardinalityChange = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = groupIds == groups(formationIdx);
    formationMapChangeCount(formationIdx) = ...
        nnz(mapCardinalityChanged(members));
    formationUncertifiedCount(formationIdx) = ...
        nnz(~mapCardinalityCertified(members));
    formationMinimumStabilitySlack(formationIdx) = ...
        min(stabilitySlack(members));
    formationMaximumPmfInfinityShift(formationIdx) = ...
        max(pmfInfinityShift(members));
    formationMeanCardinalityChange(formationIdx) = mean( ...
        candidateExpectedCardinality(members) - ...
            referenceExpectedCardinality(members));
end

risk = max(max(-stabilitySlack, 0));
details = struct();
details.contractVersion = 'lmb-cardinality-decision-stability-v1';
details.groupIds = groupIds;
details.groups = groups;
details.referenceMapCardinality = referenceMapCardinality;
details.candidateMapCardinality = candidateMapCardinality;
details.referenceMapProbability = referenceMapProbability;
details.candidateMapProbability = candidateMapProbability;
details.referenceMapMargin = referenceMapMargin;
details.pmfInfinityShift = pmfInfinityShift;
details.pmfTotalVariation = pmfTotalVariation;
details.stabilitySlack = stabilitySlack;
details.mapCardinalityChanged = mapCardinalityChanged;
details.mapCardinalityCertified = mapCardinalityCertified;
details.referenceExpectedCardinality = referenceExpectedCardinality;
details.candidateExpectedCardinality = candidateExpectedCardinality;
details.referencePmfByReceiver = referencePmfByReceiver;
details.candidatePmfByReceiver = candidatePmfByReceiver;
details.formationMapChangeCount = formationMapChangeCount;
details.formationUncertifiedCount = formationUncertifiedCount;
details.formationMinimumStabilitySlack = ...
    formationMinimumStabilitySlack;
details.formationMaximumPmfInfinityShift = ...
    formationMaximumPmfInfinityShift;
details.formationMeanCardinalityChange = ...
    formationMeanCardinalityChange;
details.risk = risk;
details.certificateIsSufficientNotNecessary = true;
details.referenceCorrectnessClaimed = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function pmf = marginalCardinalityPmf(distribution)
distribution = validateDistribution(distribution);
maximumCount = 0;
for outcomeIdx = 1:numel(distribution.summary)
    maximumCount = max(maximumCount, ...
        numel(distribution.summary{outcomeIdx}.existence));
end
pmf = zeros(1, maximumCount + 1);
for outcomeIdx = 1:numel(distribution.summary)
    existence = reshape( ...
        distribution.summary{outcomeIdx}.existence, 1, []);
    outcomePmf = 1;
    for value = existence
        outcomePmf = conv(outcomePmf, [1 - value, value]);
    end
    outcomePmf(end + 1:maximumCount + 1) = 0;
    pmf = pmf + distribution.probability(outcomeIdx) * outcomePmf;
end
pmf = pmf / sum(pmf);
end

function distribution = validateDistribution(distribution)
if ~isstruct(distribution) || ...
        ~isfield(distribution, 'probability') || ...
        ~isfield(distribution, 'summary') || ...
        ~iscell(distribution.summary)
    error('CardinalityStability:InvalidInput', ...
        'A receiver outcome distribution is incomplete.');
end
probability = reshape(distribution.probability, 1, []);
if isempty(probability) || numel(probability) ~= ...
        numel(distribution.summary) || any(~isfinite(probability)) || ...
        any(probability < 0) || abs(sum(probability) - 1) > 1e-10
    error('CardinalityStability:InvalidInput', ...
        'Outcome probabilities are invalid.');
end
for outcomeIdx = 1:numel(distribution.summary)
    summary = distribution.summary{outcomeIdx};
    if ~isstruct(summary) || ~isfield(summary, 'existence')
        error('CardinalityStability:InvalidInput', ...
            'An outcome summary is incomplete.');
    end
    existence = reshape(summary.existence, 1, []);
    if any(~isfinite(existence)) || any(existence < 0) || ...
            any(existence > 1)
        error('CardinalityStability:InvalidInput', ...
            'LMB existence probabilities are invalid.');
    end
end
distribution.probability = probability;
end

