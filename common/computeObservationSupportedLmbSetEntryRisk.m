function [risk, details] = computeObservationSupportedLmbSetEntryRisk( ...
        referenceReceiverDistributions, ...
        candidateReceiverDistributions, localPosteriorBySensor, ...
        groupIds, options)
% COMPUTEOBSERVATIONSUPPORTEDLMBSETENTRYRISK Detect unsupported MAP entries.
%
% The candidate and reference link-outcome distributions are reduced to
% their exact marginal cardinality PMFs and marginal label existences.  The
% usual LMB point extractor is then approximated by the marginal MAP
% cardinality and the corresponding highest-existence labels.  Candidate
% labels that enter this set without current receiver measurement support
% are treated as over-cardinality risk.  Label values are used only to align
% Bernoulli components, never as numeric features.

if nargin < 5 || isempty(options)
    options = struct();
end
positiveSupportThreshold = getField( ...
    options, 'positiveSupportThreshold', 0.20);
if ~iscell(referenceReceiverDistributions) || ...
        ~iscell(candidateReceiverDistributions) || ...
        ~iscell(localPosteriorBySensor) || ...
        isempty(referenceReceiverDistributions) || ...
        numel(referenceReceiverDistributions) ~= ...
            numel(candidateReceiverDistributions) || ...
        numel(referenceReceiverDistributions) ~= ...
            numel(localPosteriorBySensor) || ...
        ~isscalar(positiveSupportThreshold) || ...
        ~isfinite(positiveSupportThreshold) || ...
        positiveSupportThreshold < 0 || positiveSupportThreshold > 1
    error('SetEntryRisk:InvalidInput', ...
        'Set-entry distributions or support threshold are invalid.');
end
nodeCount = numel(localPosteriorBySensor);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
if numel(groupIds) ~= nodeCount || isempty(groups)
    error('SetEntryRisk:InvalidInput', ...
        'Formation membership is invalid.');
end

[~, cardinality] = computeLmbCardinalityDecisionStability( ...
    referenceReceiverDistributions, ...
    candidateReceiverDistributions, groupIds);
entryCount = zeros(1, nodeCount);
unsupportedEntryCount = zeros(1, nodeCount);
entryExistenceMass = zeros(1, nodeCount);
unsupportedEntryExistenceMass = zeros(1, nodeCount);
supportWeightedEntryExistenceMass = zeros(1, nodeCount);
minimumEntrySupport = ones(1, nodeCount);
meanEntrySupport = ones(1, nodeCount);
unsupportedEntryFraction = zeros(1, nodeCount);
unsupportedEntryMassFraction = zeros(1, nodeCount);
receiverRisk = zeros(1, nodeCount);
referenceExtractedLabelsByReceiver = cell(1, nodeCount);
candidateExtractedLabelsByReceiver = cell(1, nodeCount);
enteredLabelsByReceiver = cell(1, nodeCount);
enteredSupportByReceiver = cell(1, nodeCount);
enteredCandidateExistenceByReceiver = cell(1, nodeCount);

for receiverIdx = 1:nodeCount
    labels = distributionLabelUniverse( ...
        referenceReceiverDistributions{receiverIdx}, ...
        candidateReceiverDistributions{receiverIdx});
    referenceExistence = expectedExistence( ...
        referenceReceiverDistributions{receiverIdx}, labels);
    candidateExistence = expectedExistence( ...
        candidateReceiverDistributions{receiverIdx}, labels);
    referenceLabels = extractTopLabels(labels, referenceExistence, ...
        cardinality.referenceMapCardinality(receiverIdx));
    candidateLabels = extractTopLabels(labels, candidateExistence, ...
        cardinality.candidateMapCardinality(receiverIdx));
    enteredMask = ~columnMembership(candidateLabels, referenceLabels);
    enteredLabels = candidateLabels(:, enteredMask);
    enteredExistence = labelValues( ...
        labels, candidateExistence, enteredLabels);
    enteredSupport = currentAssociationMass( ...
        localPosteriorBySensor{receiverIdx}, enteredLabels);
    unsupported = enteredSupport < positiveSupportThreshold - 1e-12;

    entryCount(receiverIdx) = size(enteredLabels, 2);
    unsupportedEntryCount(receiverIdx) = nnz(unsupported);
    entryExistenceMass(receiverIdx) = sum(enteredExistence);
    unsupportedEntryExistenceMass(receiverIdx) = ...
        sum(enteredExistence(unsupported));
    supportWeightedEntryExistenceMass(receiverIdx) = ...
        sum(enteredExistence .* enteredSupport);
    if ~isempty(enteredSupport)
        minimumEntrySupport(receiverIdx) = min(enteredSupport);
        meanEntrySupport(receiverIdx) = mean(enteredSupport);
    end
    unsupportedEntryFraction(receiverIdx) = ...
        unsupportedEntryCount(receiverIdx) / max(entryCount(receiverIdx), 1);
    unsupportedEntryMassFraction(receiverIdx) = ...
        unsupportedEntryExistenceMass(receiverIdx) / max( ...
            entryExistenceMass(receiverIdx), eps);
    receiverRisk(receiverIdx) = ...
        unsupportedEntryExistenceMass(receiverIdx) / max( ...
            cardinality.referenceExpectedCardinality(receiverIdx), 1);
    referenceExtractedLabelsByReceiver{receiverIdx} = referenceLabels;
    candidateExtractedLabelsByReceiver{receiverIdx} = candidateLabels;
    enteredLabelsByReceiver{receiverIdx} = enteredLabels;
    enteredSupportByReceiver{receiverIdx} = enteredSupport;
    enteredCandidateExistenceByReceiver{receiverIdx} = enteredExistence;
end

formationCount = numel(groups);
formationEntryCount = zeros(1, formationCount);
formationUnsupportedEntryCount = zeros(1, formationCount);
formationUnsupportedEntryExistenceMass = zeros(1, formationCount);
formationMaximumReceiverRisk = zeros(1, formationCount);
formationMeanReceiverRisk = zeros(1, formationCount);
formationMinimumEntrySupport = ones(1, formationCount);
for formationIdx = 1:formationCount
    members = groupIds == groups(formationIdx);
    formationEntryCount(formationIdx) = sum(entryCount(members));
    formationUnsupportedEntryCount(formationIdx) = ...
        sum(unsupportedEntryCount(members));
    formationUnsupportedEntryExistenceMass(formationIdx) = ...
        sum(unsupportedEntryExistenceMass(members));
    formationMaximumReceiverRisk(formationIdx) = ...
        max(receiverRisk(members));
    formationMeanReceiverRisk(formationIdx) = ...
        mean(receiverRisk(members));
    formationMinimumEntrySupport(formationIdx) = ...
        min(minimumEntrySupport(members));
end

risk = max(receiverRisk);
details = struct();
details.contractVersion = ...
    'observation-supported-lmb-set-entry-risk-v1';
details.positiveSupportThreshold = positiveSupportThreshold;
details.groupIds = groupIds;
details.groups = groups;
details.cardinality = cardinality;
details.entryCount = entryCount;
details.unsupportedEntryCount = unsupportedEntryCount;
details.entryExistenceMass = entryExistenceMass;
details.unsupportedEntryExistenceMass = ...
    unsupportedEntryExistenceMass;
details.supportWeightedEntryExistenceMass = ...
    supportWeightedEntryExistenceMass;
details.minimumEntrySupport = minimumEntrySupport;
details.meanEntrySupport = meanEntrySupport;
details.unsupportedEntryFraction = unsupportedEntryFraction;
details.unsupportedEntryMassFraction = unsupportedEntryMassFraction;
details.receiverRisk = receiverRisk;
details.referenceExtractedLabelsByReceiver = ...
    referenceExtractedLabelsByReceiver;
details.candidateExtractedLabelsByReceiver = ...
    candidateExtractedLabelsByReceiver;
details.enteredLabelsByReceiver = enteredLabelsByReceiver;
details.enteredSupportByReceiver = enteredSupportByReceiver;
details.enteredCandidateExistenceByReceiver = ...
    enteredCandidateExistenceByReceiver;
details.formationEntryCount = formationEntryCount;
details.formationUnsupportedEntryCount = ...
    formationUnsupportedEntryCount;
details.formationUnsupportedEntryExistenceMass = ...
    formationUnsupportedEntryExistenceMass;
details.formationMaximumReceiverRisk = formationMaximumReceiverRisk;
details.formationMeanReceiverRisk = formationMeanReceiverRisk;
details.formationMinimumEntrySupport = formationMinimumEntrySupport;
details.risk = risk;
details.marginalLmbExtractionApproximationUsed = true;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.truthUsed = false;
details.futureInformationUsed = false;
end

function labels = distributionLabelUniverse(left, right)
labels = zeros(2, 0);
for distribution = {left, right}
    currentDistribution = validateDistribution(distribution{1});
    for outcomeIdx = 1:numel(currentDistribution.summary)
        current = currentDistribution.summary{outcomeIdx}.labels;
        for labelIdx = 1:size(current, 2)
            if isempty(labels) || ...
                    ~any(all(labels == current(:, labelIdx), 1))
                labels(:, end + 1) = current(:, labelIdx); %#ok<AGROW>
            end
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
end

function values = expectedExistence(distribution, labels)
distribution = validateDistribution(distribution);
values = zeros(1, size(labels, 2));
for outcomeIdx = 1:numel(distribution.summary)
    summary = distribution.summary{outcomeIdx};
    for labelIdx = 1:size(summary.labels, 2)
        universeIdx = find(all( ...
            labels == summary.labels(:, labelIdx), 1), 1);
        values(universeIdx) = values(universeIdx) + ...
            distribution.probability(outcomeIdx) * ...
                summary.existence(labelIdx);
    end
end
end

function labels = extractTopLabels(allLabels, existence, count)
count = min(max(round(count), 0), size(allLabels, 2));
if count == 0
    labels = zeros(2, 0);
    return;
end
ranking = [-reshape(existence, [], 1), allLabels'];
[~, order] = sortrows(ranking, [1, 2, 3]);
labels = allLabels(:, order(1:count));
end

function mask = columnMembership(left, right)
mask = false(1, size(left, 2));
for idx = 1:size(left, 2)
    mask(idx) = ~isempty(right) && ...
        any(all(right == left(:, idx), 1));
end
end

function values = labelValues(allLabels, allValues, labels)
values = zeros(1, size(labels, 2));
for idx = 1:size(labels, 2)
    universeIdx = find(all(allLabels == labels(:, idx), 1), 1);
    values(idx) = allValues(universeIdx);
end
end

function values = currentAssociationMass(objects, labels)
values = zeros(1, size(labels, 2));
for labelIdx = 1:size(labels, 2)
    for objectIdx = 1:numel(objects)
        if objects(objectIdx).numberOfGmComponents > 0 && ...
                objects(objectIdx).birthTime == labels(1, labelIdx) && ...
                objects(objectIdx).birthLocation == labels(2, labelIdx)
            values(labelIdx) = min(max(getField( ...
                objects(objectIdx), 'detectionAssociationMass', 0), 0), 1);
            break;
        end
    end
end
end

function distribution = validateDistribution(distribution)
if ~isstruct(distribution) || ...
        ~isfield(distribution, 'probability') || ...
        ~isfield(distribution, 'summary') || ...
        ~iscell(distribution.summary)
    error('SetEntryRisk:InvalidInput', ...
        'A receiver outcome distribution is incomplete.');
end
probability = reshape(distribution.probability, 1, []);
if isempty(probability) || numel(probability) ~= ...
        numel(distribution.summary) || any(~isfinite(probability)) || ...
        any(probability < 0) || abs(sum(probability) - 1) > 1e-10
    error('SetEntryRisk:InvalidInput', ...
        'Outcome probabilities are invalid.');
end
for outcomeIdx = 1:numel(distribution.summary)
    summary = distribution.summary{outcomeIdx};
    if ~isstruct(summary) || ~isfield(summary, 'labels') || ...
            ~isfield(summary, 'existence') || ...
            size(summary.labels, 1) ~= 2 || ...
            size(summary.labels, 2) ~= numel(summary.existence) || ...
            any(~isfinite(summary.labels(:))) || ...
            any(~isfinite(summary.existence)) || ...
            any(summary.existence < 0) || any(summary.existence > 1)
        error('SetEntryRisk:InvalidInput', ...
            'An LMB outcome summary is invalid.');
    end
end
distribution.probability = probability;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end

