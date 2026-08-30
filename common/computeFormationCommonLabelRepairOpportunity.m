function metrics = computeFormationCommonLabelRepairOpportunity( ...
        fusedPosteriorBySensor, localPosteriorBySensor, ...
        physicalAdjacency, sensorGroupIds, model, options)
% COMPUTEFORMATIONCOMMONLABELREPAIROPPORTUNITY Source-aware trigger signal.
%
% For each formation, enumerate only exact (label, source) pairs whose
% source is physically reachable by every receiver.  A pair scores highly
% when the source label is supported by current measurement evidence and
% compatible peer sources, yet spatially disagrees with the corresponding
% receiver labels.  The same pair must be present at every receiver.  This
% is the inexpensive semantic target for a two-stage synopsis trigger; it
% is not an authorization to transmit or a bound on tracking improvement.
%
% Numeric label values are used only to join Bernoulli objects.  They are
% never included in the score or any learned feature.

if nargin < 6 || isempty(options)
    options = struct();
end
sensorCount = numel(localPosteriorBySensor);
groupIds = reshape(sensorGroupIds, 1, []);
sourceExistenceThreshold = getField( ...
    options, 'sourceExistenceThreshold', 1e-2);
receiverPresenceThreshold = getField( ...
    options, 'receiverPresenceThreshold', 0.5);
minimumPeerCount = getField(options, 'minimumPeerCount', 1);
if ~iscell(fusedPosteriorBySensor) || ...
        numel(fusedPosteriorBySensor) ~= sensorCount || ...
        ~isequal(size(physicalAdjacency), [sensorCount, sensorCount]) || ...
        numel(groupIds) ~= sensorCount || sensorCount < 1 || ...
        any(~isfinite(groupIds)) || any(groupIds ~= round(groupIds)) || ...
        any(groupIds < 1) || ...
        ~isscalar(sourceExistenceThreshold) || ...
        ~isfinite(sourceExistenceThreshold) || ...
        sourceExistenceThreshold < 0 || sourceExistenceThreshold >= 1 || ...
        ~isscalar(receiverPresenceThreshold) || ...
        ~isfinite(receiverPresenceThreshold) || ...
        receiverPresenceThreshold < 0 || receiverPresenceThreshold > 1 || ...
        ~isscalar(minimumPeerCount) || ~isfinite(minimumPeerCount) || ...
        minimumPeerCount < 0 || minimumPeerCount ~= round(minimumPeerCount)
    error('FormationCommonRepairOpportunity:InvalidInput', ...
        'The source-aware trigger inputs or options are malformed.');
end
physicalAdjacency = logical(physicalAdjacency);
summaryBySensor = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    summaryBySensor{sensorIdx} = summarizeObjects( ...
        localPosteriorBySensor{sensorIdx}, model);
end
fusedSummaryBySensor = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    fusedSummaryBySensor{sensorIdx} = summarizeObjects( ...
        fusedPosteriorBySensor{sensorIdx}, model);
end

formationIds = unique(groupIds, 'stable');
formationCount = numel(formationIds);
opportunity = zeros(1, formationCount);
minimumRiskReduction = nan(1, formationCount);
minimumReceiverExistence = nan(1, formationCount);
commonCandidateCount = zeros(1, formationCount);
bestSource = zeros(1, formationCount);
bestLabel = zeros(2, formationCount);
bestPeerConsensus = zeros(1, formationCount);
bestReceiverCompatibility = ones(1, formationCount);
for formationIdx = 1:formationCount
    receivers = find(groupIds == formationIds(formationIdx));
    commonSources = 1:sensorCount;
    for receiverIdx = reshape(receivers, 1, [])
        commonSources = intersect(commonSources, ...
            find(physicalAdjacency(receiverIdx, :)), 'stable');
    end
    bestKey = [];
    for sourceIdx = reshape(commonSources, 1, [])
        sourceSummary = summaryBySensor{sourceIdx};
        sourceCandidates = find( ...
            sourceSummary.existence >= sourceExistenceThreshold);
        for sourceLabelIdx = reshape(sourceCandidates, 1, [])
            label = sourceSummary.labels(:, sourceLabelIdx);
            rescueByReceiver = zeros(1, numel(receivers));
            riskReductionByReceiver = zeros(1, numel(receivers));
            receiverExistence = zeros(1, numel(receivers));
            peerConsensusByReceiver = zeros(1, numel(receivers));
            compatibilityByReceiver = zeros(1, numel(receivers));
            complete = true;
            for receiverPosition = 1:numel(receivers)
                receiverIdx = receivers(receiverPosition);
                receiverSummary = fusedSummaryBySensor{receiverIdx};
                receiverLabelIdx = findLabel(receiverSummary.labels, label);
                if receiverLabelIdx == 0 || ...
                        receiverSummary.existence(receiverLabelIdx) < ...
                            receiverPresenceThreshold
                    complete = false;
                    break;
                end
                receiverExistence(receiverPosition) = ...
                    receiverSummary.existence(receiverLabelIdx);
                compatibility = labelCompatibility( ...
                    receiverSummary, receiverLabelIdx, ...
                    sourceSummary, sourceLabelIdx);
                peers = find(physicalAdjacency(receiverIdx, :));
                peers(peers == sourceIdx) = [];
                peerCompatibility = zeros(1, 0);
                for peerIdx = reshape(peers, 1, [])
                    peerSummary = summaryBySensor{peerIdx};
                    peerLabelIdx = findLabel(peerSummary.labels, label);
                    if peerLabelIdx == 0 || ...
                            peerSummary.existence(peerLabelIdx) < ...
                                sourceExistenceThreshold
                        continue;
                    end
                    peerCompatibility(end + 1) = labelCompatibility( ...
                        sourceSummary, sourceLabelIdx, ...
                        peerSummary, peerLabelIdx); %#ok<AGROW>
                end
                if numel(peerCompatibility) < minimumPeerCount
                    complete = false;
                    break;
                end
                peerConsensus = mean(peerCompatibility);
                sourceQuality = sourceSummary.existence(sourceLabelIdx) * ...
                    sourceSummary.evidenceQuality(sourceLabelIdx);
                rescueByReceiver(receiverPosition) = peerConsensus * ...
                    max(1 - compatibility, 0) * max(sourceQuality, 0);
                riskReductionByReceiver(receiverPosition) = ...
                    receiverSummary.bayesRisk(receiverLabelIdx) - ...
                    sourceSummary.bayesRisk(sourceLabelIdx);
                peerConsensusByReceiver(receiverPosition) = peerConsensus;
                compatibilityByReceiver(receiverPosition) = compatibility;
            end
            if ~complete
                continue;
            end
            commonCandidateCount(formationIdx) = ...
                commonCandidateCount(formationIdx) + 1;
            candidateOpportunity = median(rescueByReceiver);
            candidateMinimumRisk = min(riskReductionByReceiver);
            key = [-candidateOpportunity, -candidateMinimumRisk, sourceIdx, ...
                label(1), label(2)];
            if isempty(bestKey) || lexicographicallyLess(key, bestKey)
                bestKey = key;
                opportunity(formationIdx) = candidateOpportunity;
                minimumRiskReduction(formationIdx) = candidateMinimumRisk;
                minimumReceiverExistence(formationIdx) = ...
                    min(receiverExistence);
                bestSource(formationIdx) = sourceIdx;
                bestLabel(:, formationIdx) = label;
                bestPeerConsensus(formationIdx) = ...
                    median(peerConsensusByReceiver);
                bestReceiverCompatibility(formationIdx) = ...
                    median(compatibilityByReceiver);
            end
        end
    end
end

metrics = struct();
metrics.contractVersion = ...
    'formation-common-label-repair-opportunity-v188-v1';
metrics.formationIds = formationIds;
metrics.opportunity = opportunity;
metrics.minimumRiskReduction = minimumRiskReduction;
metrics.minimumReceiverExistence = minimumReceiverExistence;
metrics.commonCandidateCount = commonCandidateCount;
metrics.bestSource = bestSource;
metrics.bestLabel = bestLabel;
metrics.bestPeerConsensus = bestPeerConsensus;
metrics.bestReceiverCompatibility = bestReceiverCompatibility;
metrics.sourceExistenceThreshold = sourceExistenceThreshold;
metrics.receiverPresenceThreshold = receiverPresenceThreshold;
metrics.minimumPeerCount = minimumPeerCount;
metrics.truthUsed = false;
metrics.futureInformationUsed = false;
metrics.numericLabelIdentifiersUsedAsFeatures = false;
metrics.completePayloadRequested = false;
metrics.communicationCharged = false;
metrics.requiresSynopsisImplementation = true;
metrics.trackingLossBoundClaimed = false;
end

function summary = summarizeObjects(objects, model)
objects = reshape(objects, 1, []);
summary = struct( ...
    'labels', zeros(2, 0), ...
    'existence', zeros(1, 0), ...
    'mean', zeros(2, 0), ...
    'covariance', zeros(2, 2, 0), ...
    'evidenceQuality', zeros(1, 0), ...
    'bayesRisk', zeros(1, 0));
for objectIdx = 1:numel(objects)
    object = objects(objectIdx);
    if object.numberOfGmComponents < 1
        continue;
    end
    [risk, riskDetails] = ...
        computeObservableLmbLabelBayesRisk(object, model);
    [meanVector, covariance] = objectMoments(object);
    summary.labels(:, end + 1) = [ ...
        object.birthTime; object.birthLocation]; %#ok<AGROW>
    summary.existence(end + 1) = ...
        min(max(object.r, 0), 1); %#ok<AGROW>
    summary.mean(:, end + 1) = meanVector(1:2); %#ok<AGROW>
    summary.covariance(:, :, end + 1) = ...
        covariance(1:2, 1:2); %#ok<AGROW>
    summary.evidenceQuality(end + 1) = 0.5 * ( ...
        boundedScalar(object, 'associationConfidence') + ...
        boundedScalar(object, 'detectionAssociationMass')); %#ok<AGROW>
    summary.bayesRisk(end + 1) = risk; %#ok<AGROW>
    if abs(riskDetails.existence - summary.existence(end)) > 1e-12
        error('FormationCommonRepairOpportunity:RiskContractDrift', ...
            'The label-risk and source synopsis existence disagree.');
    end
end
if size(unique(summary.labels', 'rows'), 1) ~= ...
        size(summary.labels, 2)
    error('FormationCommonRepairOpportunity:DuplicateLabel', ...
        'Every sensor synopsis must contain unique labels.');
end
end

function [meanVector, covariance] = objectMoments(object)
weights = reshape(object.w, 1, []);
weights(~isfinite(weights)) = 0;
weights = max(weights, 0);
if numel(weights) ~= object.numberOfGmComponents || sum(weights) <= 0
    weights = ones(1, object.numberOfGmComponents);
end
weights = weights / sum(weights);
stateDimension = numel(object.mu{1});
meanVector = zeros(stateDimension, 1);
for componentIdx = 1:object.numberOfGmComponents
    meanVector = meanVector + weights(componentIdx) * ...
        object.mu{componentIdx};
end
covariance = zeros(stateDimension);
for componentIdx = 1:object.numberOfGmComponents
    delta = object.mu{componentIdx} - meanVector;
    covariance = covariance + weights(componentIdx) * ...
        (object.Sigma{componentIdx} + delta * delta');
end
covariance = regularizeCovariance(covariance);
end

function compatibility = labelCompatibility( ...
        left, leftIdx, right, rightIdx)
delta = left.mean(:, leftIdx) - right.mean(:, rightIdx);
covariance = left.covariance(:, :, leftIdx) + ...
    right.covariance(:, :, rightIdx);
covariance = regularizeCovariance(covariance);
distance = max(real(delta' * (covariance \ delta)), 0);
if ~isfinite(distance)
    distance = 1e6;
end
compatibility = exp(-0.5 * min(distance, 100));
end

function covariance = regularizeCovariance(covariance)
covariance = (covariance + covariance') / 2;
jitter = 0;
for attempt = 1:8
    [~, flag] = chol(covariance + jitter * eye(size(covariance)));
    if flag == 0
        covariance = covariance + jitter * eye(size(covariance));
        return;
    end
    jitter = max(1e-12, 10 * max(jitter, 1e-12));
end
error('FormationCommonRepairOpportunity:InvalidCovariance', ...
    'A label covariance cannot be regularized.');
end

function index = findLabel(labels, label)
index = 0;
if isempty(labels)
    return;
end
match = find(all(labels == label, 1), 1);
if ~isempty(match)
    index = match;
end
end

function value = boundedScalar(data, name)
value = 0;
if isstruct(data) && isfield(data, name) && ...
        isscalar(data.(name)) && isfinite(data.(name))
    value = min(max(data.(name), 0), 1);
end
end

function value = lexicographicallyLess(left, right)
value = false;
for idx = 1:numel(left)
    if left(idx) < right(idx) - 1e-15
        value = true;
        return;
    elseif left(idx) > right(idx) + 1e-15
        return;
    end
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
