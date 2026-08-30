function metrics = ...
        computeFormationCommonLabelRepairOpportunityFromSynopsisV188( ...
            cache, physicalAdjacency, sensorGroupIds, options)
% COMPUTEFORMATIONCOMMONLABELREPAIROPPORTUNITYFROMSYNOPSISV188 Fast bank.

if nargin < 4 || isempty(options)
    options = struct();
end
validateCache(cache);
sensorCount = cache.sensorCount;
groupIds = reshape(sensorGroupIds, 1, []);
sourceExistenceThreshold = getField( ...
    options, 'sourceExistenceThreshold', 1e-2);
receiverPresenceThreshold = getField( ...
    options, 'receiverPresenceThreshold', 0.5);
minimumPeerCount = getField(options, 'minimumPeerCount', 1);
if ~isequal(size(physicalAdjacency), [sensorCount, sensorCount]) || ...
        numel(groupIds) ~= sensorCount || ...
        any(~isfinite(groupIds)) || any(groupIds ~= round(groupIds)) || ...
        any(groupIds < 1) || ...
        ~validUnitWeight(sourceExistenceThreshold, false) || ...
        ~validUnitWeight(receiverPresenceThreshold, true) || ...
        ~isscalar(minimumPeerCount) || ~isfinite(minimumPeerCount) || ...
        minimumPeerCount < 0 || minimumPeerCount ~= round(minimumPeerCount)
    error('FormationCommonRepairSynopsisV188:InvalidInput', ...
        'The common-source synopsis inputs are malformed.');
end
physicalAdjacency = logical(physicalAdjacency);
[labels, local, fused] = alignByLabel( ...
    cache.localBySensor, cache.fusedBySensor, sensorCount);

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
    commonSources = find(all(physicalAdjacency(receivers, :), 1));
    bestKey = [];
    for sourceIdx = reshape(commonSources, 1, [])
        sourceLabels = find(local.existence(sourceIdx, :) >= ...
            sourceExistenceThreshold);
        for labelIdx = reshape(sourceLabels, 1, [])
            if any(fused.existence(receivers, labelIdx) < ...
                    receiverPresenceThreshold)
                continue;
            end
            sourceCompatibility = compatibilityToSource( ...
                local, sourceIdx, labelIdx);
            peerConsensus = zeros(1, numel(receivers));
            complete = true;
            for receiverPosition = 1:numel(receivers)
                receiverIdx = receivers(receiverPosition);
                peers = find(physicalAdjacency(receiverIdx, :));
                peers(peers == sourceIdx) = [];
                peers = peers(local.existence(peers, labelIdx) >= ...
                    sourceExistenceThreshold);
                if numel(peers) < minimumPeerCount
                    complete = false;
                    break;
                end
                peerConsensus(receiverPosition) = mean( ...
                    sourceCompatibility(peers));
            end
            if ~complete
                continue;
            end
            receiverCompatibility = compatibilityToFusedReceivers( ...
                local, fused, sourceIdx, receivers, labelIdx);
            sourceQuality = local.existence(sourceIdx, labelIdx) * ...
                local.evidenceQuality(sourceIdx, labelIdx);
            rescue = peerConsensus .* ...
                max(1 - receiverCompatibility, 0) * max(sourceQuality, 0);
            riskReduction = fused.bayesRisk(receivers, labelIdx)' - ...
                local.bayesRisk(sourceIdx, labelIdx);
            commonCandidateCount(formationIdx) = ...
                commonCandidateCount(formationIdx) + 1;
            candidateOpportunity = median(rescue);
            candidateMinimumRisk = min(riskReduction);
            key = [-candidateOpportunity, -candidateMinimumRisk, ...
                sourceIdx, labels(labelIdx, 1), labels(labelIdx, 2)];
            if isempty(bestKey) || lexicographicallyLess(key, bestKey)
                bestKey = key;
                opportunity(formationIdx) = candidateOpportunity;
                minimumRiskReduction(formationIdx) = candidateMinimumRisk;
                minimumReceiverExistence(formationIdx) = min( ...
                    fused.existence(receivers, labelIdx));
                bestSource(formationIdx) = sourceIdx;
                bestLabel(:, formationIdx) = labels(labelIdx, :)';
                bestPeerConsensus(formationIdx) = median(peerConsensus);
                bestReceiverCompatibility(formationIdx) = ...
                    median(receiverCompatibility);
            end
        end
    end
end

metrics = struct();
metrics.contractVersion = ...
    'formation-common-label-repair-opportunity-synopsis-v188-v1';
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
metrics.communicationCharged = true;
metrics.lightSynopsisApproximation = ...
    'quantized-isotropic-position-moment';
metrics.trackingLossBoundClaimed = false;
end

function [labels, local, fused] = alignByLabel( ...
        localBySensor, fusedBySensor, sensorCount)
labels = zeros(0, 2);
for sensorIdx = 1:sensorCount
    labels = [labels; localBySensor{sensorIdx}.labels']; %#ok<AGROW>
    labels = [labels; fusedBySensor{sensorIdx}.labels']; %#ok<AGROW>
end
labels = unique(labels, 'rows');
labelCount = size(labels, 1);
local = emptyAligned(sensorCount, labelCount);
fused = emptyAligned(sensorCount, labelCount);
for sensorIdx = 1:sensorCount
    local = insertSummary(local, localBySensor{sensorIdx}, ...
        sensorIdx, labels);
    fused = insertSummary(fused, fusedBySensor{sensorIdx}, ...
        sensorIdx, labels);
end
end

function aligned = emptyAligned(sensorCount, labelCount)
aligned = struct( ...
    'existence', zeros(sensorCount, labelCount), ...
    'meanX', zeros(sensorCount, labelCount), ...
    'meanY', zeros(sensorCount, labelCount), ...
    'positionTrace', zeros(sensorCount, labelCount), ...
    'evidenceQuality', zeros(sensorCount, labelCount), ...
    'bayesRisk', 0.5 * ones(sensorCount, labelCount));
end

function aligned = insertSummary(aligned, summary, sensorIdx, labels)
if summary.labelCount == 0
    return;
end
[present, indices] = ismember(summary.labels', labels, 'rows');
if ~all(present)
    error('FormationCommonRepairSynopsisV188:LabelAlignmentDrift', ...
        'A synopsis label is missing from the shared universe.');
end
aligned.existence(sensorIdx, indices) = summary.existence;
aligned.meanX(sensorIdx, indices) = summary.positionMean(1, :);
aligned.meanY(sensorIdx, indices) = summary.positionMean(2, :);
aligned.positionTrace(sensorIdx, indices) = summary.positionTrace;
aligned.evidenceQuality(sensorIdx, indices) = summary.evidenceQuality;
aligned.bayesRisk(sensorIdx, indices) = summary.bayesRisk;
end

function values = compatibilityToSource(data, sourceIdx, labelIdx)
deltaSquared = ...
    (data.meanX(:, labelIdx) - data.meanX(sourceIdx, labelIdx)) .^ 2 + ...
    (data.meanY(:, labelIdx) - data.meanY(sourceIdx, labelIdx)) .^ 2;
denominator = data.positionTrace(:, labelIdx) + ...
    data.positionTrace(sourceIdx, labelIdx);
distance = 2 * deltaSquared ./ max(denominator, eps);
values = exp(-0.5 * min(distance, 100));
values(data.existence(:, labelIdx) <= 0) = 0;
end

function values = compatibilityToFusedReceivers( ...
        local, fused, sourceIdx, receivers, labelIdx)
deltaSquared = ...
    (fused.meanX(receivers, labelIdx) - ...
        local.meanX(sourceIdx, labelIdx)) .^ 2 + ...
    (fused.meanY(receivers, labelIdx) - ...
        local.meanY(sourceIdx, labelIdx)) .^ 2;
denominator = fused.positionTrace(receivers, labelIdx) + ...
    local.positionTrace(sourceIdx, labelIdx);
distance = 2 * deltaSquared ./ max(denominator, eps);
values = reshape(exp(-0.5 * min(distance, 100)), 1, []);
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

function validateCache(cache)
if ~isstruct(cache) || ~isfield(cache, 'contractVersion') || ...
        ~strcmp(cache.contractVersion, ...
        'formation-repair-light-synopsis-cache-v188-v1')
    error('FormationCommonRepairSynopsisV188:CacheContractDrift', ...
        'A frozen V188 light-synopsis cache is required.');
end
end

function valid = validUnitWeight(value, upperInclusive)
valid = isscalar(value) && isfinite(value) && value >= 0;
if upperInclusive
    valid = valid && value <= 1;
else
    valid = valid && value < 1;
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
