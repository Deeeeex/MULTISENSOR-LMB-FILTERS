function [scores, details] = ...
    computeLabelSetDominantRoutingScores( ...
        context, receiverIndices, senderIndices, mode, options)
% COMPUTELABELSETDOMINANTROUTINGSCORES Truth-free high-weight edge scores.
%
% The residual-cycle work scores only low-weight cross-formation edges.
% This scorer instead evaluates physically available same-formation
% candidates for the dominant input.  Per-label LMB features are pooled
% without using numeric label identifiers, target truth, or future
% outcomes.  Every semantic channel is converted to a receiver-conditional
% percentile before the registered score directions are applied.

if nargin < 5 || isempty(options)
    options = struct();
end
mode = lower(strrep(char(mode), '_', '-'));
receiverIndices = reshape(receiverIndices, [], 1);
senderIndices = reshape(senderIndices, [], 1);
edgeCount = numel(receiverIndices);
if edgeCount < 1 || numel(senderIndices) ~= edgeCount
    error('Dominant-routing candidate indices are invalid.');
end

protocol = getField(options, 'protocol', ...
    getLabelSetSimulatorPolicyProtocol());
labelOptions = struct( ...
    'maxLabelCount', protocol.maxLabelCount, ...
    'velocityScale', protocol.velocityScale, ...
    'highExistenceThreshold', ...
        protocol.highExistenceThreshold);
[labelFeatures, labelMask, featureNames, metadata] = ...
    computeLabelSetDirectedActionFeatures( ...
        context, receiverIndices, senderIndices, ...
        labelOptions);
if metadata.truthUsed || metadata.groundTruthUsed || ...
        metadata.futureOutcomeUsed || ...
        metadata.numericLabelIdentifiersUsedAsFeatures
    error('Dominant-routing label features violated truth-free scope.');
end

value = @(name) labelFeatures(:, :, ...
    requireFeature(featureNames, name));
senderPresent = value('sender_present') > 0.5 & labelMask;
shared = value('shared_label') > 0.5 & labelMask;
senderExistence = value('sender_existence');
sourceCardinality = maskedSum( ...
    senderExistence, labelMask);
sourceConfidence = maskedMean( ...
    value('sender_association_confidence'), ...
    senderPresent);
sourceDetection = maskedMean( ...
    value('sender_detection_association_mass'), ...
    senderPresent);
sourceClarity = 1 - maskedMean( ...
    value('sender_association_ambiguity'), ...
    senderPresent);
sourcePrecision = -maskedMean( ...
    value('sender_normalized_position_variance'), ...
    senderPresent);
existenceTransfer = maskedSum( ...
    value('positive_existence_gain') - ...
        value('negative_existence_gap'), labelMask);
precisionTransfer = maskedSum( ...
    value('positive_precision_gain') - ...
        value('negative_precision_gap'), labelMask);
compatibility = maskedMean( ...
    value('gaussian_compatibility'), shared);
negativeDisagreement = -maskedMean( ...
    value('log_combined_mahalanobis_disagreement') + ...
        value('normalized_position_disagreement') + ...
        value('normalized_velocity_disagreement'), shared);
linkReliability = zeros(edgeCount, 1);
for edgeIdx = 1:edgeCount
    linkReliability(edgeIdx) = resolveLinkReliability( ...
        context.commConfig, senderIndices(edgeIdx), ...
        receiverIndices(edgeIdx), context.currentTime);
end

rawChannels = [ ...
    sourceCardinality, sourceConfidence, ...
    sourceDetection, sourceClarity, sourcePrecision, ...
    existenceTransfer, precisionTransfer, ...
    compatibility, negativeDisagreement, ...
    linkReliability];
channelNames = { ...
    'source_expected_cardinality', ...
    'source_association_confidence', ...
    'source_detection_mass', ...
    'source_association_clarity', ...
    'source_precision', ...
    'existence_transfer', ...
    'precision_transfer', ...
    'posterior_compatibility', ...
    'negative_state_disagreement', ...
    'link_reliability'};
channels = receiverConditionalPercentiles( ...
    rawChannels, receiverIndices);

switch mode
    case 'source-quality'
        coefficients = [ ...
            0.25, 0.20, 0.15, 0.10, 0.15, ...
            0, 0, 0, 0, 0.15];
    case 'transfer'
        coefficients = [ ...
            0.05, 0.05, 0.05, 0.05, 0.05, ...
            0.25, 0.20, 0.15, 0.10, 0.05];
    case 'compatibility'
        coefficients = [ ...
            0.05, 0.10, 0.05, 0.10, 0.05, ...
            0.05, 0.05, 0.30, 0.20, 0.05];
    case {'composite-balanced', 'composite-concentrated'}
        coefficients = [ ...
            0.15, 0.10, 0.05, 0.05, 0.10, ...
            0.15, 0.10, 0.10, 0.05, 0.05];
    otherwise
        error('Unknown dominant-routing score mode: %s.', mode);
end
scores = channels * coefficients';
if any(~isfinite(scores))
    error('Dominant-routing scores are non-finite.');
end

details = struct();
details.contractVersion = ...
    'label-set-dominant-routing-scores-v1';
details.mode = mode;
details.receiverIndices = receiverIndices;
details.senderIndices = senderIndices;
details.channelNames = channelNames;
details.rawChannels = rawChannels;
details.receiverConditionalPercentiles = channels;
details.coefficients = coefficients;
details.scores = scores;
details.labelFeatureContractSha256 = ...
    metadata.featureContractSha256;
details.posteriorUsed = true;
details.currentLinkReliabilityUsed = true;
details.numericLabelIdentifiersUsedAsFeatures = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureOutcomeUsed = false;
end

function values = maskedSum(tensor, mask)
values = sum(tensor .* double(mask), 2);
values = reshape(values, [], 1);
end

function values = maskedMean(tensor, mask)
count = sum(mask, 2);
values = sum(tensor .* double(mask), 2) ./ max(count, 1);
values(count == 0) = 0;
values = reshape(values, [], 1);
end

function transformed = receiverConditionalPercentiles( ...
        values, receiverIndices)
transformed = zeros(size(values));
receivers = unique(receiverIndices, 'stable');
for receiverCursor = 1:numel(receivers)
    rows = receiverIndices == receivers(receiverCursor);
    for columnIdx = 1:size(values, 2)
        transformed(rows, columnIdx) = ...
            averageRankPercentile(values(rows, columnIdx));
    end
end
end

function percentile = averageRankPercentile(values)
values = reshape(values, [], 1);
if any(~isfinite(values))
    error('Dominant-routing channel contains non-finite values.');
end
count = numel(values);
if count <= 1
    percentile = zeros(count, 1);
    return;
end
[sorted, order] = sort(values, 'ascend');
ranks = zeros(count, 1);
cursor = 1;
while cursor <= count
    last = cursor;
    while last < count && sorted(last + 1) == sorted(cursor)
        last = last + 1;
    end
    ranks(order(cursor:last)) = (cursor + last) / 2;
    cursor = last + 1;
end
percentile = (ranks - 1) / (count - 1);
end

function reliability = resolveLinkReliability( ...
        config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        dropProbability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        dropProbability = ...
            config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    dropProbability = config.pDropBySensor(senderIdx);
else
    dropProbability = 0;
end
reliability = max(1e-6, ...
    1 - min(max(dropProbability, 0), 1));
end

function featureIdx = requireFeature(featureNames, featureName)
featureIdx = find(strcmp(featureNames, featureName), 1);
if isempty(featureIdx)
    error('Dominant-routing scorer requires %s.', featureName);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
