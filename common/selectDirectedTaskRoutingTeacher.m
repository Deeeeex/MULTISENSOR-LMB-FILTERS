function [routedPosteriors, details] = ...
    selectDirectedTaskRoutingTeacher(context, options)
% SELECTDIRECTEDTASKROUTINGTEACHER Privileged directional fusion diagnostic.
%
% The current topology path couples two decisions: selecting an undirected
% edge and applying symmetric Metropolis fusion at both endpoints. This
% offline teacher separates them. For every receiver it greedily selects
% only source posteriors that reduce that receiver's labelled task risk and
% line-searches the KLA source weight. The sender is not modified.
%
% The teacher reads truth through evaluateLmbTopologyTaskRisk and is not
% deployable. Its purpose is to test whether directional, receiver-specific
% routing has enough residual value to justify a learned local surrogate.
%
% Link delivery is a Bernoulli event: on success the registered KLA weight
% is applied, and on failure the receiver remains local. The default
% teacher therefore scores expected risk over these two outcomes. It does
% not shrink the KLA weight by link reliability.

if nargin < 2 || isempty(options)
    options = struct();
end
timerId = tic;
posteriors = reshape(context.localPosteriorBySensor, 1, []);
nodeCount = numel(posteriors);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;

maxMessagesPerReceiver = max(0, round(getField( ...
    options, 'maxMessagesPerReceiver', 2)));
sourceWeightGrid = reshape(getField( ...
    options, 'sourceWeightGrid', [0.15, 0.30, 0.50, 0.70]), 1, []);
sourceWeightGrid = unique(sourceWeightGrid( ...
    sourceWeightGrid > 0 & sourceWeightGrid < 1));
if isempty(sourceWeightGrid)
    error('sourceWeightGrid must contain a value strictly between 0 and 1.');
end
minimumRelativeNodeGain = max(getField( ...
    options, 'minimumRelativeNodeGain', 0), 0);
deliveryExpectationMode = resolveDeliveryExpectationMode(options);
if strcmp(deliveryExpectationMode, 'bernoulli-risk') && ...
        maxMessagesPerReceiver > 1
    error(['bernoulli-risk delivery expectation currently supports ', ...
        'exactly one message per receiver.']);
end
riskOptions = getField(options, 'riskOptions', struct());

routedPosteriors = posteriors;
nodeRiskBefore = zeros(1, nodeCount);
nodeRiskAfter = zeros(1, nodeCount);
selectedSources = cell(1, nodeCount);
selectedSourceWeights = cell(1, nodeCount);
selectedEffectiveWeights = cell(1, nodeCount);
selectedStepGains = cell(1, nodeCount);
firstStepCandidateRisk = nan(nodeCount);
firstStepCandidateConditionalRisk = nan(nodeCount);
firstStepCandidateExpectedGain = nan(nodeCount);
firstStepCandidateSourceWeight = nan(nodeCount);
firstStepCandidateEffectiveWeight = nan(nodeCount);
weightCount = numel(sourceWeightGrid);
firstStepConditionalRiskByWeight = ...
    nan(nodeCount, nodeCount, weightCount);
firstStepExpectedRiskByWeight = ...
    nan(nodeCount, nodeCount, weightCount);
firstStepExpectedGainByWeight = ...
    nan(nodeCount, nodeCount, weightCount);
adjacencySchedule = false( ...
    nodeCount, nodeCount, maxMessagesPerReceiver);
payloadBytes = estimateNodePayloadBytes(posteriors, context.model);
attemptedBytes = 0;

for receiverIdx = 1:nodeCount
    currentPosterior = posteriors{receiverIdx};
    currentRisk = evaluateLmbTopologyTaskRisk( ...
        currentPosterior, context.model, context.currentTime, ...
        riskOptions);
    nodeRiskBefore(receiverIdx) = currentRisk;
    availableSources = find(physical(receiverIdx, :));
    usedSources = false(1, nodeCount);

    for messageIdx = 1:maxMessagesPerReceiver
        bestRisk = currentRisk;
        bestSource = NaN;
        bestNominalWeight = NaN;
        bestEffectiveWeight = NaN;
        bestPosterior = [];
        for senderIdx = reshape(availableSources, 1, [])
            if usedSources(senderIdx)
                continue;
            end
            senderBestRisk = inf;
            senderBestNominalWeight = NaN;
            senderBestEffectiveWeight = NaN;
            senderBestConditionalRisk = NaN;
            reliability = 1 - edgeDrop( ...
                context.commConfig, senderIdx, receiverIdx, ...
                context.currentTime);
            for weightIdx = 1:weightCount
                nominalWeight = sourceWeightGrid(weightIdx);
                [weights, effectiveWeight] = resolveCandidateWeights( ...
                    nominalWeight, reliability, ...
                    deliveryExpectationMode);
                fusionDetails = struct('eventType', [0, 2]);
                candidatePosterior = fuseLmbPosteriorsByLabel( ...
                    {currentPosterior, posteriors{senderIdx}}, ...
                    weights, context.model, weights, ...
                    fusionDetails, context.triggerConfig);
                conditionalRisk = evaluateLmbTopologyTaskRisk( ...
                    candidatePosterior, context.model, ...
                    context.currentTime, riskOptions);
                expectedRisk = expectedCandidateRisk( ...
                    currentRisk, conditionalRisk, reliability, ...
                    deliveryExpectationMode);
                if messageIdx == 1
                    firstStepConditionalRiskByWeight( ...
                        receiverIdx, senderIdx, weightIdx) = ...
                        conditionalRisk;
                    firstStepExpectedRiskByWeight( ...
                        receiverIdx, senderIdx, weightIdx) = ...
                        expectedRisk;
                    firstStepExpectedGainByWeight( ...
                        receiverIdx, senderIdx, weightIdx) = ...
                        currentRisk - expectedRisk;
                end
                if expectedRisk < bestRisk - 1e-12
                    bestRisk = expectedRisk;
                    bestSource = senderIdx;
                    bestNominalWeight = nominalWeight;
                    bestEffectiveWeight = effectiveWeight;
                    bestPosterior = candidatePosterior;
                end
                if expectedRisk < senderBestRisk
                    senderBestRisk = expectedRisk;
                    senderBestNominalWeight = nominalWeight;
                    senderBestEffectiveWeight = effectiveWeight;
                    senderBestConditionalRisk = conditionalRisk;
                end
            end
            if messageIdx == 1
                firstStepCandidateRisk(receiverIdx, senderIdx) = ...
                    senderBestRisk;
                firstStepCandidateConditionalRisk( ...
                    receiverIdx, senderIdx) = ...
                    senderBestConditionalRisk;
                firstStepCandidateExpectedGain( ...
                    receiverIdx, senderIdx) = ...
                    currentRisk - senderBestRisk;
                firstStepCandidateSourceWeight( ...
                    receiverIdx, senderIdx) = senderBestNominalWeight;
                firstStepCandidateEffectiveWeight( ...
                    receiverIdx, senderIdx) = senderBestEffectiveWeight;
            end
        end
        relativeGain = (currentRisk - bestRisk) / max(currentRisk, eps);
        if ~isfinite(bestSource) || ...
                relativeGain + 1e-12 < minimumRelativeNodeGain
            break;
        end
        selectedSources{receiverIdx}(end+1) = bestSource; %#ok<AGROW>
        selectedSourceWeights{receiverIdx}(end+1) = ...
            bestNominalWeight; %#ok<AGROW>
        selectedEffectiveWeights{receiverIdx}(end+1) = ...
            bestEffectiveWeight; %#ok<AGROW>
        selectedStepGains{receiverIdx}(end+1) = ...
            currentRisk - bestRisk; %#ok<AGROW>
        adjacencySchedule(receiverIdx, bestSource, messageIdx) = true;
        attemptedBytes = attemptedBytes + payloadBytes(bestSource);
        usedSources(bestSource) = true;
        currentPosterior = bestPosterior;
        currentRisk = bestRisk;
    end
    routedPosteriors{receiverIdx} = currentPosterior;
    nodeRiskAfter(receiverIdx) = currentRisk;
end

details = struct();
details.mode = 'privileged-directed-task-routing';
details.maxMessagesPerReceiver = maxMessagesPerReceiver;
details.sourceWeightGrid = sourceWeightGrid;
details.minimumRelativeNodeGain = minimumRelativeNodeGain;
details.deliveryExpectationMode = deliveryExpectationMode;
details.expectedDeliveryWeighting = strcmp( ...
    deliveryExpectationMode, 'legacy-weight-shrinkage');
details.adjacencySchedule = adjacencySchedule;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedSourceWeights;
details.selectedEffectiveWeightsByReceiver = selectedEffectiveWeights;
details.selectedStepGainsByReceiver = selectedStepGains;
details.firstStepCandidateRisk = firstStepCandidateRisk;
details.firstStepCandidateConditionalRisk = ...
    firstStepCandidateConditionalRisk;
details.firstStepCandidateExpectedGain = ...
    firstStepCandidateExpectedGain;
details.firstStepCandidateSourceWeight = ...
    firstStepCandidateSourceWeight;
details.firstStepCandidateEffectiveWeight = ...
    firstStepCandidateEffectiveWeight;
details.firstStepConditionalRiskByWeight = ...
    firstStepConditionalRiskByWeight;
details.firstStepExpectedRiskByWeight = ...
    firstStepExpectedRiskByWeight;
details.firstStepExpectedGainByWeight = ...
    firstStepExpectedGainByWeight;
details.nodeRiskBefore = nodeRiskBefore;
details.nodeRiskAfter = nodeRiskAfter;
details.meanRiskBefore = mean(nodeRiskBefore);
details.meanRiskAfter = mean(nodeRiskAfter);
details.worstNodeRiskBefore = max(nodeRiskBefore);
details.worstNodeRiskAfter = max(nodeRiskAfter);
details.meanRiskGainFraction = ...
    (details.meanRiskBefore - details.meanRiskAfter) / ...
    max(details.meanRiskBefore, eps);
details.worstNodeRiskGainFraction = ...
    (details.worstNodeRiskBefore - details.worstNodeRiskAfter) / ...
    max(details.worstNodeRiskBefore, eps);
details.messageCount = nnz(adjacencySchedule);
details.attemptedBytes = attemptedBytes;
details.nodePayloadBytes = payloadBytes;
details.selectionSeconds = toc(timerId);
details.adjacency = any(adjacencySchedule, 3);
details.fusionWeightMatrix = eye(nodeCount);
if maxMessagesPerReceiver == 1
    for receiverIdx = 1:nodeCount
        if isempty(selectedSources{receiverIdx})
            continue;
        end
        senderIdx = selectedSources{receiverIdx}(1);
        sourceWeight = selectedSourceWeights{receiverIdx}(1);
        details.fusionWeightMatrix(receiverIdx, receiverIdx) = ...
            1 - sourceWeight;
        details.fusionWeightMatrix(receiverIdx, senderIdx) = ...
            sourceWeight;
    end
end
end

function mode = resolveDeliveryExpectationMode(options)
if isfield(options, 'deliveryExpectationMode') && ...
        ~isempty(options.deliveryExpectationMode)
    mode = lower(strrep(char( ...
        options.deliveryExpectationMode), '_', '-'));
elseif isfield(options, 'expectedDeliveryWeighting')
    if options.expectedDeliveryWeighting
        mode = 'legacy-weight-shrinkage';
    else
        mode = 'conditional-success';
    end
else
    mode = 'bernoulli-risk';
end
allowed = {'bernoulli-risk', 'conditional-success', ...
    'legacy-weight-shrinkage'};
if ~any(strcmp(mode, allowed))
    error('Unknown deliveryExpectationMode: %s', mode);
end
end

function [weights, effectiveWeight] = resolveCandidateWeights( ...
    nominalWeight, reliability, mode)
switch mode
    case 'legacy-weight-shrinkage'
        weights = [ ...
            1 - nominalWeight, reliability * nominalWeight];
        weights = weights / max(sum(weights), eps);
        effectiveWeight = weights(2);
    otherwise
        weights = [1 - nominalWeight, nominalWeight];
        effectiveWeight = nominalWeight;
end
end

function risk = expectedCandidateRisk( ...
    localRisk, conditionalRisk, reliability, mode)
if strcmp(mode, 'bernoulli-risk')
    risk = reliability * conditionalRisk + ...
        (1 - reliability) * localRisk;
else
    risk = conditionalRisk;
end
end

function bytes = estimateNodePayloadBytes(posteriors, model)
bytes = zeros(1, numel(posteriors));
for sensorIdx = 1:numel(posteriors)
    stats = estimateLmbPayloadSize( ...
        posteriors{sensorIdx}, model, 2);
    bytes(sensorIdx) = stats.estimatedBytes;
end
end

function probability = edgeDrop( ...
    config, senderIdx, receiverIdx, currentTime)
if isfield(config, 'pDropByEdge') && ~isempty(config.pDropByEdge)
    if ndims(config.pDropByEdge) >= 3
        timeIdx = min(currentTime, size(config.pDropByEdge, 3));
        probability = config.pDropByEdge( ...
            senderIdx, receiverIdx, timeIdx);
    else
        probability = config.pDropByEdge(senderIdx, receiverIdx);
    end
elseif isfield(config, 'pDropBySensor') && ...
        numel(config.pDropBySensor) >= senderIdx
    probability = config.pDropBySensor(senderIdx);
else
    probability = 0;
end
probability = min(max(probability, 0), 1);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
