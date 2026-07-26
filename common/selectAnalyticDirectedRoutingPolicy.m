function [adjacency, details] = ...
    selectAnalyticDirectedRoutingPolicy(context, mode, options)
% SELECTANALYTICDIRECTEDROUTINGPOLICY Truth-free directed control baselines.
%
% Each receiver selects at most one physical sender. These policies share
% the directed action space and fixed KLA weight with the learned policy,
% but rank links using one observable scalar. They are intended as strong,
% interpretable controls for separating the value of directed routing from
% the value of learned edge scoring.

if nargin < 2 || isempty(mode)
    mode = 'reliability';
end
if nargin < 3 || isempty(options)
    options = getField( ...
        context.triggerConfig, ...
        'topologyAnalyticDirectedRoutingOptions', struct());
end
timerId = tic;
mode = lower(strrep(char(mode), '_', '-'));
sourceWeight = getField(options, 'sourceWeight', 0.50);
if ~isscalar(sourceWeight) || ~isfinite(sourceWeight) || ...
        sourceWeight <= 0 || sourceWeight >= 1
    error('sourceWeight must be a finite scalar strictly between 0 and 1.');
end
minimumScore = getField(options, 'minimumScore', defaultMinimumScore(mode));

nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
if ~isequal(size(physical), [nodeCount, nodeCount])
    error('physicalAdjacency must be S-by-S.');
end
physical(1:nodeCount+1:end) = false;
[scoreMatrix, scoreDetails] = ...
    computeScores(context, physical, mode, options);

selectedSources = nan(1, nodeCount);
selectedScores = -inf(1, nodeCount);
for receiverIdx = 1:nodeCount
    senders = find(physical(receiverIdx, :));
    if isempty(senders)
        continue;
    end
    [bestScore, bestCursor] = max( ...
        scoreMatrix(receiverIdx, senders));
    if isfinite(bestScore) && bestScore >= minimumScore
        selectedSources(receiverIdx) = senders(bestCursor);
        selectedScores(receiverIdx) = bestScore;
    end
end

messageBudget = max(0, floor(getField( ...
    context, 'directedMessageBudget', inf)));
selectedReceivers = find(isfinite(selectedSources));
if numel(selectedReceivers) > messageBudget
    [~, order] = sort(selectedScores(selectedReceivers), 'descend');
    dropped = selectedReceivers(order((messageBudget + 1):end));
    selectedSources(dropped) = NaN;
    selectedScores(dropped) = -inf;
end

adjacency = false(nodeCount);
fusionWeights = eye(nodeCount);
selectedWeights = zeros(1, nodeCount);
selectedReceivers = reshape(find(isfinite(selectedSources)), 1, []);
for receiverIdx = selectedReceivers
    senderIdx = selectedSources(receiverIdx);
    adjacency(receiverIdx, senderIdx) = true;
    fusionWeights(receiverIdx, receiverIdx) = 1 - sourceWeight;
    fusionWeights(receiverIdx, senderIdx) = sourceWeight;
    selectedWeights(receiverIdx) = sourceWeight;
end

details = struct();
details.mode = ['analytic-directed-', mode];
details.objective = -sum(selectedScores(isfinite(selectedScores)));
details.candidateIndex = NaN;
details.selectionSeconds = toc(timerId);
details.taskRisk = NaN;
details.baselineTaskRisk = NaN;
details.taskAdvantage = NaN;
details.taskRiskSpread = NaN;
details.validCandidateCount = nnz(adjacency);
details.directed = true;
details.fusionWeightMatrix = fusionWeights;
details.scoreMatrix = scoreMatrix;
details.scoreDetails = scoreDetails;
details.selectedSourcesByReceiver = selectedSources;
details.selectedSourceWeightsByReceiver = selectedWeights;
details.selectedScores = selectedScores;
details.messageCount = nnz(adjacency);
details.sourceWeight = sourceWeight;
end

function [scores, details] = ...
    computeScores(context, physical, mode, options)
nodeCount = size(physical, 1);
scores = -inf(nodeCount);
details = struct();
switch mode
    case 'reliability'
        for receiverIdx = 1:nodeCount
            for senderIdx = find(physical(receiverIdx, :))
                scores(receiverIdx, senderIdx) = 1 - edgeDrop( ...
                    context.commConfig, senderIdx, receiverIdx, ...
                    context.currentTime);
            end
        end
    case 'nearest'
        positions = getField(context, ...
            'positions', zeros(2, nodeCount));
        for receiverIdx = 1:nodeCount
            for senderIdx = find(physical(receiverIdx, :))
                scores(receiverIdx, senderIdx) = -norm( ...
                    positions(:, receiverIdx) - ...
                    positions(:, senderIdx));
            end
        end
    case 'quality-advantage'
        [features, names] = ...
            computeDirectedRoutingFeatures(context);
        featureIdx = find(strcmp( ...
            names, 'node_quality_advantage'), 1);
        if isempty(featureIdx)
            error('Missing node_quality_advantage feature.');
        end
        scores = reshape(features(:, :, featureIdx), ...
            nodeCount, nodeCount);
    case 'moment-compatibility'
        [features, names] = ...
            computeDirectedRoutingFeatures(context);
        reliabilityIdx = requireFeature( ...
            names, 'link_reliability');
        distanceIdx = requireFeature( ...
            names, 'normalized_sensor_distance');
        discrepancyIdx = requireFeature( ...
            names, 'normalized_state_discrepancy');
        precisionGapIdx = requireFeature( ...
            names, 'negative_precision_gap');
        reliability = reshape(features(:, :, reliabilityIdx), ...
            nodeCount, nodeCount);
        distance = reshape(features(:, :, distanceIdx), ...
            nodeCount, nodeCount);
        discrepancy = reshape(features(:, :, discrepancyIdx), ...
            nodeCount, nodeCount);
        precisionGap = reshape(features(:, :, precisionGapIdx), ...
            nodeCount, nodeCount);
        distancePenalty = max(getField( ...
            options, 'distancePenalty', 0.25), 0);
        discrepancyPenalty = max(getField( ...
            options, 'discrepancyPenalty', 0.25), 0);
        precisionPenalty = max(getField( ...
            options, 'precisionPenalty', 1.00), 0);
        scores = reliability - ...
            distancePenalty * distance - ...
            discrepancyPenalty * discrepancy - ...
            precisionPenalty * precisionGap;
        details.distancePenalty = distancePenalty;
        details.discrepancyPenalty = discrepancyPenalty;
        details.precisionPenalty = precisionPenalty;
    case 'kla-compatibility'
        [scores, details] = ...
            computeLmbKlaCompatibilityMatrix(context, options);
    otherwise
        error('Unknown analytic directed-routing mode: %s', mode);
end
scores(~physical) = -inf;
end

function idx = requireFeature(names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('Missing directed-routing feature: %s', name);
end
end

function minimum = defaultMinimumScore(mode)
if strcmp(mode, 'quality-advantage')
    minimum = 0;
else
    minimum = -inf;
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
