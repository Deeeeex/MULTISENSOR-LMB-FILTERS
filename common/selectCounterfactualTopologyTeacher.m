function [adjacency, details] = ...
    selectCounterfactualTopologyTeacher(context, mode)
% SELECTCOUNTERFACTUALTOPOLOGYTEACHER Offline task-value topology teacher.
%
% mode:
%   current     - downstream truth risk immediately after one fusion round
%   predictive  - discounted truth risk after current fusion and open-loop
%                 LMB prediction through configured future horizons
%
% The supervised label is candidate advantage in task risk only. Topology
% switching is applied later as a deployment constraint/penalty, so a GNN
% cannot learn an action-cost artefact as if it were tracking information.

if nargin < 2 || isempty(mode)
    mode = 'predictive';
end
mode = lower(strrep(char(mode), '_', '-'));
if ~any(strcmp(mode, {'current', 'predictive'}))
    error('Unknown counterfactual teacher mode: %s', mode);
end
timerId = tic;
options = resolveOptions(context.triggerConfig, mode);
[candidates, candidateMetadata] = ...
    buildDynamicTopologyCandidatePool(context, options);
candidateCount = size(candidates, 3);
nodeCount = size(candidates, 1);

taskRisks = inf(1, candidateCount);
nodeTaskRisks = nan(candidateCount, nodeCount);
switchCosts = inf(1, candidateCount);
byteMismatch = inf(1, candidateCount);
structurallyValid = false(1, candidateCount);
selectionValid = false(1, candidateCount);
cache = initializeRiskCache(nodeCount);
nodePayloadBytes = estimateNodePayloadBytes( ...
    context.localPosteriorBySensor, context.model);
referenceBytes = topologyAttemptedBytes( ...
    context.baseAdjacency, nodePayloadBytes);
scenarioConfig = context.model.dynamicTopologyScenario.config;
byteTolerance = getField(scenarioConfig, ...
    'attemptedByteToleranceFraction', 0.02);
maxReplacements = getField(scenarioConfig, ...
    'maxEdgeReplacementsPerStep', inf);

for candidateIdx = 1:candidateCount
    candidate = candidates(:, :, candidateIdx);
    if any(candidate(:) & ~context.physicalAdjacency(:)) || ...
            countEdges(candidate) > context.edgeBudget || ...
            ~isConnected(candidate)
        continue;
    end
    structurallyValid(candidateIdx) = true;
    switchCosts(candidateIdx) = countRemovedEdges( ...
        context.previousAdjacency, candidate);
    candidateBytes = topologyAttemptedBytes( ...
        candidate, nodePayloadBytes);
    byteMismatch(candidateIdx) = abs( ...
        candidateBytes - referenceBytes) / max(referenceBytes, 1);
    switchAllowed = ~any(context.previousAdjacency(:)) || ...
        switchCosts(candidateIdx) <= maxReplacements;
    selectionValid(candidateIdx) = switchAllowed && ...
        byteMismatch(candidateIdx) <= byteTolerance + 1e-12;
end

baselineIdx = findMatchingCandidate( ...
    candidates, context.baseAdjacency);
if isempty(baselineIdx)
    baselineIdx = find(structurallyValid, 1);
end
if options.evaluateAllCandidates
    evaluationIndices = find(structurallyValid);
else
    evaluationIndices = find(selectionValid);
    if ~isempty(baselineIdx) && structurallyValid(baselineIdx)
        evaluationIndices = unique([ ...
            reshape(evaluationIndices, 1, []), baselineIdx]);
    end
end
for candidateIdx = reshape(evaluationIndices, 1, [])
    [taskRisks(candidateIdx), cache, ...
        nodeTaskRisks(candidateIdx, :)] = evaluateCandidateRisk( ...
        candidates(:, :, candidateIdx), context, options, cache);
end
baselineRisk = NaN;
if ~isempty(baselineIdx) && isfinite(taskRisks(baselineIdx))
    baselineRisk = taskRisks(baselineIdx);
end
advantages = baselineRisk - taskRisks;
selectionObjectives = taskRisks + ...
    options.switchPenaltyWeight * switchCosts;
selectionObjectives(~selectionValid) = inf;

if ~any(selectionValid)
    adjacency = context.baseAdjacency & context.physicalAdjacency;
    details = buildDetails( ...
        mode, inf, NaN, 0, toc(timerId), NaN, ...
        baselineRisk, baselineIdx, NaN, candidateCount, candidateMetadata, ...
        taskRisks, nodeTaskRisks, advantages, switchCosts, byteMismatch, ...
        structurallyValid, selectionValid, options);
    return;
end

[objective, selectedIdx] = min(selectionObjectives);
adjacency = candidates(:, :, selectedIdx);
details = buildDetails( ...
    mode, objective, selectedIdx, sum(selectionValid), ...
    toc(timerId), taskRisks(selectedIdx), baselineRisk, baselineIdx, ...
    advantages(selectedIdx), candidateCount, candidateMetadata, ...
    taskRisks, nodeTaskRisks, advantages, switchCosts, byteMismatch, ...
    structurallyValid, selectionValid, options);
end

function options = resolveOptions(triggerConfig, mode)
options = struct();
options.maxCandidates = getField( ...
    triggerConfig, 'topologyTeacherMaxCandidates', 96);
options.diversityBansPerScore = getField( ...
    triggerConfig, 'topologyTeacherDiversityBansPerScore', 8);
options.expectedDeliveryWeighting = getField( ...
    triggerConfig, 'topologyTeacherExpectedDeliveryWeighting', true);
options.switchPenaltyWeight = max(getField( ...
    triggerConfig, 'topologyTeacherSwitchPenaltyWeight', 0), 0);
options.evaluateAllCandidates = getField( ...
    triggerConfig, 'topologyTeacherEvaluateAllCandidates', false);
options.discountFactor = getField( ...
    triggerConfig, 'topologyTeacherDiscountFactor', 0.9);
if strcmp(mode, 'current')
    options.horizonSteps = 0;
else
    options.horizonSteps = getField( ...
        triggerConfig, 'topologyTeacherHorizonSteps', [0, 3, 6]);
end
riskOverrides = getField( ...
    triggerConfig, 'topologyTeacherRiskOptions', struct());
riskFields = fieldnames(riskOverrides);
for fieldIdx = 1:numel(riskFields)
    options.(riskFields{fieldIdx}) = ...
        riskOverrides.(riskFields{fieldIdx});
end
end

function [risk, cache, nodeRisk] = evaluateCandidateRisk( ...
    adjacency, context, options, cache)
weights = metropolisWeights(adjacency);
nodeRisk = zeros(1, size(adjacency, 1));
for receiverIdx = 1:size(adjacency, 1)
    neighbors = find(adjacency(receiverIdx, :));
    sources = [receiverIdx, neighbors];
    localWeights = weights(receiverIdx, sources);
    if options.expectedDeliveryWeighting
        for sourceCursor = 2:numel(sources)
            senderIdx = sources(sourceCursor);
            reliability = 1 - edgeDrop( ...
                context.commConfig, senderIdx, receiverIdx, ...
                context.currentTime);
            localWeights(sourceCursor) = ...
                localWeights(sourceCursor) * reliability;
        end
    end
    positive = localWeights > eps;
    sources = sources(positive);
    localWeights = localWeights(positive);
    localWeights = localWeights / max(sum(localWeights), eps);
    cacheKey = [sprintf('%d_', sources), '|', ...
        sprintf('%.17g_', localWeights)];
    cachedIdx = find(strcmp( ...
        cache(receiverIdx).keys, cacheKey), 1);
    if ~isempty(cachedIdx)
        nodeRisk(receiverIdx) = ...
            cache(receiverIdx).risks(cachedIdx);
        continue;
    end
    fusionInputs = context.localPosteriorBySensor(sources);
    fusionDetails = struct( ...
        'eventType', [0, 2 * ones(1, numel(sources) - 1)]);
    fused = fuseLmbPosteriorsByLabel( ...
        fusionInputs, localWeights, context.model, localWeights, ...
        fusionDetails, context.triggerConfig);
    nodeRisk(receiverIdx) = evaluateLmbTopologyTaskRisk( ...
        fused, context.model, context.currentTime, options);
    cache(receiverIdx).keys{end+1} = cacheKey;
    cache(receiverIdx).risks(end+1) = nodeRisk(receiverIdx);
end
risk = aggregateNodeRisk(nodeRisk, options);
end

function risk = aggregateNodeRisk(nodeRisk, options)
finiteRisk = nodeRisk(isfinite(nodeRisk));
if isempty(finiteRisk)
    risk = inf;
    return;
end
mode = lower(strrep(getField( ...
    options, 'sensorAggregationMode', 'mean'), '_', '-'));
cvarFraction = min(max(getField( ...
    options, 'sensorCvarFraction', 0.25), eps), 1);
cvarWeight = min(max(getField( ...
    options, 'sensorCvarWeight', 0.5), 0), 1);
meanRisk = mean(finiteRisk);
sortedRisk = sort(finiteRisk, 'descend');
tailCount = max(1, ceil(cvarFraction * numel(sortedRisk)));
cvarRisk = mean(sortedRisk(1:tailCount));
switch mode
    case 'mean'
        risk = meanRisk;
    case {'cvar', 'tail'}
        risk = cvarRisk;
    case {'mean-cvar', 'risk-sensitive'}
        risk = (1 - cvarWeight) * meanRisk + ...
            cvarWeight * cvarRisk;
    otherwise
        error('Unknown sensorAggregationMode: %s', mode);
end
end

function cache = initializeRiskCache(nodeCount)
cache = repmat(struct( ...
    'keys', {{}}, ...
    'risks', []), 1, nodeCount);
end

function details = buildDetails( ...
    mode, objective, candidateIdx, validCount, selectionSeconds, ...
    taskRisk, baselineTaskRisk, baselineCandidateIndex, ...
    taskAdvantage, candidateCount, ...
    candidateMetadata, taskRisks, nodeTaskRisks, advantages, switchCosts, ...
    byteMismatch, structurallyValid, selectionValid, options)
finiteRisks = taskRisks(isfinite(taskRisks));
if isempty(finiteRisks)
    riskSpread = NaN;
else
    riskSpread = max(finiteRisks) - min(finiteRisks);
end
details = struct();
details.objective = objective;
details.candidateIndex = candidateIdx;
details.mode = mode;
details.validCandidateCount = validCount;
details.selectionSeconds = selectionSeconds;
details.taskRisk = taskRisk;
details.baselineTaskRisk = baselineTaskRisk;
details.taskAdvantage = taskAdvantage;
details.baselineCandidateIndex = baselineCandidateIndex;
details.candidateCount = candidateCount;
details.candidateSource = candidateMetadata.source;
details.candidateTaskRisks = taskRisks;
details.candidateNodeTaskRisks = nodeTaskRisks;
details.candidateAdvantages = advantages;
details.candidateSwitchCosts = switchCosts;
details.candidateByteMismatchFraction = byteMismatch;
details.structurallyValidCandidates = structurallyValid;
details.selectionValidCandidates = selectionValid;
details.taskRiskSpread = riskSpread;
details.horizonSteps = options.horizonSteps;
details.sensorAggregationMode = getField( ...
    options, 'sensorAggregationMode', 'mean');
details.sensorCvarFraction = getField( ...
    options, 'sensorCvarFraction', 0.25);
details.sensorCvarWeight = getField( ...
    options, 'sensorCvarWeight', 0.5);
details.switchPenaltyWeight = options.switchPenaltyWeight;
details.labelIncludesSwitchPenalty = false;
end

function weights = metropolisWeights(adjacency)
nodeCount = size(adjacency, 1);
degreesIncludingSelf = sum(adjacency, 2)' + 1;
weights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    neighbors = find(adjacency(receiverIdx, :));
    for senderIdx = neighbors
        weights(receiverIdx, senderIdx) = 1 / (1 + max( ...
            degreesIncludingSelf(receiverIdx), ...
            degreesIncludingSelf(senderIdx)));
    end
    weights(receiverIdx, receiverIdx) = ...
        1 - sum(weights(receiverIdx, :));
    weights(receiverIdx, :) = weights(receiverIdx, :) / ...
        max(sum(weights(receiverIdx, :)), eps);
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

function bytes = topologyAttemptedBytes(adjacency, nodePayloadBytes)
bytes = sum(sum(adjacency, 2)' .* nodePayloadBytes);
end

function index = findMatchingCandidate(candidates, reference)
index = [];
for candidateIdx = 1:size(candidates, 3)
    if isequal(candidates(:, :, candidateIdx), logical(reference))
        index = candidateIdx;
        return;
    end
end
end

function count = countRemovedEdges(previous, current)
if isempty(previous) || ~any(previous(:))
    count = 0;
else
    count = nnz(triu(previous & ~current, 1));
end
end

function count = countEdges(adjacency)
count = nnz(triu(adjacency, 1));
end

function connected = isConnected(adjacency)
nodeCount = size(adjacency, 1);
if nodeCount <= 1
    connected = true;
    return;
end
visited = false(1, nodeCount);
queue = 1;
visited(1) = true;
while ~isempty(queue)
    current = queue(1);
    queue(1) = [];
    neighbors = find(adjacency(current, :) & ~visited);
    visited(neighbors) = true;
    queue = [queue, neighbors]; %#ok<AGROW>
end
connected = all(visited);
end

function probability = edgeDrop(config, senderIdx, receiverIdx, currentTime)
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
