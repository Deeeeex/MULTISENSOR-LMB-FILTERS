function [adjacency, details] = ...
    selectClosedLoopCounterfactualTopologyTeacher( ...
        context, rolloutData, options)
% SELECTCLOSEDLOOPCOUNTERFACTUALTOPOLOGYTEACHER Future-measurement teacher.
%
% Each feasible candidate is used only for the current communication
% action. Subsequent prediction, local measurement updates, and fusion use
% identical future data and a fixed continuation graph. Candidate labels
% are therefore paired H-step tracking-risk advantages. Switch cost remains
% a deployment penalty and is never included in the supervised task label.

if nargin < 2 || ~isstruct(rolloutData) || ...
        ~isfield(rolloutData, 'measurements')
    error('rolloutData.measurements is required.');
end
if nargin < 3 || isempty(options)
    options = struct();
end
timerId = tic;
options = resolveOptions(context.triggerConfig, options);
[candidates, candidateMetadata] = ...
    buildDynamicTopologyCandidatePool(context, options);
candidateCount = size(candidates, 3);

taskRisks = inf(1, candidateCount);
switchCosts = inf(1, candidateCount);
byteMismatch = inf(1, candidateCount);
structurallyValid = false(1, candidateCount);
selectionValid = false(1, candidateCount);
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

continuationAdjacency = getField(rolloutData, ...
    'continuationAdjacency', context.baseAdjacency);
rolloutOptions = struct( ...
    'horizonSteps', options.horizonSteps, ...
    'discountFactor', options.discountFactor, ...
    'expectedDeliveryWeighting', ...
        options.expectedDeliveryWeighting, ...
    'candidatePersistenceSteps', ...
        options.candidatePersistenceSteps, ...
    'riskOptions', options.riskOptions, ...
    'sampleMask', getField(rolloutData, 'sampleMask', []));
baselineIdx = findMatchingCandidate( ...
    candidates, context.baseAdjacency);
if isempty(baselineIdx)
    baselineIdx = find(structurallyValid, 1);
end
eligible = find(selectionValid);
if ~isempty(baselineIdx) && structurallyValid(baselineIdx)
    eligible = unique([reshape(eligible, 1, []), baselineIdx]);
end
candidateSeconds = nan(1, candidateCount);
for candidateIdx = reshape(eligible, 1, [])
    candidateTimer = tic;
    taskRisks(candidateIdx) = ...
        evaluateClosedLoopTopologyTaskRisk( ...
            context.localPosteriorBySensor, ...
            candidates(:, :, candidateIdx), context.model, ...
            rolloutData.measurements, context.commConfig, ...
            context.currentTime, continuationAdjacency, ...
            context.triggerConfig, rolloutOptions);
    candidateSeconds(candidateIdx) = toc(candidateTimer);
end

if isempty(baselineIdx) || ~isfinite(taskRisks(baselineIdx))
    baselineIdx = find(isfinite(taskRisks), 1);
end
baselineRisk = NaN;
if ~isempty(baselineIdx)
    baselineRisk = taskRisks(baselineIdx);
end
advantages = baselineRisk - taskRisks;
selectionObjectives = taskRisks + ...
    options.switchPenaltyWeight * switchCosts;
selectionObjectives(~selectionValid) = inf;

if ~any(isfinite(selectionObjectives))
    adjacency = context.baseAdjacency & context.physicalAdjacency;
    details = buildDetails( ...
        inf, NaN, 0, toc(timerId), NaN, baselineRisk, ...
        baselineIdx, NaN, candidateCount, candidateMetadata, ...
        taskRisks, advantages, switchCosts, byteMismatch, ...
        structurallyValid, selectionValid, candidateSeconds, options);
    return;
end

[objective, selectedIdx] = min(selectionObjectives);
adjacency = candidates(:, :, selectedIdx);
details = buildDetails( ...
    objective, selectedIdx, sum(selectionValid), toc(timerId), ...
    taskRisks(selectedIdx), baselineRisk, baselineIdx, ...
    advantages(selectedIdx), candidateCount, candidateMetadata, ...
    taskRisks, advantages, switchCosts, byteMismatch, ...
    structurallyValid, selectionValid, candidateSeconds, options);
end

function options = resolveOptions(triggerConfig, overrides)
options = struct();
options.maxCandidates = getField( ...
    triggerConfig, 'topologyTeacherMaxCandidates', 96);
options.diversityBansPerScore = getField( ...
    triggerConfig, 'topologyTeacherDiversityBansPerScore', 8);
options.expectedDeliveryWeighting = getField( ...
    triggerConfig, 'topologyTeacherExpectedDeliveryWeighting', true);
options.horizonSteps = max(1, round(getField( ...
    triggerConfig, 'topologyTeacherClosedLoopHorizonSteps', 3)));
options.candidatePersistenceSteps = max(1, round(getField( ...
    triggerConfig, 'topologyTeacherCandidatePersistenceSteps', 1)));
options.discountFactor = getField( ...
    triggerConfig, 'topologyTeacherDiscountFactor', 0.9);
options.switchPenaltyWeight = max(getField( ...
    triggerConfig, 'topologyTeacherSwitchPenaltyWeight', 0), 0);
options.riskOptions = getField( ...
    triggerConfig, 'topologyTeacherRiskOptions', struct());
overrideFields = fieldnames(overrides);
for fieldIdx = 1:numel(overrideFields)
    options.(overrideFields{fieldIdx}) = ...
        overrides.(overrideFields{fieldIdx});
end
end

function details = buildDetails( ...
    objective, candidateIdx, validCount, selectionSeconds, taskRisk, ...
    baselineTaskRisk, baselineCandidateIndex, taskAdvantage, ...
    candidateCount, candidateMetadata, taskRisks, advantages, ...
    switchCosts, byteMismatch, structurallyValid, selectionValid, ...
    candidateSeconds, options)
finiteRisks = taskRisks(isfinite(taskRisks));
if isempty(finiteRisks)
    riskSpread = NaN;
else
    riskSpread = max(finiteRisks) - min(finiteRisks);
end
details = struct();
details.objective = objective;
details.candidateIndex = candidateIdx;
details.mode = 'closed-loop';
details.validCandidateCount = validCount;
details.selectionSeconds = selectionSeconds;
details.taskRisk = taskRisk;
details.baselineTaskRisk = baselineTaskRisk;
details.baselineCandidateIndex = baselineCandidateIndex;
details.taskAdvantage = taskAdvantage;
details.candidateCount = candidateCount;
details.candidateSource = candidateMetadata.source;
details.candidateTaskRisks = taskRisks;
details.candidateAdvantages = advantages;
details.candidateSwitchCosts = switchCosts;
details.candidateByteMismatchFraction = byteMismatch;
details.structurallyValidCandidates = structurallyValid;
details.selectionValidCandidates = selectionValid;
details.candidateRolloutSeconds = candidateSeconds;
details.taskRiskSpread = riskSpread;
details.horizonSteps = options.horizonSteps;
details.candidatePersistenceSteps = ...
    options.candidatePersistenceSteps;
details.switchPenaltyWeight = options.switchPenaltyWeight;
details.labelIncludesSwitchPenalty = false;
details.usesFutureMeasurements = true;
if options.candidatePersistenceSteps > 1
    details.continuationPolicy = ...
        'candidate-then-fixed-registered-graph';
else
    details.continuationPolicy = 'fixed-registered-graph';
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

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
