function result = measureFormationBraidTemporalTaskCoupling( ...
        config, graphData, difficulty)
% MEASUREFORMATIONBRAIDTEMPORALTASKCOUPLING Audit actual task/cut timing.
%
% Registered source/destination labels are not enough: moving formations can
% make a nominal F2->F3 route visible mainly from F1.  This audit therefore
% uses the realised nearest-visible owner of every target.  A failed initial
% tree cut is task-coupled only when at least one registered cohort is owned
% on its source side before the failure and actually transfers to the other
% side while that cut is unavailable.

requiredDifficultyFields = { ...
    'ownerByTarget', 'activeMask', ...
    'visibleFormationMaskByTargetTime'};
for fieldIdx = 1:numel(requiredDifficultyFields)
    if ~isfield(difficulty, requiredDifficultyFields{fieldIdx})
        error('TemporalTaskCoupling:MissingDifficultyField', ...
            'Missing difficulty field %s.', ...
            requiredDifficultyFields{fieldIdx});
    end
end
if ~isfield(config, 'targetRouteSourceFormationIds') || ...
        ~isfield(config, 'targetRouteDestinationFormationIds')
    error('TemporalTaskCoupling:MissingRouteMetadata', ...
        'Target source/destination metadata is required.');
end

formationCount = config.formationCount;
timeCount = config.simulationLength;
groupIds = reshape(config.sensorGroupIds, 1, []);
initialTree = collapseFormationAdjacency( ...
    logical(graphData.staticAdjacency), groupIds, formationCount);
if nnz(triu(initialTree, 1)) ~= formationCount - 1 || ...
        ~isConnected(initialTree)
    error('TemporalTaskCoupling:InitialGraphNotTree', ...
        'The registered initial formation graph must be a tree.');
end

edgePairs = findUndirectedPairs(initialTree);
lookback = getField(config, 'temporalCouplingLookbackSteps', 24);
lookahead = getField(config, 'temporalCouplingLookaheadSteps', 36);
minimumSupport = getField(config, ...
    'temporalCouplingMinimumSupportFraction', 0.50);
minimumAlignedTargets = getField(config, ...
    'temporalCouplingMinimumAlignedTargetFraction', 0.50);
records = repmat(emptyCutRecord(), 1, size(edgePairs, 1));
for edgeIdx = 1:size(edgePairs, 1)
    records(edgeIdx) = inspectCut( ...
        config, graphData, difficulty, initialTree, ...
        edgePairs(edgeIdx, :), lookback, lookahead, ...
        minimumSupport, minimumAlignedTargets);
end

failed = [records.failed];
result = struct();
result.contractVersion = ...
    'formation-braid-temporal-task-coupling-v247-result-v1';
result.initialTreePairs = edgePairs;
result.cutRecords = records;
result.failedCutCount = sum(failed);
result.temporallyCoupledFailedCutCount = sum( ...
    failed & [records.temporallyCoupled]);
result.allFailedCutsTemporallyCoupled = ...
    result.failedCutCount > 0 && all([records(failed).temporallyCoupled]);
result.minimumAlignedCohortsPerFailedCut = minOrZero( ...
    [records(failed).alignedCohortCount]);
result.lookbackSteps = lookback;
result.lookaheadSteps = lookahead;
result.minimumSupportFraction = minimumSupport;
result.minimumAlignedTargetFraction = minimumAlignedTargets;
end

function record = inspectCut(config, graphData, difficulty, ...
        initialTree, edgePair, lookback, lookahead, ...
        minimumSupport, minimumAlignedTargets)
timeCount = config.simulationLength;
formationCount = config.formationCount;
sensorGroups = reshape(config.sensorGroupIds, 1, []);
edgeAvailable = false(1, timeCount);
for timeIdx = 1:timeCount
    current = collapseFormationAdjacency( ...
        logical(graphData.physicalAdjacency(:, :, timeIdx)), ...
        sensorGroups, formationCount);
    edgeAvailable(timeIdx) = current(edgePair(1), edgePair(2));
end
failureMask = ~edgeAvailable;
firstFailure = firstTrue(failureMask);
lastFailure = lastTrue(failureMask);

cutTree = initialTree;
cutTree(edgePair(1), edgePair(2)) = false;
cutTree(edgePair(2), edgePair(1)) = false;
leftComponent = reachableFrom(cutTree, edgePair(1));
sourceIds = reshape(config.targetRouteSourceFormationIds, 1, []);
destinationIds = reshape( ...
    config.targetRouteDestinationFormationIds, 1, []);
crossingGroups = find(xor( ...
    leftComponent(sourceIds), leftComponent(destinationIds)));

cohorts = repmat(emptyCohortRecord(), 1, numel(crossingGroups));
for cohortIdx = 1:numel(crossingGroups)
    groupIdx = crossingGroups(cohortIdx);
    cohorts(cohortIdx) = inspectCohort( ...
        config, difficulty, groupIdx, leftComponent, ...
        failureMask, firstFailure, lookback, lookahead, ...
        minimumSupport, minimumAlignedTargets);
end

record = emptyCutRecord();
record.edgePair = edgePair;
record.failed = any(failureMask);
record.firstFailureTime = firstFailure;
record.lastFailureTime = lastFailure;
record.failureFraction = mean(failureMask);
record.registeredCrossingGroups = crossingGroups;
record.cohortRecords = cohorts;
if isempty(cohorts)
    record.alignedCohortCount = 0;
else
    record.alignedCohortCount = sum([cohorts.temporallyAligned]);
end
record.temporallyCoupled = record.failed && ...
    record.alignedCohortCount > 0;
end

function record = inspectCohort(config, difficulty, groupIdx, ...
        leftComponent, failureMask, firstFailure, lookback, lookahead, ...
        minimumSupport, minimumAlignedTargets)
sourceFormation = config.targetRouteSourceFormationIds(groupIdx);
destinationFormation = ...
    config.targetRouteDestinationFormationIds(groupIdx);
sourceSide = leftComponent;
if ~leftComponent(sourceFormation)
    sourceSide = ~leftComponent;
end
destinationSide = ~sourceSide;
targetIds = find(config.targetGroupIds == groupIdx);
timeCount = config.simulationLength;
if isnan(firstFailure)
    preWindow = zeros(1, 0);
    couplingWindow = zeros(1, 0);
else
    preWindow = max(1, firstFailure - lookback):(firstFailure - 1);
    couplingWindow = firstFailure:min(timeCount, firstFailure + lookahead);
end

owners = difficulty.ownerByTarget(targetIds, :);
active = difficulty.activeMask(targetIds, :);
preSourceSupport = ownerSupportFraction( ...
    owners(:, preWindow), active(:, preWindow), sourceSide);
duringDestinationSupport = ownerSupportFraction( ...
    owners(:, couplingWindow), ...
    active(:, couplingWindow) & failureMask(couplingWindow), ...
    destinationSide);

alignedTargetMask = false(1, numel(targetIds));
earliestTransition = NaN;
for localTargetIdx = 1:numel(targetIds)
    [alignedTargetMask(localTargetIdx), transitionTime] = ...
        hasAlignedSourceToDestinationTransition( ...
            owners(localTargetIdx, :), sourceSide, destinationSide, ...
            failureMask, couplingWindow);
    if alignedTargetMask(localTargetIdx)
        earliestTransition = nanMinimum( ...
            earliestTransition, transitionTime);
    end
end
alignedTargetFraction = mean(alignedTargetMask);

visible = difficulty.visibleFormationMaskByTargetTime( ...
    targetIds, :, couplingWindow);
sourceVisible = squeeze(any(visible(:, sourceSide, :), 2));
destinationVisible = squeeze(any(visible(:, destinationSide, :), 2));
if isempty(couplingWindow)
    crossCutOverlapFraction = NaN;
else
    failureSamples = repmat( ...
        failureMask(couplingWindow), numel(targetIds), 1);
    overlap = sourceVisible & destinationVisible & failureSamples;
    denominator = sum(failureSamples(:));
    crossCutOverlapFraction = sum(overlap(:)) / max(denominator, 1);
end

record = emptyCohortRecord();
record.groupId = groupIdx;
record.sourceFormation = sourceFormation;
record.destinationFormation = destinationFormation;
record.targetCount = numel(targetIds);
record.preFailureSourceSupportFraction = preSourceSupport;
record.duringFailureDestinationSupportFraction = ...
    duringDestinationSupport;
record.alignedTargetFraction = alignedTargetFraction;
record.earliestAlignedTransitionTime = earliestTransition;
record.crossCutVisibilityOverlapFraction = crossCutOverlapFraction;
record.temporallyAligned = ...
    preSourceSupport >= minimumSupport && ...
    duringDestinationSupport >= minimumSupport && ...
    alignedTargetFraction >= minimumAlignedTargets;
end

function fraction = ownerSupportFraction(owners, active, sideMask)
if isempty(owners)
    fraction = NaN;
    return;
end
supported = false(size(owners));
for formationIdx = find(sideMask)
    supported = supported | owners == formationIdx;
end
% Active but completely unobserved target samples count as unsupported;
% otherwise a blackout could artificially increase the apparent ownership
% support by disappearing from the denominator.
fraction = sum(supported(active)) / max(sum(active(:)), 1);
end

function [aligned, transitionTime] = ...
        hasAlignedSourceToDestinationTransition(owners, ...
            sourceSide, destinationSide, failureMask, couplingWindow)
sideTrace = zeros(size(owners));
for formationIdx = find(sourceSide)
    sideTrace(owners == formationIdx) = 1;
end
for formationIdx = find(destinationSide)
    sideTrace(owners == formationIdx) = 2;
end
validTimes = find(sideTrace > 0);
aligned = false;
transitionTime = NaN;
if isempty(validTimes) || isempty(couplingWindow)
    return;
end
for cursor = 2:numel(validTimes)
    previousTime = validTimes(cursor - 1);
    currentTime = validTimes(cursor);
    if sideTrace(previousTime) == 1 && sideTrace(currentTime) == 2 && ...
            any(currentTime == couplingWindow) && ...
            any(failureMask(previousTime:currentTime))
        aligned = true;
        transitionTime = currentTime;
        return;
    end
end
end

function adjacency = collapseFormationAdjacency( ...
        sensorAdjacency, groupIds, formationCount)
adjacency = false(formationCount);
for leftIdx = 1:formationCount-1
    for rightIdx = leftIdx+1:formationCount
        block = sensorAdjacency(groupIds == leftIdx, ...
            groupIds == rightIdx);
        adjacency(leftIdx, rightIdx) = any(block(:));
        adjacency(rightIdx, leftIdx) = adjacency(leftIdx, rightIdx);
    end
end
end

function pairs = findUndirectedPairs(adjacency)
[left, right] = find(triu(adjacency, 1));
pairs = sortrows([left, right], [1, 2]);
end

function visited = reachableFrom(adjacency, source)
visited = false(1, size(adjacency, 1));
frontier = source;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
end

function connected = isConnected(adjacency)
connected = all(reachableFrom(adjacency, 1));
end

function value = firstTrue(mask)
value = find(mask, 1, 'first');
if isempty(value)
    value = NaN;
end
end

function value = lastTrue(mask)
value = find(mask, 1, 'last');
if isempty(value)
    value = NaN;
end
end

function value = nanMinimum(left, right)
if isnan(left)
    value = right;
else
    value = min(left, right);
end
end

function value = minOrZero(values)
if isempty(values)
    value = 0;
else
    value = min(values);
end
end

function record = emptyCutRecord()
record = struct( ...
    'edgePair', zeros(1, 2), ...
    'failed', false, ...
    'firstFailureTime', NaN, ...
    'lastFailureTime', NaN, ...
    'failureFraction', NaN, ...
    'registeredCrossingGroups', zeros(1, 0), ...
    'cohortRecords', repmat(emptyCohortRecord(), 1, 0), ...
    'alignedCohortCount', 0, ...
    'temporallyCoupled', false);
end

function record = emptyCohortRecord()
record = struct( ...
    'groupId', NaN, ...
    'sourceFormation', NaN, ...
    'destinationFormation', NaN, ...
    'targetCount', 0, ...
    'preFailureSourceSupportFraction', NaN, ...
    'duringFailureDestinationSupportFraction', NaN, ...
    'alignedTargetFraction', NaN, ...
    'earliestAlignedTransitionTime', NaN, ...
    'crossCutVisibilityOverlapFraction', NaN, ...
    'temporallyAligned', false);
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
