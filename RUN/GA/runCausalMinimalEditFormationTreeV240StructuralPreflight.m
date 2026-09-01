function [reportPath, result] = ...
        runCausalMinimalEditFormationTreeV240StructuralPreflight(options)
% RUNCAUSALMINIMALEDITFORMATIONTREEV240STRUCTURALPREFLIGHT Route replay.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCausalMinimalEditFormationTreeV240Protocol();
presetNames = getField(options, 'presetNames', protocol.allowedPresets);
seed = getField(options, 'seed', 41);
writeReport = getField(options, 'writeReport', true);
if ischar(presetNames)
    presetNames = {presetNames};
end
if ~iscell(presetNames) || ...
        any(~ismember(presetNames, protocol.allowedPresets)) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('CausalMinimalEditFormationTreeV240:InvalidPreflightOptions', ...
        'The V240 structural preflight options are invalid.');
end

records = repmat(emptyRecord(), 1, numel(presetNames));
for presetIdx = 1:numel(presetNames)
    records(presetIdx) = replayPreset(presetNames{presetIdx}, seed);
    record = records(presetIdx);
    fprintf([ ...
        'V240 %s: scene=%d physical=%d messages=%d strong=%d ', ...
        'tree changes=%d reselections=%s initial failure=%g\n'], ...
        record.presetName, record.sceneValid, ...
        record.formationPhysicalConnectedAllTimes, ...
        record.messagesPerRound, record.sensorStrongAllTimes, ...
        record.treeChangeCount, mat2str(record.treeReselectionTimes), ...
        record.initialTreeFirstPhysicalFailureTime);
end

result = struct();
result.contractVersion = ...
    'causal-minimal-edit-formation-tree-v240-structural-result-v1';
result.generatedAt = datestr(now, 31);
result.protocol = protocol;
result.seed = seed;
result.records = records;
result.gatePassed = all([records.sceneValid]) && ...
    all([records.formationPhysicalConnectedAllTimes]) && ...
    all([records.sensorStrongAllTimes]) && ...
    all([records.physicalRouteAllTimes]) && ...
    all([records.messageParityAllTimes]) && ...
    all([records.initialTreeActuallyFails]) && ...
    all([records.treeReselectionOnlyOnInfeasibility]) && ...
    all([records.treePreservedWhenFeasible]);
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = protocol.evidenceBoundary;

reportPath = '';
if writeReport
    outputDirectory = fullfile('RUN', 'GA', 'dynamic_topology', ...
        'evidence', 'tracking_aligned_v240', 'structural_preflight');
    if exist(outputDirectory, 'dir') ~= 7
        mkdir(outputDirectory);
    end
    reportPath = fullfile(outputDirectory, ...
        'CAUSAL_MINIMAL_EDIT_FORMATION_TREE_V240_STRUCTURAL.md');
    matPath = strrep(reportPath, '.md', '.mat');
    result.reportPath = reportPath;
    result.matPath = matPath;
    writeReportFile(reportPath, result);
    save(matPath, 'result');
end
if ~result.gatePassed
    error('CausalMinimalEditFormationTreeV240:StructuralGateFailed', ...
        'The V240 structural preflight failed.');
end
fprintf('CAUSAL_MINIMAL_EDIT_FORMATION_TREE_V240_STRUCTURAL_PASSED=1\n');
end

function record = replayPreset(presetName, seed)
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
validation = validateDynamicTopologyScenario(config, sensors, targets, ...
    graph, struct('throwOnInvalid', false));
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
nodeCount = config.numberOfSensors;
timeCount = config.simulationLength;
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
initialTree = false(config.formationCount);
previousTree = false(config.formationCount);
treeChangeTimes = zeros(1, 0);
treeReselectionTimes = zeros(1, 0);
replacementCounts = zeros(1, timeCount);
messages = zeros(1, timeCount);
sensorStrong = false(1, timeCount);
physicalRoute = false(1, timeCount);
messageParity = false(1, timeCount);
treeReselectionOnlyOnInfeasibility = true;
treePreservedWhenFeasible = true;
formationPhysicalConnected = false(1, timeCount);
initialTreePhysical = false(1, timeCount);
for currentTime = 1:timeCount
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes);
    [adjacency, details] = ...
        selectCausalMinimalEditFormationTreeV240Policy(context);
    tree = logical(details.currentFormationTreeAdjacency);
    if currentTime == 1
        initialTree = tree;
    elseif ~isequal(tree, previousTree)
        treeChangeTimes(end + 1) = currentTime; %#ok<AGROW>
    end
    if details.treeReselectionUsed
        treeReselectionTimes(end + 1) = currentTime; %#ok<AGROW>
    end
    treeReselectionOnlyOnInfeasibility = ...
        treeReselectionOnlyOnInfeasibility && ...
        (~details.treeReselectionUsed || ...
         details.physicalInfeasibilityFallbackUsed || ...
         details.assignmentInfeasibilityFallbackUsed);
    treePreservedWhenFeasible = treePreservedWhenFeasible && ...
        (~details.incumbentTreeAvailable || ...
         ~details.incumbentAssignmentFeasible || ...
         details.incumbentTreePreserved);
    currentPhysical = logical(graph.physicalAdjacency(:, :, currentTime));
    formationPhysical = formationAdjacency( ...
        currentPhysical, config.sensorGroupIds);
    formationPhysicalConnected(currentTime) = ...
        isConnected(formationPhysical);
    initialTreePhysical(currentTime) = ...
        ~any(initialTree(:) & ~formationPhysical(:));
    messages(currentTime) = nnz(adjacency);
    sensorStrong(currentTime) = isStronglyConnected(adjacency);
    physicalRoute(currentTime) = ...
        ~any(adjacency(:) & ~currentPhysical(:));
    messageParity(currentTime) = ...
        messages(currentTime) == 2 * nodeCount && ...
        all(abs(sum(details.fusionWeightMatrix, 2) - 1) < 1e-12);
    replacementCounts(currentTime) = ...
        details.formationEdgeReplacementCount;
    history = cat(3, history, adjacency);
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
    previousTree = tree;
end
firstFailure = find(~initialTreePhysical, 1);
if isempty(firstFailure)
    firstFailure = NaN;
end

metrics = validation.difficulty;
record = emptyRecord();
record.presetName = presetName;
record.seed = seed;
record.sensorCount = nodeCount;
record.formationCount = config.formationCount;
record.targetCount = config.numberOfTargets;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.formationPhysicalConnectedAllTimes = ...
    all(formationPhysicalConnected);
record.sensorStrongAllTimes = all(sensorStrong);
record.physicalRouteAllTimes = all(physicalRoute);
record.messageParityAllTimes = all(messageParity);
record.messagesPerRound = unique(messages);
record.treeChangeTimes = treeChangeTimes;
record.treeChangeCount = numel(treeChangeTimes);
record.treeReselectionTimes = treeReselectionTimes;
record.treeReselectionCount = numel(treeReselectionTimes);
record.totalFormationEdgeReplacements = sum(replacementCounts);
record.initialTreeFirstPhysicalFailureTime = firstFailure;
record.initialTreeActuallyFails = isfinite(firstFailure);
record.treeReselectionOnlyOnInfeasibility = ...
    treeReselectionOnlyOnInfeasibility;
record.treePreservedWhenFeasible = treePreservedWhenFeasible;
record.minimumSensorTargetSeparation = ...
    validation.minimumSensorTargetSeparation;
record.blackoutFraction = metrics.blackoutFraction;
record.focusBlackoutFraction = metrics.focusBlackoutFraction;
record.maximumPerTargetBlackoutFraction = ...
    metrics.maximumPerTargetBlackoutFraction;
record.maximumConsecutiveBlackoutSteps = ...
    metrics.maximumConsecutiveBlackoutSteps;
record.focusMeanVisibleSensorCount = ...
    metrics.focusMeanVisibleSensorCount;
record.focusMeanExpectedTargetDetectionCount = ...
    metrics.focusMeanExpectedTargetDetectionCount;
record.focusHandovers = metrics.focusHandovers;
end

function context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes)
nodeCount = config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', config, ...
    'staticAdjacency', logical(graph.staticAdjacency)));
context.baseAdjacency = logical(graph.staticAdjacency);
context.physicalAdjacency = logical( ...
    graph.physicalAdjacency(:, :, currentTime));
context.positions = graph.positions(:, :, currentTime);
context.currentTime = currentTime;
context.commConfig = struct('pDropByEdge', pDrop(:, :, currentTime));
context.directedMessageBudget = 2 * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.previousAdjacencyHistory = history;
context.previousAdjacencyHistoryCount = size(history, 3);
context.previousAdjacencyHistoryTimes = historyTimes;
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column';
context.previousAdjacencyHistorySource = ...
    'v240-structural-policy-replay';
end

function adjacency = formationAdjacency(sensorAdjacency, groupIds)
groups = unique(groupIds, 'stable');
adjacency = false(numel(groups));
for left = 1:numel(groups)-1
    for right = left+1:numel(groups)
        block = sensorAdjacency(groupIds == groups(left), ...
            groupIds == groups(right));
        adjacency(left, right) = any(block(:));
        adjacency(right, left) = adjacency(left, right);
    end
end
end

function connected = isConnected(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end

function connected = isStronglyConnected(adjacency)
nodeCount = size(adjacency, 1);
connected = true;
for root = 1:nodeCount
    visited = false(1, nodeCount);
    frontier = root;
    while ~isempty(frontier)
        node = frontier(end);
        frontier(end) = [];
        if visited(node), continue; end
        visited(node) = true;
        frontier = [frontier, ...
            find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
    end
    if ~all(visited)
        connected = false;
        return;
    end
end
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write V240 report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V240 causal minimal-edit formation-tree preflight\n\n');
fprintf(fid, '- Seed: `%d`\n', result.seed);
fprintf(fid, '- Structural gate passed: `%d`\n', result.gatePassed);
fprintf(fid, '- Tracking outcome authorized: `0`\n\n');
fprintf(fid, ['| Preset | Sensors | Formations | Targets | Scene | ', ...
    'Physical connected | Initial tree fails | First failure | ', ...
    'Tree changes | Reselection times | Replacements | Messages/round | ', ...
    'Strong | Physical route | Min sensor-target (m) | Blackout | ', ...
    'Focus blackout | Worst target | Longest | Visible sensors | ', ...
    'Expected detections | Handovers |\n']);
fprintf(fid, ['|:--|--:|--:|--:|:--:|:--:|:--:|--:|--:|:--|--:|', ...
    '--:|:--:|:--:|--:|--:|--:|--:|--:|--:|--:|--:|\n']);
for record = result.records
    fprintf(fid, ['| %s | %d | %d | %d | %d | %d | %d | %g | ', ...
        '%d | %s | %d | %s | %d | %d | %.1f | %.3f | %.3f | ', ...
        '%.3f | %d | %.2f | %.2f | %d |\n'], ...
        record.presetName, record.sensorCount, record.formationCount, ...
        record.targetCount, record.sceneValid, ...
        record.formationPhysicalConnectedAllTimes, ...
        record.initialTreeActuallyFails, ...
        record.initialTreeFirstPhysicalFailureTime, ...
        record.treeChangeCount, mat2str(record.treeReselectionTimes), ...
        record.totalFormationEdgeReplacements, ...
        mat2str(record.messagesPerRound), record.sensorStrongAllTimes, ...
        record.physicalRouteAllTimes, ...
        record.minimumSensorTargetSeparation, ...
        record.blackoutFraction, record.focusBlackoutFraction, ...
        record.maximumPerTargetBlackoutFraction, ...
        record.maximumConsecutiveBlackoutSteps, ...
        record.focusMeanVisibleSensorCount, ...
        record.focusMeanExpectedTargetDetectionCount, ...
        record.focusHandovers);
end
fprintf(fid, '\n## Decision boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function record = emptyRecord()
record = struct('presetName', '', 'seed', NaN, ...
    'sensorCount', NaN, 'formationCount', NaN, 'targetCount', NaN, ...
    'sceneValid', false, 'sceneHardFailures', {{}}, ...
    'formationPhysicalConnectedAllTimes', false, ...
    'sensorStrongAllTimes', false, 'physicalRouteAllTimes', false, ...
    'messageParityAllTimes', false, 'messagesPerRound', zeros(1, 0), ...
    'treeChangeTimes', zeros(1, 0), 'treeChangeCount', NaN, ...
    'treeReselectionTimes', zeros(1, 0), ...
    'treeReselectionCount', NaN, ...
    'totalFormationEdgeReplacements', NaN, ...
    'initialTreeFirstPhysicalFailureTime', NaN, ...
    'initialTreeActuallyFails', false, ...
    'treeReselectionOnlyOnInfeasibility', false, ...
    'treePreservedWhenFeasible', false, ...
    'minimumSensorTargetSeparation', NaN, ...
    'blackoutFraction', NaN, 'focusBlackoutFraction', NaN, ...
    'maximumPerTargetBlackoutFraction', NaN, ...
    'maximumConsecutiveBlackoutSteps', NaN, ...
    'focusMeanVisibleSensorCount', NaN, ...
    'focusMeanExpectedTargetDetectionCount', NaN, ...
    'focusHandovers', NaN);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
