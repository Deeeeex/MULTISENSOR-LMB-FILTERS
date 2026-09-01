function [reportPath, result] = ...
        runCausalMinimumFormationBackboneV242StructuralPreflight(options)
% RUNCAUSALMINIMUMFORMATIONBACKBONEV242STRUCTURALPREFLIGHT Cross-scale gate.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCausalMinimumFormationBackboneV242Protocol();
presets = getField(options, 'presetNames', protocol.allowedPresets);
seed = getField(options, 'seed', protocol.allowedSeeds(1));
writeReport = getField(options, 'writeReport', true);
if ischar(presets), presets = {presets}; end
if ~iscell(presets) || isempty(presets) || ...
        any(~cellfun(@ischar, presets)) || ...
        any(~ismember(presets, protocol.allowedPresets)) || ...
        numel(unique(presets)) ~= numel(presets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('CausalMinimumFormationBackboneV242:InvalidPreflightOptions', ...
        'The V242 preflight options are invalid.');
end

records = repmat(emptyRecord(), 1, numel(presets));
for presetIdx = 1:numel(presets)
    records(presetIdx) = replayPreset( ...
        presets{presetIdx}, seed, protocol);
end
result = struct();
result.contractVersion = ...
    'causal-minimum-formation-backbone-v242-structural-result-v1';
result.generatedAt = datestr(now, 31);
result.protocol = protocol;
result.seed = seed;
result.records = records;
result.gatePassed = all([records.gatePassed]);
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = protocol.evidenceBoundary;

reportPath = '';
if writeReport
    outputDirectory = fullfile('RUN', 'GA', 'dynamic_topology', ...
        'evidence', 'tracking_aligned_v242', 'structural_preflight');
    if exist(outputDirectory, 'dir') ~= 7
        mkdir(outputDirectory);
    end
    reportPath = fullfile(outputDirectory, ...
        'CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_STRUCTURAL.md');
    result.reportPath = reportPath;
    result.matPath = strrep(reportPath, '.md', '.mat');
    writeReportFile(reportPath, result);
    save(result.matPath, 'result');
end
if ~result.gatePassed
    error('CausalMinimumFormationBackboneV242:StructuralGateFailed', ...
        'At least one V242 scale failed the structural gate.');
end
for record = records
    fprintf(['V242 %s: %d/%d messages (%.3f saving), strong=%d, ', ...
        'reselections=%s, mean selection=%.4fs\n'], ...
        record.presetName, record.minimumMessageCount, ...
        record.referenceMessageCount, record.messageSavingFraction, ...
        record.strongAllTimes, mat2str(record.treeReselectionTimes), ...
        record.meanSelectionSeconds);
end
fprintf('CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_STRUCTURAL_PASSED=1\n');
end

function record = replayPreset(presetName, seed, protocol)
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
validation = validateDynamicTopologyScenario( ...
    config, sensors, targets, graph, struct('throwOnInvalid', false));
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
nodeCount = config.numberOfSensors;
formationCount = numel(unique(config.sensorGroupIds));
minimumMessages = nodeCount + 2 * (formationCount - 1);
referenceMessages = 2 * nodeCount;
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
previousTree = false(formationCount);
messageCounts = zeros(1, config.simulationLength);
strong = false(1, config.simulationLength);
physical = false(1, config.simulationLength);
weightsValid = false(1, config.simulationLength);
selectionSeconds = zeros(1, config.simulationLength);
treeChangeTimes = zeros(1, 0);
treeReselectionTimes = zeros(1, 0);
reselectionOnlyOnInfeasibility = true;
for currentTime = 1:config.simulationLength
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, minimumMessages);
    timerId = tic;
    [adjacency, details] = ...
        selectCausalMinimumFormationBackboneV242Policy(context);
    selectionSeconds(currentTime) = toc(timerId);
    currentPhysical = logical( ...
        graph.physicalAdjacency(:, :, currentTime));
    weights = details.fusionWeightMatrix;
    positiveAllowed = logical(adjacency) | logical(eye(nodeCount));
    messageCounts(currentTime) = nnz(adjacency);
    strong(currentTime) = isStronglyConnected(adjacency);
    physical(currentTime) = ...
        ~any(adjacency(:) & ~currentPhysical(:));
    weightsValid(currentTime) = ...
        all(abs(sum(weights, 2) - 1) <= 1e-12) && ...
        all(weights(:) >= 0) && ...
        ~any(weights(:) > 0 & ~positiveAllowed(:));
    tree = logical(details.currentFormationTreeAdjacency);
    if currentTime > 1 && ~isequal(tree, previousTree)
        treeChangeTimes(end + 1) = currentTime; %#ok<AGROW>
    end
    previousTree = tree;
    if details.treeReselectionUsed
        treeReselectionTimes(end + 1) = currentTime; %#ok<AGROW>
        reselectionOnlyOnInfeasibility = ...
            reselectionOnlyOnInfeasibility && ...
            (details.physicalInfeasibilityFallbackUsed || ...
             details.assignmentInfeasibilityFallbackUsed);
    end
    history = cat(3, history, adjacency);
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
end

record = emptyRecord();
record.presetName = presetName;
record.nodeCount = nodeCount;
record.formationCount = formationCount;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.minimumMessageCount = minimumMessages;
record.referenceMessageCount = referenceMessages;
record.minimumMessagesObserved = min(messageCounts);
record.maximumMessagesObserved = max(messageCounts);
record.messageSavingCount = referenceMessages - minimumMessages;
record.messageSavingFraction = ...
    record.messageSavingCount / referenceMessages;
record.strongAllTimes = all(strong);
record.physicalAllTimes = all(physical);
record.weightsValidAllTimes = all(weightsValid);
record.treeChangeTimes = treeChangeTimes;
record.treeChangeCount = numel(treeChangeTimes);
record.treeReselectionTimes = treeReselectionTimes;
record.treeReselectionCount = numel(treeReselectionTimes);
record.treeReselectionOnlyOnInfeasibility = ...
    reselectionOnlyOnInfeasibility;
record.meanSelectionSeconds = mean(selectionSeconds);
record.maximumSelectionSeconds = max(selectionSeconds);
record.gatePassed = validation.isValid && ...
    all(messageCounts == minimumMessages) && ...
    all(strong) && all(physical) && all(weightsValid) && ...
    isequal(treeChangeTimes, treeReselectionTimes) && ...
    reselectionOnlyOnInfeasibility;
end

function context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageBudget)
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
context.directedMessageBudget = messageBudget;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.previousAdjacencyHistory = history;
context.previousAdjacencyHistoryCount = size(history, 3);
context.previousAdjacencyHistoryTimes = historyTimes;
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write V242 structural report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V242 causal minimum formation backbone\n\n');
fprintf(fid, '- Opened development seed: `%d`\n', result.seed);
fprintf(fid, '- Structural gate passed: `%d`\n', result.gatePassed);
fprintf(fid, '- Tracking outcome authorized: `0`\n\n');
fprintf(fid, ['| Scene | N / F | Messages candidate / 2N | Saving | ', ...
    'Strong | Physical | Necessary reselections | Mean / max select s |\n']);
fprintf(fid, '|:--|:--:|:--:|--:|:--:|:--:|:--|:--|\n');
for record = result.records
    fprintf(fid, ...
        '| %s | %d / %d | %d / %d | %.3f%% | %d | %d | %s | %.4f / %.4f |\n', ...
        record.presetName, record.nodeCount, record.formationCount, ...
        record.minimumMessageCount, record.referenceMessageCount, ...
        100 * record.messageSavingFraction, ...
        record.strongAllTimes, record.physicalAllTimes, ...
        mat2str(record.treeReselectionTimes), ...
        record.meanSelectionSeconds, record.maximumSelectionSeconds);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function record = emptyRecord()
record = struct('presetName', '', 'nodeCount', NaN, ...
    'formationCount', NaN, 'sceneValid', false, ...
    'sceneHardFailures', {{}}, 'minimumMessageCount', NaN, ...
    'referenceMessageCount', NaN, ...
    'minimumMessagesObserved', NaN, ...
    'maximumMessagesObserved', NaN, 'messageSavingCount', NaN, ...
    'messageSavingFraction', NaN, 'strongAllTimes', false, ...
    'physicalAllTimes', false, 'weightsValidAllTimes', false, ...
    'treeChangeTimes', zeros(1, 0), 'treeChangeCount', NaN, ...
    'treeReselectionTimes', zeros(1, 0), ...
    'treeReselectionCount', NaN, ...
    'treeReselectionOnlyOnInfeasibility', false, ...
    'meanSelectionSeconds', NaN, ...
    'maximumSelectionSeconds', NaN, 'gatePassed', false);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
