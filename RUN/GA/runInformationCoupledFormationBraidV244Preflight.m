function [reportPath, result] = ...
        runInformationCoupledFormationBraidV244Preflight(options)
% RUNINFORMATIONCOUPLEDFORMATIONBRAIDV244PREFLIGHT Check task/topology coupling.
%
% A scene can break an initial communication tree without making the broken
% cut relevant to target tracking.  This preflight compares the original
% paired target handoffs with a variant whose handoffs cover every initial
% formation-tree cut.  It uses geometry and target routes only; it does not
% run or score a tracker.

if nargin < 1 || isempty(options)
    options = struct();
end
seed = getField(options, 'seed', 1301);
writeReport = logical(getField(options, 'writeReport', true));
if ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('InformationCoupledBraidV244:InvalidOptions', ...
        'The seed and report option must be scalar values.');
end

presets = registeredPresets();
records = repmat(emptyRecord(), 1, numel(presets));
for presetIdx = 1:numel(presets)
    records(presetIdx) = inspectPreset(presets{presetIdx}, seed);
    record = records(presetIdx);
    fprintf([ ...
        'V244 %-53s valid=%d connected=%.3f tree=%.3f ', ...
        'failedCuts=%d covered=%d minTransfers=%d failures=%s\n'], ...
        record.presetName, record.sceneValid, ...
        record.formationConnectedFraction, ...
        record.initialTreeFeasibleFraction, ...
        record.failedInitialCutCount, ...
        record.taskCoupledFailedCutCount, ...
        record.minimumTransfersPerFailedCut, ...
        joinTokens(record.sceneHardFailures));
end

isCoupled = [records.informationCoupled];
coupledRecords = records(isCoupled);
originalRecords = records(~isCoupled);
result = struct();
result.contractVersion = ...
    'information-coupled-formation-braid-v244-preflight-result-v1';
result.generatedAt = datestr(now, 31);
result.seed = seed;
result.records = records;
result.coupledSceneGatePassed = all([coupledRecords.sceneValid]) && ...
    all([coupledRecords.formationConnectedFraction] == 1) && ...
    all([coupledRecords.initialTreeFeasibleFraction] < 1) && ...
    all([coupledRecords.allFailedInitialCutsTaskCoupled]) && ...
    all([coupledRecords.allInitialTreeCutsTaskCoupled]);
result.originalUncoupledFailureDemonstrated = ...
    all([originalRecords.failedInitialCutCount] > 0) && ...
    all(~[originalRecords.allFailedInitialCutsTaskCoupled]);
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = [ ...
    'V244 checks scene validity, physical formation connectivity, ', ...
    'initial-tree failure, and whether registered target handoffs cross ', ...
    'the affected tree cuts. It does not establish a tracking, ', ...
    'consistency, communication-byte, or generalization benefit.'];
if ~result.coupledSceneGatePassed || ...
        ~result.originalUncoupledFailureDemonstrated
    error('InformationCoupledBraidV244:PreflightFailed', ...
        'The task/topology coupling gate did not pass across all scales.');
end

reportPath = '';
if writeReport
    outputDirectory = fullfile('RUN', 'GA', 'dynamic_topology', ...
        'evidence', 'tracking_aligned_v244', ...
        'information_coupled_formation_braid');
    if exist(outputDirectory, 'dir') ~= 7
        mkdir(outputDirectory);
    end
    reportPath = fullfile(outputDirectory, ...
        'INFORMATION_COUPLED_FORMATION_BRAID_V244_PREFLIGHT.md');
    result.reportPath = reportPath;
    result.matPath = strrep(reportPath, '.md', '.mat');
    writeReportFile(reportPath, result);
    save('-mat7-binary', result.matPath, 'result');
end
end

function record = inspectPreset(presetName, seed)
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
validation = validateDynamicTopologyScenario( ...
    config, sensors, targets, graph, struct('throwOnInvalid', false));
groupIds = reshape(config.sensorGroupIds, 1, []);
initialTree = formationAdjacency( ...
    logical(graph.staticAdjacency), groupIds);
formationCount = config.formationCount;
if nnz(triu(initialTree, 1)) ~= formationCount - 1 || ...
        ~isConnected(initialTree)
    error('InformationCoupledBraidV244:InvalidInitialTree', ...
        'The registered initial formation graph is not a tree.');
end

formationConnected = false(1, config.simulationLength);
initialTreeFeasible = false(1, config.simulationLength);
edgePairs = findUndirectedPairs(initialTree);
firstFailureByEdge = NaN(size(edgePairs, 1), 1);
for currentTime = 1:config.simulationLength
    physical = logical(graph.physicalAdjacency(:, :, currentTime));
    currentFormation = formationAdjacency(physical, groupIds);
    formationConnected(currentTime) = isConnected(currentFormation);
    initialTreeFeasible(currentTime) = ...
        ~any(initialTree(:) & ~currentFormation(:));
    for edgeIdx = 1:size(edgePairs, 1)
        left = edgePairs(edgeIdx, 1);
        right = edgePairs(edgeIdx, 2);
        if isnan(firstFailureByEdge(edgeIdx)) && ...
                ~currentFormation(left, right)
            firstFailureByEdge(edgeIdx) = currentTime;
        end
    end
end

sourceIds = reshape(config.targetRouteSourceFormationIds, 1, []);
destinationIds = reshape( ...
    config.targetRouteDestinationFormationIds, 1, []);
if numel(sourceIds) ~= config.targetGroupCount || ...
        numel(destinationIds) ~= config.targetGroupCount
    error('InformationCoupledBraidV244:InvalidTargetRouteMetadata', ...
        'Target source/destination metadata is incomplete.');
end
transferCountByCut = zeros(size(edgePairs, 1), 1);
for edgeIdx = 1:size(edgePairs, 1)
    cutTree = initialTree;
    left = edgePairs(edgeIdx, 1);
    right = edgePairs(edgeIdx, 2);
    cutTree(left, right) = false;
    cutTree(right, left) = false;
    leftComponent = reachableFrom(cutTree, left);
    transferCountByCut(edgeIdx) = sum(xor( ...
        leftComponent(sourceIds), leftComponent(destinationIds)));
end
failedMask = isfinite(firstFailureByEdge);
failedTransferCounts = transferCountByCut(failedMask);

record = emptyRecord();
record.presetName = config.presetName;
record.sceneStyle = config.sceneStyle;
record.informationCoupled = strcmp(config.sceneStyle, ...
    'information-coupled-formation-braid');
record.sensorCount = config.numberOfSensors;
record.formationCount = formationCount;
record.targetCount = config.numberOfTargets;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.formationConnectedFraction = mean(formationConnected);
record.initialTreeFeasibleFraction = mean(initialTreeFeasible);
record.initialTreeFirstFailureTime = firstFalse(initialTreeFeasible);
record.initialTreePairs = edgePairs;
record.initialTreeEdgeFirstFailureTimes = firstFailureByEdge;
record.targetTransferPairs = [sourceIds(:), destinationIds(:)];
record.transferCountByInitialCut = transferCountByCut;
record.failedInitialCutCount = sum(failedMask);
record.taskCoupledFailedCutCount = sum( ...
    failedMask & transferCountByCut > 0);
record.allFailedInitialCutsTaskCoupled = ...
    record.failedInitialCutCount > 0 && all(failedTransferCounts > 0);
record.allInitialTreeCutsTaskCoupled = all(transferCountByCut > 0);
record.minimumTransfersPerFailedCut = minOrZero(failedTransferCounts);
record.taskTopologyCouplingContract = ...
    config.taskTopologyCouplingContract;
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

function value = firstFalse(trace)
index = find(~trace, 1);
if isempty(index)
    value = NaN;
else
    value = index;
end
end

function value = minOrZero(values)
if isempty(values)
    value = 0;
else
    value = min(values);
end
end

function presets = registeredPresets()
presets = { ...
    'm24-formation-fov-formation-braid', ...
    'm24-formation-fov-coupled-formation-braid', ...
    'x36-formation-fov-formation-braid', ...
    'x36-formation-fov-coupled-formation-braid', ...
    'x48-formation-fov-formation-braid', ...
    'x48-formation-fov-coupled-formation-braid'};
end

function record = emptyRecord()
record = struct( ...
    'presetName', '', 'sceneStyle', '', ...
    'informationCoupled', false, 'sensorCount', NaN, ...
    'formationCount', NaN, 'targetCount', NaN, ...
    'sceneValid', false, 'sceneHardFailures', {{}}, ...
    'formationConnectedFraction', NaN, ...
    'initialTreeFeasibleFraction', NaN, ...
    'initialTreeFirstFailureTime', NaN, ...
    'initialTreePairs', zeros(0, 2), ...
    'initialTreeEdgeFirstFailureTimes', zeros(0, 1), ...
    'targetTransferPairs', zeros(0, 2), ...
    'transferCountByInitialCut', zeros(0, 1), ...
    'failedInitialCutCount', 0, ...
    'taskCoupledFailedCutCount', 0, ...
    'allFailedInitialCutsTaskCoupled', false, ...
    'allInitialTreeCutsTaskCoupled', false, ...
    'minimumTransfersPerFailedCut', 0, ...
    'taskTopologyCouplingContract', '');
end

function writeReportFile(path, result)
fileId = fopen(path, 'w');
if fileId < 0
    error('InformationCoupledBraidV244:ReportOpenFailed', ...
        'Unable to open %s.', path);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId, '# V244 information-coupled formation-braid preflight\n\n');
fprintf(fileId, '- Seed: `%d`\n', result.seed);
fprintf(fileId, '- Coupled-scene gate passed: `%d`\n', ...
    result.coupledSceneGatePassed);
fprintf(fileId, '- Original uncoupled failure demonstrated: `%d`\n', ...
    result.originalUncoupledFailureDemonstrated);
fprintf(fileId, '- Tracking outcome authorized: `0`\n');
fprintf(fileId, '- Evidence boundary: %s\n\n', result.evidenceBoundary);
fprintf(fileId, [ ...
    '| Preset | Valid | Connected | Initial tree feasible | First ', ...
    'failure | Failed cuts | Failed cuts with target transfer | ', ...
    'All tree cuts covered | Min transfers / failed cut |\n']);
fprintf(fileId, '|:--|--:|--:|--:|--:|--:|--:|--:|--:|\n');
for idx = 1:numel(result.records)
    record = result.records(idx);
    fprintf(fileId, [ ...
        '| %s | %d | %.3f | %.3f | %s | %d | %d | %d | %d |\n'], ...
        record.presetName, record.sceneValid, ...
        record.formationConnectedFraction, ...
        record.initialTreeFeasibleFraction, ...
        scalarText(record.initialTreeFirstFailureTime), ...
        record.failedInitialCutCount, ...
        record.taskCoupledFailedCutCount, ...
        record.allInitialTreeCutsTaskCoupled, ...
        record.minimumTransfersPerFailedCut);
end
fprintf(fileId, '\n## Per-scene cut evidence\n\n');
for idx = 1:numel(result.records)
    record = result.records(idx);
    fprintf(fileId, '### %s\n\n', record.presetName);
    fprintf(fileId, '- Initial tree edges: `%s`\n', ...
        mat2str(record.initialTreePairs));
    fprintf(fileId, '- Edge first-failure times: `%s`\n', ...
        mat2str(record.initialTreeEdgeFirstFailureTimes'));
    fprintf(fileId, '- Target source/destination pairs: `%s`\n', ...
        mat2str(record.targetTransferPairs));
    fprintf(fileId, '- Target transfers across each tree cut: `%s`\n\n', ...
        mat2str(record.transferCountByInitialCut'));
end
end

function text = scalarText(value)
if isfinite(value)
    text = sprintf('%g', value);
else
    text = '--';
end
end

function text = joinTokens(tokens)
if isempty(tokens)
    text = '--';
else
    text = strjoin(tokens, '+');
end
end

function value = getField(structure, name, defaultValue)
if isstruct(structure) && isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
