function [reportPath, result] = runDynamicTopologySceneStressSurveyV243(options)
% RUNDYNAMICTOPOLOGYSCENESTRESSSURVEYV243 Qualify scene routing stress.
%
% This survey does not run a tracker.  It separates scenes that merely
% look different from scenes that actually require causal topology repair:
% the physical formation graph must stay connected while the initially
% registered formation tree becomes infeasible at least once.

if nargin < 1 || isempty(options)
    options = struct();
end
presetNames = getField(options, 'presetNames', defaultPresets());
seed = getField(options, 'seed', 1301);
writeReport = logical(getField(options, 'writeReport', true));
if ischar(presetNames)
    presetNames = {presetNames};
end
if ~iscell(presetNames) || isempty(presetNames) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('DynamicTopologySceneStressV243:InvalidOptions', ...
        'Preset names, seed, or report option are invalid.');
end

records = repmat(emptyRecord(), 1, numel(presetNames));
for presetIdx = 1:numel(presetNames)
    records(presetIdx) = inspectPreset(presetNames{presetIdx}, seed);
    record = records(presetIdx);
    fprintf([ ...
        'V243 %-45s connected=%.3f tree=%.3f exact=%.3f ', ...
        'firstTreeFailure=%g class=%s\n'], ...
        record.presetName, record.formationConnectedFraction, ...
        record.initialTreeFeasibleFraction, ...
        record.exactStaticRoutePhysicalFraction, ...
        record.initialTreeFirstFailureTime, record.qualification);
end

result = struct();
result.contractVersion = ...
    'dynamic-topology-scene-stress-survey-v243-result-v1';
result.generatedAt = datestr(now, 31);
result.seed = seed;
result.records = records;
result.dynamicTreeCandidates = ...
    {records(strcmp({records.qualification}, ...
        'dynamic-tree-candidate')).presetName};
result.dynamicGatewayCandidates = ...
    {records(strcmp({records.qualification}, ...
        'dynamic-gateway-candidate')).presetName};
result.dynamicRoutingCandidates = [ ...
    result.dynamicTreeCandidates, result.dynamicGatewayCandidates];
result.generalizationControls = ...
    {records(strcmp({records.qualification}, ...
        'static-route-control')).presetName};
result.unsuitableWithoutStoreForward = ...
    {records(strcmp({records.qualification}, ...
        'physical-formation-disconnection')).presetName};
result.sceneRepairRequired = ...
    {records(strcmp({records.qualification}, ...
        'scene-repair-required')).presetName};
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = [ ...
    'V243 uses geometry and physical communication reachability only. ', ...
    'It selects candidate scene families for later paired tracking ', ...
    'experiments but does not establish tracking, consistency, ', ...
    'communication-byte, or generalization benefit.'];

reportPath = '';
if writeReport
    outputDirectory = fullfile('RUN', 'GA', 'dynamic_topology', ...
        'evidence', 'tracking_aligned_v243', 'scene_stress_survey');
    if exist(outputDirectory, 'dir') ~= 7
        mkdir(outputDirectory);
    end
    reportPath = fullfile(outputDirectory, ...
        'DYNAMIC_TOPOLOGY_SCENE_STRESS_SURVEY_V243.md');
    result.reportPath = reportPath;
    result.matPath = strrep(reportPath, '.md', '.mat');
    writeReportFile(reportPath, result);
    save('-mat7-binary', result.matPath, 'result');
end
end

function record = inspectPreset(presetName, seed)
record = emptyRecord();
record.presetName = char(presetName);
record.seed = seed;
try
    rng(seed, 'twister');
    config = buildDynamicTopologyScenarioConfig(presetName);
    [sensors, ~] = generateMultiFormationTrajectories(config);
    [targets, ~] = generateCorridorTargetTrajectories(config);
    graph = buildDynamicTopologyGraphs(config, sensors);
    validation = validateDynamicTopologyScenario( ...
        config, sensors, targets, graph, struct('throwOnInvalid', false));
    record.sceneStyle = getField(config, 'sceneStyle', config.variant);
    record.sensorCount = config.numberOfSensors;
    record.formationCount = config.formationCount;
    record.targetCount = config.numberOfTargets;
    record.sceneValid = validation.isValid;
    record.sceneHardFailures = validation.hardFailures;
    [connected, initialTreeFeasible, exactRoutePhysical] = ...
        topologyTraces(config, graph);
    record.formationConnectedFraction = mean(connected);
    record.initialTreeFeasibleFraction = mean(initialTreeFeasible);
    record.exactStaticRoutePhysicalFraction = mean(exactRoutePhysical);
    record.formationFirstDisconnectionTime = firstFalse(connected);
    record.initialTreeFirstFailureTime = firstFalse(initialTreeFeasible);
    record.exactStaticRouteFirstFailureTime = ...
        firstFalse(exactRoutePhysical);
    record.initialTreeFailureEpisodeCount = ...
        failureEpisodeCount(initialTreeFeasible);
    record.qualification = qualify(record);
catch exception
    record.sceneValid = false;
    record.errorIdentifier = exception.identifier;
    record.errorMessage = exception.message;
    record.qualification = 'scene-repair-required';
end
end

function [connected, treeFeasible, exactPhysical] = ...
        topologyTraces(config, graph)
timeCount = config.simulationLength;
formationCount = config.formationCount;
groupIds = reshape(config.sensorGroupIds, 1, []);
initialFormationTree = formationAdjacency( ...
    logical(graph.staticAdjacency), groupIds);
connected = false(1, timeCount);
treeFeasible = false(1, timeCount);
exactPhysical = false(1, timeCount);
for currentTime = 1:timeCount
    physical = logical(graph.physicalAdjacency(:, :, currentTime));
    currentFormation = formationAdjacency(physical, groupIds);
    connected(currentTime) = isConnected(currentFormation);
    treeFeasible(currentTime) = ...
        ~any(initialFormationTree(:) & ~currentFormation(:));
    exactPhysical(currentTime) = ...
        ~any(logical(graph.staticAdjacency(:)) & ~physical(:));
end
if nnz(triu(initialFormationTree, 1)) < formationCount - 1 || ...
        ~isConnected(initialFormationTree)
    error('DynamicTopologySceneStressV243:InvalidInitialTree', ...
        'The registered static route has no formation spanning tree.');
end
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
if isempty(adjacency)
    connected = false;
    return;
end
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
connected = all(visited);
end

function qualification = qualify(record)
if ~record.sceneValid
    qualification = 'scene-repair-required';
elseif record.formationConnectedFraction < 1
    qualification = 'physical-formation-disconnection';
elseif record.initialTreeFeasibleFraction < 1
    qualification = 'dynamic-tree-candidate';
elseif record.exactStaticRoutePhysicalFraction < 1
    qualification = 'dynamic-gateway-candidate';
else
    qualification = 'static-route-control';
end
end

function value = firstFalse(trace)
index = find(~trace, 1);
if isempty(index)
    value = NaN;
else
    value = index;
end
end

function count = failureEpisodeCount(feasible)
failed = ~logical(feasible);
count = sum(failed & [true, ~failed(1:end-1)]);
end

function record = emptyRecord()
record = struct( ...
    'presetName', '', ...
    'sceneStyle', '', ...
    'seed', NaN, ...
    'sensorCount', NaN, ...
    'formationCount', NaN, ...
    'targetCount', NaN, ...
    'sceneValid', false, ...
    'sceneHardFailures', {{}}, ...
    'formationConnectedFraction', NaN, ...
    'initialTreeFeasibleFraction', NaN, ...
    'exactStaticRoutePhysicalFraction', NaN, ...
    'formationFirstDisconnectionTime', NaN, ...
    'initialTreeFirstFailureTime', NaN, ...
    'exactStaticRouteFirstFailureTime', NaN, ...
    'initialTreeFailureEpisodeCount', NaN, ...
    'qualification', '', ...
    'errorIdentifier', '', ...
    'errorMessage', '');
end

function presets = defaultPresets()
styles = { ...
    'convoy', 'relay', 'crossing', 'merge-split', ...
    'target-overlap', 'curved-corridor', ...
    'braided-handover', 'formation-braid'};
scales = {'m24', 'x36'};
presets = cell(1, numel(styles) * numel(scales));
cursor = 0;
for styleIdx = 1:numel(styles)
    for scaleIdx = 1:numel(scales)
        cursor = cursor + 1;
        presets{cursor} = sprintf('%s-formation-fov-%s', ...
            scales{scaleIdx}, styles{styleIdx});
    end
end
end

function writeReportFile(path, result)
fileId = fopen(path, 'w');
if fileId < 0
    error('DynamicTopologySceneStressV243:ReportOpenFailed', ...
        'Unable to open %s.', path);
end
cleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, '# V243 dynamic-topology scene stress survey\n\n');
fprintf(fileId, '- Seed: `%d`\n', result.seed);
fprintf(fileId, '- Tracking outcome authorized: `0`\n');
fprintf(fileId, '- Evidence boundary: %s\n\n', result.evidenceBoundary);
fprintf(fileId, [ ...
    '| Preset | Style | Valid | Formation connected | Initial tree ', ...
    'feasible | First tree failure | Failure episodes | Exact static ', ...
    'route physical | First exact-route failure | Qualification |\n']);
fprintf(fileId, [ ...
    '|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--|\n']);
for idx = 1:numel(result.records)
    record = result.records(idx);
    fprintf(fileId, ...
        ['| %s | %s | %d | %.3f | %.3f | %s | %s | %.3f | ', ...
         '%s | %s |\n'], ...
        record.presetName, record.sceneStyle, record.sceneValid, ...
        record.formationConnectedFraction, ...
        record.initialTreeFeasibleFraction, ...
        scalarText(record.initialTreeFirstFailureTime), ...
        scalarText(record.initialTreeFailureEpisodeCount), ...
        record.exactStaticRoutePhysicalFraction, ...
        scalarText(record.exactStaticRouteFirstFailureTime), ...
        record.qualification);
    if ~isempty(record.errorMessage)
        fprintf(fileId, '\n  - `%s`: %s\n', ...
            record.errorIdentifier, record.errorMessage);
    end
end
fprintf(fileId, '\n## Decision rule\n\n');
fprintf(fileId, [ ...
    '- `dynamic-tree-candidate`: the physical formation graph stays ', ...
    'connected, but the initially registered formation tree fails.\n']);
fprintf(fileId, [ ...
    '- `dynamic-gateway-candidate`: the formation tree remains feasible, ', ...
    'but one or more registered sensor gateway edges fail and must be ', ...
    'reassigned.\n']);
fprintf(fileId, [ ...
    '- `static-route-control`: useful for cross-style generalization, ', ...
    'but it cannot establish the value of dynamic repair.\n']);
fprintf(fileId, [ ...
    '- `physical-formation-disconnection`: requires store/forward or ', ...
    'intermittent-connectivity semantics beyond the current method.\n']);
fprintf(fileId, [ ...
    '- `scene-repair-required`: geometry or validation must be fixed ', ...
    'before any tracking comparison.\n']);
clear cleanup;
end

function text = scalarText(value)
if isfinite(value)
    text = sprintf('%g', value);
else
    text = '--';
end
end

function value = getField(structure, name, defaultValue)
if isstruct(structure) && isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
