function [reportPath, result] = ...
        analyzeDynamicRoutingSceneEligibilityV238(options)
% ANALYZEDYNAMICROUTINGSCENEELIGIBILITYV238 Topology-only scene screen.
%
% This screen separates sensing richness from actual communication-topology
% pressure. It never generates measurements or runs a tracking filter.

if nargin < 1 || isempty(options)
    options = struct();
end
presets = getField(options, 'presets', defaultPresets());
seeds = reshape(getField(options, 'seeds', 41), 1, []);
if ~iscell(presets) || isempty(presets) || ...
        ~isnumeric(seeds) || isempty(seeds) || ...
        any(~isfinite(seeds)) || any(seeds ~= round(seeds))
    error('DynamicRoutingSceneV238:InvalidOptions', ...
        'V238 requires nonempty preset and integer seed lists.');
end

records = repmat(emptyRecord(), 1, numel(presets) * numel(seeds));
cursor = 0;
for presetIndex = 1:numel(presets)
    for seedIndex = 1:numel(seeds)
        cursor = cursor + 1;
        fprintf('V238 scene %d/%d: %s seed %d\n', ...
            cursor, numel(records), presets{presetIndex}, ...
            seeds(seedIndex));
        records(cursor) = analyzeCase( ...
            presets{presetIndex}, seeds(seedIndex));
        item = records(cursor);
        fprintf(['  physical intersection/union %.3f/%.3f; ', ...
            'changes %d; t1 survival %.3f; genuine pressure %d\n'], ...
            item.physicalIntersectionFraction, ...
            item.physicalUnionFraction, item.physicalPageChangeCount, ...
            item.initialRouteSurvivalFraction, ...
            item.genuinePhysicalAdaptationPressure);
    end
end

result = struct();
result.contractVersion = ...
    'dynamic-routing-scene-eligibility-v238-result-v1';
result.generatedAt = datestr(now, 31);
gitState = resolveResearchGitState();
result.generationGitCommit = gitState.commit;
result.presets = presets;
result.seeds = seeds;
result.records = records;
result.caseCount = numel(records);
result.genuinePressureCaseCount = nnz( ...
    [records.genuinePhysicalAdaptationPressure]);
result.allRuntimeSelectorsTruthFree = ...
    all(~[records.initialSelectorTruthUsed]);
result.trackingOutcomeScored = false;
result.validationClaimAllowed = false;
result.developmentEvidenceOnly = true;
result.evidenceBoundary = [ ...
    'V238 is a geometry/link-schedule eligibility screen. It measures ', ...
    'physical graph variation and whether the causal t=1 V227 route ', ...
    'eventually becomes infeasible. It does not measure tracking value, ', ...
    'does not authorize a scene for paper results, and does not claim ', ...
    'that physical churn alone improves estimation.'];

outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v238', 'scene_eligibility')));
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'DYNAMIC_ROUTING_SCENE_ELIGIBILITY_V238.mat');
reportPath = fullfile(outputRoot, ...
    'DYNAMIC_ROUTING_SCENE_ELIGIBILITY_V238.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V238 scene eligibility: %s\n', reportPath);
end

function item = analyzeCase(presetName, seed)
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensorTrajectories, ~] = ...
    generateMultiFormationTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensorTrajectories);
[pDropByEdge, ~] = buildDynamicTopologyLinkSchedule(config, graph);
physical = logical(graph.physicalAdjacency);
nodeCount = config.numberOfSensors;
timeCount = config.simulationLength;
completeEdgeCount = nodeCount * (nodeCount - 1);
intersection = all(physical, 3);
unionGraph = any(physical, 3);
pageEdgeCounts = pageCounts(physical);
pageChanged = false(1, timeCount);
pageSymmetricDifference = zeros(1, timeCount);
for currentTime = 2:timeCount
    previous = physical(:, :, currentTime - 1);
    current = physical(:, :, currentTime);
    pageChanged(currentTime) = ~isequal(previous, current);
    pageSymmetricDifference(currentTime) = nnz(xor(previous, current));
end

formationPages = collapseToFormations( ...
    physical, config.sensorGroupIds);
formationConnected = false(1, timeCount);
for currentTime = 1:timeCount
    formationConnected(currentTime) = ...
        isConnected(formationPages(:, :, currentTime));
end

[initialRoute, initialDetails, selectionError] = ...
    selectInitialRoute(config, graph, pDropByEdge);
initialRouteSelected = isempty(selectionError);
initialRoutePhysicalByTime = false(1, timeCount);
if initialRouteSelected
    for currentTime = 1:timeCount
        initialRoutePhysicalByTime(currentTime) = ~any(any( ...
            initialRoute & ~physical(:, :, currentTime)));
    end
end
firstFailureTime = find(~initialRoutePhysicalByTime, 1);
if isempty(firstFailureTime) || ~initialRouteSelected
    firstFailureTime = NaN;
end

staticAllTimePhysical = all(all(all(repmat( ...
    logical(graph.staticAdjacency), 1, 1, timeCount) <= physical)));
item = emptyRecord();
item.presetName = presetName;
item.sceneStyle = getField(config, 'sceneStyle', config.variant);
item.seed = seed;
item.nodeCount = nodeCount;
item.formationCount = config.formationCount;
item.commRange = config.commRange;
item.registeredStaticEdgeCount = nnz(graph.staticAdjacency);
item.registeredStaticAllTimePhysical = staticAllTimePhysical;
item.completeDirectedEdgeCount = completeEdgeCount;
item.physicalIntersectionEdgeCount = nnz(intersection);
item.physicalUnionEdgeCount = nnz(unionGraph);
item.physicalIntersectionFraction = ...
    nnz(intersection) / max(completeEdgeCount, 1);
item.physicalUnionFraction = ...
    nnz(unionGraph) / max(completeEdgeCount, 1);
item.minimumPhysicalEdgeCount = min(pageEdgeCounts);
item.meanPhysicalEdgeCount = mean(pageEdgeCounts);
item.maximumPhysicalEdgeCount = max(pageEdgeCounts);
item.physicalPageChangeCount = nnz(pageChanged);
item.physicalPageChangeFraction = nnz(pageChanged) / max(timeCount - 1, 1);
item.meanPageSymmetricDifference = mean(pageSymmetricDifference(2:end));
item.allFormationPagesConnected = all(formationConnected);
item.minimumFormationDegree = minimumPageDegree(formationPages);
item.initialRouteSelected = initialRouteSelected;
item.initialRouteSelectionError = selectionError;
item.initialRouteEdgeCount = nnz(initialRoute);
item.initialRouteAllTimePhysical = ...
    initialRouteSelected && all(initialRoutePhysicalByTime);
item.initialRouteSurvivalFraction = ...
    mean(initialRoutePhysicalByTime);
item.initialRouteFirstFailureTime = firstFailureTime;
item.initialSelectorTruthUsed = getField( ...
    initialDetails, 'truthUsed', true);
item.initialSelectorFutureOutcomeUsed = getField( ...
    initialDetails, 'futureOutcomeUsed', true);
item.genuinePhysicalAdaptationPressure = ...
    initialRouteSelected && ~item.initialRouteAllTimePhysical && ...
    item.allFormationPagesConnected && item.physicalPageChangeCount > 0;
item.trackingOutcomeScored = false;
end

function [adjacency, details, errorText] = ...
        selectInitialRoute(config, graph, pDropByEdge)
nodeCount = config.numberOfSensors;
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
model = struct('dynamicTopologyScenario', struct( ...
    'config', config, ...
    'staticAdjacency', logical(graph.staticAdjacency)));
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = model;
context.baseAdjacency = logical(graph.staticAdjacency);
context.physicalAdjacency = logical(graph.physicalAdjacency(:, :, 1));
context.positions = graph.positions(:, :, 1);
context.currentTime = 1;
context.commConfig = struct('pDropByEdge', pDropByEdge(:, :, 1));
context.directedMessageBudget = 2 * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
adjacency = false(nodeCount);
details = struct();
errorText = '';
try
    [adjacency, details] = ...
        selectCorrectedDynamicFormationBackboneV227Policy(context);
catch errorInfo
    errorText = sprintf('%s: %s', errorInfo.identifier, errorInfo.message);
end
end

function pages = collapseToFormations(sensorPages, groupIds)
groups = unique(reshape(groupIds, 1, []), 'stable');
timeCount = size(sensorPages, 3);
pages = false(numel(groups), numel(groups), timeCount);
for currentTime = 1:timeCount
    sensor = sensorPages(:, :, currentTime);
    for receiverGroup = 1:numel(groups)
        receivers = groupIds == groups(receiverGroup);
        for senderGroup = 1:numel(groups)
            if receiverGroup == senderGroup
                continue;
            end
            senders = groupIds == groups(senderGroup);
            pages(receiverGroup, senderGroup, currentTime) = ...
                any(any(sensor(receivers, senders)));
        end
    end
end
end

function minimum = minimumPageDegree(pages)
minimum = inf;
for currentTime = 1:size(pages, 3)
    minimum = min(minimum, min(sum(pages(:, :, currentTime), 2)));
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
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
connected = all(visited);
end

function counts = pageCounts(pages)
counts = zeros(1, size(pages, 3));
for currentTime = 1:size(pages, 3)
    counts(currentTime) = nnz(pages(:, :, currentTime));
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('DynamicRoutingSceneV238:ReportWriteFailed', ...
        'Could not create the V238 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V238 dynamic-routing scene eligibility\n\n');
fprintf(fid, '- Source commit: `%s`\n', result.generationGitCommit);
fprintf(fid, '- Seeds: `%s`\n', mat2str(result.seeds));
fprintf(fid, '- Genuine-pressure cases: `%d / %d`\n\n', ...
    result.genuinePressureCaseCount, result.caseCount);
fprintf(fid, ['| Preset | Style | N | Comm range | Static edges | ', ...
    'Physical intersection / union | Page changes | Formation connected | ', ...
    't1 route survival | First failure | Genuine pressure |\n']);
fprintf(fid, '|:--|:--|--:|--:|--:|--:|--:|:--:|--:|--:|:--:|\n');
for item = result.records
    fprintf(fid, ['| %s | %s | %d | %.0f | %d | %.3f / %.3f | ', ...
        '%d | `%d` | %.3f | %s | `%d` |\n'], ...
        item.presetName, item.sceneStyle, item.nodeCount, ...
        item.commRange, item.registeredStaticEdgeCount, ...
        item.physicalIntersectionFraction, item.physicalUnionFraction, ...
        item.physicalPageChangeCount, item.allFormationPagesConnected, ...
        item.initialRouteSurvivalFraction, ...
        numberText(item.initialRouteFirstFailureTime), ...
        item.genuinePhysicalAdaptationPressure);
end
fprintf(fid, '\n## Interpretation\n\n');
fprintf(fid, ['A scene has genuine physical adaptation pressure only when ', ...
    'the causal t=1 route later becomes physically infeasible while the ', ...
    'formation graph remains connected. A changing physical graph whose ', ...
    'selected t=1 route survives the full episode cannot, by itself, ', ...
    'identify the value of temporal route reselection.\n']);
fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function text = numberText(value)
if isfinite(value)
    text = sprintf('%d', value);
else
    text = '-';
end
end

function presets = defaultPresets()
styles = {'', '-convoy', '-relay', '-crossing', '-merge-split', ...
    '-curved-corridor', '-braided-handover'};
presets = cell(1, 2 * numel(styles));
cursor = 0;
for scale = {'m24', 'x36'}
    for style = styles
        cursor = cursor + 1;
        presets{cursor} = [scale{1}, '-formation-fov', style{1}];
    end
end
end

function item = emptyRecord()
item = struct('presetName', '', 'sceneStyle', '', 'seed', NaN, ...
    'nodeCount', NaN, 'formationCount', NaN, 'commRange', NaN, ...
    'registeredStaticEdgeCount', NaN, ...
    'registeredStaticAllTimePhysical', false, ...
    'completeDirectedEdgeCount', NaN, ...
    'physicalIntersectionEdgeCount', NaN, ...
    'physicalUnionEdgeCount', NaN, ...
    'physicalIntersectionFraction', NaN, ...
    'physicalUnionFraction', NaN, ...
    'minimumPhysicalEdgeCount', NaN, ...
    'meanPhysicalEdgeCount', NaN, ...
    'maximumPhysicalEdgeCount', NaN, ...
    'physicalPageChangeCount', NaN, ...
    'physicalPageChangeFraction', NaN, ...
    'meanPageSymmetricDifference', NaN, ...
    'allFormationPagesConnected', false, ...
    'minimumFormationDegree', NaN, ...
    'initialRouteSelected', false, ...
    'initialRouteSelectionError', '', ...
    'initialRouteEdgeCount', NaN, ...
    'initialRouteAllTimePhysical', false, ...
    'initialRouteSurvivalFraction', NaN, ...
    'initialRouteFirstFailureTime', NaN, ...
    'initialSelectorTruthUsed', true, ...
    'initialSelectorFutureOutcomeUsed', true, ...
    'genuinePhysicalAdaptationPressure', false, ...
    'trackingOutcomeScored', false);
end

function value = getField(structure, name, defaultValue)
if isstruct(structure) && isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
