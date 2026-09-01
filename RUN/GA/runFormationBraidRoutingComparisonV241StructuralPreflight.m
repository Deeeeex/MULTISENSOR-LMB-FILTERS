function [reportPath, result] = ...
        runFormationBraidRoutingComparisonV241StructuralPreflight(options)
% RUNFORMATIONBRAIDROUTINGCOMPARISONV241STRUCTURALPREFLIGHT Three arms.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationBraidRoutingComparisonV241Protocol();
presetName = getField(options, 'presetName', ...
    'm24-formation-fov-formation-braid');
seed = getField(options, 'seed', protocol.allowedSeeds(1));
writeReport = getField(options, 'writeReport', true);
if ~ischar(presetName) || ...
        ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds) || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('FormationBraidRoutingV241:InvalidPreflightOptions', ...
        'The V241 preflight options are invalid.');
end

rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensors, ~] = generateMultiFormationTrajectories(config);
[targets, ~] = generateCorridorTargetTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensors);
validation = validateDynamicTopologyScenario(config, sensors, targets, ...
    graph, struct('throwOnInvalid', false));
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
registration = initialRegistration( ...
    config, graph, pDrop, identity);

armNames = {'static-dropout', 'always-replan', 'causal'};
arms = repmat(emptyArm(), 1, numel(armNames));
for armIdx = 1:numel(armNames)
    arms(armIdx) = replayArm(armNames{armIdx}, config, graph, ...
        pDrop, identity, registration);
end

static = arms(1);
always = arms(2);
causal = arms(3);
result = struct();
result.contractVersion = ...
    'formation-braid-routing-comparison-v241-structural-result-v1';
result.generatedAt = datestr(now, 31);
result.protocol = protocol;
result.presetName = presetName;
result.seed = seed;
result.sceneValid = validation.isValid;
result.sceneHardFailures = validation.hardFailures;
result.arms = arms;
result.gatePassed = validation.isValid && ...
    static.physicalAllTimes && static.dropMessageCount > 0 && ...
    static.strongFraction < 1 && static.treeChangeCount == 0 && ...
    always.physicalAllTimes && always.messageParityAllTimes && ...
    always.strongFraction == 1 && ...
    causal.physicalAllTimes && causal.messageParityAllTimes && ...
    causal.strongFraction == 1 && causal.treeReselectionCount > 0 && ...
    causal.treeReselectionOnlyOnInfeasibility && ...
    causal.treeChangeCount <= always.treeChangeCount;
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = protocol.evidenceBoundary;

reportPath = '';
if writeReport
    outputDirectory = fullfile('RUN', 'GA', 'dynamic_topology', ...
        'evidence', 'tracking_aligned_v241', 'structural_preflight');
    if exist(outputDirectory, 'dir') ~= 7
        mkdir(outputDirectory);
    end
    reportPath = fullfile(outputDirectory, sprintf( ...
        'FORMATION_BRAID_ROUTING_V241_%s_STRUCTURAL.md', ...
        upper(strrep(presetName, '-', '_'))));
    matPath = strrep(reportPath, '.md', '.mat');
    result.reportPath = reportPath;
    result.matPath = matPath;
    writeReportFile(reportPath, result);
    save(matPath, 'result');
end
if ~result.gatePassed
    error('FormationBraidRoutingV241:StructuralGateFailed', ...
        'The V241 matched structural comparison failed.');
end
fprintf(['V241 structural %s: static messages=%d..%d strong=%.3f; ', ...
    'always changes=%d; causal changes=%d reselections=%s\n'], ...
    presetName, static.minimumMessages, static.maximumMessages, ...
    static.strongFraction, always.treeChangeCount, ...
    causal.treeChangeCount, mat2str(causal.treeReselectionTimes));
fprintf('FORMATION_BRAID_ROUTING_V241_STRUCTURAL_PASSED=1\n');
end

function registration = initialRegistration(config, graph, pDrop, identity)
context = buildContext(config, graph, pDrop, identity, 1, ...
    false(config.numberOfSensors, config.numberOfSensors, 0), ...
    zeros(1, 0));
[adjacency, details] = ...
    selectCausalMinimalEditFormationTreeV240Policy(context);
registration = struct('contractVersion', ...
    'formation-braid-v241-initial-route-registration-v1', ...
    'adjacency', logical(adjacency), ...
    'fusionWeightMatrix', details.fusionWeightMatrix, ...
    'selectionDetails', details, 'messageCount', nnz(adjacency), ...
    'selectionTime', 1, 'targetTruthUsed', false, ...
    'futurePhysicalPageUsed', false);
end

function arm = replayArm(name, config, graph, pDrop, identity, registration)
nodeCount = config.numberOfSensors;
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
previousTree = false(config.formationCount);
messages = zeros(1, config.simulationLength);
strong = false(1, config.simulationLength);
physical = false(1, config.simulationLength);
parity = false(1, config.simulationLength);
treeChangeTimes = zeros(1, 0);
reselectionTimes = zeros(1, 0);
dropMessages = zeros(1, config.simulationLength);
reselectionOnlyOnInfeasibility = true;
for currentTime = 1:config.simulationLength
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes);
    switch name
        case 'static-dropout'
            [adjacency, details] = ...
                selectFrozenFormationBraidDropoutV241Policy( ...
                    context, registration);
        case 'always-replan'
            [adjacency, details] = ...
                selectAlwaysReplanFormationTreeV241Policy(context);
        case 'causal'
            [adjacency, details] = ...
                selectCausalMinimalEditFormationTreeV240Policy(context);
        otherwise
            error('Unknown V241 structural arm: %s.', name);
    end
    if ~isscalar(getField(details, 'objective', NaN))
        error('FormationBraidRoutingV241:NonScalarDiagnostic', ...
            'The topology objective must fit the scalar runtime trace.');
    end
    currentPhysical = logical( ...
        graph.physicalAdjacency(:, :, currentTime));
    messages(currentTime) = nnz(adjacency);
    strong(currentTime) = isStronglyConnected(adjacency);
    physical(currentTime) = ...
        ~any(adjacency(:) & ~currentPhysical(:));
    parity(currentTime) = nnz(adjacency) == 2 * nodeCount;
    dropMessages(currentTime) = ...
        getField(details, 'dropoutMessageCount', 0);
    if isfield(details, 'currentFormationTreeAdjacency')
        tree = logical(details.currentFormationTreeAdjacency);
        if currentTime > 1 && ~isequal(tree, previousTree)
            treeChangeTimes(end + 1) = currentTime; %#ok<AGROW>
        end
        previousTree = tree;
    end
    if isfield(details, 'treeReselectionUsed') && ...
            details.treeReselectionUsed
        reselectionTimes(end + 1) = currentTime; %#ok<AGROW>
        reselectionOnlyOnInfeasibility = ...
            reselectionOnlyOnInfeasibility && ...
            (getField(details, ...
                'physicalInfeasibilityFallbackUsed', false) || ...
             getField(details, ...
                'assignmentInfeasibilityFallbackUsed', false) || ...
             strcmp(name, 'always-replan'));
    end
    history = cat(3, history, adjacency);
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
end
arm = emptyArm();
arm.name = name;
arm.minimumMessages = min(messages);
arm.maximumMessages = max(messages);
arm.messageParityAllTimes = all(parity);
arm.strongFraction = mean(strong);
arm.physicalAllTimes = all(physical);
arm.treeChangeTimes = treeChangeTimes;
arm.treeChangeCount = numel(treeChangeTimes);
arm.treeReselectionTimes = reselectionTimes;
arm.treeReselectionCount = numel(reselectionTimes);
arm.treeReselectionOnlyOnInfeasibility = ...
    reselectionOnlyOnInfeasibility;
arm.dropMessageCount = sum(dropMessages);
arm.firstDropTime = findFirst(dropMessages > 0);
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

function value = findFirst(mask)
value = find(mask, 1);
if isempty(value), value = NaN; end
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write V241 structural report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V241 formation-braid matched structural comparison\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Scene valid: `%d`\n', result.sceneValid);
fprintf(fid, '- Structural gate passed: `%d`\n', result.gatePassed);
fprintf(fid, '- Tracking outcome authorized: `0`\n\n');
fprintf(fid, ['| Arm | Messages min--max | Strong fraction | Physical | ', ...
    'Tree changes | Reselection times | First fixed-tree dropout | ', ...
    'Dropped route-messages |\n']);
fprintf(fid, '|:--|:--:|--:|:--:|--:|:--|--:|--:|\n');
for arm = result.arms
    fprintf(fid, '| %s | %d--%d | %.3f | %d | %d | %s | %g | %d |\n', ...
        arm.name, arm.minimumMessages, arm.maximumMessages, ...
        arm.strongFraction, arm.physicalAllTimes, ...
        arm.treeChangeCount, formatReselections(arm), ...
        arm.firstDropTime, arm.dropMessageCount);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function value = formatReselections(arm)
if strcmp(arm.name, 'always-replan')
    value = sprintf('every step (%d)', arm.treeReselectionCount);
else
    value = mat2str(arm.treeReselectionTimes);
end
end

function arm = emptyArm()
arm = struct('name', '', 'minimumMessages', NaN, ...
    'maximumMessages', NaN, 'messageParityAllTimes', false, ...
    'strongFraction', NaN, 'physicalAllTimes', false, ...
    'treeChangeTimes', zeros(1, 0), 'treeChangeCount', NaN, ...
    'treeReselectionTimes', zeros(1, 0), ...
    'treeReselectionCount', NaN, ...
    'treeReselectionOnlyOnInfeasibility', false, ...
    'dropMessageCount', NaN, 'firstDropTime', NaN);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
