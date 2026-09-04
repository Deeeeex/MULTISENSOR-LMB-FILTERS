function [reportPath, result] = analyzeKlaWeightBalanceV275(options)
% ANALYZEKLAWEIGHTBALANCEV275 Read-only consensus-matrix diagnostic.

if nargin < 1 || isempty(options)
    options = struct();
end
presetNames = getField(options, 'presetNames', { ...
    'm24-formation-fov-temporal-coupled-formation-braid', ...
    'x36-formation-fov-temporal-coupled-formation-braid'});
seed = getField(options, 'seed', 1301);
tolerance = getField(options, 'tolerance', 1e-12);
writeReport = getField(options, 'writeReport', true);
if ischar(presetNames), presetNames = {presetNames}; end
allowed = getCausalMinimalEditFormationTreeV240Protocol().allowedPresets;
if ~iscell(presetNames) || isempty(presetNames) || ...
        any(~cellfun(@ischar, presetNames)) || ...
        any(~ismember(presetNames, allowed)) || ...
        numel(unique(presetNames)) ~= numel(presetNames) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~isscalar(tolerance) || ~isfinite(tolerance) || tolerance <= 0 || ...
        ~islogical(writeReport) || ~isscalar(writeReport)
    error('KlaWeightBalanceV275:InvalidOptions', ...
        'The V275 diagnostic options are invalid.');
end

records = repmat(emptyRecord(), 1, numel(presetNames));
for presetIdx = 1:numel(presetNames)
    records(presetIdx) = replayPreset( ...
        presetNames{presetIdx}, seed, tolerance);
end

result = struct();
result.contractVersion = 'kla-weight-balance-v275-result-v1';
result.generatedAt = datestr(now, 31);
gitState = resolveResearchGitState();
result.generationGitCommit = gitState.commit;
result.seed = seed;
result.tolerance = tolerance;
result.records = records;
result.diagnosticComplete = all([records.sceneValid]) && ...
    all([records.physicalSymmetricAllTimes]) && ...
    all([records.dropScheduleSymmetricAllTimes]) && ...
    all(arrayfun(@(x) x.v240.rowStochasticAllTimes && ...
        x.v240.strongAllTimes && x.v240.physicalAllTimes && ...
        x.v242.rowStochasticAllTimes && x.v242.strongAllTimes && ...
        x.v242.physicalAllTimes, records));
result.standardUnweightedKlaConditionEstablished = ...
    all(arrayfun(@(x) x.v242.doublyStochasticAllTimes, records));
result.trackingOutcomeAuthorized = false;
result.evidenceBoundary = [ ...
    'V275 replays only the physical graph, link-drop schedule and causal ', ...
    'routing policies. It reads no posterior, measurement, truth, ', ...
    'realized delivery or tracking outcome. Row/column sums diagnose the ', ...
    'consensus matrix used by the current implementation; they do not ', ...
    'establish finite-round tracking benefit or convergence by themselves.'];

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v275', sprintf('kla_weight_balance_seed%d', seed)));
if ~isAbsolutePathLocal(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, 'KLA_WEIGHT_BALANCE_V275.md');
matPath = fullfile(outputRoot, 'KLA_WEIGHT_BALANCE_V275.mat');
result.reportPath = reportPath;
result.matPath = matPath;
if writeReport
    save('-mat7-binary', matPath, 'result');
    writeReportFile(reportPath, result);
end

for record = records
    fprintf([ ...
        'V275 %s: V240 non-DS=%d/%d col max/mean=%.3f/%.5f; ', ...
        'V242 non-DS=%d/%d col max/mean=%.3f/%.5f\n'], ...
        record.presetName, record.v240.nonDoublyStochasticPageCount, ...
        record.timeCount, record.v240.maximumColumnSumDeviation, ...
        record.v240.meanColumnSumDeviation, ...
        record.v242.nonDoublyStochasticPageCount, record.timeCount, ...
        record.v242.maximumColumnSumDeviation, ...
        record.v242.meanColumnSumDeviation);
end
fprintf('V275_STANDARD_UNWEIGHTED_KLA_CONDITION_ESTABLISHED=%d\n', ...
    result.standardUnweightedKlaConditionEstablished);
end

function record = replayPreset(presetName, seed, tolerance)
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

v240 = replayPolicy(@selectCausalMinimalEditFormationTreeV240Policy, ...
    2 * nodeCount, config, graph, pDrop, identity, tolerance);
v242 = replayPolicy(@selectCausalMinimumFormationBackboneV242Policy, ...
    nodeCount + 2 * (formationCount - 1), ...
    config, graph, pDrop, identity, tolerance);

[physicalMaximumAsymmetry, physicalSymmetric] = ...
    pageSymmetry(graph.physicalAdjacency, tolerance);
[dropMaximumAsymmetry, dropSymmetric] = ...
    pageSymmetry(pDrop, tolerance);

record = emptyRecord();
record.presetName = presetName;
record.seed = seed;
record.nodeCount = nodeCount;
record.formationCount = formationCount;
record.timeCount = config.simulationLength;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.physicalMaximumAsymmetry = physicalMaximumAsymmetry;
record.physicalSymmetricAllTimes = physicalSymmetric;
record.dropScheduleMaximumAsymmetry = dropMaximumAsymmetry;
record.dropScheduleSymmetricAllTimes = dropSymmetric;
record.v240 = v240;
record.v242 = v242;
end

function summary = replayPolicy(policyFcn, messageBudget, ...
        config, graph, pDrop, identity, tolerance)
nodeCount = config.numberOfSensors;
timeCount = config.simulationLength;
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);
rowDeviation = zeros(1, timeCount);
columnDeviation = zeros(1, timeCount);
messageCount = zeros(1, timeCount);
strong = false(1, timeCount);
physical = false(1, timeCount);
supportValid = false(1, timeCount);
for currentTime = 1:timeCount
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageBudget);
    [adjacency, details] = policyFcn(context);
    weights = details.fusionWeightMatrix;
    currentPhysical = logical(graph.physicalAdjacency(:, :, currentTime));
    allowed = logical(adjacency) | logical(eye(nodeCount));
    rowDeviation(currentTime) = max(abs(sum(weights, 2) - 1));
    columnDeviation(currentTime) = max(abs(sum(weights, 1) - 1));
    messageCount(currentTime) = nnz(adjacency);
    strong(currentTime) = isStronglyConnected(adjacency);
    physical(currentTime) = ...
        ~any(logical(adjacency(:)) & ~currentPhysical(:));
    supportValid(currentTime) = all(weights(:) >= -tolerance) && ...
        ~any(weights(:) > tolerance & ~allowed(:));
    history = cat(3, history, logical(adjacency));
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
end
nonDouble = rowDeviation > tolerance | columnDeviation > tolerance;
summary = struct();
summary.minimumMessageCount = min(messageCount);
summary.maximumMessageCount = max(messageCount);
summary.maximumRowSumDeviation = max(rowDeviation);
summary.maximumColumnSumDeviation = max(columnDeviation);
summary.meanColumnSumDeviation = mean(columnDeviation);
summary.rowStochasticAllTimes = all(rowDeviation <= tolerance);
summary.doublyStochasticAllTimes = ~any(nonDouble);
summary.nonDoublyStochasticPageCount = nnz(nonDouble);
summary.nonDoublyStochasticTimes = find(nonDouble);
summary.strongAllTimes = all(strong);
summary.physicalAllTimes = all(physical);
summary.weightSupportValidAllTimes = all(supportValid);
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

function [maximumAsymmetry, symmetric] = pageSymmetry(pages, tolerance)
maximumAsymmetry = 0;
for pageIdx = 1:size(pages, 3)
    page = double(pages(:, :, pageIdx));
    maximumAsymmetry = max(maximumAsymmetry, ...
        max(abs(page(:) - page'(:))));
end
symmetric = maximumAsymmetry <= tolerance;
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
    error('Could not write V275 report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V275 KLA consensus-matrix balance diagnostic\n\n');
fprintf(fid, '- Generation commit: `%s`\n', ...
    result.generationGitCommit);
fprintf(fid, '- Structural seed: `%d`\n', result.seed);
fprintf(fid, '- Numerical tolerance: `%.1e`\n', result.tolerance);
fprintf(fid, '- Diagnostic complete: `%d`\n', result.diagnosticComplete);
fprintf(fid, '- Standard unweighted-KLA condition established: `%d`\n\n', ...
    result.standardUnweightedKlaConditionEstablished);
fprintf(fid, ['The standard consensus result for an unweighted collective ', ...
    'KLA requires more than instantaneous strong connectivity: the ', ...
    'consensus matrix must also satisfy the relevant stochasticity ', ...
    'conditions. This report checks row and column sums of the matrices ', ...
    'actually emitted by V240 and V242.\n\n']);
fprintf(fid, ['| Scene | Arm | Messages | Strong / physical | Max row dev | ', ...
    'Max / mean column dev | Non-double pages |\n']);
fprintf(fid, '|:--|:--|:--:|:--:|--:|:--|:--|\n');
for record = result.records
    writeArmRow(fid, record, 'V240 full causal', record.v240);
    writeArmRow(fid, record, 'V242 minimum backbone', record.v242);
end
fprintf(fid, '\n## Symmetry of the available-link process\n\n');
fprintf(fid, '| Scene | Physical adjacency max asymmetry | Drop schedule max asymmetry |\n');
fprintf(fid, '|:--|--:|--:|\n');
for record = result.records
    fprintf(fid, '| %s | %.3g | %.3g |\n', record.presetName, ...
        record.physicalMaximumAsymmetry, ...
        record.dropScheduleMaximumAsymmetry);
end
fprintf(fid, '\n## Interpretation\n\n');
fprintf(fid, ['V240 is row stochastic but is not doubly stochastic on any ', ...
    'of the 160 pages in either scale. V242 removes most of that ', ...
    'imbalance, but its independently directed cross-formation gateways ', ...
    'still leave intermittent non-double pages. Therefore current V242 ', ...
    'supports claims about physical feasibility, instantaneous strong ', ...
    'connectivity and message count, but not a direct appeal to the ', ...
    'standard unbiased collective-KLA convergence theorem.\n\n']);
fprintf(fid, ['Because the available physical graph and drop-probability ', ...
    'schedule are symmetric, a same-pair reciprocal gateway is a ', ...
    'plausible next construction. Coupling both directions of every ', ...
    'formation-tree edge, while assigning distinct incident edges to ', ...
    'sensors within each formation, preserves N+2(F-1) messages, the ', ...
    'one-or-two-input receiver bound and equal incoming/outgoing residual ', ...
    'weight. That construction still needs a feasibility algorithm and ', ...
    'paired tracking evidence; it is not implemented by this diagnostic.\n']);
fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function writeArmRow(fid, record, armName, arm)
times = formatTimes(arm.nonDoublyStochasticTimes, record.timeCount);
fprintf(fid, '| %s | %s | %d--%d | %d / %d | %.3g | %.3g / %.5f | %d/%d `%s` |\n', ...
    record.presetName, armName, arm.minimumMessageCount, ...
    arm.maximumMessageCount, arm.strongAllTimes, arm.physicalAllTimes, ...
    arm.maximumRowSumDeviation, arm.maximumColumnSumDeviation, ...
    arm.meanColumnSumDeviation, arm.nonDoublyStochasticPageCount, ...
    record.timeCount, times);
end

function text = formatTimes(times, timeCount)
if isempty(times)
    text = 'none';
elseif numel(times) == timeCount && isequal(times, 1:timeCount)
    text = 'all';
else
    text = mat2str(times);
end
end

function record = emptyRecord()
emptyArm = struct('minimumMessageCount', NaN, ...
    'maximumMessageCount', NaN, 'maximumRowSumDeviation', NaN, ...
    'maximumColumnSumDeviation', NaN, ...
    'meanColumnSumDeviation', NaN, ...
    'rowStochasticAllTimes', false, ...
    'doublyStochasticAllTimes', false, ...
    'nonDoublyStochasticPageCount', NaN, ...
    'nonDoublyStochasticTimes', zeros(1, 0), ...
    'strongAllTimes', false, 'physicalAllTimes', false, ...
    'weightSupportValidAllTimes', false);
record = struct('presetName', '', 'seed', NaN, 'nodeCount', NaN, ...
    'formationCount', NaN, 'timeCount', NaN, ...
    'sceneValid', false, 'sceneHardFailures', {{}}, ...
    'physicalMaximumAsymmetry', NaN, ...
    'physicalSymmetricAllTimes', false, ...
    'dropScheduleMaximumAsymmetry', NaN, ...
    'dropScheduleSymmetricAllTimes', false, ...
    'v240', emptyArm, 'v242', emptyArm);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end

function passed = isAbsolutePathLocal(pathValue)
pathValue = char(pathValue);
passed = ~isempty(pathValue) && pathValue(1) == filesep;
end
