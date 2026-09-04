function [reportPath, result] = analyzeTemporalGatewayCoverageV271(options)
% ANALYZETEMPORALGATEWAYCOVERAGEV271 Diagnose temporal V242 concentration.

if nargin < 1 || isempty(options)
    options = struct();
end
presetNames = getField(options, 'presetNames', { ...
    'm24-formation-fov-temporal-coupled-formation-braid', ...
    'x36-formation-fov-temporal-coupled-formation-braid'});
seed = getField(options, 'seed', 1301);
horizonGrid = reshape(getField(options, 'horizonGrid', [3, 6, 12]), 1, []);
eventWindow = reshape(getField(options, 'eventWindow', 58:73), 1, []);
eventFormationIndex = getField(options, 'eventFormationIndex', 4);
trackingResultPath = char(getField(options, 'trackingResultPath', ''));
writeReport = logical(getField(options, 'writeReport', true));

if ischar(presetNames), presetNames = {presetNames}; end
if ~iscell(presetNames) || isempty(presetNames) || ...
        any(~cellfun(@ischar, presetNames)) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        any(~isfinite(horizonGrid)) || any(horizonGrid < 1) || ...
        any(horizonGrid ~= round(horizonGrid)) || ...
        any(~isfinite(eventWindow)) || any(eventWindow < 1) || ...
        any(eventWindow ~= round(eventWindow)) || ...
        ~isscalar(eventFormationIndex) || ...
        eventFormationIndex < 1 || ...
        eventFormationIndex ~= round(eventFormationIndex) || ...
        ~isscalar(writeReport)
    error('TemporalGatewayCoverageV271:InvalidOptions', ...
        'The V271 diagnostic options are invalid.');
end

protocol = getCausalMinimumFormationBackboneV242Protocol();
if any(~ismember(presetNames, protocol.allowedPresets))
    error('TemporalGatewayCoverageV271:UnregisteredPreset', ...
        'V271 requires registered V242 scenarios.');
end

records = repmat(emptyRecord(), 1, numel(presetNames));
for presetIdx = 1:numel(presetNames)
    records(presetIdx) = replayPreset( ...
        presetNames{presetIdx}, seed, horizonGrid);
end

event = emptyEvent();
if ~isempty(trackingResultPath)
    event = alignM24TrackingEvent( ...
        records, trackingResultPath, eventWindow, eventFormationIndex);
end

result = struct();
result.contractVersion = ...
    'temporal-gateway-coverage-v271-result-v1';
result.generatedAt = datestr(now, 31);
gitState = resolveResearchGitState();
result.generationGitCommit = gitState.commit;
result.seed = seed;
result.horizonGrid = horizonGrid;
result.records = records;
result.event = event;
result.allScaleConcentrationPassed = ...
    all([records.concentrationGatePassed]);
result.allScaleMixingHeadroomPassed = ...
    all([records.mixingHeadroomPassed]);
result.eventImbalancePassed = event.available && ...
    event.imbalanceGatePassed;
result.v272Authorized = result.allScaleConcentrationPassed && ...
    result.allScaleMixingHeadroomPassed && ...
    result.eventImbalancePassed;
result.nextDecision = ternary(result.v272Authorized, ...
    'implement-v272-temporally-balanced-minimum-backbone', ...
    'close-temporal-gateway-balancing-before-tracking');
result.evidenceBoundary = [ ...
    'V271 replays the causal V242 route without changing posterior state, ', ...
    'measurements, fusion weights, messages or random streams. M24 ', ...
    'tracking outcomes are joined only after route construction. The ', ...
    'diagnostic may authorize one development action family; it is not a ', ...
    'tracking, deployment, validation or generalization result.'];

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v271', 'temporal_gateway_coverage')));
if ~isAbsolutePathLocal(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'TEMPORAL_GATEWAY_COVERAGE_V271.md');
matPath = fullfile(outputRoot, ...
    'TEMPORAL_GATEWAY_COVERAGE_V271.mat');
result.reportPath = reportPath;
result.matPath = matPath;
if writeReport
    save('-mat7-binary', matPath, 'result');
    writeReportFile(reportPath, result);
end

for record = records
    fprintf(['V271 %s: coverage min %.3f, receiver share max %.3f, ', ...
        'age max %.1f, H%d mean/max TV %.3f/%.3f, gate %d\n'], ...
        record.presetName, min(record.crossReceiverCoverageByFormation), ...
        max(record.maximumCrossReceiverShareByFormation), ...
        max(record.maximumDirectInputAgeByFormation), ...
        record.horizonGrid(end), record.meanRowTvByHorizon(end), ...
        record.maximumRowTvByHorizon(end), ...
        record.concentrationGatePassed);
end
if event.available
    fprintf(['V271 M24 F%d t%d-%d: age/RMSE corr %.3f, ', ...
        'age range %.3f, direct fraction range %.3f, gate %d\n'], ...
        event.formationIndex, event.window(1), event.window(end), ...
        event.ageRmseCorrelation, rangeLocal(event.meanDirectInputAge), ...
        rangeLocal(event.directInputFraction), event.imbalanceGatePassed);
end
fprintf('TEMPORAL_GATEWAY_COVERAGE_V271_AUTHORIZED=%d\n', ...
    result.v272Authorized);
end

function record = replayPreset(presetName, seed, horizonGrid)
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
groupIds = reshape(config.sensorGroupIds, 1, []);
groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
timeCount = config.simulationLength;
messageBudget = nodeCount + 2 * (formationCount - 1);

crossReceiverByTime = false(nodeCount, timeCount);
crossSenderByTime = false(nodeCount, timeCount);
localInjectionDistanceByTime = nan(nodeCount, timeCount);
fusionWeightsByTime = zeros(nodeCount, nodeCount, timeCount);
messageCountByTime = zeros(1, timeCount);
history = false(nodeCount, nodeCount, 0);
historyTimes = zeros(1, 0);

for currentTime = 1:timeCount
    context = buildContext(config, graph, pDrop, identity, ...
        currentTime, history, historyTimes, messageBudget);
    [adjacency, details] = ...
        selectCausalMinimumFormationBackboneV242Policy(context);
    cross = logical(details.crossResidualAdjacency);
    crossReceiverByTime(:, currentTime) = any(cross, 2);
    crossSenderByTime(:, currentTime) = any(cross, 1)';
    fusionWeightsByTime(:, :, currentTime) = ...
        details.fusionWeightMatrix;
    messageCountByTime(currentTime) = nnz(adjacency);

    for formationIdx = 1:formationCount
        members = find(groupIds == groupLabels(formationIdx));
        injections = members(crossReceiverByTime(members, currentTime));
        localInjectionDistanceByTime(members, currentTime) = ...
            directedDistancesFromSources( ...
                details.dominantAdjacency, members, injections);
    end

    history = cat(3, history, adjacency);
    historyTimes(end + 1) = currentTime; %#ok<AGROW>
    if size(history, 3) > 2
        history = history(:, :, end-1:end);
        historyTimes = historyTimes(end-1:end);
    end
end

directInputAgeByTime = ageSinceLastInput(crossReceiverByTime);
coverage = zeros(1, formationCount);
maximumShare = zeros(1, formationCount);
meanAge = zeros(1, formationCount);
maximumAge = zeros(1, formationCount);
meanDistance = zeros(1, formationCount);
maximumDistance = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = find(groupIds == groupLabels(formationIdx));
    counts = sum(crossReceiverByTime(members, :), 2);
    formationAge = directInputAgeByTime(members, :);
    formationDistance = localInjectionDistanceByTime(members, :);
    coverage(formationIdx) = nnz(counts > 0) / numel(members);
    maximumShare(formationIdx) = max(counts) / max(1, sum(counts));
    meanAge(formationIdx) = mean(formationAge(:));
    maximumAge(formationIdx) = max(formationAge(:));
    meanDistance(formationIdx) = mean(formationDistance(:));
    maximumDistance(formationIdx) = max(formationDistance(:));
end
[meanTv, maximumTv] = mixingDiagnostics( ...
    fusionWeightsByTime, horizonGrid);

record = emptyRecord();
record.presetName = presetName;
record.nodeCount = nodeCount;
record.formationCount = formationCount;
record.timeCount = timeCount;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.sensorPhysicalUids = identity.sensorPhysicalUids;
record.groupIds = groupIds;
record.groupLabels = groupLabels;
record.crossReceiverByTime = crossReceiverByTime;
record.crossSenderByTime = crossSenderByTime;
record.crossReceiverCountBySensor = sum(crossReceiverByTime, 2)';
record.crossSenderCountBySensor = sum(crossSenderByTime, 2)';
record.directInputAgeByTime = directInputAgeByTime;
record.localInjectionDistanceByTime = localInjectionDistanceByTime;
record.crossReceiverCoverageByFormation = coverage;
record.maximumCrossReceiverShareByFormation = maximumShare;
record.meanDirectInputAgeByFormation = meanAge;
record.maximumDirectInputAgeByFormation = maximumAge;
record.meanLocalInjectionDistanceByFormation = meanDistance;
record.maximumLocalInjectionDistanceByFormation = maximumDistance;
record.horizonGrid = horizonGrid;
record.meanRowTvByHorizon = meanTv;
record.maximumRowTvByHorizon = maximumTv;
record.messageCountMinimum = min(messageCountByTime);
record.messageCountMaximum = max(messageCountByTime);
record.messageParityPassed = ...
    all(messageCountByTime == messageBudget);
record.concentrationGatePassed = any( ...
    coverage <= 0.5 | maximumShare >= 0.5);
record.mixingHeadroomPassed = meanTv(end) > 0.05;
record.gatePassed = validation.isValid && ...
    record.messageParityPassed && ...
    record.concentrationGatePassed && record.mixingHeadroomPassed;
end

function event = alignM24TrackingEvent( ...
        records, trackingPath, window, formationIndex)
event = emptyEvent();
if exist(trackingPath, 'file') ~= 2
    error('TemporalGatewayCoverageV271:MissingTrackingResult', ...
        'The registered M24 tracking result does not exist.');
end
recordIdx = find(cellfun(@(name) strncmp(name, 'm24-', 4), ...
    {records.presetName}), 1);
if isempty(recordIdx)
    error('TemporalGatewayCoverageV271:MissingM24Replay', ...
        'Tracking alignment requires an M24 route replay.');
end
record = records(recordIdx);
if formationIndex > record.formationCount || ...
        any(window > record.timeCount)
    error('TemporalGatewayCoverageV271:InvalidEvent', ...
        'The requested event is outside the M24 replay.');
end

loaded = load(trackingPath, 'result');
if ~isfield(loaded, 'result')
    error('TemporalGatewayCoverageV271:InvalidTrackingResult', ...
        'The tracking file has no result envelope.');
end
tracking = loaded.result;
if isfield(tracking, 'minimumBackbone')
    arm = tracking.minimumBackbone;
elseif isfield(tracking, 'candidate')
    arm = tracking.candidate;
else
    error('TemporalGatewayCoverageV271:InvalidTrackingResult', ...
        'The tracking result has no V242 arm.');
end
if ~isfield(arm, 'positionRmseBySensorTime') || ...
        size(arm.positionRmseBySensorTime, 1) ~= record.nodeCount || ...
        size(arm.positionRmseBySensorTime, 2) < max(window)
    error('TemporalGatewayCoverageV271:InvalidTrackingResult', ...
        'The V242 result lacks aligned per-sensor RMSE.');
end

members = find(record.groupIds == record.groupLabels(formationIndex));
meanAge = mean(record.directInputAgeByTime(members, window), 2)';
meanDistance = mean( ...
    record.localInjectionDistanceByTime(members, window), 2)';
directFraction = mean( ...
    record.crossReceiverByTime(members, window), 2)';
meanRmse = mean(arm.positionRmseBySensorTime(members, window), 2)';
[~, worstLocal] = max(meanRmse);

event.available = true;
event.trackingResultPath = trackingPath;
event.formationIndex = formationIndex;
event.window = window;
event.sensorIndices = members;
event.sensorPhysicalUids = record.sensorPhysicalUids(members);
event.meanDirectInputAge = meanAge;
event.meanLocalInjectionDistance = meanDistance;
event.directInputFraction = directFraction;
event.meanPositionRmse = meanRmse;
event.ageRmseCorrelation = pearsonCorrelation(meanAge, meanRmse);
event.distanceRmseCorrelation = ...
    pearsonCorrelation(meanDistance, meanRmse);
event.worstSensorIndex = members(worstLocal);
event.worstSensorPhysicalUid = ...
    record.sensorPhysicalUids(members(worstLocal));
event.worstSensorMeanRmse = meanRmse(worstLocal);
event.worstSensorMeanAge = meanAge(worstLocal);
event.worstSensorDirectInputFraction = directFraction(worstLocal);
event.imbalanceGatePassed = ...
    rangeLocal(meanAge) >= 2 || ...
    rangeLocal(meanDistance) >= 1 || ...
    rangeLocal(directFraction) >= 0.25;
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

function distances = directedDistancesFromSources( ...
        adjacency, members, sources)
if isempty(sources)
    error('TemporalGatewayCoverageV271:MissingCrossInput', ...
        'Every formation must receive at least one cross input.');
end
nodeCount = size(adjacency, 1);
distanceByNode = inf(1, nodeCount);
distanceByNode(sources) = 0;
frontier = reshape(sources, 1, []);
while ~isempty(frontier)
    source = frontier(1);
    frontier(1) = [];
    receivers = find(adjacency(:, source))';
    receivers = receivers(ismember(receivers, members));
    for receiver = receivers
        candidate = distanceByNode(source) + 1;
        if candidate < distanceByNode(receiver)
            distanceByNode(receiver) = candidate;
            frontier(end + 1) = receiver; %#ok<AGROW>
        end
    end
end
distances = distanceByNode(members)';
if any(~isfinite(distances))
    error('TemporalGatewayCoverageV271:DisconnectedLocalCycle', ...
        'A V242 local cycle does not propagate from its gateway.');
end
end

function age = ageSinceLastInput(mask)
[nodeCount, timeCount] = size(mask);
age = zeros(nodeCount, timeCount);
last = nan(nodeCount, 1);
for timeIdx = 1:timeCount
    current = mask(:, timeIdx);
    last(current) = timeIdx;
    for nodeIdx = 1:nodeCount
        if isfinite(last(nodeIdx))
            age(nodeIdx, timeIdx) = timeIdx - last(nodeIdx);
        else
            age(nodeIdx, timeIdx) = timeIdx;
        end
    end
end
end

function [meanTv, maximumTv] = mixingDiagnostics(weights, horizons)
nodeCount = size(weights, 1);
timeCount = size(weights, 3);
meanTv = nan(1, numel(horizons));
maximumTv = nan(1, numel(horizons));
for horizonIdx = 1:numel(horizons)
    horizon = horizons(horizonIdx);
    if horizon > timeCount
        continue;
    end
    windowMean = zeros(1, timeCount - horizon + 1);
    windowMaximum = zeros(1, timeCount - horizon + 1);
    cursor = 0;
    for endTime = horizon:timeCount
        product = eye(nodeCount);
        for timeIdx = (endTime - horizon + 1):endTime
            product = weights(:, :, timeIdx) * product;
        end
        cursor = cursor + 1;
        pairTv = zeros(1, nodeCount * (nodeCount - 1) / 2);
        pairCursor = 0;
        for left = 1:nodeCount-1
            for right = left+1:nodeCount
                pairCursor = pairCursor + 1;
                pairTv(pairCursor) = 0.5 * sum(abs( ...
                    product(left, :) - product(right, :)));
            end
        end
        windowMean(cursor) = mean(pairTv);
        windowMaximum(cursor) = max(pairTv);
    end
    meanTv(horizonIdx) = mean(windowMean);
    maximumTv(horizonIdx) = mean(windowMaximum);
end
end

function value = pearsonCorrelation(left, right)
left = reshape(left, 1, []);
right = reshape(right, 1, []);
valid = isfinite(left) & isfinite(right);
left = left(valid);
right = right(valid);
if numel(left) < 2 || std(left) <= eps || std(right) <= eps
    value = NaN;
    return;
end
centeredLeft = left - mean(left);
centeredRight = right - mean(right);
value = sum(centeredLeft .* centeredRight) / sqrt( ...
    sum(centeredLeft .^ 2) * sum(centeredRight .^ 2));
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('TemporalGatewayCoverageV271:ReportOpen', ...
        'Could not write V271 report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V271 temporal gateway coverage diagnostic\n\n');
fprintf(fid, '- Generation commit: `%s`\n', result.generationGitCommit);
fprintf(fid, '- Structural seed: `%d`\n', result.seed);
fprintf(fid, '- V272 authorized: `%d`\n', result.v272Authorized);
fprintf(fid, '- Next decision: `%s`\n\n', result.nextDecision);
fprintf(fid, ['| Scene | N / F | Minimum receiver coverage | Maximum ', ...
    'receiver share | Maximum mean / absolute age | Mean / maximum local ', ...
    'distance | Hmax mean / maximum row TV | Concentrated / mixing ', ...
    'headroom |\n']);
fprintf(fid, '|:--|:--:|--:|--:|:--|:--|:--|:--:|\n');
for record = result.records
    fprintf(fid, ...
        '| %s | %d / %d | %.3f | %.3f | %.3f / %.0f | %.3f / %.0f | H%d %.3f / %.3f | %d / %d |\n', ...
        record.presetName, record.nodeCount, record.formationCount, ...
        min(record.crossReceiverCoverageByFormation), ...
        max(record.maximumCrossReceiverShareByFormation), ...
        max(record.meanDirectInputAgeByFormation), ...
        max(record.maximumDirectInputAgeByFormation), ...
        max(record.meanLocalInjectionDistanceByFormation), ...
        max(record.maximumLocalInjectionDistanceByFormation), ...
        record.horizonGrid(end), record.meanRowTvByHorizon(end), ...
        record.maximumRowTvByHorizon(end), ...
        record.concentrationGatePassed, record.mixingHeadroomPassed);
end

fprintf(fid, '\n## Per-formation route concentration\n\n');
fprintf(fid, ['| Scene | Formation | Receiver coverage | Maximum receiver ', ...
    'share | Mean / maximum age | Mean / maximum local distance |\n']);
fprintf(fid, '|:--|--:|--:|--:|:--|:--|\n');
for record = result.records
    for formationIdx = 1:record.formationCount
        fprintf(fid, '| %s | %d | %.3f | %.3f | %.3f / %.0f | %.3f / %.0f |\n', ...
            record.presetName, formationIdx, ...
            record.crossReceiverCoverageByFormation(formationIdx), ...
            record.maximumCrossReceiverShareByFormation(formationIdx), ...
            record.meanDirectInputAgeByFormation(formationIdx), ...
            record.maximumDirectInputAgeByFormation(formationIdx), ...
            record.meanLocalInjectionDistanceByFormation(formationIdx), ...
            record.maximumLocalInjectionDistanceByFormation(formationIdx));
    end
end

fprintf(fid, '\n## Finite-horizon nominal mixing\n\n');
fprintf(fid, '| Scene | Horizon | Mean row TV | Maximum row TV |\n');
fprintf(fid, '|:--|--:|--:|--:|\n');
for record = result.records
    for horizonIdx = 1:numel(record.horizonGrid)
        fprintf(fid, '| %s | %d | %.6f | %.6f |\n', ...
            record.presetName, record.horizonGrid(horizonIdx), ...
            record.meanRowTvByHorizon(horizonIdx), ...
            record.maximumRowTvByHorizon(horizonIdx));
    end
end

if result.event.available
    event = result.event;
    fprintf(fid, '\n## M24 tail alignment\n\n');
    fprintf(fid, '- Formation / window: `F%d / t=%d--%d`\n', ...
        event.formationIndex, event.window(1), event.window(end));
    fprintf(fid, '- Direct-input age / RMSE correlation: `%.6f`\n', ...
        event.ageRmseCorrelation);
    fprintf(fid, '- Local-distance / RMSE correlation: `%.6f`\n', ...
        event.distanceRmseCorrelation);
    fprintf(fid, '- Event imbalance gate: `%d`\n\n', ...
        event.imbalanceGatePassed);
    fprintf(fid, ['| Sensor UID | Direct-input fraction | Mean direct age | ', ...
        'Mean local distance | Mean RMSE |\n']);
    fprintf(fid, '|--:|--:|--:|--:|--:|\n');
    for idx = 1:numel(event.sensorPhysicalUids)
        fprintf(fid, '| %.0f | %.3f | %.3f | %.3f | %.6f |\n', ...
            event.sensorPhysicalUids(idx), ...
            event.directInputFraction(idx), ...
            event.meanDirectInputAge(idx), ...
            event.meanLocalInjectionDistance(idx), ...
            event.meanPositionRmse(idx));
    end
end

fprintf(fid, '\n## Decision\n\n');
if result.v272Authorized
    fprintf(fid, ['V242 leaves a cross-scale temporal concentration and ', ...
        'mixing gap that is aligned with the opened M24 tail. Implement ', ...
        'V272 as a same-message-count temporal balancing policy before ', ...
        'running tracking.\n']);
else
    fprintf(fid, ['The registered concentration, tail-alignment and mixing ', ...
        'conditions do not all hold. Close temporal gateway balancing ', ...
        'without a tracking run.\n']);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function record = emptyRecord()
record = struct('presetName', '', 'nodeCount', NaN, ...
    'formationCount', NaN, 'timeCount', NaN, ...
    'sceneValid', false, 'sceneHardFailures', {{}}, ...
    'sensorPhysicalUids', zeros(1, 0), ...
    'groupIds', zeros(1, 0), 'groupLabels', zeros(1, 0), ...
    'crossReceiverByTime', false(0, 0), ...
    'crossSenderByTime', false(0, 0), ...
    'crossReceiverCountBySensor', zeros(1, 0), ...
    'crossSenderCountBySensor', zeros(1, 0), ...
    'directInputAgeByTime', zeros(0, 0), ...
    'localInjectionDistanceByTime', zeros(0, 0), ...
    'crossReceiverCoverageByFormation', zeros(1, 0), ...
    'maximumCrossReceiverShareByFormation', zeros(1, 0), ...
    'meanDirectInputAgeByFormation', zeros(1, 0), ...
    'maximumDirectInputAgeByFormation', zeros(1, 0), ...
    'meanLocalInjectionDistanceByFormation', zeros(1, 0), ...
    'maximumLocalInjectionDistanceByFormation', zeros(1, 0), ...
    'horizonGrid', zeros(1, 0), ...
    'meanRowTvByHorizon', zeros(1, 0), ...
    'maximumRowTvByHorizon', zeros(1, 0), ...
    'messageCountMinimum', NaN, 'messageCountMaximum', NaN, ...
    'messageParityPassed', false, ...
    'concentrationGatePassed', false, ...
    'mixingHeadroomPassed', false, 'gatePassed', false);
end

function event = emptyEvent()
event = struct('available', false, 'trackingResultPath', '', ...
    'formationIndex', NaN, 'window', zeros(1, 0), ...
    'sensorIndices', zeros(1, 0), ...
    'sensorPhysicalUids', zeros(1, 0), ...
    'meanDirectInputAge', zeros(1, 0), ...
    'meanLocalInjectionDistance', zeros(1, 0), ...
    'directInputFraction', zeros(1, 0), ...
    'meanPositionRmse', zeros(1, 0), ...
    'ageRmseCorrelation', NaN, ...
    'distanceRmseCorrelation', NaN, ...
    'worstSensorIndex', NaN, 'worstSensorPhysicalUid', NaN, ...
    'worstSensorMeanRmse', NaN, 'worstSensorMeanAge', NaN, ...
    'worstSensorDirectInputFraction', NaN, ...
    'imbalanceGatePassed', false);
end

function value = rangeLocal(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values) - min(values);
end
end

function value = ternary(condition, whenTrue, whenFalse)
if condition
    value = whenTrue;
else
    value = whenFalse;
end
end

function value = getField(input, name, fallback)
if isstruct(input) && isfield(input, name)
    value = input.(name);
else
    value = fallback;
end
end

function absolute = isAbsolutePathLocal(path)
absolute = ~isempty(path) && path(1) == filesep;
end
