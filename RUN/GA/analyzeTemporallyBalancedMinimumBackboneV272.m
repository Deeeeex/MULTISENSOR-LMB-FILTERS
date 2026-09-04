function [reportPath, result] = ...
        analyzeTemporallyBalancedMinimumBackboneV272(options)
% ANALYZETEMPORALLYBALANCEDMINIMUMBACKBONEV272 Structural comparison.

if nargin < 1 || isempty(options)
    options = struct();
end
presetNames = getField(options, 'presetNames', { ...
    'm24-formation-fov-temporal-coupled-formation-braid', ...
    'x36-formation-fov-temporal-coupled-formation-braid'});
seed = getField(options, 'seed', 1301);
horizonGrid = reshape(getField(options, ...
    'horizonGrid', [3, 6, 12]), 1, []);
writeReport = logical(getField(options, 'writeReport', true));
if ischar(presetNames), presetNames = {presetNames}; end
if ~iscell(presetNames) || isempty(presetNames) || ...
        any(~cellfun(@ischar, presetNames)) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        any(~isfinite(horizonGrid)) || any(horizonGrid < 1) || ...
        any(horizonGrid ~= round(horizonGrid)) || ...
        ~isscalar(writeReport)
    error('TemporallyBalancedBackboneV272:InvalidAnalysisOptions', ...
        'The V272 structural-analysis options are invalid.');
end

protocol = getTemporallyBalancedMinimumBackboneV272Protocol();
if any(~ismember(presetNames, protocol.allowedPresets))
    error('TemporallyBalancedBackboneV272:UnregisteredPreset', ...
        'V272 requires registered V242 scenarios.');
end

records = repmat(emptyRecord(), 1, numel(presetNames));
for presetIdx = 1:numel(presetNames)
    records(presetIdx) = replayPreset( ...
        presetNames{presetIdx}, seed, horizonGrid, protocol);
end

result = struct();
result.contractVersion = ...
    'temporally-balanced-minimum-backbone-v272-structural-result-v1';
result.generatedAt = datestr(now, 31);
gitState = resolveResearchGitState();
result.generationGitCommit = gitState.commit;
result.seed = seed;
result.horizonGrid = horizonGrid;
result.protocol = protocol;
result.records = records;
result.allScaleStructuralGatePassed = all([records.gatePassed]);
result.trackingScreenAuthorized = result.allScaleStructuralGatePassed;
result.nextDecision = ternary(result.trackingScreenAuthorized, ...
    'run-paired-m24-event-window-before-full-episode', ...
    'revise-or-close-v272-before-tracking');
result.evidenceBoundary = [ ...
    'V272 replays the V242 and temporally balanced routes on the same ', ...
    'physical graph and link-reliability schedules. No filter state, ', ...
    'measurement, truth, tracking outcome or realized delivery is read. ', ...
    'The report can authorize one paired development screen, but cannot ', ...
    'establish tracking benefit, deployment value or generalization.'];

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v272', 'structural_preflight')));
if ~isAbsolutePathLocal(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'TEMPORALLY_BALANCED_BACKBONE_V272_STRUCTURAL.md');
matPath = fullfile(outputRoot, ...
    'TEMPORALLY_BALANCED_BACKBONE_V272_STRUCTURAL.mat');
result.reportPath = reportPath;
result.matPath = matPath;
if writeReport
    save('-mat7-binary', matPath, 'result');
    writeReportFile(reportPath, result);
end

for record = records
    fprintf([ ...
        'V272 %s: coverage %.3f->%.3f, share %.3f->%.3f, ', ...
        'eligible %.3f->%.3f, missed %.0f->%.0f, ', ...
        'H%d TV %.3f->%.3f (%+.2f%%), external %+.2f%%, gate %d\n'], ...
        record.presetName, record.reference.minimumReceiverCoverage, ...
        record.candidate.minimumReceiverCoverage, ...
        record.reference.maximumReceiverShare, ...
        record.candidate.maximumReceiverShare, ...
        record.reference.eligibleReceiverCoverage, ...
        record.candidate.eligibleReceiverCoverage, ...
        record.reference.maximumMissedEligibleOpportunityCount, ...
        record.candidate.maximumMissedEligibleOpportunityCount, ...
        horizonGrid(end), record.reference.meanRowTvByHorizon(end), ...
        record.candidate.meanRowTvByHorizon(end), ...
        record.meanTvGainPercent, ...
        record.minimumExternalInfluenceGainPercent, ...
        record.gatePassed);
end
fprintf('V272_TRACKING_SCREEN_AUTHORIZED=%d\n', ...
    result.trackingScreenAuthorized);
end

function record = replayPreset(presetName, seed, horizons, protocol)
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
timeCount = config.simulationLength;
messageBudget = nodeCount + 2 * (formationCount - 1);
referenceTrace = initializeTrace(nodeCount, timeCount);
candidateTrace = initializeTrace(nodeCount, timeCount);
referenceHistory = false(nodeCount, nodeCount, 0);
candidateHistory = false(nodeCount, nodeCount, 0);
referenceTimes = zeros(1, 0);
candidateTimes = zeros(1, 0);

for currentTime = 1:timeCount
    referenceContext = buildContext(config, graph, pDrop, identity, ...
        currentTime, referenceHistory, referenceTimes, messageBudget);
    candidateContext = buildContext(config, graph, pDrop, identity, ...
        currentTime, candidateHistory, candidateTimes, messageBudget);
    [referenceAdjacency, referenceDetails] = ...
        selectCausalMinimumFormationBackboneV242Policy( ...
            referenceContext);
    [candidateAdjacency, candidateDetails] = ...
        selectTemporallyBalancedMinimumBackboneV272Policy( ...
            candidateContext);

    reliability = 1 - pDrop(:, :, currentTime)';
    referenceTrace = recordPage(referenceTrace, currentTime, ...
        referenceAdjacency, referenceDetails, reliability, config, ...
        false);
    candidateTrace = recordPage(candidateTrace, currentTime, ...
        candidateAdjacency, candidateDetails, reliability, config, ...
        true);
    referenceTrace.receiverOpportunity(:, currentTime) = ...
        candidateTrace.receiverOpportunity(:, currentTime);
    [referenceHistory, referenceTimes] = appendHistory( ...
        referenceHistory, referenceTimes, referenceAdjacency, ...
        currentTime, protocol.historyDepth);
    [candidateHistory, candidateTimes] = appendHistory( ...
        candidateHistory, candidateTimes, candidateAdjacency, ...
        currentTime, protocol.historyDepth);
end

reference = summarizeTrace(referenceTrace, config, horizons);
candidate = summarizeTrace(candidateTrace, config, horizons);
meanTvGain = percentReduction( ...
    reference.meanRowTvByHorizon(end), ...
    candidate.meanRowTvByHorizon(end));
maximumTvGain = percentReduction( ...
    reference.maximumRowTvByHorizon(end), ...
    candidate.maximumRowTvByHorizon(end));
coveragePassed = candidate.minimumReceiverCoverage > ...
    reference.minimumReceiverCoverage + 1e-12;
concentrationPassed = candidate.maximumReceiverShare <= ...
    reference.maximumReceiverShare - 0.05;
opportunityPassed = candidate.eligibleReceiverCoverage >= ...
        reference.eligibleReceiverCoverage + 1e-12 && ...
    candidate.maximumMissedEligibleOpportunityCount < ...
        reference.maximumMissedEligibleOpportunityCount;
externalInfluenceGain = percentIncrease( ...
    reference.meanMinimumExternalInfluenceByHorizon(end), ...
    candidate.meanMinimumExternalInfluenceByHorizon(end));
mixingPassed = meanTvGain > 0 && maximumTvGain >= -1e-9 && ...
    externalInfluenceGain > 0;
qualityPassed = candidate.realizedMinimumReliabilityRatio >= ...
        protocol.minimumReliabilityRatio - 1e-12 && ...
    candidate.realizedMaximumReliabilityDrop <= ...
        protocol.maximumReliabilityDrop + 1e-12;
hardPassed = validation.isValid && ...
    reference.messageParityPassed && candidate.messageParityPassed && ...
    candidate.allPagesStrong && qualityPassed;

record = emptyRecord();
record.presetName = presetName;
record.nodeCount = nodeCount;
record.formationCount = formationCount;
record.timeCount = timeCount;
record.sceneValid = validation.isValid;
record.sceneHardFailures = validation.hardFailures;
record.reference = reference;
record.candidate = candidate;
record.meanTvGainPercent = meanTvGain;
record.maximumTvGainPercent = maximumTvGain;
record.minimumExternalInfluenceGainPercent = externalInfluenceGain;
record.coverageGatePassed = coveragePassed;
record.concentrationGatePassed = concentrationPassed;
record.opportunityGatePassed = opportunityPassed;
record.mixingGatePassed = mixingPassed;
record.qualityGatePassed = qualityPassed;
record.hardGatePassed = hardPassed;
record.gatePassed = hardPassed && coveragePassed && ...
    concentrationPassed && opportunityPassed && mixingPassed;
end

function trace = initializeTrace(nodeCount, timeCount)
trace = struct();
trace.crossReceiver = false(nodeCount, timeCount);
trace.crossSender = false(nodeCount, timeCount);
trace.fusionWeights = zeros(nodeCount, nodeCount, timeCount);
trace.adjacency = false(nodeCount, nodeCount, timeCount);
trace.messageCount = zeros(1, timeCount);
trace.meanCrossReliability = zeros(1, timeCount);
trace.minimumCrossReliability = zeros(1, timeCount);
trace.minimumReliabilityRatio = ones(1, timeCount);
trace.maximumReliabilityDrop = zeros(1, timeCount);
trace.strong = false(1, timeCount);
trace.receiverOpportunity = false(nodeCount, timeCount);
end

function trace = recordPage(trace, currentTime, adjacency, details, ...
        reliability, config, candidate)
formationBySensor = reshape(config.sensorGroupIds, 1, []);
crossMask = formationBySensor(:) ~= formationBySensor(:)';
cross = logical(adjacency) & crossMask;
trace.crossReceiver(:, currentTime) = any(cross, 2);
trace.crossSender(:, currentTime) = any(cross, 1)';
trace.fusionWeights(:, :, currentTime) = details.fusionWeightMatrix;
trace.adjacency(:, :, currentTime) = logical(adjacency);
trace.messageCount(currentTime) = nnz(adjacency);
q = reliability(cross);
trace.meanCrossReliability(currentTime) = mean(q);
trace.minimumCrossReliability(currentTime) = min(q);
if candidate
    trace.minimumReliabilityRatio(currentTime) = ...
        details.realizedMinimumReliabilityRatio;
    trace.maximumReliabilityDrop(currentTime) = ...
        details.realizedMaximumReliabilityDrop;
    for formationIdx = 1:numel(details.selectionByReceivingFormation)
        selected = details.selectionByReceivingFormation{formationIdx};
        trace.receiverOpportunity( ...
            selected.eligibleReceiverIndices, currentTime) = true;
    end
end
trace.strong(currentTime) = isStronglyConnected(adjacency);
end

function summary = summarizeTrace(trace, config, horizons)
groupIds = reshape(config.sensorGroupIds, 1, []);
groupLabels = unique(groupIds);
formationCount = numel(groupLabels);
receiverAge = ageSinceLastInput(trace.crossReceiver);
senderAge = ageSinceLastInput(trace.crossSender);
[eligibleCoverage, maximumMissedOpportunities] = ...
    opportunityDiagnostics( ...
        trace.crossReceiver, trace.receiverOpportunity);
receiverCoverage = zeros(1, formationCount);
senderCoverage = zeros(1, formationCount);
receiverShare = zeros(1, formationCount);
senderShare = zeros(1, formationCount);
maximumAge = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = find(groupIds == groupLabels(formationIdx));
    receiverCounts = sum(trace.crossReceiver(members, :), 2);
    senderCounts = sum(trace.crossSender(members, :), 2);
    receiverCoverage(formationIdx) = ...
        nnz(receiverCounts > 0) / numel(members);
    senderCoverage(formationIdx) = ...
        nnz(senderCounts > 0) / numel(members);
    receiverShare(formationIdx) = ...
        max(receiverCounts) / max(1, sum(receiverCounts));
    senderShare(formationIdx) = ...
        max(senderCounts) / max(1, sum(senderCounts));
    formationAge = receiverAge(members, :);
    maximumAge(formationIdx) = max(formationAge(:));
end
[meanTv, maximumTv, minimumExternal, externalSpread] = ...
    mixingDiagnostics(trace.fusionWeights, horizons, groupIds);
routeChanged = false(1, size(trace.adjacency, 3));
for timeIdx = 2:size(trace.adjacency, 3)
    routeChanged(timeIdx) = ~isequal( ...
        trace.adjacency(:, :, timeIdx), ...
        trace.adjacency(:, :, timeIdx - 1));
end
summary = struct();
summary.minimumReceiverCoverage = min(receiverCoverage);
summary.minimumSenderCoverage = min(senderCoverage);
summary.maximumReceiverShare = max(receiverShare);
summary.maximumSenderShare = max(senderShare);
summary.maximumDirectInputAge = max(maximumAge);
summary.eligibleReceiverCoverage = eligibleCoverage;
summary.maximumMissedEligibleOpportunityCount = ...
    maximumMissedOpportunities;
summary.receiverCoverageByFormation = receiverCoverage;
summary.senderCoverageByFormation = senderCoverage;
summary.receiverShareByFormation = receiverShare;
summary.senderShareByFormation = senderShare;
summary.maximumAgeByFormation = maximumAge;
summary.meanRowTvByHorizon = meanTv;
summary.maximumRowTvByHorizon = maximumTv;
summary.meanMinimumExternalInfluenceByHorizon = minimumExternal;
summary.meanExternalInfluenceSpreadByHorizon = externalSpread;
summary.horizonGrid = horizons;
summary.messageCountMinimum = min(trace.messageCount);
summary.messageCountMaximum = max(trace.messageCount);
summary.messageParityPassed = all(trace.messageCount == ...
    config.numberOfSensors + 2 * (formationCount - 1));
summary.allPagesStrong = all(trace.strong);
summary.meanCrossReliability = mean(trace.meanCrossReliability);
summary.minimumCrossReliability = min(trace.minimumCrossReliability);
summary.realizedMinimumReliabilityRatio = ...
    min(trace.minimumReliabilityRatio);
summary.realizedMaximumReliabilityDrop = ...
    max(trace.maximumReliabilityDrop);
summary.routeChangeFraction = mean(routeChanged(2:end));
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

function [history, times] = appendHistory( ...
        history, times, adjacency, currentTime, depth)
history = cat(3, history, logical(adjacency));
times(end + 1) = currentTime;
if size(history, 3) > depth
    history = history(:, :, end-depth+1:end);
    times = times(end-depth+1:end);
end
end

function age = ageSinceLastInput(mask)
[nodeCount, timeCount] = size(mask);
age = zeros(nodeCount, timeCount);
last = nan(nodeCount, 1);
for timeIdx = 1:timeCount
    current = mask(:, timeIdx);
    last(current) = timeIdx;
    unseen = ~isfinite(last);
    age(~unseen, timeIdx) = timeIdx - last(~unseen);
    age(unseen, timeIdx) = timeIdx;
end
end

function [coverage, maximumMissed] = ...
        opportunityDiagnostics(service, opportunity)
everEligible = any(opportunity, 2);
everServed = any(service, 2);
coverage = nnz(everEligible & everServed) / max(1, nnz(everEligible));
missed = zeros(size(service, 1), 1);
maximumMissed = 0;
for timeIdx = 1:size(service, 2)
    missed(service(:, timeIdx)) = 0;
    waiting = opportunity(:, timeIdx) & ~service(:, timeIdx);
    missed(waiting) = missed(waiting) + 1;
    maximumMissed = max(maximumMissed, max(missed));
end
end

function [meanTv, maximumTv, minimumExternal, externalSpread] = ...
        mixingDiagnostics(weights, horizons, groupIds)
nodeCount = size(weights, 1);
timeCount = size(weights, 3);
meanTv = nan(1, numel(horizons));
maximumTv = nan(1, numel(horizons));
minimumExternal = nan(1, numel(horizons));
externalSpread = nan(1, numel(horizons));
for horizonIdx = 1:numel(horizons)
    horizon = horizons(horizonIdx);
    windowMean = zeros(1, timeCount - horizon + 1);
    windowMaximum = zeros(1, timeCount - horizon + 1);
    windowMinimumExternal = zeros(1, timeCount - horizon + 1);
    windowExternalSpread = zeros(1, timeCount - horizon + 1);
    cursor = 0;
    for endTime = horizon:timeCount
        product = eye(nodeCount);
        for timeIdx = (endTime - horizon + 1):endTime
            product = weights(:, :, timeIdx) * product;
        end
        pairTv = zeros(1, nodeCount * (nodeCount - 1) / 2);
        pairCursor = 0;
        for left = 1:nodeCount-1
            for right = left+1:nodeCount
                pairCursor = pairCursor + 1;
                pairTv(pairCursor) = 0.5 * sum(abs( ...
                    product(left, :) - product(right, :)));
            end
        end
        cursor = cursor + 1;
        windowMean(cursor) = mean(pairTv);
        windowMaximum(cursor) = max(pairTv);
        external = zeros(1, nodeCount);
        for receiver = 1:nodeCount
            external(receiver) = sum(product(receiver, ...
                groupIds ~= groupIds(receiver)));
        end
        windowMinimumExternal(cursor) = min(external);
        groupSpread = zeros(1, numel(unique(groupIds)));
        labels = unique(groupIds);
        for groupIdx = 1:numel(labels)
            values = external(groupIds == labels(groupIdx));
            groupSpread(groupIdx) = max(values) - min(values);
        end
        windowExternalSpread(cursor) = max(groupSpread);
    end
    meanTv(horizonIdx) = mean(windowMean);
    maximumTv(horizonIdx) = mean(windowMaximum);
    minimumExternal(horizonIdx) = mean(windowMinimumExternal);
    externalSpread(horizonIdx) = mean(windowExternalSpread);
end
end

function writeReportFile(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('TemporallyBalancedBackboneV272:ReportOpenFailed', ...
        'Could not write V272 report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V272 temporally balanced minimum backbone\n\n');
fprintf(fid, '- Generation commit: `%s`\n', result.generationGitCommit);
fprintf(fid, '- Structural seed: `%d`\n', result.seed);
fprintf(fid, '- Tracking screen authorized: `%d`\n', ...
    result.trackingScreenAuthorized);
fprintf(fid, '- Next decision: `%s`\n\n', result.nextDecision);
fprintf(fid, ['| Scene | Receiver coverage | Maximum receiver share | ', ...
    'Eligible coverage / missed opportunities | Hmax mean row TV | ', ...
    'Hmax minimum external influence | ', ...
    'Link ratio / drop | Gate |\n']);
fprintf(fid, '|:--|:--|:--|:--|:--|:--|:--|:--:|\n');
for record = result.records
    fprintf(fid, ['| %s | %.3f -> %.3f | %.3f -> %.3f | ', ...
        '%.3f -> %.3f / %.0f -> %.0f | ', ...
        '%.3f -> %.3f (%+.2f%%) | ', ...
        '%.4f -> %.4f (%+.2f%%) | %.3f / %.3f | %d |\n'], ...
        record.presetName, ...
        record.reference.minimumReceiverCoverage, ...
        record.candidate.minimumReceiverCoverage, ...
        record.reference.maximumReceiverShare, ...
        record.candidate.maximumReceiverShare, ...
        record.reference.eligibleReceiverCoverage, ...
        record.candidate.eligibleReceiverCoverage, ...
        record.reference.maximumMissedEligibleOpportunityCount, ...
        record.candidate.maximumMissedEligibleOpportunityCount, ...
        record.reference.meanRowTvByHorizon(end), ...
        record.candidate.meanRowTvByHorizon(end), ...
        record.meanTvGainPercent, ...
        record.reference.meanMinimumExternalInfluenceByHorizon(end), ...
        record.candidate.meanMinimumExternalInfluenceByHorizon(end), ...
        record.minimumExternalInfluenceGainPercent, ...
        record.candidate.realizedMinimumReliabilityRatio, ...
        record.candidate.realizedMaximumReliabilityDrop, ...
        record.gatePassed);
end

fprintf(fid, '\n## Gate components\n\n');
fprintf(fid, ['| Scene | Hard | Coverage | Concentration | Opportunity | ', ...
    'Mixing | Quality |\n']);
fprintf(fid, '|:--|:--:|:--:|:--:|:--:|:--:|:--:|\n');
for record = result.records
    fprintf(fid, '| %s | %d | %d | %d | %d | %d | %d |\n', ...
        record.presetName, record.hardGatePassed, ...
        record.coverageGatePassed, ...
        record.concentrationGatePassed, record.opportunityGatePassed, ...
        record.mixingGatePassed, record.qualityGatePassed);
end

fprintf(fid, '\n## Communication and route behavior\n\n');
fprintf(fid, ['| Scene / arm | Messages | Mean / minimum cross-link ', ...
    'reliability | Receiver / sender coverage | Receiver / sender ', ...
    'maximum share | Route-change fraction |\n']);
fprintf(fid, '|:--|:--|:--|:--|:--|--:|\n');
for record = result.records
    writeBehaviorRow(fid, record.presetName, 'V242', record.reference);
    writeBehaviorRow(fid, record.presetName, 'V272', record.candidate);
end

fprintf(fid, '\n## Decision\n\n');
if result.trackingScreenAuthorized
    fprintf(fid, ['The same-message-count V272 route clears the registered ', ...
        'M24 and X36 structural gates. Run one paired M24 event-window ', ...
        'tracking screen before any full-episode claim.\n']);
else
    fprintf(fid, ['V272 does not clear every cross-scale structural gate. ', ...
        'Do not spend a tracking run until the failed mechanism component ', ...
        'is revised or the action family is closed.\n']);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function writeBehaviorRow(fid, scene, armName, value)
fprintf(fid, ['| %s / %s | %d--%d | %.3f / %.3f | %.3f / %.3f | ', ...
    '%.3f / %.3f | %.3f |\n'], scene, armName, ...
    value.messageCountMinimum, value.messageCountMaximum, ...
    value.meanCrossReliability, value.minimumCrossReliability, ...
    value.minimumReceiverCoverage, value.minimumSenderCoverage, ...
    value.maximumReceiverShare, value.maximumSenderShare, ...
    value.routeChangeFraction);
end

function value = percentReduction(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), realmin);
end

function value = percentIncrease(reference, candidate)
value = 100 * (candidate - reference) / max(abs(reference), realmin);
end

function record = emptyRecord()
record = struct('presetName', '', 'nodeCount', NaN, ...
    'formationCount', NaN, 'timeCount', NaN, ...
    'sceneValid', false, 'sceneHardFailures', {{}}, ...
    'reference', struct(), 'candidate', struct(), ...
    'meanTvGainPercent', NaN, 'maximumTvGainPercent', NaN, ...
    'minimumExternalInfluenceGainPercent', NaN, ...
    'coverageGatePassed', false, ...
    'concentrationGatePassed', false, 'opportunityGatePassed', false, ...
    'mixingGatePassed', false, 'qualityGatePassed', false, ...
    'hardGatePassed', false, 'gatePassed', false);
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
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; ...
        %#ok<AGROW>
end
passed = all(visited);
end
