function [reportPath, result] = ...
    runTrackingAlignedRoutingLeverageV58Attribution(options)
% RUNTRACKINGALIGNEDROUTINGLEVERAGEV58ATTRIBUTION V58 Stage-A screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getTrackingAlignedRoutingLeverageV58Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('TrackingAlignedV58:DirtySource', ...
        'V58 attribution requires a clean source checkout.');
end
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

recordCount = sum(arrayfun( ...
    @(entry) numel(entry.snapshotTimes), protocol.cases));
records = repmat(emptyRecord(), 1, recordCount);
recordCursor = 0;
cacheGenerationCommit = '';
for caseIdx = 1:numel(protocol.cases)
    entry = protocol.cases(caseIdx);
    for currentTime = entry.snapshotTimes
        recordCursor = recordCursor + 1;
        state = loadFormationH3ObservableState( ...
            entry.presetName, entry.seed, currentTime, struct( ...
                'cacheRoot', entry.cacheRoot));
        loaded = load(state.cachePath, 'behaviorBundle');
        bundle = loaded.behaviorBundle;
        if ~isfield(bundle, 'protocolId') || ...
                ~strcmp(bundle.protocolId, protocol.id) || ...
                ~isfield(bundle, 'generationGitCommit') || ...
                bundle.policyTruthUsed || bundle.policyFutureOutcomeUsed
            error('TrackingAlignedV58:InvalidCacheProvenance', ...
                'A V58 reference cache violates the source-only boundary.');
        end
        if isempty(cacheGenerationCommit)
            cacheGenerationCommit = bundle.generationGitCommit;
        elseif ~strcmp(cacheGenerationCommit, bundle.generationGitCommit)
            error('TrackingAlignedV58:MixedCacheGeneration', ...
                'Radial and convoy caches were not generated together.');
        end
        fprintf('V58 signature %d/%d: %s t=%d ...\n', ...
            recordCursor, recordCount, entry.presetName, currentTime);
        signature = computeFormationRoutingLeverageSignature( ...
            state.context, state.groupIds);
        target = resolveOpenedTarget( ...
            protocol, entry.presetName, entry.seed, currentTime);
        records(recordCursor) = makeRecord( ...
            entry, currentTime, state, signature, target);
        fprintf([ ...
            '  event=%.6f max-debt=%.3f%% rescued=%.3f%% ', ...
            'crossings=%g selected=%s\n'], ...
            signature.eventScore, ...
            100 * signature.maximumRetentionDebtFraction, ...
            100 * signature.maximumRescuedExistenceFraction, ...
            signature.maximumRescuedDecisionCrossingCount, ...
            mat2str(signature.selectedFormationIds));
    end
end
if recordCursor ~= recordCount
    error('TrackingAlignedV58:IncompleteAttribution', ...
        'The V58 state table is incomplete.');
end

knownMask = isfinite([records.knownMeanGainPercent]);
known = records(knownMask);
knownGain = [known.knownMeanGainPercent];
eventScore = arrayfun( ...
    @(record) record.signature.eventScore, known);
maximumDebt = arrayfun( ...
    @(record) record.signature.maximumRetentionDebtFraction, known);
rescuedFraction = arrayfun( ...
    @(record) record.signature.maximumRescuedExistenceFraction, known);
rescuedCrossings = arrayfun( ...
    @(record) record.signature.maximumRescuedDecisionCrossingCount, known);
strongMask = knownGain >= 5 - 1e-12;
weakMask = knownGain < 2 - 1e-12;

convoyMask = strcmp( ...
    {records.presetName}, 'm24-formation-fov-convoy');
convoyRecords = records(convoyMask);
convoyTimes = [convoyRecords.currentTime];
convoyDebt = arrayfun( ...
    @(record) record.signature.maximumRetentionDebtFraction, ...
    convoyRecords);
eligibleMask = ...
    convoyDebt >= protocol.retentionDebtOnFraction - 1e-12 & ...
    ~ismember(convoyTimes, protocol.excludedOpenedConvoyTimes);
selectedConvoyEventTimes = selectSeparatedEvents( ...
    convoyTimes(eligibleMask), convoyDebt(eligibleMask), ...
    protocol.selectedConvoyEventCount, ...
    protocol.minimumSelectedEventSeparation);
[~, convoyDebtOrder] = sortrows( ...
    [-convoyDebt(:), convoyTimes(:)], [1, 2]);
topCount = min(5, numel(convoyDebtOrder));
topConvoyDebtTimes = convoyTimes(convoyDebtOrder(1:topCount));

result = struct();
result.contractVersion = ...
    'tracking-aligned-routing-leverage-v58-attribution-v1';
result.protocol = protocol;
result.generatedAt = datestr(now, 31);
result.featureGenerationGitCommit = gitState.commit;
result.cacheGenerationGitCommit = cacheGenerationCommit;
result.records = records;
result.openedRecordIndices = find(knownMask);
result.pearsonEventScoreToKnownGain = ...
    pearson(eventScore, knownGain);
result.pearsonMaximumDebtToKnownGain = ...
    pearson(maximumDebt, knownGain);
result.pearsonMaximumRescuedExistenceToKnownGain = ...
    pearson(rescuedFraction, knownGain);
result.pearsonMaximumRescuedCrossingsToKnownGain = ...
    pearson(rescuedCrossings, knownGain);
result.eventScoreSeparatesOpenedStrongWeak = ...
    separatesHigher(eventScore, strongMask, weakMask);
result.maximumDebtSeparatesOpenedStrongWeak = ...
    separatesHigher(maximumDebt, strongMask, weakMask);
result.maximumRescuedExistenceSeparatesOpenedStrongWeak = ...
    separatesHigher(rescuedFraction, strongMask, weakMask);
result.maximumRescuedCrossingsSeparatesOpenedStrongWeak = ...
    separatesHigher(rescuedCrossings, strongMask, weakMask);
result.convoyTimes = convoyTimes;
result.convoyMaximumDebtFraction = convoyDebt;
result.convoyEligibleEventMask = eligibleMask;
result.selectedConvoyEventTimes = selectedConvoyEventTimes;
result.topConvoyDebtTimes = topConvoyDebtTimes;
result.selectionUsesOpenedTrackingTargets = false;
result.selectionUsesTruth = false;
result.selectionUsesFutureMeasurements = false;
result.selectionUsesFutureLinkOutcomes = false;
result.trackingOutcomeScored = false;
result.x36OutcomeOpened = false;
result.modelTrainingAuthorized = false;
result.openedDevelopmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;

matPath = fullfile(outputRoot, ...
    'TRACKING_ALIGNED_ROUTING_LEVERAGE_V58_ATTRIBUTION.mat');
reportPath = fullfile(outputRoot, ...
    'TRACKING_ALIGNED_ROUTING_LEVERAGE_V58_ATTRIBUTION.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V58 routing-leverage attribution: %s\n', reportPath);
end

function record = makeRecord(entry, currentTime, state, signature, target)
record = emptyRecord();
record.caseId = entry.caseId;
record.presetName = entry.presetName;
record.seed = entry.seed;
record.currentTime = currentTime;
record.cachePath = state.cachePath;
record.cacheSha256 = state.cacheSha256;
record.signature = signature;
record.knownMeanGainPercent = target.meanGainPercent;
record.knownTargetSource = target.source;
end

function record = emptyRecord()
record = struct( ...
    'caseId', '', ...
    'presetName', '', ...
    'seed', NaN, ...
    'currentTime', NaN, ...
    'cachePath', '', ...
    'cacheSha256', '', ...
    'signature', struct(), ...
    'knownMeanGainPercent', NaN, ...
    'knownTargetSource', '');
end

function target = resolveOpenedTarget( ...
        protocol, presetName, seed, currentTime)
targets = protocol.openedTrackingTargets;
mask = strcmp({targets.presetName}, presetName) & ...
    [targets.seed] == seed & [targets.currentTime] == currentTime;
if nnz(mask) > 1
    error('TrackingAlignedV58:DuplicateOpenedTarget', ...
        'An opened V58 target is duplicated.');
elseif nnz(mask) == 1
    target = targets(find(mask, 1));
else
    target = struct( ...
        'meanGainPercent', NaN, 'source', 'not-opened');
end
end

function times = selectSeparatedEvents( ...
        candidateTimes, scores, desiredCount, minimumSeparation)
candidateTimes = reshape(candidateTimes, 1, []);
scores = reshape(scores, 1, []);
if numel(candidateTimes) ~= numel(scores)
    error('TrackingAlignedV58:InvalidEventCandidates', ...
        'V58 event times and scores do not align.');
end
[~, order] = sortrows([-scores(:), candidateTimes(:)], [1, 2]);
times = zeros(1, 0);
for candidateIdx = reshape(order, 1, [])
    currentTime = candidateTimes(candidateIdx);
    if isempty(times) || all(abs(times - currentTime) >= minimumSeparation)
        times(end + 1) = currentTime; %#ok<AGROW>
        if numel(times) == desiredCount
            break;
        end
    end
end
times = sort(times);
end

function value = pearson(left, right)
left = reshape(left, 1, []);
right = reshape(right, 1, []);
valid = isfinite(left) & isfinite(right);
left = left(valid);
right = right(valid);
if numel(left) < 2
    value = NaN;
    return;
end
left = left - mean(left);
right = right - mean(right);
denominator = sqrt(sum(left .^ 2) * sum(right .^ 2));
if denominator <= eps
    value = NaN;
else
    value = sum(left .* right) / denominator;
end
end

function value = separatesHigher(feature, strongMask, weakMask)
value = any(strongMask) && any(weakMask) && ...
    min(feature(strongMask)) > max(feature(weakMask)) + 1e-12;
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('TrackingAlignedV58:WriteFailed', ...
        'Could not create the V58 attribution report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V58 tracking-aligned routing-leverage attribution\n\n');
fprintf(fid, '## Opened-state association\n\n');
fprintf(fid, '| Observable | Pearson with best known H=3 gain | Separates opened strong/weak |\n');
fprintf(fid, '|:--|--:|:--:|\n');
fprintf(fid, '| Former posterior-contrast event score | %+.4f | %d |\n', ...
    result.pearsonEventScoreToKnownGain, ...
    result.eventScoreSeparatesOpenedStrongWeak);
fprintf(fid, '| Maximum formation retention debt | %+.4f | %d |\n', ...
    result.pearsonMaximumDebtToKnownGain, ...
    result.maximumDebtSeparatesOpenedStrongWeak);
fprintf(fid, '| Maximum rescued existence fraction | %+.4f | %d |\n', ...
    result.pearsonMaximumRescuedExistenceToKnownGain, ...
    result.maximumRescuedExistenceSeparatesOpenedStrongWeak);
fprintf(fid, '| Maximum rescued decision crossings | %+.4f | %d |\n\n', ...
    result.pearsonMaximumRescuedCrossingsToKnownGain, ...
    result.maximumRescuedCrossingsSeparatesOpenedStrongWeak);

fprintf(fid, '| Preset | Time | Event score | Max debt | Rescued existence | Crossings | Known gain | Source |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|:--|\n');
for record = result.records(result.openedRecordIndices)
    signature = record.signature;
    fprintf(fid, '| `%s` | %d | %.4f | %.3f%% | %.3f%% | %g | %+.3f%% | %s |\n', ...
        record.presetName, record.currentTime, signature.eventScore, ...
        100 * signature.maximumRetentionDebtFraction, ...
        100 * signature.maximumRescuedExistenceFraction, ...
        signature.maximumRescuedDecisionCrossingCount, ...
        record.knownMeanGainPercent, record.knownTargetSource);
end

fprintf(fid, '\n## Full convoy event scan\n\n');
fprintf(fid, '- Registered debt threshold: `%.2f%%`\n', ...
    100 * result.protocol.retentionDebtOnFraction);
fprintf(fid, '- Previously opened times excluded from selection: `%s`\n', ...
    mat2str(result.protocol.excludedOpenedConvoyTimes));
fprintf(fid, '- Selected fresh event times: `%s`\n', ...
    mat2str(result.selectedConvoyEventTimes));
fprintf(fid, '- Top five convoy debt times: `%s`\n\n', ...
    mat2str(result.topConvoyDebtTimes));
fprintf(fid, '| Time | Event score | Max debt | Above threshold | Rescued existence | Crossings | Selected formations |\n');
fprintf(fid, '|--:|--:|--:|:--:|--:|--:|:--|\n');
convoy = result.records(strcmp( ...
    {result.records.presetName}, 'm24-formation-fov-convoy'));
for record = convoy
    signature = record.signature;
    fprintf(fid, '| %d | %.4f | %.3f%% | %d | %.3f%% | %g | `%s` |\n', ...
        record.currentTime, signature.eventScore, ...
        100 * signature.maximumRetentionDebtFraction, ...
        signature.maximumRetentionDebtFraction >= ...
            result.protocol.retentionDebtOnFraction - 1e-12, ...
        100 * signature.maximumRescuedExistenceFraction, ...
        signature.maximumRescuedDecisionCrossingCount, ...
        mat2str(signature.selectedFormationIds));
end

fprintf(fid, '\n## Evidence boundary\n\n');
fprintf(fid, ['The features and fresh-event selection use no target truth, ', ...
    'future measurement, or future link outcome. Previously opened H=3 ', ...
    'tracking gains are joined only for mechanism attribution. No new ', ...
    'tracking outcome, X36 result, model training, or validation claim is ', ...
    'authorized by this report.\n']);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
