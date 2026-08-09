function [reportPath, result] = ...
    runObservationSupportedRoutingLeverageV60Attribution(options)
% RUNOBSERVATIONSUPPORTEDROUTINGLEVERAGEV60ATTRIBUTION V60 cache analysis.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getObservationSupportedRoutingLeverageV60Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('ObservationSupportedV60:DirtySource', ...
        'V60 attribution requires a clean source checkout.');
end
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end

recordCount = sum(arrayfun( ...
    @(entry) numel(entry.snapshotTimes), protocol.cases));
records = repmat(emptyRecord(), 1, recordCount);
cursor = 0;
for caseIdx = 1:numel(protocol.cases)
    entry = protocol.cases(caseIdx);
    for currentTime = entry.snapshotTimes
        cursor = cursor + 1;
        state = loadFormationH3ObservableState( ...
            entry.presetName, entry.seed, currentTime, struct( ...
                'cacheRoot', entry.cacheRoot));
        fprintf('V60 support attribution %d/%d: %s t=%d ...\n', ...
            cursor, recordCount, entry.presetName, currentTime);
        [signature, control] = ...
            computeFormationRoutingLeverageSignature( ...
                state.context, state.groupIds);
        support = computeObservationSupportedRoutingLeverage( ...
            control, state.context, state.groupIds, ...
            protocol.positiveSupportThreshold);
        target = resolveOpenedTarget( ...
            protocol, entry.presetName, entry.seed, currentTime);
        records(cursor) = makeRecord( ...
            entry, currentTime, state, signature, support, target);
        fprintf('  raw=%.3f%% supported=%.3f%% weighted=%.3f%% gain=%+.3f%%\n', ...
            100 * signature.maximumRescuedExistenceFraction, ...
            100 * support. ...
                maximumPositiveSupportedRescuedFractionOfReference, ...
            100 * support. ...
                maximumSupportWeightedRescuedFractionOfReference, ...
            target.meanGainPercent);
    end
end

knownMask = isfinite([records.knownMeanGainPercent]);
known = records(knownMask);
knownGain = [known.knownMeanGainPercent];
rawRescue = arrayfun(@(record) ...
    record.signature.maximumRescuedExistenceFraction, known);
supportedRescue = arrayfun(@(record) record.support. ...
    maximumPositiveSupportedRescuedFractionOfReference, known);
weightedRescue = arrayfun(@(record) record.support. ...
    maximumSupportWeightedRescuedFractionOfReference, known);
proportionalRescue = arrayfun(@(record) record.support. ...
    maximumSupportWeightedProportionalRescue, known);
supportedCrossings = arrayfun(@(record) record.support. ...
    maximumPositiveSupportedDecisionCrossings, known);
strongMask = knownGain >= protocol.strongGainPercent - 1e-12;
weakMask = knownGain < protocol.weakGainPercent - 1e-12;

convoyMask = strcmp( ...
    {records.presetName}, 'm24-formation-fov-convoy');
convoy = records(convoyMask);
convoyTimes = [convoy.currentTime];
convoyWeighted = arrayfun(@(record) record.support. ...
    maximumSupportWeightedRescuedFractionOfReference, convoy);
eligible = ~ismember( ...
    convoyTimes, protocol.excludedOpenedConvoyTimes);
selectedTimes = selectSeparatedEvents( ...
    convoyTimes(eligible), convoyWeighted(eligible), ...
    protocol.selectedConvoyEventCount, ...
    protocol.minimumSelectedEventSeparation);

result = struct();
result.contractVersion = ...
    'observation-supported-routing-leverage-v60-attribution-v1';
result.protocol = protocol;
result.generatedAt = datestr(now, 31);
result.featureGenerationGitCommit = gitState.commit;
result.records = records;
result.openedRecordIndices = find(knownMask);
result.pearsonRawRescueToKnownGain = pearson(rawRescue, knownGain);
result.pearsonSupportedRescueToKnownGain = ...
    pearson(supportedRescue, knownGain);
result.pearsonWeightedRescueToKnownGain = ...
    pearson(weightedRescue, knownGain);
result.pearsonProportionalRescueToKnownGain = ...
    pearson(proportionalRescue, knownGain);
result.pearsonSupportedCrossingsToKnownGain = ...
    pearson(supportedCrossings, knownGain);
result.rawRescueSeparatesStrongWeak = ...
    separatesHigher(rawRescue, strongMask, weakMask);
result.supportedRescueSeparatesStrongWeak = ...
    separatesHigher(supportedRescue, strongMask, weakMask);
result.weightedRescueSeparatesStrongWeak = ...
    separatesHigher(weightedRescue, strongMask, weakMask);
result.proportionalRescueSeparatesStrongWeak = ...
    separatesHigher(proportionalRescue, strongMask, weakMask);
result.supportedCrossingsSeparateStrongWeak = ...
    separatesHigher(supportedCrossings, strongMask, weakMask);
result.convoyTimes = convoyTimes;
result.convoyWeightedRescue = convoyWeighted;
result.selectedFreshConvoyTimes = selectedTimes;
result.selectionUsesOpenedTrackingTargets = false;
result.trackingOutcomeScored = false;
result.x36OutcomeOpened = false;
result.modelTrainingAuthorized = false;
result.openedDevelopmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.evidenceBoundary = protocol.evidenceBoundary;

matPath = fullfile(outputRoot, ...
    'OBSERVATION_SUPPORTED_ROUTING_LEVERAGE_V60_ATTRIBUTION.mat');
reportPath = fullfile(outputRoot, ...
    'OBSERVATION_SUPPORTED_ROUTING_LEVERAGE_V60_ATTRIBUTION.md');
result.matPath = matPath;
result.reportPath = reportPath;
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V60 observation-supported attribution: %s\n', reportPath);
end

function record = makeRecord( ...
        entry, currentTime, state, signature, support, target)
record = emptyRecord();
record.presetName = entry.presetName;
record.seed = entry.seed;
record.currentTime = currentTime;
record.cachePath = state.cachePath;
record.signature = signature;
record.support = support;
record.knownMeanGainPercent = target.meanGainPercent;
record.knownTargetSource = target.source;
end

function record = emptyRecord()
record = struct( ...
    'presetName', '', ...
    'seed', NaN, ...
    'currentTime', NaN, ...
    'cachePath', '', ...
    'signature', struct(), ...
    'support', struct(), ...
    'knownMeanGainPercent', NaN, ...
    'knownTargetSource', '');
end

function target = resolveOpenedTarget( ...
        protocol, presetName, seed, currentTime)
targets = protocol.openedTrackingTargets;
mask = strcmp({targets.presetName}, presetName) & ...
    [targets.seed] == seed & [targets.currentTime] == currentTime;
if nnz(mask) > 1
    error('ObservationSupportedV60:DuplicateTarget', ...
        'An opened V60 target is duplicated.');
elseif nnz(mask) == 1
    target = targets(find(mask, 1));
else
    target = struct('meanGainPercent', NaN, 'source', 'not-opened');
end
end

function times = selectSeparatedEvents( ...
        candidateTimes, scores, desiredCount, minimumSeparation)
[~, order] = sortrows([-scores(:), candidateTimes(:)], [1, 2]);
times = zeros(1, 0);
for idx = reshape(order, 1, [])
    currentTime = candidateTimes(idx);
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
    error('ObservationSupportedV60:WriteFailed', ...
        'Could not create the V60 attribution report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V60 observation-supported routing leverage\n\n');
fprintf(fid, '- Positive-support threshold: `%.2f`\n', ...
    result.protocol.positiveSupportThreshold);
fprintf(fid, '- Fresh convoy candidates: `%s`\n\n', ...
    mat2str(result.selectedFreshConvoyTimes));
fprintf(fid, '| Observable | Pearson with opened H=3 gain | Separates strong/weak |\n');
fprintf(fid, '|:--|--:|:--:|\n');
writeAssociationRow(fid, 'V58 raw rescued existence', ...
    result.pearsonRawRescueToKnownGain, ...
    result.rawRescueSeparatesStrongWeak);
writeAssociationRow(fid, 'Positive-supported rescued existence', ...
    result.pearsonSupportedRescueToKnownGain, ...
    result.supportedRescueSeparatesStrongWeak);
writeAssociationRow(fid, 'Association-mass-weighted rescue', ...
    result.pearsonWeightedRescueToKnownGain, ...
    result.weightedRescueSeparatesStrongWeak);
writeAssociationRow(fid, 'Weighted proportional rescue', ...
    result.pearsonProportionalRescueToKnownGain, ...
    result.proportionalRescueSeparatesStrongWeak);
writeAssociationRow(fid, 'Positive-supported crossings', ...
    result.pearsonSupportedCrossingsToKnownGain, ...
    result.supportedCrossingsSeparateStrongWeak);

fprintf(fid, '\n| Preset | Time | Raw rescue | Supported rescue | Weighted rescue | Supported crossings | Known gain |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|\n');
for record = result.records(result.openedRecordIndices)
    fprintf(fid, '| `%s` | %d | %.3f%% | %.3f%% | %.3f%% | %g | %+.3f%% |\n', ...
        record.presetName, record.currentTime, ...
        100 * record.signature.maximumRescuedExistenceFraction, ...
        100 * record.support. ...
            maximumPositiveSupportedRescuedFractionOfReference, ...
        100 * record.support. ...
            maximumSupportWeightedRescuedFractionOfReference, ...
        record.support.maximumPositiveSupportedDecisionCrossings, ...
        record.knownMeanGainPercent);
end

fprintf(fid, '\n## Evidence boundary\n\n%s\n', result.evidenceBoundary);
end

function writeAssociationRow(fid, name, correlation, separates)
fprintf(fid, '| %s | %+.4f | %d |\n', name, correlation, separates);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
