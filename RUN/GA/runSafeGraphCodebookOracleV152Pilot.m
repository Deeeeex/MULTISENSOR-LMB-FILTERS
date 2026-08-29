function [reportPath, summary] = ...
    runSafeGraphCodebookOracleV152Pilot(presetName, seed, options)
% RUNSAFEGRAPHCODEBOOKORACLEV152PILOT Run one opened H=8 codebook pilot.

if nargin < 1 || isempty(presetName)
    presetName = 'm24-formation-fov';
end
if nargin < 2 || isempty(seed)
    seed = 83;
end
if nargin < 3 || isempty(options)
    options = struct();
end
protocol = getSafeGraphCodebookOracleV152Protocol();
presetIdx = find(strcmp(protocol.presetNames, presetName));
if numel(presetIdx) ~= 1 || ...
        ~ismember(seed, protocol.openedDevelopmentSeeds)
    error('V152 pilot case is outside the frozen opened matrix.');
end
cacheDirectory = getField(options, 'cacheDirectory', '');
if isstring(cacheDirectory) && isscalar(cacheDirectory)
    cacheDirectory = char(cacheDirectory);
end
if isempty(cacheDirectory) || exist(cacheDirectory, 'dir') ~= 7
    error('V152 pilot requires an explicit opened cacheDirectory.');
end

rankArms = arrayfun(@(rankIndex) sprintf('%s%d', ...
    protocol.proposalArmPrefix, rankIndex), ...
    protocol.proposalRanks, 'UniformOutput', false);
armNames = [protocol.staticArmModes, rankArms];
outputDirectory = getField(options, 'outputDirectory', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v152', 'safe_graph_codebook_oracle', ...
    strrep(presetName, '-', '_'), sprintf('seed%d', seed)));
runnerOptions = struct( ...
    'armNames', {armNames}, ...
    'maxTimeSteps', protocol.anchorTimes(presetIdx) + ...
        protocol.horizon - 1, ...
    'continuationStartTime', protocol.anchorTimes(presetIdx), ...
    'behaviorCacheDirectory', cacheDirectory, ...
    'generateMissingBehaviorCache', false, ...
    'evidenceSplit', protocol.evidenceSplit, ...
    'v152ExecutionAuthorized', true, ...
    'outputDirectory', outputDirectory, ...
    'writeReport', true);
[rawReportPath, rawSummary] = ...
    runDynamicTopologyOracleGapScreen( ...
        presetName, seed, runnerOptions);

summary = summarizePilot(rawSummary, protocol, presetIdx);
summary.rawReportPath = rawReportPath;
summary.rawMatPath = rawSummary.matPath;
reportPath = fullfile(outputDirectory, ...
    'SAFE_GRAPH_CODEBOOK_ORACLE_V152_PILOT.md');
matPath = fullfile(outputDirectory, ...
    'safe_graph_codebook_oracle_v152_pilot.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writePilotReport(reportPath, summary);
fprintf('V152 pilot summary: %s\n', reportPath);
end

function summary = summarizePilot(raw, protocol, presetIdx)
records = reshape(raw.records, 1, []);
staticMask = ismember({records.armMode}, protocol.staticArmModes);
dynamicMask = startsWithCompat( ...
    {records.armMode}, protocol.proposalArmPrefix);
if nnz(staticMask) ~= 2 || ...
        nnz(dynamicMask) ~= protocol.proposalCount
    error('V152 raw pilot lacks the complete frozen codebook.');
end
staticRecords = records(staticMask);
[~, bestStaticLocalIdx] = min([staticRecords.meanEospa]);
reference = staticRecords(bestStaticLocalIdx);
candidateRecords = records(dynamicMask);
admissible = false(1, numel(candidateRecords));
for idx = 1:numel(candidateRecords)
    candidate = candidateRecords(idx);
    formationGain = minimumFormationGain( ...
        reference, candidate, raw.scenarioConfigSnapshot.sensorGroupIds);
    admissible(idx) = ...
        candidate.meanEospa < reference.meanEospa && ...
        relativeGain(reference.worstSensorEospa, ...
            candidate.worstSensorEospa) >= ...
                -protocol.maximumWorstSensorRegressionPercent - 1e-10 && ...
        formationGain >= ...
            -protocol.maximumMinimumFormationRegressionPercent - 1e-10 && ...
        relativeGain(reference.consensusOspa, ...
            candidate.consensusOspa) >= ...
                -protocol.maximumConsensusRegressionPercent - 1e-10 && ...
        relativeSaving(reference.attemptedBytes, ...
            candidate.attemptedBytes) >= ...
                -protocol.maximumAttemptedByteIncreasePercent - 1e-10 && ...
        candidate.selectedRollingB3SensorStrongFraction == 1 && ...
        candidate.selectedRollingB3FormationStrongFraction == 1 && ...
        abs(candidate.meanDirectedMessageCount - ...
            protocol.exactSelectedMessageCount(presetIdx)) <= 1e-12;
end
admissibleIndices = find(admissible);
if isempty(admissibleIndices)
    selected = reference;
    selectedIsDynamic = false;
else
    [~, localBest] = min([candidateRecords(admissibleIndices).meanEospa]);
    selected = candidateRecords(admissibleIndices(localBest));
    selectedIsDynamic = true;
end

summary = struct();
summary.protocolId = protocol.id;
summary.contractVersion = protocol.contractVersion;
summary.generatedAt = datestr(now, 31);
summary.presetName = raw.presetName;
summary.seed = raw.seeds;
summary.analysisWindow = reference.analysisWindow;
summary.referenceArmMode = reference.armMode;
summary.referenceMeanEospa = reference.meanEospa;
summary.selectedArmMode = selected.armMode;
summary.selectedMeanEospa = selected.meanEospa;
summary.selectedIsDynamic = selectedIsDynamic;
summary.oracleGainPercent = relativeGain( ...
    reference.meanEospa, selected.meanEospa);
summary.worstSensorGainPercent = relativeGain( ...
    reference.worstSensorEospa, selected.worstSensorEospa);
summary.minimumFormationGainPercent = minimumFormationGain( ...
    reference, selected, raw.scenarioConfigSnapshot.sensorGroupIds);
summary.consensusGainPercent = relativeGain( ...
    reference.consensusOspa, selected.consensusOspa);
summary.attemptedByteSavingPercent = relativeSaving( ...
    reference.attemptedBytes, selected.attemptedBytes);
summary.admissibleDynamicCount = nnz(admissible);
summary.dynamicArmCount = numel(candidateRecords);
summary.pilotMeanGatePassed = ...
    summary.oracleGainPercent >= ...
        protocol.minimumMeanOracleGainPercent;
summary.codebook = records;
summary.dynamicAdmissible = admissible;
summary.claimBoundary = protocol.claimBoundary;
end

function gain = minimumFormationGain(reference, candidate, groupIds)
groups = unique(groupIds, 'stable');
gain = inf;
for groupIdx = 1:numel(groups)
    mask = groupIds == groups(groupIdx);
    referenceMean = mean(reference.sensorMeanEospa(mask));
    candidateMean = mean(candidate.sensorMeanEospa(mask));
    gain = min(gain, relativeGain(referenceMean, candidateMean));
end
end

function writePilotReport(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write V152 pilot report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V152 safe graph-codebook oracle pilot\n\n');
fprintf(fid, '- Preset / seed / window: `%s / %d / %d:%d`\n', ...
    summary.presetName, summary.seed, summary.analysisWindow);
fprintf(fid, '- Better static reference: `%s`\n', ...
    summary.referenceArmMode);
fprintf(fid, '- Selected oracle action: `%s`\n', ...
    summary.selectedArmMode);
fprintf(fid, '- Admissible dynamic graphs: `%d / %d`\n\n', ...
    summary.admissibleDynamicCount, summary.dynamicArmCount);
fprintf(fid, ['| Mean gain | Worst-sensor gain | Minimum-formation gain | ', ...
    'Consensus gain | Attempted-byte saving | 5%% pilot gate |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|:--:|\n');
fprintf(fid, '| %+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% | %d |\n\n', ...
    summary.oracleGainPercent, summary.worstSensorGainPercent, ...
    summary.minimumFormationGainPercent, ...
    summary.consensusGainPercent, ...
    summary.attemptedByteSavingPercent, ...
    summary.pilotMeanGatePassed);
fprintf(fid, '## Codebook\n\n');
fprintf(fid, '| Arm | Mean E-OSPA | Worst sensor | Consensus | Bytes | Admissible |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|:--:|\n');
dynamicCursor = 0;
for idx = 1:numel(summary.codebook)
    record = summary.codebook(idx);
    isDynamic = startsWithCompat( ...
        {record.armMode}, 'v152-safe-graph-rank');
    if isDynamic
        dynamicCursor = dynamicCursor + 1;
        admissible = summary.dynamicAdmissible(dynamicCursor);
    else
        admissible = strcmp(record.armMode, summary.referenceArmMode);
    end
    fprintf(fid, '| `%s` | %.6f | %.6f | %.6f | %.0f | %d |\n', ...
        record.armMode, record.meanEospa, ...
        record.worstSensorEospa, record.consensusOspa, ...
        record.attemptedBytes, admissible);
end
fprintf(fid, '\n## Boundary\n\n%s\n', summary.claimBoundary);
end

function result = startsWithCompat(values, prefix)
result = cellfun(@(value) strncmp(value, prefix, numel(prefix)), values);
end

function gain = relativeGain(referenceValue, candidateValue)
gain = 100 * (referenceValue - candidateValue) / ...
    max(abs(referenceValue), eps);
end

function saving = relativeSaving(referenceValue, candidateValue)
saving = 100 * (referenceValue - candidateValue) / ...
    max(abs(referenceValue), eps);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
