function [reportPath, probe] = ...
    runFormationH3CoordinatedSubsetRepairProbe(options)
% RUNFORMATIONH3COORDINATEDSUBSETREPAIRPROBE Higher-order repair at step 3.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationH3CoordinatedSubsetRepairProbeProtocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('Official coordinated-subset probe requires clean source.');
end
outputRoot = getField(options, 'outputDirectory', protocol.outputRoot);
probePath = fullfile(outputRoot, ...
    'formation_h3_coordinated_subset_repair_probe_v1.mat');
reportPath = fullfile(outputRoot, ...
    'FORMATION_H3_COORDINATED_SUBSET_REPAIR_PROBE_V1.md');
probeExists = exist(probePath, 'file') == 2;
reportExists = exist(reportPath, 'file') == 2;
if xor(probeExists, reportExists)
    error('Coordinated-subset probe artifacts are partial.');
elseif probeExists
    loaded = load(probePath, 'probe');
    if ~isfield(loaded, 'probe') || ...
            ~strcmp(loaded.probe.protocolId, protocol.id) || ...
            ~strcmp(loaded.probe.generationGitCommit, gitState.commit)
        error('Coordinated-subset probe provenance mismatch.');
    end
    probe = loaded.probe;
    fprintf('Reused coordinated-subset probe: %s\n', probePath);
    return;
end

reference = protocol.referenceActionIndex;
prefix = protocol.prefixActionIndices;
repairActions = protocol.coordinatedRepairActionIndices;
sequences = [ ...
    reference, reference, reference; ...
    prefix, reference; ...
    repmat(prefix, numel(repairActions), 1), repairActions(:)];
screenDirectory = fullfile(outputRoot, 'screen');
screenOptions = struct( ...
    'cacheRoot', protocol.cacheRoot, ...
    'outputDirectory', screenDirectory, ...
    'interventionBankType', protocol.interventionBankType, ...
    'actionSequenceIndices', sequences, ...
    'writeReport', true);
[screenReportPath, screen] = ...
    runFormationModeH3OpenedReturnScreen( ...
        protocol.presetName, protocol.seed, ...
        protocol.currentTime, screenOptions);
if screen.bank.actionCount ~= protocol.expectedExpandedActionCount
    error('Coordinated-subset action count changed.');
end
targets = targetMatrix(screen);
prefixRow = find(all(screen.actionSequenceIndices == ...
    [prefix, reference], 2));
if numel(prefixRow) ~= 1 || ...
        max(abs(targets(prefixRow, :) - ...
            protocol.expectedPrefixTargets)) > ...
                protocol.reproductionTolerancePercent
    error('Coordinated-subset probe does not reproduce the v18 prefix.');
end

strictFeasible = all(targets >= -1e-12, 2);
strictIndices = find(strictFeasible);
[strictGain, localBest] = max(targets(strictIndices, 1));
strictOracleIndex = strictIndices(localBest);
nonreference = ~all(sequences == reference, 2);
nonreferenceIndices = find(nonreference);
[bestConsensus, localConsensus] = max( ...
    targets(nonreferenceIndices, 4));
bestConsensusIndex = nonreferenceIndices(localConsensus);

probe = struct();
probe.contractVersion = ...
    'formation-h3-coordinated-subset-repair-probe-result-v1';
probe.protocolId = protocol.id;
probe.generatedAt = datestr(now, 31);
probe.generationGitCommit = gitState.commit;
probe.generationTrackedWorktreeDirty = ...
    gitState.trackedWorktreeDirty;
probe.generationUntrackedSourceFiles = ...
    gitState.untrackedSourceFiles;
probe.cacheProtocolId = protocol.cacheProtocolId;
probe.cacheGenerationGitCommit = ...
    protocol.cacheGenerationGitCommit;
probe.presetName = protocol.presetName;
probe.seed = protocol.seed;
probe.currentTime = protocol.currentTime;
probe.sequences = sequences;
probe.targets = targets;
probe.actionNames = screen.bank.actionNames;
probe.actionFormationIndices = ...
    screen.bank.actionFormationIndices;
probe.actionInterventionOrder = ...
    screen.bank.actionInterventionOrder;
probe.strictFeasible = strictFeasible;
probe.strictFeasibleSequenceCount = nnz(strictFeasible);
probe.strictOracleIndex = strictOracleIndex;
probe.strictOracleSequence = sequences(strictOracleIndex, :);
probe.strictOracleTargets = targets(strictOracleIndex, :);
probe.strictOracleGainPercent = strictGain;
probe.strongSafeSequenceFound = ...
    strictGain >= protocol.strongGainPercent - 1e-12;
probe.bestConsensusIndex = bestConsensusIndex;
probe.bestConsensusSequence = sequences(bestConsensusIndex, :);
probe.bestConsensusTargets = targets(bestConsensusIndex, :);
probe.bestConsensusGainPercent = bestConsensus;
probe.screen = screen;
probe.screenReportPath = screenReportPath;
probe.actionSelectionUsesTruth = protocol.actionSelectionUsesTruth;
probe.actionSelectionUsesFutureMeasurements = ...
    protocol.actionSelectionUsesFutureMeasurements;
probe.runtimePolicyUsesTruth = protocol.runtimePolicyUsesTruth;
probe.openedTrainingMechanismProbeOnly = true;
probe.finalModelTrainingAuthorized = false;
probe.validationClaimAllowed = false;
probe.evidenceBoundary = protocol.evidenceBoundary;
probe.probePath = probePath;
probe.reportPath = reportPath;

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
save('-mat7-binary', probePath, 'probe');
writeReport(reportPath, probe);
fprintf('Coordinated-subset repair probe: %s\n', probePath);
fprintf('Strict oracle: [%s], gain=%+.6f%%, strong=%d\n', ...
    strtrim(sprintf('%d ', probe.strictOracleSequence)), ...
    probe.strictOracleGainPercent, probe.strongSafeSequenceFound);
end

function targets = targetMatrix(screen)
records = screen.records;
outcomes = screen.outcomes;
reference = outcomes(screen.referenceSubsetIndex);
delivered = zeros(numel(outcomes), 1);
for idx = 1:numel(outcomes)
    delivered(idx) = 100 * ...
        (reference.deliveredBytes - outcomes(idx).deliveredBytes) / ...
        max(reference.deliveredBytes, eps);
end
targets = [ ...
    reshape([records.meanGainPercent], [], 1), ...
    reshape([records.minimumFormationGainPercent], [], 1), ...
    reshape([records.worstGainPercent], [], 1), ...
    reshape([records.consensusGainPercent], [], 1), ...
    reshape([records.attemptedByteSavingPercent], [], 1), ...
    delivered];
end

function writeReport(path, probe)
fid = fopen(path, 'w');
if fid < 0
    error('Cannot write coordinated-subset repair report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# Formation H=3 coordinated-subset repair probe\n\n');
fprintf(fid, '- Contract / protocol: `%s / %s`\n', ...
    probe.contractVersion, probe.protocolId);
fprintf(fid, '- Generation commit: `%s`\n', ...
    probe.generationGitCommit);
fprintf(fid, '- Cache protocol / commit: `%s / %s`\n', ...
    probe.cacheProtocolId, probe.cacheGenerationGitCommit);
fprintf(fid, '- Preset / seed / time: `%s / %d / %d`\n', ...
    probe.presetName, probe.seed, probe.currentTime);
fprintf(fid, '- Strict feasible: `%d/%d`\n', ...
    probe.strictFeasibleSequenceCount, size(probe.sequences, 1));
fprintf(fid, '- Strict oracle / gain: `%s / %+.6f%%`\n', ...
    mat2str(probe.strictOracleSequence), ...
    probe.strictOracleGainPercent);
fprintf(fid, '- Strong safe sequence found: `%d`\n\n', ...
    probe.strongSafeSequenceFound);

fprintf(fid, ['| Sequence | Third action | Order | Mean | ', ...
    'Min. formation | Worst sensor | Consensus | Attempted bytes | ', ...
    'Delivered bytes | Strict |\n']);
fprintf(fid, '|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--:|\n');
for idx = 1:size(probe.sequences, 1)
    sequence = probe.sequences(idx, :);
    actionIdx = sequence(3);
    targets = probe.targets(idx, :);
    fprintf(fid, ['| `%s` | %s | %d | %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% | %d |\n'], ...
        strtrim(sprintf('%d ', sequence)), ...
        probe.actionNames{actionIdx}, ...
        probe.actionInterventionOrder(actionIdx), ...
        targets(1), targets(2), targets(3), targets(4), ...
        targets(5), targets(6), probe.strictFeasible(idx));
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    probe.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
