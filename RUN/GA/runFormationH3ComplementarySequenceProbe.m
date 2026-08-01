function [reportPath, probe] = ...
    runFormationH3ComplementarySequenceProbe(options)
% RUNFORMATIONH3COMPLEMENTARYSEQUENCEPROBE Privileged local-action beam.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationH3ComplementarySequenceProbeProtocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('Official complementary-sequence probe requires clean source.');
end
outputRoot = getField(options, 'outputDirectory', protocol.outputRoot);
probePath = fullfile(outputRoot, ...
    'formation_h3_complementary_sequence_probe_v1.mat');
reportPath = fullfile(outputRoot, ...
    'FORMATION_H3_COMPLEMENTARY_SEQUENCE_PROBE_V1.md');
probeExists = exist(probePath, 'file') == 2;
reportExists = exist(reportPath, 'file') == 2;
if xor(probeExists, reportExists)
    error('Complementary-sequence probe artifacts are partial.');
elseif probeExists
    loaded = load(probePath, 'probe');
    if ~isfield(loaded, 'probe') || ...
            ~strcmp(loaded.probe.protocolId, protocol.id) || ...
            ~strcmp(loaded.probe.generationGitCommit, gitState.commit)
        error('Complementary-sequence probe provenance mismatch.');
    end
    probe = loaded.probe;
    fprintf('Reused complementary-sequence probe: %s\n', probePath);
    return;
end

reference = protocol.referenceActionIndex;
repairs = reshape(protocol.repairActionIndices, 1, []);
stageOneSequences = [ ...
    repmat(reference, 1, protocol.horizonSteps); ...
    [repmat(protocol.exploitActionIndex, numel(repairs), 1), ...
     repairs(:), repmat(reference, numel(repairs), 1)]];
stageOne = loadOrRunStage( ...
    fullfile(outputRoot, 'stage1'), stageOneSequences, ...
    protocol, gitState.commit);
stageOneTargets = targetMatrix(stageOne);

exploitRow = find(all(stageOne.actionSequenceIndices == ...
    [protocol.exploitActionIndex, reference, reference], 2));
if numel(exploitRow) ~= 1 || ...
        abs(stageOneTargets(exploitRow, 1) - ...
            protocol.expectedExploitMeanGainPercent) > ...
                protocol.reproductionTolerancePercent || ...
        abs(stageOneTargets(exploitRow, 4) - ...
            protocol.expectedExploitConsensusGainPercent) > ...
                protocol.reproductionTolerancePercent
    error('Stage-one exploit arm does not reproduce the v17 ceiling.');
end

repairRows = zeros(1, numel(repairs));
for idx = 1:numel(repairs)
    repairRows(idx) = findSequence(stageOne, ...
        [protocol.exploitActionIndex, repairs(idx), reference]);
end
[beamActions, beamSelection] = ...
    selectFormationH3ComplementarySequenceBeams( ...
        repairs, stageOneTargets(repairRows, :), ...
        protocol.beamWidth);

thirdActions = repairs(repairs ~= reference);
stageTwoSequences = repmat(reference, 1, protocol.horizonSteps);
for beamAction = beamActions
    additions = [ ...
        repmat(protocol.exploitActionIndex, numel(thirdActions), 1), ...
        repmat(beamAction, numel(thirdActions), 1), ...
        thirdActions(:)];
    stageTwoSequences = [stageTwoSequences; additions]; %#ok<AGROW>
end
if size(unique(stageTwoSequences, 'rows'), 1) ~= ...
        size(stageTwoSequences, 1)
    error('Stage-two complementary sequences are not unique.');
end

stageTwo = loadOrRunStage( ...
    fullfile(outputRoot, 'stage2'), stageTwoSequences, ...
    protocol, gitState.commit);
assertReferenceEqual(stageOne, stageTwo);
stageTwoTargets = targetMatrix(stageTwo);

stageTwoNonreference = ~all( ...
    stageTwo.actionSequenceIndices == reference, 2);
allSequences = [ ...
    stageOne.actionSequenceIndices; ...
    stageTwo.actionSequenceIndices(stageTwoNonreference, :)];
allTargets = [ ...
    stageOneTargets; stageTwoTargets(stageTwoNonreference, :)];
if size(unique(allSequences, 'rows'), 1) ~= size(allSequences, 1)
    error('Combined complementary sequence set contains duplicates.');
end

strictFeasible = all(allTargets >= -1e-12, 2);
strictIndices = find(strictFeasible);
[strictGain, localBest] = max(allTargets(strictIndices, 1));
strictOracleIndex = strictIndices(localBest);
strongSafeSequenceFound = ...
    strictGain >= protocol.strongGainPercent - 1e-12;

probe = struct();
probe.contractVersion = ...
    'formation-h3-complementary-sequence-probe-result-v1';
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
probe.stageOne = stageOne;
probe.stageTwo = stageTwo;
probe.stageOneSequences = stageOneSequences;
probe.stageOneTargets = stageOneTargets;
probe.beamActions = beamActions;
probe.beamSelection = beamSelection;
probe.stageTwoSequences = stageTwoSequences;
probe.stageTwoTargets = stageTwoTargets;
probe.allSequences = allSequences;
probe.allTargets = allTargets;
probe.strictFeasible = strictFeasible;
probe.strictFeasibleSequenceCount = nnz(strictFeasible);
probe.strictOracleIndex = strictOracleIndex;
probe.strictOracleSequence = allSequences(strictOracleIndex, :);
probe.strictOracleTargets = allTargets(strictOracleIndex, :);
probe.strictOracleGainPercent = strictGain;
probe.strongSafeSequenceFound = strongSafeSequenceFound;
probe.actionSelectionUsesTruth = ...
    protocol.actionSelectionUsesTruth;
probe.actionSelectionUsesFutureMeasurements = ...
    protocol.actionSelectionUsesFutureMeasurements;
probe.runtimePolicyUsesTruth = protocol.runtimePolicyUsesTruth;
probe.openedTrainingMechanismProbeOnly = ...
    protocol.openedTrainingMechanismProbeOnly;
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
fprintf('Complementary-sequence probe: %s\n', probePath);
fprintf('Strict oracle: [%s], gain=%+.6f%%, strong=%d\n', ...
    strtrim(sprintf('%d ', probe.strictOracleSequence)), ...
    probe.strictOracleGainPercent, probe.strongSafeSequenceFound);
end

function screen = loadOrRunStage( ...
        directory, sequences, protocol, generationCommit)
stem = sprintf('FORMATION_MODE_H3_%s_SEED%d_T%d', ...
    upper(strrep(protocol.presetName, '-', '_')), ...
    protocol.seed, protocol.currentTime);
matPath = fullfile(directory, [stem, '.mat']);
reportPath = fullfile(directory, [stem, '.md']);
matExists = exist(matPath, 'file') == 2;
reportExists = exist(reportPath, 'file') == 2;
if xor(matExists, reportExists)
    error('Complementary-sequence stage artifacts are partial.');
elseif matExists
    loaded = load(matPath, 'screen');
    screen = loaded.screen;
    if ~strcmp(screen.generationGitCommit, generationCommit) || ...
            ~screen.explicitActionSequenceEnabled || ...
            ~isequal(screen.actionSequenceIndices, sequences)
        error('Complementary-sequence stage provenance mismatch.');
    end
    fprintf('Reused complementary-sequence stage: %s\n', matPath);
    return;
end
options = struct( ...
    'cacheRoot', protocol.cacheRoot, ...
    'outputDirectory', directory, ...
    'actionSequenceIndices', sequences, ...
    'writeReport', true);
[~, screen] = runFormationModeH3OpenedReturnScreen( ...
    protocol.presetName, protocol.seed, protocol.currentTime, options);
end

function row = findSequence(screen, sequence)
row = find(all(screen.actionSequenceIndices == sequence, 2));
if numel(row) ~= 1
    error('Complementary-sequence stage lacks a unique expected arm.');
end
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

function assertReferenceEqual(first, second)
a = first.outcomes(first.referenceSubsetIndex);
b = second.outcomes(second.referenceSubsetIndex);
fields = { ...
    'meanEospa', 'worstSensorEospa', 'formationMeanEospa', ...
    'consensusOspa', 'attemptedBytes', 'deliveredBytes', ...
    'selectedAdjacency', 'deliveredAdjacency'};
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isequaln(a.(name), b.(name))
        error('Stage reference outcomes differ in %s.', name);
    end
end
end

function writeReport(path, probe)
fid = fopen(path, 'w');
if fid < 0
    error('Cannot write complementary-sequence report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# Formation H=3 complementary-sequence beam probe\n\n');
fprintf(fid, '- Contract / protocol: `%s / %s`\n', ...
    probe.contractVersion, probe.protocolId);
fprintf(fid, '- Generation commit: `%s`\n', ...
    probe.generationGitCommit);
fprintf(fid, '- Cache protocol / commit: `%s / %s`\n', ...
    probe.cacheProtocolId, probe.cacheGenerationGitCommit);
fprintf(fid, '- Preset / seed / time: `%s / %d / %d`\n', ...
    probe.presetName, probe.seed, probe.currentTime);
fprintf(fid, '- Stage-one / stage-two / total arms: `%d / %d / %d`\n', ...
    size(probe.stageOneSequences, 1), ...
    size(probe.stageTwoSequences, 1), ...
    size(probe.allSequences, 1));
fprintf(fid, '- Beam second actions: `%s`\n', ...
    mat2str(probe.beamActions));
fprintf(fid, '- Strict-feasible sequences: `%d/%d`\n', ...
    probe.strictFeasibleSequenceCount, size(probe.allSequences, 1));
fprintf(fid, '- Strict oracle sequence / gain: `%s / %+.6f%%`\n', ...
    mat2str(probe.strictOracleSequence), ...
    probe.strictOracleGainPercent);
fprintf(fid, '- Strong safe sequence found: `%d`\n\n', ...
    probe.strongSafeSequenceFound);

fprintf(fid, '## Beam selection\n\n');
fprintf(fid, '| Second action | Criterion | Six targets |\n');
fprintf(fid, '|--:|:--|:--|\n');
for idx = 1:numel(probe.beamActions)
    fprintf(fid, '| %d | %s | `%s` |\n', ...
        probe.beamActions(idx), ...
        probe.beamSelection.selectionCriteria{idx}, ...
        compactVector(probe.beamSelection.selectedTargets(idx, :)));
end

fprintf(fid, '\n## All evaluated sequences\n\n');
fprintf(fid, ['| Sequence | Mean | Min. formation | Worst sensor | ', ...
    'Consensus | Attempted bytes | Delivered bytes | Strict |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|:--:|\n');
for idx = 1:size(probe.allSequences, 1)
    targets = probe.allTargets(idx, :);
    fprintf(fid, '| `%s` | %+.3f%% | %+.3f%% | %+.3f%% | ', ...
        strtrim(sprintf('%d ', probe.allSequences(idx, :))), ...
        targets(1), targets(2), targets(3));
    fprintf(fid, '%+.3f%% | %+.3f%% | %+.3f%% | %d |\n', ...
        targets(4), targets(5), targets(6), ...
        probe.strictFeasible(idx));
end

fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    probe.evidenceBoundary);
end

function text = compactVector(values)
text = strtrim(sprintf('%+.3f ', values));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
