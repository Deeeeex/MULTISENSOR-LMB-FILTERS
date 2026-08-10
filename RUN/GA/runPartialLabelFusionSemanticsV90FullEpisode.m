function [reportPath, result] = ...
    runPartialLabelFusionSemanticsV90FullEpisode(options)
% RUNPARTIALLABELFUSIONSEMANTICSV90FULLEPISODE Paired receiver comparison.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPartialLabelFusionSemanticsV90Protocol();
sourceProtocol = getRepeatedMultiGatewayHandoverV89Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('PartialLabelFusionV90:DirtySource', ...
        'The V90 full-episode outcome requires clean frozen source.');
end

presetNames = normalizeSelection(getField( ...
    options, 'presetNames', protocol.presets), protocol.presets, ...
    'PartialLabelFusionV90:InvalidPresetSelection');
receiverModes = normalizeSelection(getField( ...
    options, 'receiverModes', protocol.receiverModes), ...
    protocol.receiverModes, ...
    'PartialLabelFusionV90:InvalidReceiverModeSelection');
reuseLegacy = logical(getField(options, 'reuseLegacyV89', true));
resume = logical(getField(options, 'resume', true));

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = getField(options, 'outputRoot', protocol.outputRoot);
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'PARTIAL_LABEL_FUSION_SEMANTICS_V90_FULL_EPISODE.mat');
reportPath = fullfile(outputRoot, ...
    'PARTIAL_LABEL_FUSION_SEMANTICS_V90_FULL_EPISODE.md');

result = initializeResult( ...
    protocol, gitState.commit, matPath, reportPath);
if resume && exist(matPath, 'file') == 2
    loaded = load(matPath, 'result');
    if ~isfield(loaded, 'result') || ...
            ~strcmp(loaded.result.contractVersion, ...
                result.contractVersion) || ...
            ~strcmp(loaded.result.generationGitCommit, gitState.commit)
        error('PartialLabelFusionV90:InvalidResumeState', ...
            'The V90 checkpoint belongs to another source contract.');
    end
    result = loaded.result;
end

legacy = struct();
if any(strcmp(receiverModes, 'support-renormalized')) && reuseLegacy
    legacy = loadLegacyResult(repoRoot);
end

for modeIdx = 1:numel(receiverModes)
    receiverMode = receiverModes{modeIdx};
    for presetIdx = 1:numel(presetNames)
        presetName = presetNames{presetIdx};
        if hasRecord(result.records, receiverMode, presetName)
            fprintf('V90 resume: %s / %s already complete.\n', ...
                receiverMode, presetName);
            continue;
        end
        fprintf('\nV90 receiver %s: %s (%d/%d, %d/%d)\n', ...
            receiverMode, presetName, modeIdx, numel(receiverModes), ...
            presetIdx, numel(presetNames));
        if strcmp(receiverMode, 'support-renormalized') && reuseLegacy
            record = reuseLegacyRecord(legacy, presetName);
        else
            record = executeRecord( ...
                receiverMode, presetName, protocol, sourceProtocol);
        end
        result.records(end + 1) = record; %#ok<AGROW>
        result.completedAt = datestr(now, 31);
        result = refreshCompletionFlags(result, protocol);
        save('-mat7-binary', matPath, 'result');
        writeReport(reportPath, result);
        fprintf(['  V90 improvement: full %+.3f%% focus %+.3f%% ', ...
            'worst %+.3f%% formation %+.3f%% consensus %+.3f%% ', ...
            'strong=%d\n'], ...
            record.improvement.fullHorizonMeanGainPercent, ...
            record.improvement.focusWindowMeanGainPercent, ...
            record.improvement.worstSensorGainPercent, ...
            record.improvement.minimumFormationGainPercent, ...
            record.improvement.focusConsensusGainPercent, ...
            record.improvement.strongGatePassed);
    end
end
result.completedAt = datestr(now, 31);
result = refreshCompletionFlags(result, protocol);
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('\nV90 full-episode result: %s\n', reportPath);
end

function record = executeRecord( ...
        receiverMode, presetName, protocol, sourceProtocol)
registryIdx = find(strcmp(presetName, protocol.presets), 1);
seed = protocol.allSeeds;
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
if inputs.config.numberOfSensors ~= ...
        protocol.expectedNodeCounts(registryIdx) || ...
        inputs.config.formationCount ~= ...
            protocol.expectedFormationCounts(registryIdx)
    error('PartialLabelFusionV90:ScenarioScaleDrift', ...
        'The generated V90 scene changed its registered scale.');
end
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
deliverySeed = sourceProtocol.deliverySeedOffset + ...
    100 * seed + registryIdx;
[inputs.commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        deliverySeed, identity.sensorPhysicalUids, ...
        inputs.config.simulationLength);
filterSeed = sourceProtocol.filterSeedOffset + 100 * seed + registryIdx;
filterModel = removeRealizedTargetTruth(inputs.model);
representatives = selectFormationRepresentatives( ...
    inputs.config.sensorGroupIds);
armIds = {protocol.referenceArmId, protocol.candidateArmId};
arms = cell(1, 2);
for armIdx = 1:2
    armId = armIds{armIdx};
    config = buildPartialLabelFusionSemanticsV90TriggerConfig( ...
        armId, inputs.config.numberOfSensors, receiverMode);
    context = buildPartialLabelFusionSemanticsV90ExecutionContext( ...
        presetName, seed, armId, receiverMode, ...
        inputs.config.simulationLength);
    rng(filterSeed, 'twister');
    armTimer = tic;
    fprintf('  running %s / %s ...\n', armId, receiverMode);
    [estimates, diagnostics] = ...
        runEventTriggeredDistributedLmbFilter( ...
            filterModel, inputs.measurements, ...
            inputs.sensorTrajectories, inputs.neighborMap, ...
            inputs.commConfig, config, context);
    arms{armIdx} = summarizeRepeatedMultiGatewayFullEpisodeArm( ...
        armId, estimates, diagnostics, inputs.groundTruthRfs, ...
        inputs.config, representatives, sourceProtocol, toc(armTimer));
    fprintf(['    E-OSPA full %.3f focus %.3f; gateways %d; ', ...
        'fallbacks %d; %.1f s\n'], ...
        arms{armIdx}.fullHorizonPositionEospa, ...
        arms{armIdx}.focusWindowPositionEospa, ...
        arms{armIdx}.selectedGatewayCount, ...
        arms{armIdx}.referenceFallbackCount, ...
        arms{armIdx}.elapsedSeconds);
    clear estimates diagnostics;
end
record = emptyRecord();
record.receiverMode = receiverMode;
record.presetName = presetName;
record.seed = seed;
record.numberOfSensors = inputs.config.numberOfSensors;
record.formationCount = inputs.config.formationCount;
record.deliverySeed = deliverySeed;
record.filterSeed = filterSeed;
record.reference = arms{1};
record.candidate = arms{2};
record.improvement = ...
    summarizeRepeatedMultiGatewayFullEpisodeImprovement( ...
        record.reference, record.candidate, sourceProtocol);
record.reusedLegacyV89 = false;
end

function legacy = loadLegacyResult(repoRoot)
path = fullfile(repoRoot, 'RUN', 'GA', 'dynamic_topology', ...
    'evidence', 'tracking_aligned_v89', ...
    'multi_gateway_handover_full_episode', ...
    'REPEATED_MULTI_GATEWAY_HANDOVER_V89_FULL_EPISODE.mat');
loaded = load(path, 'result');
if ~isfield(loaded, 'result') || ...
        ~strcmp(loaded.result.contractVersion, ...
            ['repeated-multi-gateway-handover-v89-', ...
             'full-episode-result-v1'])
    error('PartialLabelFusionV90:MissingLegacyControl', ...
        'A completed V89 legacy result is required for reuse.');
end
legacy = loaded.result;
end

function record = reuseLegacyRecord(legacy, presetName)
idx = find(strcmp({legacy.records.presetName}, presetName), 1);
if isempty(idx)
    error('PartialLabelFusionV90:LegacyScaleMissing', ...
        'The V89 legacy result does not contain %s.', presetName);
end
source = legacy.records(idx);
record = emptyRecord();
record.receiverMode = 'support-renormalized';
record.presetName = source.presetName;
record.seed = source.seed;
record.numberOfSensors = source.numberOfSensors;
record.formationCount = source.formationCount;
record.deliverySeed = source.deliverySeed;
record.filterSeed = source.filterSeed;
record.reference = source.reference;
record.candidate = source.candidate;
record.improvement = source.improvement;
record.reusedLegacyV89 = true;
end

function result = initializeResult(protocol, commit, matPath, reportPath)
result = struct();
result.contractVersion = ...
    'partial-label-fusion-semantics-v90-full-episode-result-v1';
result.protocol = protocol;
result.startedAt = datestr(now, 31);
result.completedAt = '';
result.generationGitCommit = commit;
result.records = repmat(emptyRecord(), 1, 0);
result.allModeScaleRecordsComplete = false;
result.fovAwareBothScalesStrongGatePassed = false;
result.multistyleScoringAuthorized = false;
result.modelTrainingAuthorized = false;
result.validationClaimAllowed = false;
result.developmentEvidenceOnly = true;
result.matPath = matPath;
result.reportPath = reportPath;
end

function result = refreshCompletionFlags(result, protocol)
complete = true;
for modeIdx = 1:numel(protocol.receiverModes)
    for presetIdx = 1:numel(protocol.presets)
        complete = complete && hasRecord( ...
            result.records, protocol.receiverModes{modeIdx}, ...
            protocol.presets{presetIdx});
    end
end
fovStrong = true;
for presetIdx = 1:numel(protocol.presets)
    idx = findRecord(result.records, ...
        'fov-aware-censored', protocol.presets{presetIdx});
    fovStrong = fovStrong && ~isempty(idx) && ...
        result.records(idx).improvement.strongGatePassed;
end
result.allModeScaleRecordsComplete = complete;
result.fovAwareBothScalesStrongGatePassed = fovStrong;
result.multistyleScoringAuthorized = complete && fovStrong;
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('PartialLabelFusionV90:ReportWriteFailed', ...
        'Could not create the V90 full-episode report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V90 partial-label fusion semantics result\n\n');
fprintf(fid, ['Positive percentages mean the frozen V89 route is ', ...
    'better than the physical-tree reference under the same receiver.\n\n']);
fprintf(fid, ['| Receiver | Scale | Reference | Candidate | Full | Focus | ', ...
    'Worst | Formation | Consensus | Card. | Strong | Reused |\n']);
fprintf(fid, '|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|\n');
for record = result.records
    gain = record.improvement;
    fprintf(fid, ['| %s | %s | %.3f | %.3f | %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% | %+.3f%% | %+.3f%% | %+.3f%% | %d | %d |\n'], ...
        record.receiverMode, record.presetName, ...
        record.reference.fullHorizonPositionEospa, ...
        record.candidate.fullHorizonPositionEospa, ...
        gain.fullHorizonMeanGainPercent, ...
        gain.focusWindowMeanGainPercent, ...
        gain.worstSensorGainPercent, ...
        gain.minimumFormationGainPercent, ...
        gain.focusConsensusGainPercent, ...
        gain.cardinalityGainPercent, ...
        gain.strongGatePassed, record.reusedLegacyV89);
end
fprintf(fid, '\n- All receiver/scale controls complete: `%d`\n', ...
    result.allModeScaleRecordsComplete);
fprintf(fid, '- FoV-aware route passes both scales: `%d`\n', ...
    result.fovAwareBothScalesStrongGatePassed);
fprintf(fid, '- Multistyle scoring authorized: `%d`\n', ...
    result.multistyleScoringAuthorized);
fprintf(fid, '- Validation claim allowed: `0`\n');
fprintf(fid, '\n## Receiver-semantics activity\n\n');
fprintf(fid, ['| Receiver | Scale | Arm | Missing inputs | Censored ', ...
    'inside FoV | Legacy excluded | FoV-excluded | Stale ignored | ', ...
    'Vetoed labels |\n']);
fprintf(fid, '|:--|:--|:--|--:|--:|--:|--:|--:|--:|\n');
for record = result.records
    writeReceiverActivityRow(fid, record.receiverMode, ...
        record.presetName, 'reference', record.reference);
    writeReceiverActivityRow(fid, record.receiverMode, ...
        record.presetName, 'candidate', record.candidate);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.protocol.evidenceBoundary);
end

function writeReceiverActivityRow(fid, receiverMode, presetName, arm, summary)
fprintf(fid, ['| %s | %s | %s | %s | %s | %s | %s | ', ...
    '%s | %s |\n'], ...
    receiverMode, presetName, arm, ...
    formatSummaryCount(summary, 'missingLabelSourceCount'), ...
    formatSummaryCount( ...
        summary, 'missingLabelObservableCensoredSourceCount'), ...
    formatSummaryCount(summary, 'missingLabelLegacyExcludedSourceCount'), ...
    formatSummaryCount( ...
        summary, 'missingLabelUninformativeExcludedSourceCount'), ...
    formatSummaryCount(summary, 'missingLabelStaleIgnoredSourceCount'), ...
    formatSummaryCount(summary, 'missingLabelStrictVetoedLabelCount'));
end

function value = formatSummaryCount(summary, name)
if isstruct(summary) && isfield(summary, name)
    value = sprintf('%.0f', summary.(name));
else
    value = '--';
end
end

function found = hasRecord(records, receiverMode, presetName)
found = ~isempty(findRecord(records, receiverMode, presetName));
end

function idx = findRecord(records, receiverMode, presetName)
idx = [];
for recordIdx = 1:numel(records)
    if strcmp(records(recordIdx).receiverMode, receiverMode) && ...
            strcmp(records(recordIdx).presetName, presetName)
        idx = recordIdx;
        return;
    end
end
end

function values = normalizeSelection(values, allowed, errorId)
if ischar(values)
    values = {values};
end
if ~iscell(values) || isempty(values) || ...
        any(~cellfun(@(value) ischar(value) && ...
            any(strcmp(value, allowed)), values))
    error(errorId, 'The requested V90 selection is not registered.');
end
values = unique(values, 'stable');
end

function record = emptyRecord()
record = struct( ...
    'receiverMode', '', ...
    'presetName', '', ...
    'seed', NaN, ...
    'numberOfSensors', NaN, ...
    'formationCount', NaN, ...
    'deliverySeed', NaN, ...
    'filterSeed', NaN, ...
    'reference', struct(), ...
    'candidate', struct(), ...
    'improvement', struct(), ...
    'reusedLegacyV89', false);
end

function model = removeRealizedTargetTruth(model)
if isfield(model, 'explicitTargetTrajectories')
    model = rmfield(model, 'explicitTargetTrajectories');
end
fields = {'targetTrajectories', 'target'};
for fieldIdx = 1:numel(fields)
    if isfield(model.dynamicTopologyScenario, fields{fieldIdx})
        model.dynamicTopologyScenario = rmfield( ...
            model.dynamicTopologyScenario, fields{fieldIdx});
    end
end
end

function sensors = selectFormationRepresentatives(groupIds)
groups = unique(groupIds, 'stable');
sensors = zeros(1, numel(groups));
for formationIdx = 1:numel(groups)
    sensors(formationIdx) = find(groupIds == groups(formationIdx), 1);
end
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && path(1) == filesep;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
