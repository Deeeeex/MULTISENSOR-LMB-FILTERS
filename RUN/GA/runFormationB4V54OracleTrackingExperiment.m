function result = runFormationB4V54OracleTrackingExperiment( ...
        sourceResultPath, presetNames, seeds, outputDirectory)
% RUNFORMATIONB4V54ORACLETRACKINGEXPERIMENT Paired X36 headroom gate.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin < 1 || isempty(sourceResultPath)
    sourceResultPath = findLatestV49Result(repoRoot);
elseif ~isAbsolutePath(sourceResultPath)
    sourceResultPath = fullfile(repoRoot, sourceResultPath);
end
loaded = load(sourceResultPath, 'result');
if ~isfield(loaded, 'result') || ...
        ~strcmp(loaded.result.contractVersion, ...
            'formation-b4-v49-paired-tracking-development-result-v1')
    error('FormationB4V54Tracking:InvalidSourceResult', ...
        'A completed V49 paired result is required.');
end
source = loaded.result;
if nargin < 2 || isempty(presetNames)
    presetNames = {'x36-formation-fov-convoy'};
elseif ischar(presetNames)
    presetNames = {presetNames};
end
if nargin < 3 || isempty(seeds)
    seeds = 1009;
end
if nargin < 4 || isempty(outputDirectory)
    outputDirectory = fullfile(repoRoot, 'RUN', 'GA', ...
        'dynamic_topology', 'evidence', ...
        'formation_b4_v54_oracle_tracking_development');
elseif ~isAbsolutePath(outputDirectory)
    outputDirectory = fullfile(repoRoot, outputDirectory);
end
if exist(outputDirectory, 'dir') == 0
    mkdir(outputDirectory);
end

protocol = getFormationB4V49RuntimeProtocol();
candidateArmId = 'v54-receiver-safe-label-oracle';
cases = repmat(struct(), 1, numel(presetNames) * numel(seeds));
cursor = 0;
startedAt = datestr(now, 31);
totalTimer = tic;
for presetIdx = 1:numel(presetNames)
    presetName = presetNames{presetIdx};
    for seedIdx = 1:numel(seeds)
        seed = seeds(seedIdx);
        sourceRecord = findSourceRecord(source.cases, presetName, seed);
        cursor = cursor + 1;
        fprintf('\nV54 oracle tracking: %s seed %d (%d/%d)\n', ...
            presetName, seed, cursor, numel(cases));

        inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
        identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
        [inputs.commConfig.linkUniforms, ~] = ...
            materializePhysicalUidDirectedDeliveryUniforms( ...
                sourceRecord.deliverySeed, ...
                identity.sensorPhysicalUids, ...
                inputs.config.simulationLength);
        filterModel = removeRealizedTargetTruth(inputs.model);
        representativeSensors = selectFormationRepresentatives( ...
            inputs.config.sensorGroupIds);
        triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
            protocol.referenceArmId, inputs.config.numberOfSensors);
        triggerConfig.receiverSafeLabelFusionEnabled = true;
        triggerConfig.receiverSafeLabelFusionMode = 'oracle';
        triggerConfig.receiverSafeOracleByteBudgetFraction = 1;
        triggerConfig.receiverSafeSupportedExistenceThreshold = 0.50;
        triggerConfig.receiverSafeMaximumLogOddsDrop = log(4);
        context = buildFormationB4V49PairedTrackingDevelopmentContext( ...
            presetName, seed, protocol.referenceArmId);

        rng(sourceRecord.filterSeed, 'twister');
        armTimer = tic;
        [stateEstimates, diagnostics] = ...
            runEventTriggeredDistributedLmbFilter( ...
                filterModel, inputs.measurements, ...
                inputs.sensorTrajectories, inputs.neighborMap, ...
                inputs.commConfig, triggerConfig, context);
        elapsedSeconds = toc(armTimer);
        candidate = summarizeFormationB4TrackingArm( ...
            candidateArmId, stateEstimates, diagnostics, ...
            inputs.groundTruthRfs, inputs.config, ...
            representativeSensors, protocol, elapsedSeconds);
        candidate.receiverSafeOracle = summarizeReceiverSafeOracle( ...
            diagnostics);

        record = sourceRecord;
        record.candidate = candidate;
        record.improvement = summarizeFormationB4TrackingImprovement( ...
            record.reference, candidate);
        record.gate = assessGate(record);
        cases(cursor) = record;
        fprintf(['  V54 E-OSPA %.3f; improvement full %+.2f%%, ', ...
            'focus %+.2f%%, cardinality %+.2f%%, bytes %+.2f%%; ', ...
            'clamps=%d unresolved=%d gate=%s\n'], ...
            candidate.fullHorizonPositionEospa, ...
            record.improvement.fullHorizonPositionEospaPct, ...
            record.improvement.focusWindowPositionEospaPct, ...
            record.improvement.meanAbsoluteCardinalityErrorPct, ...
            percentImprovement(record.reference.attemptedPayloadBytes, ...
                candidate.attemptedPayloadBytes), ...
            candidate.receiverSafeOracle.retentionClampCount, ...
            candidate.receiverSafeOracle.retentionUnresolvedCount, ...
            record.gate.decision);
        clear inputs filterModel stateEstimates diagnostics;
    end
end

result = struct();
result.contractVersion = ...
    'formation-b4-v54-oracle-tracking-development-result-v1';
result.startedAt = startedAt;
result.completedAt = datestr(now, 31);
result.elapsedSeconds = toc(totalTimer);
result.referenceArmId = protocol.referenceArmId;
result.candidateArmId = candidateArmId;
result.sourceV49ResultPath = sourceResultPath;
result.cases = cases;
result.referenceRerun = false;
result.oracleUsesFullSenderPosteriors = true;
result.compactSynopsisBytesIncluded = true;
result.quantizationCalibrationPending = true;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.trainGnnAllowed = all(arrayfun( ...
    @(record) record.gate.passed, result.cases));

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
baseName = sprintf( ...
    'FORMATION_B4_V54_ORACLE_TRACKING_%s', timestamp);
result.matPath = fullfile(outputDirectory, [baseName, '.mat']);
result.reportPath = fullfile(outputDirectory, [baseName, '.md']);
save('-mat7-binary', result.matPath, 'result');
writeReport(result.reportPath, result);
fprintf('\nSaved: %s\nReport: %s\nTrain GNN allowed: %d\n', ...
    result.matPath, result.reportPath, result.trainGnnAllowed);
end

function summary = summarizeReceiverSafeOracle(diagnostics)
summary = struct();
summary.selectiveEdgeTimeCount = ...
    diagnostics.summary.receiverSafeSelectiveEdgeTimeCount;
summary.baselineFullBytesOnSelectiveEdges = ...
    diagnostics.summary.receiverSafeBaselineFullBytes;
summary.controlSynopsisBytes = ...
    diagnostics.summary.receiverSafeAttemptedControlSynopsisBytes;
summary.selectedPayloadBytes = ...
    diagnostics.summary.receiverSafeAttemptedSelectedPayloadBytes;
summary.selectiveByteSavingFraction = ...
    diagnostics.summary.receiverSafeAttemptedByteSavingFraction;
summary.meanActiveLabelCount = ...
    diagnostics.summary.receiverSafeMeanActiveLabelCount;
summary.maxActiveLabelCount = ...
    diagnostics.summary.receiverSafeMaxActiveLabelCount;
summary.positiveSupportCount = ...
    diagnostics.summary.receiverSafePositiveSupportCount;
summary.credibleNegativeCount = ...
    diagnostics.summary.receiverSafeCredibleNegativeCount;
summary.unsupportedAbsenceCount = ...
    diagnostics.summary.receiverSafeUnsupportedAbsenceCount;
summary.ambiguousEvidenceCount = ...
    diagnostics.summary.receiverSafeAmbiguousEvidenceCount;
summary.retentionViolationCount = ...
    diagnostics.summary.receiverSafeRetentionViolationCount;
summary.retentionRemovalCount = ...
    diagnostics.summary.receiverSafeRetentionRemovalCount;
summary.retentionClampCount = ...
    diagnostics.summary.receiverSafeRetentionClampCount;
summary.retentionFallbackCount = ...
    diagnostics.summary.receiverSafeRetentionFallbackCount;
summary.retentionUnresolvedCount = ...
    diagnostics.summary.receiverSafeRetentionUnresolvedCount;
end

function gate = assessGate(record)
fullImprove = record.improvement.fullHorizonPositionEospaPct;
cardImprove = record.improvement.meanAbsoluteCardinalityErrorPct;
focusImprove = record.improvement.focusWindowPositionEospaPct;
bytesPassed = record.candidate.attemptedPayloadBytes <= ...
    record.reference.attemptedPayloadBytes;
retentionPassed = ...
    record.candidate.receiverSafeOracle.retentionUnresolvedCount == 0;
gate = struct();
gate.fullHorizonPassed = fullImprove >= 2;
gate.cardinalityPassed = cardImprove >= 2;
gate.focusPassed = focusImprove >= -0.5;
gate.bytesPassed = bytesPassed;
gate.retentionPassed = retentionPassed;
gate.passed = gate.fullHorizonPassed && gate.cardinalityPassed && ...
    gate.focusPassed && gate.bytesPassed && gate.retentionPassed;
if gate.passed
    gate.decision = 'advance-to-set-gnn';
else
    gate.decision = 'stop-before-gnn';
end
end

function record = findSourceRecord(cases, presetName, seed)
matches = strcmp({cases.presetName}, presetName) & [cases.seed] == seed;
if nnz(matches) ~= 1
    error('FormationB4V54Tracking:MissingSourceCase', ...
        'The V49 result lacks one unique %s seed %d case.', ...
        presetName, seed);
end
record = cases(find(matches, 1));
end

function path = findLatestV49Result(repoRoot)
directory = fullfile(repoRoot, 'RUN', 'GA', 'dynamic_topology', ...
    'evidence', 'formation_b4_v49_paired_tracking_development');
files = dir(fullfile(directory, ...
    'FORMATION_B4_V49_PAIRED_TRACKING_*.mat'));
if isempty(files)
    error('FormationB4V54Tracking:MissingSourceResult', ...
        'No completed V49 paired result was found.');
end
[~, newest] = max([files.datenum]);
path = fullfile(files(newest).folder, files(newest).name);
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
formations = unique(groupIds, 'stable');
sensors = zeros(1, numel(formations));
for formationIdx = 1:numel(formations)
    sensors(formationIdx) = find( ...
        groupIds == formations(formationIdx), 1);
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('FormationB4V54Tracking:ReportWriteFailed', ...
        'Could not open %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# V54 receiver-safe oracle tracking result\n\n');
fprintf(fid, ['The saved paired V46 baseline is reused. V54 keeps the ', ...
    'V46 route and changes only cross-residual label payloads plus the ', ...
    'positive-evidence Bernoulli projection. Synopsis bytes are included.\n\n']);
fprintf(fid, ['| Scene | Seed | V46 E-OSPA | V54 E-OSPA | Full improve | ', ...
    'Focus improve | Card. improve | Attempted-byte improve | ', ...
    'Selective-byte improve | Clamps | Unresolved | Decision |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--|\n');
for caseIdx = 1:numel(result.cases)
    record = result.cases(caseIdx);
    oracle = record.candidate.receiverSafeOracle;
    fprintf(fid, ['| %s | %d | %.3f | %.3f | %+.2f%% | %+.2f%% | ', ...
        '%+.2f%% | %+.2f%% | %+.2f%% | %d | %d | %s |\n'], ...
        record.presetName, record.seed, ...
        record.reference.fullHorizonPositionEospa, ...
        record.candidate.fullHorizonPositionEospa, ...
        record.improvement.fullHorizonPositionEospaPct, ...
        record.improvement.focusWindowPositionEospaPct, ...
        record.improvement.meanAbsoluteCardinalityErrorPct, ...
        percentImprovement(record.reference.attemptedPayloadBytes, ...
            record.candidate.attemptedPayloadBytes), ...
        100 * oracle.selectiveByteSavingFraction, ...
        oracle.retentionClampCount, oracle.retentionUnresolvedCount, ...
        record.gate.decision);
end
fprintf(fid, ['\nGate: full E-OSPA and cardinality each +2%% or better, ', ...
    'focus no worse than -0.5%%, total attempted bytes no greater than ', ...
    'V46, and zero unresolved retention violations.\n\n']);
fprintf(fid, ['Development evidence only. The oracle reads full sender ', ...
    'posteriors and is not deployable; compact-synopsis quantization ', ...
    'calibration remains pending.\n']);
clear cleanup;
end

function value = percentImprovement(reference, candidate)
if reference == 0
    value = 0;
    if candidate > 0
        value = -Inf;
    end
else
    value = 100 * (reference - candidate) / reference;
end
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && path(1) == filesep;
end
