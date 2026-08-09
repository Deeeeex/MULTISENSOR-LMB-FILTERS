function result = runFormationB4V55AttributionTrackingExperiment( ...
        sourceResultPath, armIds, presetNames, seeds, outputDirectory)
% RUNFORMATIONB4V55ATTRIBUTIONTRACKINGEXPERIMENT V55 causal ablation.
%
% The runner reuses the frozen V46 result and supports resuming one arm at a
% time.  Runtime modes are connected separately after the V54 gate finishes.

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
    error('FormationB4V55Attribution:InvalidSourceResult', ...
        'A completed V49 paired result is required.');
end
source = loaded.result;

catalog = armCatalog();
if nargin < 2 || isempty(armIds)
    armIds = {catalog.id};
elseif ischar(armIds)
    armIds = {armIds};
end
arms = selectArms(catalog, armIds);
if nargin < 3 || isempty(presetNames)
    presetNames = {'x36-formation-fov-convoy'};
elseif ischar(presetNames)
    presetNames = {presetNames};
end
if nargin < 4 || isempty(seeds)
    seeds = 1009;
end
if nargin < 5 || isempty(outputDirectory)
    outputDirectory = fullfile(repoRoot, 'RUN', 'GA', ...
        'dynamic_topology', 'evidence', ...
        'formation_b4_v55_attribution_tracking_development');
elseif ~isAbsolutePath(outputDirectory)
    outputDirectory = fullfile(repoRoot, outputDirectory);
end
if exist(outputDirectory, 'dir') == 0
    mkdir(outputDirectory);
end

protocol = getFormationB4V49RuntimeProtocol();
caseCount = numel(arms) * numel(presetNames) * numel(seeds);
cases = repmat(struct(), 1, caseCount);
cursor = 0;
startedAt = datestr(now, 31);
totalTimer = tic;
for armIdx = 1:numel(arms)
    arm = arms(armIdx);
    for presetIdx = 1:numel(presetNames)
        presetName = presetNames{presetIdx};
        for seedIdx = 1:numel(seeds)
            seed = seeds(seedIdx);
            sourceRecord = findSourceRecord( ...
                source.cases, presetName, seed);
            cursor = cursor + 1;
            fprintf('\nV55 attribution: %s / %s seed %d (%d/%d)\n', ...
                arm.id, presetName, seed, cursor, caseCount);

            inputs = generateDynamicTopologyScenarioInputs( ...
                presetName, seed);
            identity = buildDynamicTopologyPhysicalIdentityRegistry( ...
                inputs.config);
            [inputs.commConfig.linkUniforms, ~] = ...
                materializePhysicalUidDirectedDeliveryUniforms( ...
                    sourceRecord.deliverySeed, ...
                    identity.sensorPhysicalUids, ...
                    inputs.config.simulationLength);
            filterModel = removeRealizedTargetTruth(inputs.model);
            representativeSensors = selectFormationRepresentatives( ...
                inputs.config.sensorGroupIds);
            triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
                protocol.referenceArmId, ...
                inputs.config.numberOfSensors);
            triggerConfig.receiverSafeLabelFusionEnabled = true;
            triggerConfig.receiverSafeLabelFusionMode = arm.runtimeMode;
            triggerConfig.receiverSafeOracleByteBudgetFraction = 1;
            triggerConfig.receiverSafeSupportedExistenceThreshold = 0.50;
            triggerConfig.receiverSafeMaximumLogOddsDrop = log(4);
            context = ...
                buildFormationB4V49PairedTrackingDevelopmentContext( ...
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
                arm.id, stateEstimates, diagnostics, ...
                inputs.groundTruthRfs, inputs.config, ...
                representativeSensors, protocol, elapsedSeconds);
            candidate.receiverSafe = summarizeReceiverSafe( ...
                diagnostics);

            record = sourceRecord;
            record.armId = arm.id;
            record.runtimeMode = arm.runtimeMode;
            record.messageSelectionEnabled = ...
                arm.messageSelectionEnabled;
            record.receiverProjectionEnabled = ...
                arm.receiverProjectionEnabled;
            record.candidate = candidate;
            record.improvement = ...
                summarizeFormationB4TrackingImprovement( ...
                    record.reference, candidate);
            record.combinedGate = assessCombinedGate(record, arm);
            cases(cursor) = record;
            fprintf(['  E-OSPA %.3f; full %+.2f%%, focus %+.2f%%, ', ...
                'cardinality %+.2f%%, bytes %+.2f%%; ', ...
                'clamps=%d gate=%s\n'], ...
                candidate.fullHorizonPositionEospa, ...
                record.improvement.fullHorizonPositionEospaPct, ...
                record.improvement.focusWindowPositionEospaPct, ...
                record.improvement.meanAbsoluteCardinalityErrorPct, ...
                percentImprovement( ...
                    record.reference.attemptedPayloadBytes, ...
                    candidate.attemptedPayloadBytes), ...
                candidate.receiverSafe.retentionClampCount, ...
                record.combinedGate.decision);
            clear inputs filterModel stateEstimates diagnostics;
        end
    end
end

result = struct();
result.contractVersion = ...
    'formation-b4-v55-attribution-tracking-development-result-v1';
result.startedAt = startedAt;
result.completedAt = datestr(now, 31);
result.elapsedSeconds = toc(totalTimer);
result.referenceArmId = protocol.referenceArmId;
result.sourceV49ResultPath = sourceResultPath;
result.armCatalog = catalog;
result.executedArmIds = {arms.id};
result.cases = cases;
result.referenceRerun = false;
result.pairedDeliveryAndFilterSeeds = true;
result.targetTruthUsedByRuntimePolicy = false;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
baseName = sprintf( ...
    'FORMATION_B4_V55_ATTRIBUTION_TRACKING_%s', timestamp);
result.matPath = fullfile(outputDirectory, [baseName, '.mat']);
result.reportPath = fullfile(outputDirectory, [baseName, '.md']);
save('-mat7-binary', result.matPath, 'result');
writeReport(result.reportPath, result);
fprintf('\nSaved: %s\nReport: %s\n', ...
    result.matPath, result.reportPath);
end

function catalog = armCatalog()
catalog = struct( ...
    'id', { ...
        'v55-projection-only', ...
        'v55-context-selection-only', ...
        'v55-context-combined'}, ...
    'runtimeMode', { ...
        'projection-only', ...
        'context-aware-selection-only', ...
        'context-aware-combined'}, ...
    'messageSelectionEnabled', {false, true, true}, ...
    'receiverProjectionEnabled', {true, false, true});
end

function arms = selectArms(catalog, armIds)
arms = repmat(catalog(1), 1, 0);
for requestedIdx = 1:numel(armIds)
    match = find(strcmp({catalog.id}, armIds{requestedIdx}));
    if numel(match) ~= 1
        error('FormationB4V55Attribution:UnknownArm', ...
            'Unknown attribution arm: %s.', armIds{requestedIdx});
    end
    arms(end+1) = catalog(match); %#ok<AGROW>
end
end

function summary = summarizeReceiverSafe(diagnostics)
summary = struct();
summary.selectiveEdgeTimeCount = getSummaryField( ...
    diagnostics, 'receiverSafeSelectiveEdgeTimeCount');
summary.baselineFullBytesOnSelectiveEdges = getSummaryField( ...
    diagnostics, 'receiverSafeBaselineFullBytes');
summary.controlSynopsisBytes = getSummaryField( ...
    diagnostics, 'receiverSafeAttemptedControlSynopsisBytes');
summary.selectedPayloadBytes = getSummaryField( ...
    diagnostics, 'receiverSafeAttemptedSelectedPayloadBytes');
summary.selectiveByteSavingFraction = getSummaryField( ...
    diagnostics, 'receiverSafeAttemptedByteSavingFraction');
summary.retentionViolationCount = getSummaryField( ...
    diagnostics, 'receiverSafeRetentionViolationCount');
summary.retentionRemovalCount = getSummaryField( ...
    diagnostics, 'receiverSafeRetentionRemovalCount');
summary.retentionClampCount = getSummaryField( ...
    diagnostics, 'receiverSafeRetentionClampCount');
summary.retentionFallbackCount = getSummaryField( ...
    diagnostics, 'receiverSafeRetentionFallbackCount');
summary.retentionUnresolvedCount = getSummaryField( ...
    diagnostics, 'receiverSafeRetentionUnresolvedCount');
end

function value = getSummaryField(diagnostics, fieldName)
value = 0;
if isstruct(diagnostics) && isfield(diagnostics, 'summary') && ...
        isfield(diagnostics.summary, fieldName)
    value = diagnostics.summary.(fieldName);
end
end

function gate = assessCombinedGate(record, arm)
gate = struct();
gate.applicable = strcmp(arm.id, 'v55-context-combined');
if ~gate.applicable
    gate.passed = false;
    gate.decision = 'attribution-only';
    return;
end
fullImprove = record.improvement.fullHorizonPositionEospaPct;
cardImprove = record.improvement.meanAbsoluteCardinalityErrorPct;
focusImprove = record.improvement.focusWindowPositionEospaPct;
gate.fullHorizonPassed = fullImprove >= 2;
gate.cardinalityPassed = cardImprove >= 2;
gate.focusPassed = focusImprove >= -0.5;
gate.bytesPassed = record.candidate.attemptedPayloadBytes <= ...
    record.reference.attemptedPayloadBytes;
gate.retentionPassed = ...
    record.candidate.receiverSafe.retentionUnresolvedCount == 0;
gate.passed = gate.fullHorizonPassed && ...
    gate.cardinalityPassed && gate.focusPassed && ...
    gate.bytesPassed && gate.retentionPassed;
if gate.passed
    gate.decision = 'advance-to-multiscene';
else
    gate.decision = 'revise-before-learning';
end
end

function record = findSourceRecord(cases, presetName, seed)
matches = strcmp({cases.presetName}, presetName) & ...
    [cases.seed] == seed;
if nnz(matches) ~= 1
    error('FormationB4V55Attribution:MissingSourceCase', ...
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
    error('FormationB4V55Attribution:MissingSourceResult', ...
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
    error('FormationB4V55Attribution:ReportWriteFailed', ...
        'Could not open %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# V55 context-aware attribution result\n\n');
fprintf(fid, ['The saved paired V46 baseline is reused. The three arms ', ...
    'separate receiver projection, context-aware label selection, and ', ...
    'their combination.\n\n']);
fprintf(fid, ['| Arm | Scene | Seed | Full improve | Focus improve | ', ...
    'Card. improve | Attempted-byte improve | Selective-byte improve | ', ...
    'Clamps | Decision |\n']);
fprintf(fid, '|:--|:--|--:|--:|--:|--:|--:|--:|--:|:--|\n');
for caseIdx = 1:numel(result.cases)
    record = result.cases(caseIdx);
    fprintf(fid, ['| %s | %s | %d | %+.2f%% | %+.2f%% | %+.2f%% | ', ...
        '%+.2f%% | %+.2f%% | %d | %s |\n'], ...
        record.armId, record.presetName, record.seed, ...
        record.improvement.fullHorizonPositionEospaPct, ...
        record.improvement.focusWindowPositionEospaPct, ...
        record.improvement.meanAbsoluteCardinalityErrorPct, ...
        percentImprovement(record.reference.attemptedPayloadBytes, ...
            record.candidate.attemptedPayloadBytes), ...
        100 * record.candidate.receiverSafe. ...
            selectiveByteSavingFraction, ...
        record.candidate.receiverSafe.retentionClampCount, ...
        record.combinedGate.decision);
end
fprintf(fid, ['\nThe combined development gate requires full E-OSPA ', ...
    'and cardinality each +2%% or better, focus no worse than -0.5%%, ', ...
    'no more attempted bytes than V46, and zero unresolved retention ', ...
    'violations. The other arms are attribution evidence only.\n\n']);
fprintf(fid, ['Development evidence only. Multi-scene and held-out-seed ', ...
    'claims remain forbidden.\n']);
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
