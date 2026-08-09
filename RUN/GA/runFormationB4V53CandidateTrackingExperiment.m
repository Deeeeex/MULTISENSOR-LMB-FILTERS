function result = runFormationB4V53CandidateTrackingExperiment( ...
        sourceResultPath, presetNames, seeds, outputDirectory)
% RUNFORMATIONB4V53CANDIDATETRACKINGEXPERIMENT Reuse saved V46 baseline.

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
    error('FormationB4V53Tracking:InvalidSourceResult', ...
        'A completed V49 paired result is required.');
end
source = loaded.result;
if nargin < 2 || isempty(presetNames)
    presetNames = unique({source.cases.presetName}, 'stable');
elseif ischar(presetNames)
    presetNames = {presetNames};
end
if nargin < 3 || isempty(seeds)
    seeds = unique([source.cases.seed], 'stable');
end
if nargin < 4 || isempty(outputDirectory)
    outputDirectory = fullfile(repoRoot, 'RUN', 'GA', ...
        'dynamic_topology', 'evidence', ...
        'formation_b4_v53_candidate_tracking_development');
elseif ~isAbsolutePath(outputDirectory)
    outputDirectory = fullfile(repoRoot, outputDirectory);
end
if exist(outputDirectory, 'dir') == 0
    mkdir(outputDirectory);
end

protocol = getFormationB4V53RuntimeProtocol();
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
        fprintf('\nV53 candidate tracking: %s seed %d (%d/%d)\n', ...
            presetName, seed, cursor, numel(cases));

        inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
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
        triggerConfig = buildFormationB4V53FixedTriggerConfig( ...
            protocol.candidateArmId, inputs.config.numberOfSensors);
        context = ...
            buildFormationB4V53PairedTrackingDevelopmentContext( ...
                presetName, seed, protocol.candidateArmId);
        rng(sourceRecord.filterSeed, 'twister');
        armTimer = tic;
        [stateEstimates, diagnostics] = ...
            runEventTriggeredDistributedLmbFilter( ...
                filterModel, inputs.measurements, ...
                inputs.sensorTrajectories, inputs.neighborMap, ...
                inputs.commConfig, triggerConfig, context);
        elapsedSeconds = toc(armTimer);
        candidate = summarizeFormationB4TrackingArm( ...
            protocol.candidateArmId, stateEstimates, diagnostics, ...
            inputs.groundTruthRfs, inputs.config, ...
            representativeSensors, protocol, elapsedSeconds);
        candidate.exactSelectiveGate = summarizeExactSelectiveGate( ...
            diagnostics, protocol, inputs.config.simulationLength);

        record = sourceRecord;
        record.candidate = candidate;
        record.improvement = summarizeFormationB4TrackingImprovement( ...
            record.reference, candidate);
        cases(cursor) = record;
        fprintf(['  V53 E-OSPA %.3f; improvement full %+.2f%%, ', ...
            'focus %+.2f%%, cardinality %+.2f%%, consensus %+.2f%%; ', ...
            'deferred pulses %.1f%%\n'], ...
            candidate.fullHorizonPositionEospa, ...
            record.improvement.fullHorizonPositionEospaPct, ...
            record.improvement.focusWindowPositionEospaPct, ...
            record.improvement.meanAbsoluteCardinalityErrorPct, ...
            record.improvement.meanInterFormationPositionOspaPct, ...
            100 * candidate.exactSelectiveGate.pulseDeferralFraction);
        clear inputs filterModel stateEstimates diagnostics;
    end
end

result = struct();
result.contractVersion = ...
    'formation-b4-v53-candidate-tracking-development-result-v1';
result.startedAt = startedAt;
result.completedAt = datestr(now, 31);
result.elapsedSeconds = toc(totalTimer);
result.protocolId = protocol.id;
result.referenceArmId = protocol.referenceArmId;
result.candidateArmId = protocol.candidateArmId;
result.sourceV49ResultPath = sourceResultPath;
result.cases = cases;
result.referenceRerun = false;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.controlPlaneBytesIncluded = false;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
baseName = sprintf( ...
    'FORMATION_B4_V53_CANDIDATE_TRACKING_%s', timestamp);
result.matPath = fullfile(outputDirectory, [baseName, '.mat']);
result.reportPath = fullfile(outputDirectory, [baseName, '.md']);
save('-mat7-binary', result.matPath, 'result');
writeReport(result.reportPath, result);
fprintf('\nSaved: %s\nReport: %s\n', ...
    result.matPath, result.reportPath);
end

function summary = summarizeExactSelectiveGate( ...
        diagnostics, protocol, timeCount)
pulseTimes = 1:protocol.period:timeCount;
deferredFormationCount = zeros(size(pulseTimes));
deferredCrossEdgeCount = zeros(size(pulseTimes));
requestedFormationCount = zeros(size(pulseTimes));
jointRetentionRisk = zeros(size(pulseTimes));
temporalSensorStrong = false(size(pulseTimes));
temporalFormationStrong = false(size(pulseTimes));
for pulseIdx = 1:numel(pulseTimes)
    schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
        pulseTimes(pulseIdx)};
    gate = schedule.exactSelectiveGateDetails;
    deferredFormationCount(pulseIdx) = ...
        numel(gate.selectedDeferredFormationIds);
    deferredCrossEdgeCount(pulseIdx) = ...
        gate.deferredCrossEdgeCount;
    requestedFormationCount(pulseIdx) = ...
        nnz(gate.requestedDeferredFormationMask);
    jointRetentionRisk(pulseIdx) = gate.jointRetentionRisk;
    temporalSensorStrong(pulseIdx) = gate.temporalSensorStrong;
    temporalFormationStrong(pulseIdx) = gate.temporalFormationStrong;
end
summary = struct();
summary.pulseCount = numel(pulseTimes);
summary.pulseDeferralCount = nnz(deferredFormationCount > 0);
summary.pulseDeferralFraction = mean(deferredFormationCount > 0);
summary.meanDeferredFormationCount = mean(deferredFormationCount);
summary.meanDeferredCrossEdgeCount = mean(deferredCrossEdgeCount);
summary.meanRequestedFormationCount = mean(requestedFormationCount);
summary.meanJointRetentionRisk = mean(jointRetentionRisk);
summary.maximumJointRetentionRisk = max(jointRetentionRisk);
summary.temporalSensorStrongFraction = mean(temporalSensorStrong);
summary.temporalFormationStrongFraction = ...
    mean(temporalFormationStrong);
end

function record = findSourceRecord(cases, presetName, seed)
matches = strcmp({cases.presetName}, presetName) & [cases.seed] == seed;
if nnz(matches) ~= 1
    error('FormationB4V53Tracking:MissingSourceCase', ...
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
    error('FormationB4V53Tracking:MissingSourceResult', ...
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
    error('FormationB4V53Tracking:ReportWriteFailed', ...
        'Could not open %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# V53 exact selective cross-pulse tracking result\n\n');
fprintf(fid, ['The saved paired V46 baseline is reused. V53 keeps ', ...
    'the original local residual timing and selectively holds only ', ...
    'cross-formation inputs using exact LMB counterfactuals.\n\n']);
fprintf(fid, ['| Scene | Seed | V46 E-OSPA | V53 E-OSPA | Full improve | ', ...
    'Focus improve | Card. improve | Consensus improve | ', ...
    'Deferred pulses | Mean deferred formations | Mean retention risk |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|\n');
for caseIdx = 1:numel(result.cases)
    record = result.cases(caseIdx);
    gate = record.candidate.exactSelectiveGate;
    fprintf(fid, ['| %s | %d | %.3f | %.3f | %+.2f%% | %+.2f%% | ', ...
        '%+.2f%% | %+.2f%% | %.1f%% | %.2f | %.4f |\n'], ...
        record.presetName, record.seed, ...
        record.reference.fullHorizonPositionEospa, ...
        record.candidate.fullHorizonPositionEospa, ...
        record.improvement.fullHorizonPositionEospaPct, ...
        record.improvement.focusWindowPositionEospaPct, ...
        record.improvement.meanAbsoluteCardinalityErrorPct, ...
        record.improvement.meanInterFormationPositionOspaPct, ...
        100 * gate.pulseDeferralFraction, ...
        gate.meanDeferredFormationCount, ...
        gate.meanJointRetentionRisk);
end
fprintf(fid, ['\nDevelopment evidence only; route-control metadata bytes ', ...
    'are not included.\n']);
clear cleanup;
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && path(1) == filesep;
end
