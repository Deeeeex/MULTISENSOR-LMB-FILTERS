function result = runFormationB4V49PairedTrackingExperiment( ...
    presetNames, seeds, outputDirectory)
% RUNFORMATIONB4V49PAIREDTRACKINGEXPERIMENT Small paired V46/V49 study.
%
% Examples:
%   runFormationB4V49PairedTrackingExperiment( ...
%       {'x36-formation-fov-convoy'}, 1009)
%   runFormationB4V49PairedTrackingExperiment( ...
%       {'m24-formation-fov-convoy', ...
%        'x36-formation-fov-convoy', ...
%        'x36-formation-fov-crossing'}, [1009, 1013])
%
% Both arms reuse the same generated scene, measurements, physical-UID
% delivery draws, and filter random seed.  The output is development
% evidence for choosing the next method revision, not a validation claim.

if nargin < 1 || isempty(presetNames)
    presetNames = { ...
        'm24-formation-fov-convoy', ...
        'x36-formation-fov-convoy', ...
        'x36-formation-fov-crossing'};
elseif ischar(presetNames)
    presetNames = {presetNames};
end
if nargin < 2 || isempty(seeds)
    seeds = 1009;
end
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin < 3 || isempty(outputDirectory)
    outputDirectory = fullfile(repoRoot, 'RUN', 'GA', ...
        'dynamic_topology', 'evidence', ...
        'formation_b4_v49_paired_tracking_development');
elseif ~isAbsolutePath(outputDirectory)
    outputDirectory = fullfile(repoRoot, outputDirectory);
end
validateInputs(presetNames, seeds, outputDirectory);
if exist(outputDirectory, 'dir') == 0
    mkdir(outputDirectory);
end

protocol = getFormationB4V49RuntimeProtocol();
caseCount = numel(presetNames) * numel(seeds);
cases = repmat(struct(), 1, caseCount);
caseCursor = 0;
startedAt = datestr(now, 31);
totalTimer = tic;
for presetIdx = 1:numel(presetNames)
    presetName = presetNames{presetIdx};
    sceneOrdinal = resolveSceneOrdinal(presetName);
    for seedIdx = 1:numel(seeds)
        seed = seeds(seedIdx);
        caseCursor = caseCursor + 1;
        fprintf('\nV49 paired tracking: %s seed %d (%d/%d)\n', ...
            presetName, seed, caseCursor, caseCount);

        inputs = generateDynamicTopologyScenarioInputs( ...
            presetName, seed);
        identity = buildDynamicTopologyPhysicalIdentityRegistry( ...
            inputs.config);
        deliverySeed = 49000000 + 100 * seed + sceneOrdinal;
        [inputs.commConfig.linkUniforms, ~] = ...
            materializePhysicalUidDirectedDeliveryUniforms( ...
                deliverySeed, identity.sensorPhysicalUids, ...
                inputs.config.simulationLength);
        filterModel = removeRealizedTargetTruth(inputs.model);
        representativeSensors = selectFormationRepresentatives( ...
            inputs.config.sensorGroupIds);
        filterSeed = 49500000 + 100 * seed + sceneOrdinal;

        armRecords = cell(1, 2);
        for armIdx = 1:2
            armId = protocol.primaryArms{armIdx};
            if strcmp(armId, protocol.referenceArmId)
                triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
                    armId, inputs.config.numberOfSensors);
            else
                triggerConfig = buildFormationB4V49FixedTriggerConfig( ...
                    armId, inputs.config.numberOfSensors);
            end
            context = ...
                buildFormationB4V49PairedTrackingDevelopmentContext( ...
                    presetName, seed, armId);
            rng(filterSeed, 'twister');
            armTimer = tic;
            fprintf('  running %s ...\n', armId);
            [stateEstimates, diagnostics] = ...
                runEventTriggeredDistributedLmbFilter( ...
                    filterModel, inputs.measurements, ...
                    inputs.sensorTrajectories, inputs.neighborMap, ...
                    inputs.commConfig, triggerConfig, context);
            elapsedSeconds = toc(armTimer);
            armRecords{armIdx} = summarizeArm( ...
                armId, stateEstimates, diagnostics, ...
                inputs.groundTruthRfs, inputs.config, ...
                representativeSensors, protocol, elapsedSeconds);
            fprintf('    E-OSPA %.3f, focus %.3f, card %.3f, %.1f s\n', ...
                armRecords{armIdx}.fullHorizonPositionEospa, ...
                armRecords{armIdx}.focusWindowPositionEospa, ...
                armRecords{armIdx}.meanAbsoluteCardinalityError, ...
                elapsedSeconds);
            clear stateEstimates diagnostics;
        end

        record = struct();
        record.presetName = presetName;
        record.sceneStyle = inputs.config.sceneStyle;
        record.seed = seed;
        record.numberOfSensors = inputs.config.numberOfSensors;
        record.formationCount = inputs.config.formationCount;
        record.simulationLength = inputs.config.simulationLength;
        record.focusWindow = reshape(inputs.config.focusWindow, 1, []);
        record.deliverySeed = deliverySeed;
        record.filterSeed = filterSeed;
        record.reference = armRecords{1};
        record.candidate = armRecords{2};
        record.improvement = summarizeImprovement( ...
            record.reference, record.candidate);
        cases(caseCursor) = record;
        fprintf(['  V49 improvement: full %+.2f%%, focus %+.2f%%, ', ...
            'cardinality %+.2f%%, inter-formation consensus %+.2f%%\n'], ...
            record.improvement.fullHorizonPositionEospaPct, ...
            record.improvement.focusWindowPositionEospaPct, ...
            record.improvement.meanAbsoluteCardinalityErrorPct, ...
            record.improvement.meanInterFormationPositionOspaPct);
        clear inputs filterModel armRecords;
    end
end

result = struct();
result.contractVersion = ...
    'formation-b4-v49-paired-tracking-development-result-v1';
result.startedAt = startedAt;
result.completedAt = datestr(now, 31);
result.elapsedSeconds = toc(totalTimer);
result.protocolId = protocol.id;
result.referenceArmId = protocol.referenceArmId;
result.candidateArmId = protocol.candidateArmId;
result.presetNames = presetNames;
result.seeds = reshape(seeds, 1, []);
result.cases = cases;
result.developmentEvidenceOnly = true;
result.validationClaimAllowed = false;
result.controlPlaneBytesIncluded = false;

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
baseName = sprintf('FORMATION_B4_V49_PAIRED_TRACKING_%s', timestamp);
result.matPath = fullfile(outputDirectory, [baseName, '.mat']);
result.reportPath = fullfile(outputDirectory, [baseName, '.md']);
save('-mat7-binary', result.matPath, 'result');
writeReport(result.reportPath, result);
fprintf('\nSaved: %s\nReport: %s\n', ...
    result.matPath, result.reportPath);
end

function validateInputs(presetNames, seeds, outputDirectory)
if ~iscell(presetNames) || isempty(presetNames) || ...
        any(~cellfun(@(value) ischar(value) && isrow(value), ...
            presetNames)) || ...
        ~isa(seeds, 'double') || ~isreal(seeds) || ...
        isempty(seeds) || ~isvector(seeds) || ...
        any(~isfinite(seeds)) || any(seeds ~= floor(seeds)) || ...
        ~ischar(outputDirectory) || ~isrow(outputDirectory) || ...
        isempty(outputDirectory)
    error('FormationB4V49Tracking:InvalidExperimentInput', ...
        'Preset names, integer seeds, and an output directory are required.');
end
for presetIdx = 1:numel(presetNames)
    resolveSceneOrdinal(presetNames{presetIdx});
end
end

function ordinal = resolveSceneOrdinal(presetName)
presets = { ...
    'm24-formation-fov', ...
    'm24-formation-fov-convoy', ...
    'm24-formation-fov-relay', ...
    'm24-formation-fov-crossing', ...
    'x36-formation-fov', ...
    'x36-formation-fov-convoy', ...
    'x36-formation-fov-relay', ...
    'x36-formation-fov-crossing', ...
    'm24-formation-fov-merge-split', ...
    'm24-formation-fov-curved-corridor', ...
    'x36-formation-fov-merge-split', ...
    'x36-formation-fov-curved-corridor'};
ordinal = find(strcmp(presetName, presets), 1);
if isempty(ordinal)
    error('FormationB4V49Tracking:UnknownPreset', ...
        'Unknown M24/X36 formation-FoV preset: %s.', presetName);
end
end

function model = removeRealizedTargetTruth(model)
if isfield(model, 'explicitTargetTrajectories')
    model = rmfield(model, 'explicitTargetTrajectories');
end
if ~isfield(model, 'dynamicTopologyScenario') || ...
        ~isstruct(model.dynamicTopologyScenario) || ...
        ~isscalar(model.dynamicTopologyScenario)
    error('FormationB4V49Tracking:MissingScenario', ...
        'The generated model is missing its dynamic-topology scenario.');
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
groupIds = reshape(groupIds, 1, []);
formations = unique(groupIds, 'stable');
sensors = zeros(1, numel(formations));
for formationIdx = 1:numel(formations)
    sensors(formationIdx) = find( ...
        groupIds == formations(formationIdx), 1);
end
end

function summary = summarizeArm(armId, estimates, diagnostics, ...
        truth, config, representativeSensors, protocol, elapsedSeconds)
sensorCount = numel(estimates);
timeCount = numel(truth.x);
cutoff = config.ospaPositionCutoff;
order = 2;
positionIndices = [1, 2];
eospa = zeros(sensorCount, timeCount);
cardinalityError = zeros(sensorCount, timeCount);
for sensorIdx = 1:sensorCount
    for currentTime = 1:timeCount
        components = computePositionEuclideanOspa( ...
            truth.x{currentTime}, ...
            estimates{sensorIdx}.mu{currentTime}, ...
            cutoff, order, positionIndices);
        eospa(sensorIdx, currentTime) = components(1);
        cardinalityError(sensorIdx, currentTime) = abs( ...
            numel(estimates{sensorIdx}.mu{currentTime}) - ...
            numel(truth.x{currentTime}));
    end
end

interFormation = zeros(1, timeCount);
for currentTime = 1:timeCount
    pairValues = zeros(1, nchoosek(numel(representativeSensors), 2));
    pairCursor = 0;
    for leftIdx = 1:numel(representativeSensors)-1
        for rightIdx = leftIdx+1:numel(representativeSensors)
            pairCursor = pairCursor + 1;
            components = computePositionEuclideanOspa( ...
                estimates{representativeSensors(leftIdx)}.mu{currentTime}, ...
                estimates{representativeSensors(rightIdx)}.mu{currentTime}, ...
                cutoff, order, positionIndices);
            pairValues(pairCursor) = components(1);
        end
    end
    interFormation(currentTime) = mean(pairValues);
end

focusTimes = config.focusWindow(1):config.focusWindow(2);
perSensorMean = mean(eospa, 2);
attempted = logical(diagnostics.attempted);
delivered = logical(diagnostics.delivered);
alignedEnds = protocol.period:protocol.period:timeCount;
deliveredStrong = false(size(alignedEnds));
for windowIdx = 1:numel(alignedEnds)
    windowEnd = alignedEnds(windowIdx);
    window = windowEnd-protocol.period+1:windowEnd;
    deliveredStrong(windowIdx) = isStronglyConnectedLocal( ...
        any(delivered(:, :, window), 3));
end

phaseOneTimes = 1:protocol.period:timeCount;
cycleSelected = false(size(phaseOneTimes));
if strcmp(armId, protocol.candidateArmId)
    for phaseIdx = 1:numel(phaseOneTimes)
        schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
            phaseOneTimes(phaseIdx)};
        cycleSelected(phaseIdx) = isstruct(schedule) && ...
            isfield(schedule, 'cycleSelected') && ...
            schedule.cycleSelected;
    end
end

groups = reshape(config.sensorGroupIds, 1, []);
crossMask = groups(:) ~= groups(:)';
burstCrossChanges = zeros(1, max(0, numel(phaseOneTimes) - 1));
for phaseIdx = 2:numel(phaseOneTimes)
    previous = logical(diagnostics.topologyActiveEdge( ...
        :, :, phaseOneTimes(phaseIdx - 1)));
    current = logical(diagnostics.topologyActiveEdge( ...
        :, :, phaseOneTimes(phaseIdx)));
    burstCrossChanges(phaseIdx - 1) = nnz( ...
        xor(previous, current) & crossMask);
end

selectedObjective = diagnostics.topologyPolicyObjective(phaseOneTimes);
selectedObjective = selectedObjective(cycleSelected & ...
    isfinite(selectedObjective));
summary = struct();
summary.armId = armId;
summary.elapsedSeconds = elapsedSeconds;
summary.fullHorizonPositionEospa = mean(eospa(:));
summary.focusWindowPositionEospa = mean( ...
    reshape(eospa(:, focusTimes), 1, []));
summary.worstSensorPositionEospa = max(perSensorMean);
summary.meanAbsoluteCardinalityError = mean(cardinalityError(:));
summary.meanInterFormationPositionOspa = mean(interFormation);
summary.focusInterFormationPositionOspa = mean( ...
    interFormation(focusTimes));
summary.terminalInterFormationPositionOspa = interFormation(end);
summary.attemptedMessageCount = nnz(attempted);
summary.deliveredMessageCount = nnz(delivered);
summary.attemptedPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(:));
summary.deliveredPayloadBytes = sum( ...
    diagnostics.attemptedPayloadBytes(delivered));
summary.deliveredAlignedB4StrongFraction = mean(deliveredStrong);
summary.cycleSelectedCount = nnz(cycleSelected);
summary.cycleOpportunityCount = numel(phaseOneTimes);
summary.cycleSelectionFraction = mean(cycleSelected);
summary.meanSelectedStructuralGain = 0;
if ~isempty(selectedObjective)
    summary.meanSelectedStructuralGain = mean(selectedObjective);
end
summary.meanBurstCrossEdgeChanges = 0;
summary.burstRouteChangeFraction = 0;
if ~isempty(burstCrossChanges)
    summary.meanBurstCrossEdgeChanges = mean(burstCrossChanges);
    summary.burstRouteChangeFraction = mean(burstCrossChanges > 0);
end
end

function improvement = summarizeImprovement(reference, candidate)
fields = { ...
    'fullHorizonPositionEospa', ...
    'focusWindowPositionEospa', ...
    'worstSensorPositionEospa', ...
    'meanAbsoluteCardinalityError', ...
    'meanInterFormationPositionOspa', ...
    'focusInterFormationPositionOspa'};
improvement = struct();
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    improvement.([fieldName, 'Pct']) = percentImprovement( ...
        reference.(fieldName), candidate.(fieldName));
end
improvement.attemptedMessageSavingPct = percentImprovement( ...
    reference.attemptedMessageCount, candidate.attemptedMessageCount);
improvement.deliveredMessageSavingPct = percentImprovement( ...
    reference.deliveredMessageCount, candidate.deliveredMessageCount);
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

function connected = isStronglyConnectedLocal(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('FormationB4V49Tracking:ReportWriteFailed', ...
        'Could not open the result report: %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# V49 paired tracking development result\n\n');
fprintf(fid, ['V46 and V49 use the same scene, measurements, delivery ', ...
    'draws and filter seed. Positive improvement means V49 is better. ', ...
    'Inter-formation consensus uses one fixed representative sensor ', ...
    'from each formation.\n\n']);
fprintf(fid, ['| Scene | Seed | V46 E-OSPA | V49 E-OSPA | Full improve | ', ...
    'Focus improve | Card. improve | Consensus improve | Cycle use | ', ...
    'Burst cross-edge changes |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|\n');
for caseIdx = 1:numel(result.cases)
    record = result.cases(caseIdx);
    fprintf(fid, ['| %s | %d | %.3f | %.3f | %+.2f%% | %+.2f%% | ', ...
        '%+.2f%% | %+.2f%% | %.1f%% | %.2f |\n'], ...
        record.presetName, record.seed, ...
        record.reference.fullHorizonPositionEospa, ...
        record.candidate.fullHorizonPositionEospa, ...
        record.improvement.fullHorizonPositionEospaPct, ...
        record.improvement.focusWindowPositionEospaPct, ...
        record.improvement.meanAbsoluteCardinalityErrorPct, ...
        record.improvement.meanInterFormationPositionOspaPct, ...
        100 * record.candidate.cycleSelectionFraction, ...
        record.candidate.meanBurstCrossEdgeChanges);
end
fprintf(fid, '\n| Scene | Seed | Attempts V46/V49 | Delivered V46/V49 | ');
fprintf(fid, 'Delivered B4 strong V46/V49 | Runtime V46/V49 (s) |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|\n');
for caseIdx = 1:numel(result.cases)
    record = result.cases(caseIdx);
    fprintf(fid, '| %s | %d | %d / %d | %d / %d | %.3f / %.3f | %.1f / %.1f |\n', ...
        record.presetName, record.seed, ...
        record.reference.attemptedMessageCount, ...
        record.candidate.attemptedMessageCount, ...
        record.reference.deliveredMessageCount, ...
        record.candidate.deliveredMessageCount, ...
        record.reference.deliveredAlignedB4StrongFraction, ...
        record.candidate.deliveredAlignedB4StrongFraction, ...
        record.reference.elapsedSeconds, ...
        record.candidate.elapsedSeconds);
end
fprintf(fid, ['\nThis is development evidence for method selection. It does ', ...
    'not include route-control metadata bytes and is not a validation ', ...
    'or statistical-generalization claim.\n']);
clear cleanup;
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && path(1) == filesep;
end
