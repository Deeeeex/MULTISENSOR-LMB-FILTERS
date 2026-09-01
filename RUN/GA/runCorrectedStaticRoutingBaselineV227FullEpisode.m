function [reportPath, result] = ...
        runCorrectedStaticRoutingBaselineV227FullEpisode(options)
% RUNCORRECTEDSTATICROUTINGBASELINEV227FULLEPISODE Paired outcome.
%
% Arms can be run separately and resumed.  A stored static arm is reused
% only under the same source commit, scene, seed and frozen split.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCorrectedStaticRoutingBaselineV227Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles) || ...
        gitState.researchWorktreeDirty
    error('CorrectedStaticRoutingV227:DirtySource', ...
        'The official V227 outcome requires clean frozen source.');
end

presetName = char(getField( ...
    options, 'presetName', 'x36-formation-fov'));
seed = getField(options, 'seed', 1301);
splitName = char(getField(options, 'splitName', 'training'));
[allowedPresets, allowedSeeds] = splitMembership(protocol, splitName);
if ~ismember(presetName, allowedPresets) || ...
        ~isscalar(seed) || ~ismember(seed, allowedSeeds)
    error('CorrectedStaticRoutingV227:SplitMembership', ...
        'The requested scene and seed are outside the frozen V227 split.');
end
armNames = normalizeArmNames(getField( ...
    options, 'armNames', {'static', 'dynamic'}));
resume = logical(getField(options, 'resume', true));

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    protocol.outputRoot, splitName, sprintf('%s_seed%d', ...
        strrep(presetName, '-', '_'), seed))));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'CORRECTED_STATIC_ROUTING_BASELINE_V227_FULL_EPISODE.mat');
reportPath = fullfile(outputRoot, ...
    'CORRECTED_STATIC_ROUTING_BASELINE_V227_FULL_EPISODE.md');
result = initializeResult( ...
    protocol, gitState.commit, presetName, seed, splitName, ...
    matPath, reportPath);
if resume && exist(matPath, 'file') == 2
    loaded = load(matPath, 'result');
    validateExisting(loaded.result, result);
    result = loaded.result;
end
if all(cellfun(@(name) armComplete(result, name), armNames))
    fprintf('Reused completed V227 arm set: %s\n', reportPath);
    return;
end

fprintf('\nV227 corrected static-routing baseline: %s seed %d\n', ...
    presetName, seed);
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
if isempty(fieldnames(result.preflight))
    fprintf('  running full structural preflight ...\n');
    preflight = preflightCorrectedStaticRoutingBaselineV227(inputs);
    result.preflight = compactPreflight(preflight);
    result.staticRegistration = preflight.staticRegistration;
    save('-mat7-binary', matPath, 'result');
else
    preflight = result.preflight;
    preflight.staticRegistration = result.staticRegistration;
end
fprintf(['  routes: static unique=%d, dynamic unique=%d, ', ...
    'changes=%d, messages/round=%d, edge Jaccard=%.4f\n'], ...
    preflight.staticUniqueTopologyCount, ...
    preflight.dynamicUniqueTopologyCount, ...
    preflight.dynamicTopologyChangeCount, ...
    preflight.messageCountPerRound, ...
    preflight.meanDynamicStaticEdgeJaccard);

filterModel = ...
    removeRealizedTargetTruthFromDynamicTopologyModel(inputs.model);
representatives = selectFormationRepresentativesLocal( ...
    inputs.config.sensorGroupIds);
metricWindow = struct( ...
    'activationStartTime', inputs.config.focusWindow(1), ...
    'activationEndTime', inputs.config.focusWindow(end));
for armIdx = 1:numel(armNames)
    armName = armNames{armIdx};
    if armComplete(result, armName)
        fprintf('  resume: %s arm already complete.\n', armName);
        continue;
    end
    armId = resolveArmId(armName, protocol);
    registration = struct();
    if strcmp(armName, 'static')
        registration = result.staticRegistration;
    end
    config = buildCorrectedStaticRoutingBaselineV227Config( ...
        armId, inputs.config, registration);
    context = ...
        buildCorrectedStaticRoutingBaselineV227ExecutionContext( ...
            presetName, seed, splitName, armId, ...
            inputs.config.simulationLength);
    rng(seed + protocol.filterSeedOffset, 'twister');
    timerId = tic;
    fprintf('  running %s full episode ...\n', armName);
    [estimates, diagnostics] = ...
        runEventTriggeredDistributedLmbFilter( ...
            filterModel, inputs.measurements, ...
            inputs.sensorTrajectories, inputs.neighborMap, ...
            inputs.commConfig, config, context);
    arm = summarizeRepeatedMultiGatewayFullEpisodeArm( ...
        armId, estimates, diagnostics, inputs.groundTruthRfs, ...
        inputs.config, representatives, metricWindow, toc(timerId));
    arm = attachTopologySummary(arm, diagnostics);
    assertRuntimeArm(armName, arm, result.preflight);
    result.(armName) = arm;
    result.completedAt = datestr(now, 31);
    save('-mat7-binary', matPath, 'result');
    writeReport(reportPath, result);
    fprintf(['    E-OSPA %.3f; RMSE %.3f; consensus %.3f; ', ...
        'bytes %.0f; routes %d; %.1f s\n'], ...
        arm.fullHorizonPositionEospa, ...
        arm.fullHorizonPositionRmse, ...
        arm.focusInterFormationPositionOspa, ...
        arm.attemptedPayloadBytes, arm.uniqueTopologyCount, ...
        arm.elapsedSeconds);
    clear estimates diagnostics arm;
end

result.allRequestedArmsComplete = ...
    all(cellfun(@(name) armComplete(result, name), armNames));
result.pairedOutcomeAvailable = ...
    armComplete(result, 'static') && armComplete(result, 'dynamic');
if result.pairedOutcomeAvailable
    result.comparison = compareDynamicOverStatic( ...
        result.dynamic, result.static);
end
result.completedAt = datestr(now, 31);
save('-mat7-binary', matPath, 'result');
writeReport(reportPath, result);
fprintf('V227 result: %s\n', reportPath);
end

function arm = attachTopologySummary(arm, diagnostics)
pages = logical(diagnostics.topologyActiveEdge);
arm.uniqueTopologyCount = uniquePageCount(pages);
arm.topologyChangeCount = pageChangeCount(pages);
arm.routeMessageCountByTime = pageCounts(pages);
arm.routeMessageCountMinimum = min(arm.routeMessageCountByTime);
arm.routeMessageCountMaximum = max(arm.routeMessageCountByTime);
arm.policyTruthUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyTruthUsed);
arm.policyFutureOutcomeUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyFutureOutcomeUsed);
end

function assertRuntimeArm(armName, arm, preflight)
if ~arm.exactMessageParityPassed || ...
        ~arm.physicalFeasibilityPassed || ...
        arm.routeMessageCountMinimum ~= preflight.messageCountPerRound || ...
        arm.routeMessageCountMaximum ~= preflight.messageCountPerRound || ...
        arm.policyTruthUsed || arm.policyFutureOutcomeUsed || ...
        (strcmp(armName, 'static') && ...
         (arm.uniqueTopologyCount ~= 1 || arm.topologyChangeCount ~= 0))
    error('CorrectedStaticRoutingV227:RuntimeContractFailed', ...
        'An executed V227 arm violated the matched-routing contract.');
end
end

function comparison = compareDynamicOverStatic(dynamic, static)
comparison = struct();
comparison.fullHorizonEospaGainPercent = lowerGain( ...
    static.fullHorizonPositionEospa, dynamic.fullHorizonPositionEospa);
comparison.focusWindowEospaGainPercent = lowerGain( ...
    static.focusWindowPositionEospa, dynamic.focusWindowPositionEospa);
comparison.worstSensorEospaGainPercent = lowerGain( ...
    static.worstSensorPositionEospa, dynamic.worstSensorPositionEospa);
comparison.formationEospaGainPercent = lowerGainVector( ...
    static.perFormationPositionEospa, ...
    dynamic.perFormationPositionEospa);
comparison.minimumFormationEospaGainPercent = ...
    min(comparison.formationEospaGainPercent);
comparison.fullHorizonRmseGainPercent = lowerGain( ...
    static.fullHorizonPositionRmse, dynamic.fullHorizonPositionRmse);
comparison.focusWindowRmseGainPercent = lowerGain( ...
    static.focusWindowPositionRmse, dynamic.focusWindowPositionRmse);
comparison.worstSensorRmseGainPercent = lowerGain( ...
    static.worstSensorPositionRmse, dynamic.worstSensorPositionRmse);
comparison.formationRmseGainPercent = lowerGainVector( ...
    static.perFormationPositionRmse, ...
    dynamic.perFormationPositionRmse);
comparison.minimumFormationRmseGainPercent = ...
    min(comparison.formationRmseGainPercent);
comparison.fullConsensusGainPercent = lowerGain( ...
    static.meanInterFormationPositionOspa, ...
    dynamic.meanInterFormationPositionOspa);
comparison.focusConsensusGainPercent = lowerGain( ...
    static.focusInterFormationPositionOspa, ...
    dynamic.focusInterFormationPositionOspa);
comparison.terminalConsensusGainPercent = lowerGain( ...
    static.terminalInterFormationPositionOspa, ...
    dynamic.terminalInterFormationPositionOspa);
comparison.attemptedByteSavingPercent = lowerGain( ...
    static.attemptedPayloadBytes, dynamic.attemptedPayloadBytes);
comparison.deliveredByteSavingPercent = lowerGain( ...
    static.deliveredPayloadBytes, dynamic.deliveredPayloadBytes);
comparison.cardinalityGainPercent = lowerGain( ...
    static.meanAbsoluteCardinalityError, ...
    dynamic.meanAbsoluteCardinalityError);
comparison.jointMeanDirectionPassed = ...
    comparison.fullHorizonEospaGainPercent > 0 && ...
    comparison.fullHorizonRmseGainPercent > 0 && ...
    comparison.focusConsensusGainPercent > 0 && ...
    comparison.attemptedByteSavingPercent > 0;
comparison.nonnegativeFormationTailPassed = ...
    comparison.minimumFormationEospaGainPercent >= 0 && ...
    comparison.minimumFormationRmseGainPercent >= 0;
end

function result = initializeResult( ...
        protocol, commit, presetName, seed, splitName, matPath, reportPath)
result = struct();
result.contractVersion = ...
    'corrected-static-routing-v227-full-episode-result-v1';
result.protocolId = protocol.id;
result.protocol = protocol;
result.startedAt = datestr(now, 31);
result.completedAt = '';
result.generationGitCommit = commit;
result.presetName = presetName;
result.seed = seed;
result.splitName = splitName;
result.preflight = struct();
result.staticRegistration = struct();
result.static = struct();
result.dynamic = struct();
result.comparison = struct();
result.allRequestedArmsComplete = false;
result.pairedOutcomeAvailable = false;
result.fixedStaticRouteBaselineIncluded = true;
result.validationClaimAllowed = false;
result.developmentEvidenceOnly = true;
result.matPath = matPath;
result.reportPath = reportPath;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function validateExisting(actual, expected)
required = {'contractVersion', 'protocolId', 'generationGitCommit', ...
    'presetName', 'seed', 'splitName', 'static', 'dynamic'};
if ~isstruct(actual) || ~isscalar(actual) || ...
        ~all(isfield(actual, required)) || ...
        ~strcmp(actual.contractVersion, expected.contractVersion) || ...
        ~strcmp(actual.protocolId, expected.protocolId) || ...
        ~strcmp(actual.generationGitCommit, ...
            expected.generationGitCommit) || ...
        ~strcmp(actual.presetName, expected.presetName) || ...
        actual.seed ~= expected.seed || ...
        ~strcmp(actual.splitName, expected.splitName)
    error('CorrectedStaticRoutingV227:InvalidResumeState', ...
        'The V227 checkpoint belongs to another source contract.');
end
end

function preflight = compactPreflight(preflight)
if isfield(preflight, 'dynamicFusionWeightMatrixByTime')
    preflight = rmfield( ...
        preflight, 'dynamicFusionWeightMatrixByTime');
end
if isfield(preflight, 'staticRegistration')
    preflight = rmfield(preflight, 'staticRegistration');
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('CorrectedStaticRoutingV227:ReportWriteFailed', ...
        'Could not create the V227 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V227 corrected static versus dynamic routing\n\n');
fprintf(fid, '- Scene / seed / split: `%s / %d / %s`\n', ...
    result.presetName, result.seed, result.splitName);
fprintf(fid, '- Source commit: `%s`\n', result.generationGitCommit);
fprintf(fid, '- Fixed static baseline included: `1`\n');
fprintf(fid, '- Validation claim allowed: `0`\n\n');
if ~isempty(fieldnames(result.preflight))
    fprintf(fid, ['Structural parity: `%d` messages/round; static routes ', ...
        '`%d`; dynamic routes `%d`; dynamic changes `%d`; ', ...
        'mean edge Jaccard `%.4f`.\n\n'], ...
        result.preflight.messageCountPerRound, ...
        result.preflight.staticUniqueTopologyCount, ...
        result.preflight.dynamicUniqueTopologyCount, ...
        result.preflight.dynamicTopologyChangeCount, ...
        result.preflight.meanDynamicStaticEdgeJaccard);
end
fprintf(fid, ['| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | ', ...
    'Focus RMSE | Focus consensus | Attempted bytes | Routes |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|\n');
writeArmRow(fid, 'Static', result.static);
writeArmRow(fid, 'Dynamic', result.dynamic);
if result.pairedOutcomeAvailable
    gain = result.comparison;
    fprintf(fid, '\n## Dynamic over static\n\n');
    fprintf(fid, '| Metric | Gain |\n|:--|--:|\n');
    fprintf(fid, '| Full E-OSPA | `%+.3f%%` |\n', ...
        gain.fullHorizonEospaGainPercent);
    fprintf(fid, '| Focus E-OSPA | `%+.3f%%` |\n', ...
        gain.focusWindowEospaGainPercent);
    fprintf(fid, '| Full RMSE | `%+.3f%%` |\n', ...
        gain.fullHorizonRmseGainPercent);
    fprintf(fid, '| Focus RMSE | `%+.3f%%` |\n', ...
        gain.focusWindowRmseGainPercent);
    fprintf(fid, '| Worst-sensor E-OSPA | `%+.3f%%` |\n', ...
        gain.worstSensorEospaGainPercent);
    fprintf(fid, '| Worst-sensor RMSE | `%+.3f%%` |\n', ...
        gain.worstSensorRmseGainPercent);
    fprintf(fid, '| Focus consistency | `%+.3f%%` |\n', ...
        gain.focusConsensusGainPercent);
    fprintf(fid, '| Attempted-byte saving | `%+.3f%%` |\n', ...
        gain.attemptedByteSavingPercent);
    fprintf(fid, '| Weakest formation E-OSPA | `%+.3f%%` |\n', ...
        gain.minimumFormationEospaGainPercent);
    fprintf(fid, '| Weakest formation RMSE | `%+.3f%%` |\n', ...
        gain.minimumFormationRmseGainPercent);
    fprintf(fid, '\n- Joint mean direction passed: `%d`\n', ...
        gain.jointMeanDirectionPassed);
    fprintf(fid, '- Nonnegative formation tail passed: `%d`\n', ...
        gain.nonnegativeFormationTailPassed);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function writeArmRow(fid, label, arm)
if isempty(fieldnames(arm))
    fprintf(fid, '| %s | — | — | — | — | — | — | — |\n', label);
    return;
end
fprintf(fid, '| %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.0f | %d |\n', ...
    label, arm.fullHorizonPositionEospa, ...
    arm.focusWindowPositionEospa, arm.fullHorizonPositionRmse, ...
    arm.focusWindowPositionRmse, ...
    arm.focusInterFormationPositionOspa, ...
    arm.attemptedPayloadBytes, arm.uniqueTopologyCount);
end

function names = normalizeArmNames(value)
if ischar(value)
    value = {value};
end
if ~iscell(value) || isempty(value) || ...
        any(~cellfun(@ischar, value))
    error('CorrectedStaticRoutingV227:InvalidArmSelection', ...
        'armNames must contain static and/or dynamic.');
end
names = cellfun(@(name) lower(strtrim(name)), ...
    reshape(value, 1, []), 'UniformOutput', false);
if numel(unique(names)) ~= numel(names) || ...
        any(~ismember(names, {'static', 'dynamic'}))
    error('CorrectedStaticRoutingV227:InvalidArmSelection', ...
        'armNames must contain unique static and/or dynamic entries.');
end
end

function armId = resolveArmId(name, protocol)
if strcmp(name, 'static')
    armId = protocol.staticArmId;
else
    armId = protocol.dynamicArmId;
end
end

function value = armComplete(result, name)
value = isfield(result, name) && isstruct(result.(name)) && ...
    isscalar(result.(name)) && ~isempty(fieldnames(result.(name)));
end

function sensors = selectFormationRepresentativesLocal(groupIds)
groups = unique(reshape(groupIds, 1, []), 'stable');
sensors = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    sensors(groupIdx) = find(groupIds == groups(groupIdx), 1);
end
end

function [presets, seeds] = splitMembership(protocol, splitName)
switch splitName
    case 'training'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.trainingSeeds;
    case 'calibration'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.calibrationSeeds;
    case 'evaluation'
        presets = protocol.split.mainPresets;
        seeds = protocol.split.evaluationSeeds;
    case 'stress'
        presets = protocol.split.stressPresets;
        seeds = protocol.split.stressSeeds;
    case 'scale-extrapolation'
        presets = protocol.split.scaleExtrapolationPresets;
        seeds = protocol.split.scaleExtrapolationSeeds;
    otherwise
        error('CorrectedStaticRoutingV227:UnknownSplit', ...
            'The requested V227 split is not registered.');
end
end

function value = lowerGain(reference, candidate)
if ~isfinite(reference) || ~isfinite(candidate)
    value = NaN;
elseif abs(reference) <= eps
    value = 0;
    if candidate > 0
        value = -Inf;
    end
else
    value = 100 * (reference - candidate) / abs(reference);
end
end

function values = lowerGainVector(reference, candidate)
values = nan(size(reference));
for idx = 1:numel(reference)
    values(idx) = lowerGain(reference(idx), candidate(idx));
end
end

function count = uniquePageCount(pages)
flat = reshape(logical(pages), [], size(pages, 3))';
count = size(unique(flat, 'rows'), 1);
end

function count = pageChangeCount(pages)
count = 0;
for timeIdx = 2:size(pages, 3)
    count = count + ~isequal( ...
        pages(:, :, timeIdx), pages(:, :, timeIdx - 1));
end
end

function counts = pageCounts(pages)
counts = zeros(1, size(pages, 3));
for timeIdx = 1:size(pages, 3)
    counts(timeIdx) = nnz(pages(:, :, timeIdx));
end
end

function value = anyFiniteNonzero(values)
values = values(isfinite(values));
value = any(values ~= 0);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end

function result = isAbsolutePath(pathValue)
if ispc
    result = ~isempty(regexp(pathValue, '^[A-Za-z]:[\\/]', 'once'));
else
    result = ~isempty(pathValue) && pathValue(1) == filesep;
end
end
