function [reportPath, result] = ...
        runFormationBraidRoutingComparisonV241FullEpisode(options)
% RUNFORMATIONBRAIDROUTINGCOMPARISONV241FULLEPISODE Matched M24 pilot.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getFormationBraidRoutingComparisonV241Protocol();
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('FormationBraidRoutingV241:DirtySource', ...
        'The V241 full episode requires clean tracked source.');
end
presetName = char(getField(options, 'presetName', ...
    'm24-formation-fov-formation-braid'));
seed = getField(options, 'seed', protocol.allowedSeeds(1));
if ~ismember(presetName, protocol.allowedPresets) || ...
        ~ismember(seed, protocol.allowedSeeds)
    error('FormationBraidRoutingV241:UnregisteredCase', ...
        'The requested V241 scene or seed is not registered.');
end
armNames = normalizeArmNames(getField( ...
    options, 'armNames', {'static-dropout', 'causal'}));
resume = logical(getField(options, 'resume', true));

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    protocol.outputRoot, sprintf('%s_seed%d', ...
        strrep(presetName, '-', '_'), seed))));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'FORMATION_BRAID_ROUTING_COMPARISON_V241_FULL_EPISODE.mat');
reportPath = fullfile(outputRoot, ...
    'FORMATION_BRAID_ROUTING_COMPARISON_V241_FULL_EPISODE.md');
result = initializeResult(protocol, gitState.commit, presetName, seed, ...
    matPath, reportPath);
if resume && exist(matPath, 'file') == 2
    loaded = load(matPath, 'result');
    crossCommitResume = validateExisting( ...
        loaded.result, result, armNames);
    result = loaded.result;
    result.protocol = protocol;
    result.evidenceBoundary = protocol.evidenceBoundary;
    result.currentExecutionGitCommit = gitState.commit;
    if crossCommitResume
        result.resumeSourceGenerationGitCommit = ...
            result.generationGitCommit;
        result.staticDropout = attachGenerationCommit( ...
            result.staticDropout, result.generationGitCommit);
    end
end
if all(cellfun(@(name) armComplete(result, name), armNames))
    fprintf('Reused completed V241 arm set: %s\n', reportPath);
    return;
end

fprintf('\nV241 formation-braid routing: %s seed %d\n', ...
    presetName, seed);
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
if isempty(fieldnames(result.preflight)) || ...
        ~isfield(result.preflight, 'observableRuntimeContextPassed')
    fprintf('  running matched structural preflight ...\n');
    [~, preflight] = ...
        runFormationBraidRoutingComparisonV241StructuralPreflight( ...
            struct('presetName', presetName, 'seed', seed, ...
                'writeReport', false));
    result.preflight = preflight;
    if isempty(fieldnames(result.initialRegistration))
        result.initialRegistration = ...
            buildInitialFormationBraidRouteV241Registration(inputs);
    end
    save('-mat7-binary', matPath, 'result');
else
    preflight = result.preflight;
end
fprintf(['  structural: static strong %.3f, always changes %d, ', ...
    'causal changes %d, messages %d/receiver inputs\n'], ...
    preflight.arms(1).strongFraction, ...
    preflight.arms(2).treeChangeCount, ...
    preflight.arms(3).treeChangeCount, ...
    protocol.directedInputsPerReceiver);

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
    if strcmp(armName, 'static-dropout')
        registration = result.initialRegistration;
    end
    config = buildFormationBraidRoutingComparisonV241Config( ...
        armId, inputs.config, registration);
    context = ...
        buildFormationBraidRoutingComparisonV241ExecutionContext( ...
            presetName, seed, armId, inputs.config.simulationLength);
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
    arm = attachTopologySummary(arm, diagnostics, inputs.config);
    arm.generationGitCommit = gitState.commit;
    assertRuntimeArm(armName, arm, protocol, inputs.config);
    result.(armField(armName)) = arm;
    result.completedAt = datestr(now, 31);
    updateComparisonsAndWrite();
    fprintf(['    E-OSPA %.3f; RMSE %.3f; focus consensus %.3f; ', ...
        'bytes %.0f; messages %d..%d; routes %d; %.1f s\n'], ...
        arm.fullHorizonPositionEospa, ...
        arm.fullHorizonPositionRmse, ...
        arm.focusInterFormationPositionOspa, ...
        arm.attemptedPayloadBytes, arm.routeMessageCountMinimum, ...
        arm.routeMessageCountMaximum, arm.uniqueTopologyCount, ...
        arm.elapsedSeconds);
    clear estimates diagnostics arm;
end
updateComparisonsAndWrite();
fprintf('V241 result: %s\n', reportPath);

    function updateComparisonsAndWrite()
        result.allRequestedArmsComplete = all(cellfun( ...
            @(name) armComplete(result, name), armNames));
        result.causalStaticPairAvailable = ...
            armComplete(result, 'static-dropout') && ...
            armComplete(result, 'causal');
        result.causalAlwaysPairAvailable = ...
            armComplete(result, 'always-replan') && ...
            armComplete(result, 'causal');
        if result.causalStaticPairAvailable
            result.causalOverStaticDropout = compareArm( ...
                result.causal, result.staticDropout);
        end
        if result.causalAlwaysPairAvailable
            result.causalOverAlwaysReplan = compareArm( ...
                result.causal, result.alwaysReplan);
        end
        save('-mat7-binary', matPath, 'result');
        writeReport(reportPath, result);
    end
end

function arm = attachTopologySummary(arm, diagnostics, config)
pages = logical(diagnostics.topologyActiveEdge);
arm.uniqueTopologyCount = uniquePageCount(pages);
arm.topologyChangeCount = pageChangeCount(pages);
arm.routeMessageCountByTime = reshape( ...
    diagnostics.topologyDirectedMessageCount, 1, []);
arm.routeMessageCountMinimum = min(arm.routeMessageCountByTime);
arm.routeMessageCountMaximum = max(arm.routeMessageCountByTime);
arm.policyTruthUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyTruthUsed);
arm.policyFutureOutcomeUsed = anyFiniteNonzero( ...
    diagnostics.topologyPolicyFutureOutcomeUsed);
policyPages = permute(pages, [2, 1, 3]);
strong = false(1, size(policyPages, 3));
for currentTime = 1:numel(strong)
    strong(currentTime) = ...
        isStronglyConnected(policyPages(:, :, currentTime));
end
arm.instantaneousStrongFraction = mean(strong);
arm.instantaneousStrongAllTimes = all(strong);
arm.treeReselectionTimes = zeros(1, 0);
arm.dropoutMessageCountByTime = zeros(1, numel(strong));
for currentTime = 1:numel(strong)
    certificate = ...
        diagnostics.topologyPolicyScheduleCertificate{currentTime};
    if ~isstruct(certificate)
        continue;
    end
    if logical(getField(certificate, 'treeReselectionUsed', false))
        arm.treeReselectionTimes(end + 1) = currentTime; %#ok<AGROW>
    end
    arm.dropoutMessageCountByTime(currentTime) = ...
        getField(certificate, 'dropoutMessageCount', 0);
end
arm.treeReselectionCount = numel(arm.treeReselectionTimes);
arm.totalDropoutMessages = sum(arm.dropoutMessageCountByTime);
arm.firstDropoutTime = findFirst(arm.dropoutMessageCountByTime > 0);
arm.expectedFullMessageCount = ...
    2 * config.numberOfSensors;
end

function assertRuntimeArm(armName, arm, protocol, config)
basePassed = arm.physicalFeasibilityPassed && ...
    ~arm.policyTruthUsed && ~arm.policyFutureOutcomeUsed && ...
    arm.routeMessageCountMaximum <= ...
        protocol.directedInputsPerReceiver * config.numberOfSensors;
if strcmp(armName, 'static-dropout')
    armPassed = arm.routeMessageCountMinimum < ...
            arm.routeMessageCountMaximum && ...
        arm.totalDropoutMessages > 0 && ...
        arm.instantaneousStrongFraction < 1 && ...
        arm.treeReselectionCount == 0;
else
    armPassed = arm.exactMessageParityPassed && ...
        arm.routeMessageCountMinimum == ...
            protocol.directedInputsPerReceiver * config.numberOfSensors && ...
        arm.routeMessageCountMaximum == ...
            protocol.directedInputsPerReceiver * config.numberOfSensors && ...
        arm.instantaneousStrongAllTimes;
end
if ~basePassed || ~armPassed
    error('FormationBraidRoutingV241:RuntimeContractFailed', ...
        'The executed %s arm violated its matched boundary.', armName);
end
end

function comparison = compareArm(candidate, reference)
comparison = struct();
comparison.fullHorizonEospaGainPercent = lowerGain( ...
    reference.fullHorizonPositionEospa, ...
    candidate.fullHorizonPositionEospa);
comparison.focusWindowEospaGainPercent = lowerGain( ...
    reference.focusWindowPositionEospa, ...
    candidate.focusWindowPositionEospa);
comparison.fullHorizonRmseGainPercent = lowerGain( ...
    reference.fullHorizonPositionRmse, ...
    candidate.fullHorizonPositionRmse);
comparison.focusWindowRmseGainPercent = lowerGain( ...
    reference.focusWindowPositionRmse, ...
    candidate.focusWindowPositionRmse);
comparison.focusConsensusGainPercent = lowerGain( ...
    reference.focusInterFormationPositionOspa, ...
    candidate.focusInterFormationPositionOspa);
comparison.terminalConsensusGainPercent = lowerGain( ...
    reference.terminalInterFormationPositionOspa, ...
    candidate.terminalInterFormationPositionOspa);
comparison.attemptedByteSavingPercent = lowerGain( ...
    reference.attemptedPayloadBytes, candidate.attemptedPayloadBytes);
comparison.deliveredByteSavingPercent = lowerGain( ...
    reference.deliveredPayloadBytes, candidate.deliveredPayloadBytes);
comparison.attemptedByteOverheadPercent = ...
    -comparison.attemptedByteSavingPercent;
comparison.formationEospaGainPercent = lowerGainVector( ...
    reference.perFormationPositionEospa, ...
    candidate.perFormationPositionEospa);
comparison.minimumFormationEospaGainPercent = ...
    min(comparison.formationEospaGainPercent);
comparison.formationRmseGainPercent = lowerGainVector( ...
    reference.perFormationPositionRmse, ...
    candidate.perFormationPositionRmse);
comparison.minimumFormationRmseGainPercent = ...
    min(comparison.formationRmseGainPercent);
comparison.meanDirectionPassed = ...
    comparison.fullHorizonEospaGainPercent > 0 && ...
    comparison.fullHorizonRmseGainPercent > 0 && ...
    comparison.focusConsensusGainPercent > 0;
comparison.nonnegativeFormationTailPassed = ...
    comparison.minimumFormationEospaGainPercent >= 0 && ...
    comparison.minimumFormationRmseGainPercent >= 0;
end

function result = initializeResult(protocol, commit, presetName, seed, ...
        matPath, reportPath)
result = struct();
result.contractVersion = ...
    'formation-braid-routing-comparison-v241-full-episode-result-v1';
result.protocolId = protocol.id;
result.protocol = protocol;
result.startedAt = datestr(now, 31);
result.completedAt = '';
result.generationGitCommit = commit;
result.currentExecutionGitCommit = commit;
result.resumeSourceGenerationGitCommit = '';
result.presetName = presetName;
result.seed = seed;
result.preflight = struct();
result.initialRegistration = struct();
result.staticDropout = struct();
result.alwaysReplan = struct();
result.causal = struct();
result.causalOverStaticDropout = struct();
result.causalOverAlwaysReplan = struct();
result.allRequestedArmsComplete = false;
result.causalStaticPairAvailable = false;
result.causalAlwaysPairAvailable = false;
result.validationClaimAllowed = false;
result.developmentEvidenceOnly = true;
result.matPath = matPath;
result.reportPath = reportPath;
result.evidenceBoundary = protocol.evidenceBoundary;
end

function crossCommitResume = validateExisting(actual, expected, armNames)
required = {'contractVersion', 'protocolId', 'generationGitCommit', ...
    'presetName', 'seed', 'staticDropout', 'alwaysReplan', 'causal'};
baseValid = isstruct(actual) && isscalar(actual) && ...
    all(isfield(actual, required)) && ...
    strcmp(actual.contractVersion, expected.contractVersion) && ...
    strcmp(actual.protocolId, expected.protocolId) && ...
    strcmp(actual.presetName, expected.presetName) && ...
    actual.seed == expected.seed;
sameCommit = baseValid && strcmp(actual.generationGitCommit, ...
    expected.generationGitCommit);
crossCommitResume = baseValid && ~sameCommit && ...
    strcmp(actual.generationGitCommit, ...
        expected.protocol.compatibleStaticReferenceCommit) && ...
    numel(armNames) == 1 && strcmp(armNames{1}, 'causal') && ...
    armComplete(actual, 'static-dropout') && ...
    ~armComplete(actual, 'causal') && ...
    compatibleResumeDiffPassed(expected.protocol);
if ~sameCommit && ~crossCommitResume
    error('FormationBraidRoutingV241:InvalidResumeState', ...
        'The V241 checkpoint belongs to another source contract.');
end
end

function passed = compatibleResumeDiffPassed(protocol)
command = sprintf('git diff --name-only %s..HEAD --', ...
    protocol.compatibleStaticReferenceCommit);
[status, output] = system(command);
if status ~= 0
    passed = false;
    return;
end
output = strtrim(output);
if isempty(output)
    changedFiles = {};
else
    changedFiles = strsplit(output, sprintf('\n'));
end
passed = all(ismember(changedFiles, ...
    protocol.compatibleResumeChangedFiles));
end

function arm = attachGenerationCommit(arm, commit)
if ~isempty(fieldnames(arm)) && ...
        ~isfield(arm, 'generationGitCommit')
    arm.generationGitCommit = commit;
end
end

function writeReport(path, result)
fid = fopen(path, 'w');
if fid < 0
    error('FormationBraidRoutingV241:ReportWriteFailed', ...
        'Could not create the V241 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V241 formation-braid routing comparison\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    result.presetName, result.seed);
fprintf(fid, '- Source commit: `%s`\n', result.generationGitCommit);
fprintf(fid, '- Current execution commit: `%s`\n', ...
    getField(result, 'currentExecutionGitCommit', ...
        result.generationGitCommit));
if ~isempty(getField(result, 'resumeSourceGenerationGitCommit', ''))
    fprintf(fid, '- Compatible fixed-arm checkpoint reused: `%s`\n', ...
        result.resumeSourceGenerationGitCommit);
end
fprintf(fid, '- Fixed arm commit: `%s`\n', ...
    armGenerationCommit(result.staticDropout, ...
        result.generationGitCommit));
fprintf(fid, '- Causal arm commit: `%s`\n', ...
    armGenerationCommit(result.causal, 'pending'));
fprintf(fid, '- Fixed no-rerouting baseline included: `1`\n');
fprintf(fid, '- Validation claim allowed: `0`\n\n');
if ~isempty(fieldnames(result.preflight))
    fprintf(fid, ['Structural preflight: static strong fraction `%.3f`; ', ...
        'always-replan changes `%d`; causal changes `%d`.\n\n'], ...
        result.preflight.arms(1).strongFraction, ...
        result.preflight.arms(2).treeChangeCount, ...
        result.preflight.arms(3).treeChangeCount);
end
fprintf(fid, ['| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | ', ...
    'Focus RMSE | Focus consensus | Attempted bytes | Messages | ', ...
    'Strong fraction | Routes |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|:--:|--:|--:|\n');
writeArmRow(fid, 'Fixed formation tree', result.staticDropout);
writeArmRow(fid, 'Always replan', result.alwaysReplan);
writeArmRow(fid, 'Causal minimal edit', result.causal);
if result.causalStaticPairAvailable
    writeComparison(fid, 'Causal over fixed formation tree', ...
        result.causalOverStaticDropout);
end
if result.causalAlwaysPairAvailable
    writeComparison(fid, 'Causal over always replan', ...
        result.causalOverAlwaysReplan);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function commit = armGenerationCommit(arm, fallback)
if isempty(fieldnames(arm))
    commit = fallback;
else
    commit = getField(arm, 'generationGitCommit', fallback);
end
end

function writeArmRow(fid, label, arm)
if isempty(fieldnames(arm))
    fprintf(fid, '| %s | — | — | — | — | — | — | — | — | — |\n', ...
        label);
    return;
end
fprintf(fid, ['| %s | %.3f | %.3f | %.3f | %.3f | %.3f | %.0f | ', ...
    '%d--%d | %.3f | %d |\n'], label, ...
    arm.fullHorizonPositionEospa, arm.focusWindowPositionEospa, ...
    arm.fullHorizonPositionRmse, arm.focusWindowPositionRmse, ...
    arm.focusInterFormationPositionOspa, arm.attemptedPayloadBytes, ...
    arm.routeMessageCountMinimum, arm.routeMessageCountMaximum, ...
    arm.instantaneousStrongFraction, arm.uniqueTopologyCount);
end

function writeComparison(fid, title, gain)
fprintf(fid, '\n## %s\n\n', title);
fprintf(fid, '| Metric | Gain |\n|:--|--:|\n');
fprintf(fid, '| Full E-OSPA | `%+.3f%%` |\n', ...
    gain.fullHorizonEospaGainPercent);
fprintf(fid, '| Focus E-OSPA | `%+.3f%%` |\n', ...
    gain.focusWindowEospaGainPercent);
fprintf(fid, '| Full RMSE | `%+.3f%%` |\n', ...
    gain.fullHorizonRmseGainPercent);
fprintf(fid, '| Focus RMSE | `%+.3f%%` |\n', ...
    gain.focusWindowRmseGainPercent);
fprintf(fid, '| Focus consistency | `%+.3f%%` |\n', ...
    gain.focusConsensusGainPercent);
fprintf(fid, '| Attempted-byte saving | `%+.3f%%` |\n', ...
    gain.attemptedByteSavingPercent);
fprintf(fid, '| Weakest formation E-OSPA | `%+.3f%%` |\n', ...
    gain.minimumFormationEospaGainPercent);
fprintf(fid, '| Weakest formation RMSE | `%+.3f%%` |\n', ...
    gain.minimumFormationRmseGainPercent);
fprintf(fid, '\n- Mean direction passed: `%d`\n', ...
    gain.meanDirectionPassed);
fprintf(fid, '- Nonnegative formation tail passed: `%d`\n', ...
    gain.nonnegativeFormationTailPassed);
end

function names = normalizeArmNames(value)
if ischar(value)
    value = {value};
end
allowed = {'static-dropout', 'always-replan', 'causal'};
if ~iscell(value) || isempty(value) || ...
        any(~cellfun(@ischar, value)) || ...
        any(~ismember(value, allowed)) || ...
        numel(unique(value)) ~= numel(value)
    error('FormationBraidRoutingV241:InvalidArmSelection', ...
        'armNames contains an unknown or duplicate V241 arm.');
end
names = reshape(value, 1, []);
end

function id = resolveArmId(name, protocol)
if strcmp(name, 'static-dropout')
    id = protocol.staticDropoutArmId;
elseif strcmp(name, 'always-replan')
    id = protocol.alwaysReplanArmId;
else
    id = protocol.causalArmId;
end
end

function field = armField(name)
if strcmp(name, 'static-dropout')
    field = 'staticDropout';
elseif strcmp(name, 'always-replan')
    field = 'alwaysReplan';
else
    field = 'causal';
end
end

function complete = armComplete(result, name)
field = armField(name);
complete = isfield(result, field) && isstruct(result.(field)) && ...
    isscalar(result.(field)) && ~isempty(fieldnames(result.(field))) && ...
    isfield(result.(field), 'fullHorizonPositionEospa');
end

function count = uniquePageCount(pages)
keys = cell(1, size(pages, 3));
for pageIdx = 1:size(pages, 3)
    keys{pageIdx} = sprintf('%d', pages(:, :, pageIdx));
end
count = numel(unique(keys));
end

function count = pageChangeCount(pages)
count = 0;
for pageIdx = 2:size(pages, 3)
    count = count + ~isequal( ...
        pages(:, :, pageIdx), pages(:, :, pageIdx - 1));
end
end

function connected = isStronglyConnected(adjacency)
connected = reachesAll(adjacency) && reachesAll(adjacency');
end

function passed = reachesAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node), continue; end
    visited(node) = true;
    frontier = [frontier, find(adjacency(node, :) & ~visited)]; %#ok<AGROW>
end
passed = all(visited);
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = lowerGainVector(reference, candidate)
value = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function value = findFirst(mask)
value = find(mask, 1);
if isempty(value), value = NaN; end
end

function value = anyFiniteNonzero(values)
values = values(isfinite(values));
value = any(values ~= 0);
end

function sensors = selectFormationRepresentativesLocal(groupIds)
groups = unique(groupIds, 'stable');
sensors = zeros(1, numel(groups));
for groupIdx = 1:numel(groups)
    sensors(groupIdx) = find(groupIds == groups(groupIdx), 1);
end
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && (path(1) == '/' || ...
    (~isempty(regexp(path, '^[A-Za-z]:[\\/]', 'once'))));
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
