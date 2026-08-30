function [reportPath, preflight] = ...
        preflightTemporalHybridFormationRepairV186(options)
% PREFLIGHTTEMPORALHYBRIDFORMATIONREPAIRV186 Check Top-K advertisement.
%
% This replays the opened V180 t=79 state with ideal delivery so the only
% changed variable is the source-local synopsis cap. Truth is read after
% selection to reject a cap that changes the known-safe common action.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getTemporalHybridFormationRepairV186Protocol();
screenPath = getField(options, 'screenPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v180', 'recursive_rollout_capture', ...
    'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v186', 'source_local_topk_preflight'));
if exist(screenPath, 'file') ~= 2
    error('TemporalHybridV186:MissingOpenedState', ...
        'The opened V180 recursive snapshot is required.');
end

loaded = load(screenPath, 'screen');
screen = loaded.screen;
outcomeIdx = find(strcmp({screen.records.actionName}, ...
    'v179-analytic-f3-plus-rollout-aware-f5-repair'), 1);
pageIdx = find(screen.returnTimes == 79, 1);
if isempty(outcomeIdx) || isempty(pageIdx)
    error('TemporalHybridV186:SnapshotContractDrift', ...
        'The V179 t=79 opened state is unavailable.');
end
outcome = screen.outcomes(outcomeIdx);
inputs = generateDynamicTopologyScenarioInputs( ...
    screen.presetName, screen.seed);
model = inputs.model;
model.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(screen.sensorGroupIds, 1, []);
receiverIds = find(groupIds == 5);
physical = logical(inputs.graphData.physicalAdjacency(:, :, 79));
physical = physical | physical';
physical(1:numel(groupIds)+1:end) = false;
posterior = outcome.fusedPosteriorSnapshotsByTime{pageIdx};
locals = outcome.localPosteriorSnapshotsByTime{pageIdx};

baseConfig = selectorConfig(protocol);
uncappedConfig = baseConfig;
uncappedConfig. ...
    formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource = ...
    inf;
[uncappedPosterior, uncappedDetails] = ...
    selectFormationCoordinatedPosteriorRepair( ...
        posterior, locals, physical, physical, uncappedConfig, ...
        receiverIds, 79, model);
[cappedPosterior, cappedDetails] = ...
    selectFormationCoordinatedPosteriorRepair( ...
        posterior, locals, physical, physical, baseConfig, ...
        receiverIds, 79, model);

[uncappedLabels, uncappedSources, uncappedApplied] = ...
    selectedActions(uncappedDetails, receiverIds);
[cappedLabels, cappedSources, cappedApplied] = ...
    selectedActions(cappedDetails, receiverIds);
sameAction = isequal(uncappedLabels, cappedLabels) && ...
    isequal(uncappedSources, cappedSources);

uncappedEospaGain = zeros(1, numel(receiverIds));
cappedEospaGain = zeros(1, numel(receiverIds));
uncappedRmseGain = zeros(1, numel(receiverIds));
cappedRmseGain = zeros(1, numel(receiverIds));
for receiverPosition = 1:numel(receiverIds)
    receiverIdx = receiverIds(receiverPosition);
    baselineEospa = evaluateLmbTopologyCurrentEospa( ...
        posterior{receiverIdx}, model, 79, struct());
    baselineRmse = currentPosteriorRmse( ...
        posterior{receiverIdx}, model, 79);
    uncappedEospaGain(receiverPosition) = baselineEospa - ...
        evaluateLmbTopologyCurrentEospa( ...
            uncappedPosterior{receiverIdx}, model, 79, struct());
    cappedEospaGain(receiverPosition) = baselineEospa - ...
        evaluateLmbTopologyCurrentEospa( ...
            cappedPosterior{receiverIdx}, model, 79, struct());
    uncappedRmseGain(receiverPosition) = baselineRmse - ...
        currentPosteriorRmse(uncappedPosterior{receiverIdx}, model, 79);
    cappedRmseGain(receiverPosition) = baselineRmse - ...
        currentPosteriorRmse(cappedPosterior{receiverIdx}, model, 79);
end

uncappedSynopsisBytes = totalDetailField( ...
    uncappedDetails, receiverIds, 'attemptedSynopsisBytes');
cappedSynopsisBytes = totalDetailField( ...
    cappedDetails, receiverIds, 'attemptedSynopsisBytes');
uncappedTotalBytes = totalDetailField( ...
    uncappedDetails, receiverIds, 'attemptedTotalBytes');
cappedTotalBytes = totalDetailField( ...
    cappedDetails, receiverIds, 'attemptedTotalBytes');

preflight = struct();
preflight.contractVersion = ...
    'temporal-hybrid-formation-repair-v186-preflight-v1';
preflight.screenPath = screenPath;
preflight.currentTime = 79;
preflight.receivers = receiverIds;
preflight.maximumAdvertisedLabelsPerSource = protocol. ...
    formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource;
preflight.uncappedLabels = uncappedLabels;
preflight.uncappedSources = uncappedSources;
preflight.cappedLabels = cappedLabels;
preflight.cappedSources = cappedSources;
preflight.uncappedApplied = uncappedApplied;
preflight.cappedApplied = cappedApplied;
preflight.sameAction = sameAction;
preflight.uncappedEospaGain = uncappedEospaGain;
preflight.cappedEospaGain = cappedEospaGain;
preflight.uncappedRmseGain = uncappedRmseGain;
preflight.cappedRmseGain = cappedRmseGain;
preflight.uncappedSynopsisBytes = uncappedSynopsisBytes;
preflight.cappedSynopsisBytes = cappedSynopsisBytes;
preflight.synopsisSavingPercent = relativeSavingPercent( ...
    uncappedSynopsisBytes, cappedSynopsisBytes);
preflight.uncappedTotalBytes = uncappedTotalBytes;
preflight.cappedTotalBytes = cappedTotalBytes;
preflight.sideChannelSavingPercent = relativeSavingPercent( ...
    uncappedTotalBytes, cappedTotalBytes);
preflight.gatePassed = sameAction && all(uncappedApplied) && ...
    all(cappedApplied) && min(cappedEospaGain) >= -1e-9 && ...
    min(cappedRmseGain) > 1e-9 && cappedSynopsisBytes < ...
        uncappedSynopsisBytes;
preflight.truthUsedForDiagnosticReadout = true;
preflight.selectionUsesTruth = false;
preflight.recursiveEvaluationRun = false;
preflight.evidenceBoundary = [ ...
    'V186 preflight replays the opened V180 X36 seed-211 t=79 ', ...
    'pre-side-channel state with ideal delivery. It compares the frozen ', ...
    'uncapped formation selector against a source-local Top-4 synopsis ', ...
    'ranked only by current posterior position uncertainty. Truth is ', ...
    'read after selection for immediate E-OSPA/RMSE rejection. A pass ', ...
    'authorizes one recursive development run only; it is not validation.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'TEMPORAL_HYBRID_FORMATION_REPAIR_V186_PREFLIGHT.mat');
reportPath = fullfile(outputRoot, ...
    'TEMPORAL_HYBRID_FORMATION_REPAIR_V186_PREFLIGHT.md');
preflight.matPath = matPath;
preflight.reportPath = reportPath;
save('-mat7-binary', matPath, 'preflight');
writeReport(reportPath, preflight);
fprintf('V186 source-local Top-K preflight: %s\n', reportPath);
end

function config = selectorConfig(protocol)
config = struct();
config.observableOneHopRiskLabelExistenceThreshold = ...
    protocol.activeExistenceThreshold;
config.learnedOneHopSafeLabelModelPath = protocol.learnedModelPath;
config.learnedOneHopSafeLabelPolicyPath = protocol.learnedPolicyPath;
config.learnedOneHopSafeLabelTimes = protocol.learnedRepairTimes;
config.learnedOneHopSafeLabelMaximumEdits = ...
    protocol.learnedMaximumLabelEdits;
config.learnedOneHopSafeLabelSynopsisHeaderBytes = ...
    protocol.learnedSynopsisHeaderBytes;
config.learnedOneHopSafeLabelSynopsisBytesPerLabel = ...
    protocol.learnedSynopsisBytesPerLabel;
config.learnedOneHopSafeLabelRequestHeaderBytes = ...
    protocol.learnedRequestHeaderBytes;
config.learnedOneHopSafeLabelRequestBytesPerLabel = ...
    protocol.learnedRequestBytesPerLabel;
config.formationCoordinatedPosteriorRepairTimes = ...
    protocol.formationCoordinatedPosteriorRepairTimes;
config.formationCoordinatedPosteriorRepairModelPath = ...
    protocol.formationCoordinatedPosteriorRepairModelPath;
config.formationCoordinatedPosteriorRepairPolicyPath = ...
    protocol.formationCoordinatedPosteriorRepairPolicyPath;
config.formationCoordinatedPosteriorRepairCommonNeighborPrefilterEnabled = ...
    protocol.formationCoordinatedPosteriorRepairCommonNeighborPrefilterEnabled;
config.formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource = ...
    protocol. ...
        formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource;
end

function [labels, sources, applied] = selectedActions(details, receivers)
labels = zeros(2, numel(receivers));
sources = zeros(1, numel(receivers));
applied = false(1, numel(receivers));
for idx = 1:numel(receivers)
    detail = details{receivers(idx)};
    if detail.selectedLabelCount == 1
        labels(:, idx) = detail.selectedLabels(:, 1);
        sources(idx) = detail.selectedSources(1);
    end
    applied(idx) = detail.applied;
end
end

function total = totalDetailField(details, receivers, fieldName)
total = 0;
for receiverIdx = reshape(receivers, 1, [])
    total = total + details{receiverIdx}.(fieldName);
end
end

function value = relativeSavingPercent(reference, candidate)
if reference <= 0
    value = NaN;
else
    value = 100 * (reference - candidate) / reference;
end
end

function value = currentPosteriorRmse(posterior, model, currentTime)
objects = reshape(posterior, 1, []);
if ~isempty(objects)
    objects = objects([objects.r] > model.existenceThreshold);
end
estimate = zeros(2, 0);
if ~isempty(objects)
    [cardinality, indices] = lmbMapCardinalityEstimate([objects.r]);
    estimate = zeros(2, cardinality);
    for idx = 1:cardinality
        object = objects(indices(idx));
        weights = reshape(object.w, 1, []);
        weights(~isfinite(weights)) = -inf;
        [~, componentIdx] = max(weights);
        if isempty(componentIdx) || ~isfinite(weights(componentIdx))
            componentIdx = 1;
        end
        estimate(:, idx) = object.mu{componentIdx}(1:2);
    end
end
truth = zeros(2, 0);
trajectories = model.dynamicTopologyScenario.targetTrajectories;
for targetIdx = 1:numel(trajectories)
    if currentTime <= size(trajectories{targetIdx}, 2)
        state = trajectories{targetIdx}(:, currentTime);
        if all(isfinite(state))
            truth(:, end + 1) = state(1:2); %#ok<AGROW>
        end
    end
end
if isempty(truth) || isempty(estimate)
    value = NaN;
    return;
end
distances = zeros(size(truth, 2), size(estimate, 2));
for truthIdx = 1:size(truth, 2)
    delta = bsxfun(@minus, estimate, truth(:, truthIdx));
    distances(truthIdx, :) = sqrt(sum(delta .^ 2, 1));
end
[matching, ~] = Hungarian(distances);
matched = distances(matching == 1);
if isempty(matched)
    value = NaN;
else
    value = sqrt(mean(matched .^ 2));
end
end

function writeReport(path, preflight)
fid = fopen(path, 'w');
if fid < 0
    error('TemporalHybridV186:ReportOpenFailed', ...
        'Could not open the V186 preflight report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V186 source-local Top-K advertisement preflight\n\n');
fprintf(fid, '- Gate passed: `%d`\n', preflight.gatePassed);
fprintf(fid, '- Source-local label cap: `%d`\n', ...
    preflight.maximumAdvertisedLabelsPerSource);
fprintf(fid, '- Same action as uncapped selector: `%d`\n', ...
    preflight.sameAction);
fprintf(fid, '- Uncapped action: `[%d,%d] <- %d`\n', ...
    preflight.uncappedLabels(1, 1), ...
    preflight.uncappedLabels(2, 1), preflight.uncappedSources(1));
fprintf(fid, '- Capped action: `[%d,%d] <- %d`\n', ...
    preflight.cappedLabels(1, 1), preflight.cappedLabels(2, 1), ...
    preflight.cappedSources(1));
fprintf(fid, '- Synopsis bytes: `%d -> %d` (`%+.3f%%` saving)\n', ...
    preflight.uncappedSynopsisBytes, preflight.cappedSynopsisBytes, ...
    preflight.synopsisSavingPercent);
fprintf(fid, '- Total side-channel bytes: `%d -> %d` (`%+.3f%%` saving)\n\n', ...
    preflight.uncappedTotalBytes, preflight.cappedTotalBytes, ...
    preflight.sideChannelSavingPercent);
fprintf(fid, '| Receiver | Capped E-OSPA gain | Capped RMSE gain |\n');
fprintf(fid, '|--:|--:|--:|\n');
for idx = 1:numel(preflight.receivers)
    fprintf(fid, '| %d | %+.6f | %+.6f |\n', ...
        preflight.receivers(idx), preflight.cappedEospaGain(idx), ...
        preflight.cappedRmseGain(idx));
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    preflight.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
