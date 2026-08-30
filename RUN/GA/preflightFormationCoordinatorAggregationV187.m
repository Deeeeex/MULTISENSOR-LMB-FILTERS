function [reportPath, preflight] = ...
        preflightFormationCoordinatorAggregationV187(options)
% PREFLIGHTFORMATIONCOORDINATORAGGREGATIONV187 Test one-copy summaries.

if nargin < 1 || isempty(options)
    options = struct();
end
screenPath = getField(options, 'screenPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v180', 'recursive_rollout_capture', ...
    'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
referencePath = getField(options, 'referencePath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v186', 'source_local_topk_preflight', ...
    'TEMPORAL_HYBRID_FORMATION_REPAIR_V186_PREFLIGHT.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v187', 'formation_coordinator_preflight'));
if exist(screenPath, 'file') ~= 2 || exist(referencePath, 'file') ~= 2
    error('FormationCoordinatorV187:MissingOpenedState', ...
        'The opened V180 state and uncapped V186 reference are required.');
end
loadedReference = load(referencePath, 'preflight');
reference = loadedReference.preflight;
loaded = load(screenPath, 'screen');
screen = loaded.screen;
outcomeIdx = find(strcmp({screen.records.actionName}, ...
    'v179-analytic-f3-plus-rollout-aware-f5-repair'), 1);
pageIdx = find(screen.returnTimes == 79, 1);
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

protocol = getTemporalHybridFormationRepairV186Protocol();
config = selectorConfig(protocol);
[coordinatorPosterior, coordinatorDetails] = ...
    selectFormationCoordinatedPosteriorRepair( ...
        posterior, locals, physical, physical, config, ...
        receiverIds, 79, model);
[labels, sources, applied] = selectedActions( ...
    coordinatorDetails, receiverIds);
sameAction = isequal(labels, reference.uncappedLabels) && ...
    isequal(sources, reference.uncappedSources);
eospaGain = zeros(1, numel(receiverIds));
rmseGain = zeros(1, numel(receiverIds));
for receiverPosition = 1:numel(receiverIds)
    receiverIdx = receiverIds(receiverPosition);
    baselineEospa = evaluateLmbTopologyCurrentEospa( ...
        posterior{receiverIdx}, model, 79, struct());
    baselineRmse = currentPosteriorRmse( ...
        posterior{receiverIdx}, model, 79);
    eospaGain(receiverPosition) = baselineEospa - ...
        evaluateLmbTopologyCurrentEospa( ...
            coordinatorPosterior{receiverIdx}, model, 79, struct());
    rmseGain(receiverPosition) = baselineRmse - ...
        currentPosteriorRmse( ...
            coordinatorPosterior{receiverIdx}, model, 79);
end
attemptedSynopsisBytes = totalDetailField( ...
    coordinatorDetails, receiverIds, 'attemptedSynopsisBytes');
attemptedRequestBytes = totalDetailField( ...
    coordinatorDetails, receiverIds, 'attemptedRequestBytes');
attemptedResponseBytes = totalDetailField( ...
    coordinatorDetails, receiverIds, 'attemptedResponseBytes');
attemptedTotalBytes = attemptedSynopsisBytes + ...
    attemptedRequestBytes + attemptedResponseBytes;

preflight = struct();
preflight.contractVersion = ...
    'formation-coordinator-aggregation-v187-preflight-v1';
preflight.screenPath = screenPath;
preflight.referencePath = referencePath;
preflight.currentTime = 79;
preflight.receivers = receiverIds;
preflight.synopsisBytesPerLabel = 64;
preflight.coordinatorLabels = labels;
preflight.coordinatorSources = sources;
preflight.applied = applied;
preflight.sameActionAsUncapped = sameAction;
preflight.eospaGain = eospaGain;
preflight.rmseGain = rmseGain;
preflight.referenceAttemptedTotalBytes = ...
    reference.uncappedTotalBytes;
preflight.attemptedSynopsisBytes = attemptedSynopsisBytes;
preflight.attemptedRequestBytes = attemptedRequestBytes;
preflight.attemptedResponseBytes = attemptedResponseBytes;
preflight.attemptedTotalBytes = attemptedTotalBytes;
preflight.sideChannelSavingPercent = 100 * ...
    (reference.uncappedTotalBytes - attemptedTotalBytes) / ...
    reference.uncappedTotalBytes;
preflight.gatePassed = sameAction && all(applied) && ...
    min(eospaGain) >= -1e-9 && min(rmseGain) > 1e-9 && ...
    attemptedTotalBytes < reference.uncappedTotalBytes;
preflight.selectionUsesTruth = false;
preflight.truthUsedForDiagnosticReadout = true;
preflight.recursiveEvaluationRun = false;
preflight.evidenceBoundary = [ ...
    'V187 replays the opened V180 X36 seed-211 t=79 state with ideal ', ...
    'delivery. One formation coordinator receives each source inventory ', ...
    'once and five peer-receiver summaries once. Every per-label message ', ...
    'is charged as a 64-byte position-moment synopsis; selection consumes ', ...
    'only the reconstructed synopsis objects. Truth is read after ', ...
    'selection for immediate E-OSPA/RMSE rejection. A pass authorizes ', ...
    'one recursive development run only.'];
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'FORMATION_COORDINATOR_AGGREGATION_V187_PREFLIGHT.mat');
reportPath = fullfile(outputRoot, ...
    'FORMATION_COORDINATOR_AGGREGATION_V187_PREFLIGHT.md');
preflight.matPath = matPath;
preflight.reportPath = reportPath;
save('-mat7-binary', matPath, 'preflight');
writeReport(reportPath, preflight);
fprintf('V187 formation-coordinator preflight: %s\n', reportPath);
end

function config = selectorConfig(protocol)
config = struct();
config.observableOneHopRiskLabelExistenceThreshold = ...
    protocol.activeExistenceThreshold;
config.learnedOneHopSafeLabelModelPath = protocol.learnedModelPath;
config.learnedOneHopSafeLabelPolicyPath = protocol.learnedPolicyPath;
config.learnedOneHopSafeLabelTimes = protocol.learnedRepairTimes;
config.learnedOneHopSafeLabelMaximumEdits = 1;
config.learnedOneHopSafeLabelSynopsisHeaderBytes = ...
    protocol.learnedSynopsisHeaderBytes;
config.learnedOneHopSafeLabelSynopsisBytesPerLabel = ...
    protocol.learnedSynopsisBytesPerLabel;
config.learnedOneHopSafeLabelRequestHeaderBytes = ...
    protocol.learnedRequestHeaderBytes;
config.learnedOneHopSafeLabelRequestBytesPerLabel = ...
    protocol.learnedRequestBytesPerLabel;
config.formationCoordinatedPosteriorRepairTimes = 79;
config.formationCoordinatedPosteriorRepairModelPath = ...
    protocol.formationCoordinatedPosteriorRepairModelPath;
config.formationCoordinatedPosteriorRepairPolicyPath = ...
    protocol.formationCoordinatedPosteriorRepairPolicyPath;
config.formationCoordinatedPosteriorRepairCommonNeighborPrefilterEnabled = ...
    false;
config.formationCoordinatedPosteriorRepairMaximumAdvertisedLabelsPerSource = ...
    inf;
config.formationCoordinatedPosteriorRepairCoordinatorAggregationEnabled = ...
    true;
config.formationCoordinatedPosteriorRepairCoordinatorHeaderBytes = 16;
config.formationCoordinatedPosteriorRepairCoordinatorSynopsisBytesPerLabel = ...
    64;
config.formationCoordinatedPosteriorRepairCoordinatorRequestBytes = 48;
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
    error('FormationCoordinatorV187:ReportOpenFailed', ...
        'Could not open the V187 preflight report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V187 formation-coordinator aggregation preflight\n\n');
fprintf(fid, '- Gate passed: `%d`\n', preflight.gatePassed);
fprintf(fid, '- Same action as uncapped selector: `%d`\n', ...
    preflight.sameActionAsUncapped);
fprintf(fid, '- Coordinator action: `[%d,%d] <- %d`\n', ...
    preflight.coordinatorLabels(1, 1), ...
    preflight.coordinatorLabels(2, 1), ...
    preflight.coordinatorSources(1));
fprintf(fid, '- Synopsis bytes per label: `%d`\n', ...
    preflight.synopsisBytesPerLabel);
fprintf(fid, ['- Side-channel bytes: `%d -> %d` ', ...
    '(`%+.3f%%` saving)\n'], ...
    preflight.referenceAttemptedTotalBytes, ...
    preflight.attemptedTotalBytes, ...
    preflight.sideChannelSavingPercent);
fprintf(fid, ['- Coordinator synopsis / request / response bytes: ', ...
    '`%d / %d / %d`\n\n'], ...
    preflight.attemptedSynopsisBytes, ...
    preflight.attemptedRequestBytes, ...
    preflight.attemptedResponseBytes);
fprintf(fid, '| Receiver | E-OSPA gain | RMSE gain |\n');
fprintf(fid, '|--:|--:|--:|\n');
for idx = 1:numel(preflight.receivers)
    fprintf(fid, '| %d | %+.6f | %+.6f |\n', ...
        preflight.receivers(idx), preflight.eospaGain(idx), ...
        preflight.rmseGain(idx));
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
