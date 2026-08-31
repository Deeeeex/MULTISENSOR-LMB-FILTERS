function [reportPath, dataset] = ...
        generateOneHopLabelActionValueDatasetV166(options)
% GENERATEONEHOPLABELACTIONVALUEDATASETV166 Grouped action-value dataset.
%
% The dataset is a development learnability gate. Features are current and
% truth-free; current truth scores the immediate E-OSPA and RMSE change of a
% complete one-hop label replacement. Splits are receiver-time groups, never
% individual candidate rows.

if nargin < 1 || isempty(options)
    options = struct();
end
v157 = getPositiveValueReferenceLabelV157Protocol();
snapshotPath = getField(options, 'snapshotScreenPath', fullfile( ...
    v157.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v166', 'one_hop_action_value'));
shortlistPerCriterion = getField(options, ...
    'shortlistPerCriterion', 1);
snapshotActionName = getField(options, ...
    'snapshotActionName', v157.candidateActionName);
repairPages = reshape(getField(options, ...
    'repairPages', [5, 7, 8]), 1, []);
repairFormationsByPage = getField(options, ...
    'repairFormationsByPage', {[3], [3, 5], [3, 5]});
splitBySchedule = reshape(getField(options, ...
    'splitBySchedule', zeros(1, 0)), 1, []);
splitNames = getField(options, 'splitNames', ...
    {'training', 'calibration', 'heldout'});
datasetContractVersion = getField(options, ...
    'datasetContractVersion', ...
    'one-hop-label-action-value-dataset-v166-v1');
resultFileStem = getField(options, 'resultFileStem', ...
    'ONE_HOP_LABEL_ACTION_VALUE_DATASET_V166');
reportTitle = getField(options, 'reportTitle', ...
    'V166 one-hop label-action value dataset');
if exist(snapshotPath, 'file') ~= 2 || ...
        ~isscalar(shortlistPerCriterion) || ...
        shortlistPerCriterion < 1 || ...
        shortlistPerCriterion ~= round(shortlistPerCriterion) || ...
        isempty(repairPages) || numel(unique(repairPages)) ~= ...
            numel(repairPages) || any(repairPages < 1) || ...
        any(repairPages ~= round(repairPages)) || ...
        ~iscell(repairFormationsByPage) || ...
        numel(repairFormationsByPage) ~= numel(repairPages) || ...
        (~isempty(splitBySchedule) && ...
         (numel(splitBySchedule) ~= numel(repairPages) || ...
          any(~ismember(splitBySchedule, 1:3)))) || ...
        ~iscell(splitNames) || numel(splitNames) ~= 3 || ...
        any(~cellfun(@(name) ischar(name) && ~isempty(name), ...
            splitNames)) || ...
        ~ischar(snapshotActionName) || isempty(snapshotActionName) || ...
        ~ischar(datasetContractVersion) || isempty(datasetContractVersion) || ...
        ~ischar(resultFileStem) || isempty(resultFileStem) || ...
        ~ischar(reportTitle) || isempty(reportTitle)
    error('OneHopActionDatasetV166:InvalidInput', ...
        'A V157 snapshot and positive shortlist size are required.');
end

loaded = load(snapshotPath, 'screen');
screen = loaded.screen;
outcome = outcomeByAction(screen, snapshotActionName);
if isempty(outcome.fusedPosteriorSnapshotsByTime) || ...
        isempty(outcome.localPosteriorSnapshotsByTime)
    error('OneHopActionDatasetV166:MissingSnapshots', ...
        'Fused and local posterior snapshots are required.');
end
inputs = generateDynamicTopologyScenarioInputs( ...
    screen.presetName, screen.seed);
model = inputs.model;
model.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(screen.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
activeThreshold = 1e-2;
rows = repmat(emptyRow(), 1, 0);
cells = repmat(emptyCell(), 1, 0);
featureNames = cell(1, 0);
cellId = 0;
totalCellCount = 0;
for scheduleIdx = 1:numel(repairPages)
    totalCellCount = totalCellCount + nnz(ismember( ...
        groupIds, repairFormationsByPage{scheduleIdx}));
end

for scheduleIdx = 1:numel(repairPages)
    pageIdx = repairPages(scheduleIdx);
    currentTime = screen.returnTimes(pageIdx);
    receivers = find(ismember( ...
        groupIds, repairFormationsByPage{scheduleIdx}));
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    for receiverIdx = reshape(receivers, 1, [])
        cellId = cellId + 1;
        fprintf('One-hop dataset cell %d/%d: t=%d F=%d receiver=%d\n', ...
            cellId, totalCellCount, currentTime, ...
            groupIds(receiverIdx), receiverIdx);
        baseline = outcome.fusedPosteriorSnapshotsByTime{pageIdx}{receiverIdx};
        locals = outcome.localPosteriorSnapshotsByTime{pageIdx};
        neighbors = reshape(find(physical(receiverIdx, :)), 1, []);
        [rawCandidates, cellFeatureNames] = buildCandidates( ...
            baseline, locals, neighbors, receiverIdx, currentTime, ...
            model, groupIds, activeThreshold);
        if isempty(featureNames)
            featureNames = cellFeatureNames;
        elseif ~isequal(featureNames, cellFeatureNames)
            error('OneHopActionDatasetV166:FeatureContractDrift', ...
                'Candidate feature names changed between cells.');
        end
        shortlisted = shortlistCandidates( ...
            rawCandidates, shortlistPerCriterion);
        baselineEospa = evaluateLmbTopologyCurrentEospa( ...
            baseline, model, currentTime, struct());
        baselineRmse = currentPosteriorRmse( ...
            baseline, model, currentTime);
        cellRowStart = numel(rows) + 1;
        for candidate = reshape(shortlisted, 1, [])
            trial = replaceLabelObject(baseline, candidate.object);
            row = emptyRow();
            row.cellId = cellId;
            row.split = resolveSplit( ...
                scheduleIdx, currentTime, groupIds(receiverIdx), ...
                splitBySchedule);
            row.page = pageIdx;
            row.time = currentTime;
            row.formation = groupIds(receiverIdx);
            row.receiver = receiverIdx;
            row.source = candidate.source;
            row.label = candidate.label;
            row.features = candidate.features;
            row.eospaGain = baselineEospa - ...
                evaluateLmbTopologyCurrentEospa( ...
                    trial, model, currentTime, struct());
            row.rmseGain = baselineRmse - ...
                currentPosteriorRmse(trial, model, currentTime);
            row.payloadBytes = candidate.payloadBytes;
            rows(end + 1) = row; %#ok<AGROW>
        end
        cellRows = rows(cellRowStart:end);
        cellRecord = emptyCell();
        cellRecord.cellId = cellId;
        cellRecord.split = resolveSplit( ...
            scheduleIdx, currentTime, groupIds(receiverIdx), ...
            splitBySchedule);
        cellRecord.page = pageIdx;
        cellRecord.time = currentTime;
        cellRecord.formation = groupIds(receiverIdx);
        cellRecord.receiver = receiverIdx;
        cellRecord.physicalNeighborCount = numel(neighbors);
        cellRecord.rawCandidateCount = numel(rawCandidates);
        cellRecord.shortlistedCandidateCount = numel(shortlisted);
        cellRecord.safeCandidateCount = nnz( ...
            [cellRows.eospaGain] >= -1e-9 & ...
            [cellRows.rmseGain] > 1e-9);
        cellRecord.safeUniqueLabelCount = safeUniqueLabelCount(cellRows);
        cells(end + 1) = cellRecord; %#ok<AGROW>
    end
end

dataset = struct();
dataset.contractVersion = datasetContractVersion;
dataset.featureContractVersion = ...
    'observable-one-hop-label-action-features-v1';
dataset.presetName = screen.presetName;
dataset.seed = screen.seed;
dataset.snapshotPath = snapshotPath;
dataset.snapshotActionName = snapshotActionName;
dataset.repairPages = repairPages;
dataset.repairTimes = screen.returnTimes(repairPages);
dataset.repairFormationsByPage = repairFormationsByPage;
dataset.activeExistenceThreshold = activeThreshold;
dataset.shortlistPerCriterion = shortlistPerCriterion;
dataset.shortlistCriteria = { ...
    'risk_reduction', ...
    'receiver_source_compatibility', ...
    'peer_consensus_mean', ...
    'source_evidence_quality', ...
    'source_observation_opportunity', ...
    'credibility_weighted_risk', ...
    'source_low_risk_percentile'};
dataset.featureNames = featureNames;
dataset.featureCount = numel(featureNames);
dataset.rows = rows;
dataset.cells = cells;
dataset.splitNames = reshape(splitNames, 1, []);
dataset.trainingCellIds = [cells([cells.split] == 1).cellId];
dataset.calibrationCellIds = [cells([cells.split] == 2).cellId];
dataset.heldoutCellIds = [cells([cells.split] == 3).cellId];
dataset.featuresUseTruth = false;
dataset.featuresUseFutureInformation = false;
dataset.numericLabelIdentifiersUsedAsFeatures = false;
dataset.targetsUseCurrentTruth = true;
dataset.targetsUseFutureOutcome = false;
dataset.policyTrained = false;
dataset.recursiveEvaluationRun = false;
dataset.deployable = false;
defaultEvidenceBoundary = [ ...
    'V166 is an opened X36 seed-211 single-step learnability dataset. ', ...
    'Feature rows contain current posterior, physical-neighbor agreement, ', ...
    'FoV opportunity, formation-role and payload summaries but exclude ', ...
    'numeric label keys, truth, future measurements and alternative-arm ', ...
    'state. Current truth supplies immediate complete-label E-OSPA and ', ...
    'matched-position RMSE targets. Candidate shortlisting is the union ', ...
    'of seven truth-free rankings. Training uses t=76/78 cells, ', ...
    'calibration uses t=79 F3 cells, and heldout testing uses t=79 F5 ', ...
    'cells. This same-seed temporal/formation split is a learnability ', ...
    'screen, not independent validation or generalization evidence.'];
dataset.evidenceBoundary = getField(options, ...
    'evidenceBoundary', defaultEvidenceBoundary);

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, [resultFileStem, '.mat']);
reportPath = fullfile(outputRoot, [resultFileStem, '.md']);
dataset.matPath = matPath;
dataset.reportPath = reportPath;
save('-mat7-binary', matPath, 'dataset');
writeReport(reportPath, dataset, reportTitle);
fprintf('V166 action-value dataset: %s\n', reportPath);
end

function [candidates, featureNames] = buildCandidates( ...
        receiverPosterior, locals, neighbors, receiverIdx, currentTime, ...
        model, groupIds, activeThreshold)
labels = zeros(2, 0);
sourceActiveCount = zeros(1, numel(locals));
for sourceIdx = neighbors
    active = activeObjects(locals{sourceIdx}, activeThreshold);
    sourceActiveCount(sourceIdx) = numel(active);
    for object = reshape(active, 1, [])
        label = [object.birthTime; object.birthLocation];
        if isempty(labels) || ~any(all(bsxfun(@eq, labels, label), 1))
            labels(:, end + 1) = label; %#ok<AGROW>
        end
    end
end
if ~isempty(labels)
    labels = sortrows(labels')';
end
receiverActiveCount = numel(activeObjects( ...
    receiverPosterior, activeThreshold));
candidates = repmat(emptyCandidate(), 1, 0);
featureNames = cell(1, 0);
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    receiverObject = findLabelObject(receiverPosterior, label);
    sourceIds = zeros(1, 0);
    sourceObjects = repmat(emptyObjectLike(locals), 1, 0);
    for sourceIdx = neighbors
        sourceObject = findLabelObject(locals{sourceIdx}, label);
        if ~isempty(sourceObject) && sourceObject.r > activeThreshold && ...
                sourceObject.numberOfGmComponents > 0
            sourceIds(end + 1) = sourceIdx; %#ok<AGROW>
            sourceObjects(end + 1) = sourceObject; %#ok<AGROW>
        end
    end
    for localIdx = 1:numel(sourceIds)
        sourceIdx = sourceIds(localIdx);
        sourceObject = sourceObjects(localIdx);
        peers = sourceObjects(setdiff(1:numel(sourceIds), localIdx));
        context = struct( ...
            'receiverFormation', groupIds(receiverIdx), ...
            'sourceFormation', groupIds(sourceIdx), ...
            'nodeCount', numel(groupIds), ...
            'physicalNeighborCount', numel(neighbors), ...
            'receiverActiveLabelCount', receiverActiveCount, ...
            'sourceActiveLabelCount', sourceActiveCount(sourceIdx), ...
            'formationSize', nnz(groupIds == groupIds(receiverIdx)));
        [features, names, featureDetails] = ...
            computeObservableOneHopLabelActionFeatures( ...
                receiverObject, sourceObject, peers, model, ...
                receiverIdx, sourceIdx, currentTime, context);
        if isempty(featureNames)
            featureNames = names;
        elseif ~isequal(featureNames, names)
            error('OneHopActionDatasetV166:FeatureNameDrift', ...
                'Candidate feature names changed within a cell.');
        end
        candidate = emptyCandidate();
        candidate.label = label;
        candidate.source = sourceIdx;
        candidate.object = sourceObject;
        candidate.features = features;
        candidate.payloadBytes = featureDetails.payloadBytes;
        candidates(end + 1) = candidate; %#ok<AGROW>
    end
end
end

function shortlisted = shortlistCandidates(candidates, count)
shortlisted = repmat(emptyCandidate(), 1, 0);
if isempty(candidates)
    return;
end
labels = unique(reshape([candidates.label], 2, [])', 'rows', 'stable')';
for labelIdx = 1:size(labels, 2)
    label = labels(:, labelIdx);
    local = candidates(arrayfun(@(item) ...
        isequal(item.label, label), candidates));
    featureNames = featureNameContract();
    risk = featureColumn(local, featureNames, 'risk_reduction');
    compatibility = featureColumn(local, featureNames, ...
        'receiver_source_compatibility');
    consensus = featureColumn(local, featureNames, 'peer_consensus_mean');
    evidence = featureColumn(local, featureNames, ...
        'source_evidence_quality');
    opportunity = featureColumn(local, featureNames, ...
        'source_observation_opportunity');
    credibility = featureColumn(local, featureNames, ...
        'spatial_credibility');
    lowRisk = featureColumn(local, featureNames, ...
        'source_low_risk_percentile');
    criteria = [risk; compatibility; consensus; evidence; opportunity; ...
        risk .* sqrt(max(credibility, 0)); lowRisk];
    chosen = zeros(1, 0);
    for criterionIdx = 1:size(criteria, 1)
        ranking = [-criteria(criterionIdx, :)', [local.source]'];
        [~, order] = sortrows(ranking, [1, 2]);
        chosen = unique([chosen, ...
            order(1:min(count, numel(order)))'], 'stable'); %#ok<AGROW>
    end
    shortlisted = [shortlisted, local(chosen)]; %#ok<AGROW>
end
end

function names = featureNameContract()
persistent cached;
if isempty(cached)
    % Names are stable and duplicated here only to index already-built rows.
    cached = { ...
        'receiver_present', 'receiver_existence', 'source_existence', ...
        'receiver_bayes_risk', 'source_bayes_risk', 'risk_reduction', ...
        'receiver_position_trace_normalized', ...
        'source_position_trace_normalized', ...
        'receiver_evidence_quality', 'source_evidence_quality', ...
        'receiver_observation_opportunity', ...
        'source_observation_opportunity', 'existence_gain', ...
        'precision_gain', 'evidence_gap', 'opportunity_gap', ...
        'log_mahalanobis_disagreement', ...
        'receiver_source_compatibility', 'source_quality', ...
        'confidence_disagreement_score', 'handover_rescue_score', ...
        'log_payload_kib', 'peer_count_fraction', ...
        'peer_consensus_mean', 'peer_consensus_median', ...
        'peer_consensus_maximum', 'peer_support_chi2_95', ...
        'peer_support_chi2_99', 'source_low_risk_percentile', ...
        'spatial_credibility', 'same_formation', 'label_age_fraction', ...
        'physical_degree_fraction', 'receiver_active_label_fraction', ...
        'source_active_label_fraction', 'formation_size_fraction', ...
        'log_source_receiver_trace_ratio'};
end
names = cached;
end

function values = featureColumn(candidates, names, name)
idx = find(strcmp(names, name), 1);
if isempty(idx)
    error('OneHopActionDatasetV166:MissingFeature', ...
        'Missing shortlist feature: %s', name);
end
matrix = vertcat(candidates.features);
values = matrix(:, idx)';
end

function value = resolveSplit( ...
        scheduleIdx, currentTime, formation, splitBySchedule)
if ~isempty(splitBySchedule)
    value = splitBySchedule(scheduleIdx);
elseif currentTime <= 78
    value = 1;
elseif formation == 3
    value = 2;
else
    value = 3;
end
end

function count = safeUniqueLabelCount(rows)
if isempty(rows)
    count = 0;
    return;
end
safe = rows([rows.eospaGain] >= -1e-9 & [rows.rmseGain] > 1e-9);
if isempty(safe)
    count = 0;
else
    count = size(unique(reshape([safe.label], 2, [])', ...
        'rows', 'stable'), 1);
end
end

function objects = activeObjects(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
end

function template = emptyObjectLike(locals)
template = [];
for idx = 1:numel(locals)
    if ~isempty(locals{idx})
        template = locals{idx}(1);
        template = template([]);
        return;
    end
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

function object = findLabelObject(objects, label)
idx = findLabelIndex(objects, label);
if idx == 0
    object = [];
else
    object = objects(idx);
end
end

function posterior = replaceLabelObject(posterior, object)
posterior = reshape(posterior, 1, []);
idx = findLabelIndex(posterior, ...
    [object.birthTime; object.birthLocation]);
if idx == 0
    posterior(end + 1) = object;
else
    posterior(idx) = object;
end
end

function idx = findLabelIndex(objects, label)
idx = 0;
for objectIdx = 1:numel(objects)
    if objects(objectIdx).birthTime == label(1) && ...
            objects(objectIdx).birthLocation == label(2)
        idx = objectIdx;
        return;
    end
end
end

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('OneHopActionDatasetV166:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function candidate = emptyCandidate()
candidate = struct( ...
    'label', zeros(2, 1), 'source', 0, 'object', [], ...
    'features', zeros(1, 0), 'payloadBytes', 0);
end

function row = emptyRow()
row = struct( ...
    'cellId', 0, 'split', 0, 'page', 0, 'time', 0, ...
    'formation', 0, 'receiver', 0, 'source', 0, ...
    'label', zeros(2, 1), 'features', zeros(1, 0), ...
    'eospaGain', NaN, 'rmseGain', NaN, 'payloadBytes', 0);
end

function cellRecord = emptyCell()
cellRecord = struct( ...
    'cellId', 0, 'split', 0, 'page', 0, 'time', 0, ...
    'formation', 0, 'receiver', 0, ...
    'physicalNeighborCount', 0, 'rawCandidateCount', 0, ...
    'shortlistedCandidateCount', 0, 'safeCandidateCount', 0, ...
    'safeUniqueLabelCount', 0);
end

function writeReport(path, dataset, reportTitle)
fid = fopen(path, 'w');
if fid < 0
    error('OneHopActionDatasetV166:ReportOpenFailed', ...
        'Could not open the V166 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# %s\n\n', reportTitle);
fprintf(fid, '- Preset / seed: `%s / %d`\n', ...
    dataset.presetName, dataset.seed);
fprintf(fid, '- Feature count: `%d`\n', dataset.featureCount);
fprintf(fid, '- Candidate rows / cells: `%d / %d`\n', ...
    numel(dataset.rows), numel(dataset.cells));
fprintf(fid, '- Shortlist per criterion: `%d`\n\n', ...
    dataset.shortlistPerCriterion);
fprintf(fid, ['| Split | Cells | Rows | Joint-positive rows | ', ...
    'Cells with >=4 safe labels |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|\n');
for splitIdx = 1:numel(dataset.splitNames)
    cells = dataset.cells([dataset.cells.split] == splitIdx);
    rows = dataset.rows([dataset.rows.split] == splitIdx);
    safeRows = nnz([rows.eospaGain] >= -1e-9 & ...
        [rows.rmseGain] > 1e-9);
    fprintf(fid, '| %s | %d | %d | %d | %d |\n', ...
        dataset.splitNames{splitIdx}, numel(cells), numel(rows), ...
        safeRows, nnz([cells.safeUniqueLabelCount] >= 4));
end
fprintf(fid, '\n| t | F | Receiver | Raw | Shortlist | Safe | Safe labels | Split |\n');
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|:--|\n');
for cellRecord = dataset.cells
    fprintf(fid, '| %d | %d | %d | %d | %d | %d | %d | %s |\n', ...
        cellRecord.time, cellRecord.formation, cellRecord.receiver, ...
        cellRecord.rawCandidateCount, ...
        cellRecord.shortlistedCandidateCount, ...
        cellRecord.safeCandidateCount, ...
        cellRecord.safeUniqueLabelCount, ...
        dataset.splitNames{cellRecord.split});
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    dataset.evidenceBoundary);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
