function [reportPath, dataset] = ...
        generateConditionalSecondLabelActionDatasetV173(options)
% GENERATECONDITIONALSECONDLABELACTIONDATASETV173 Sequential action values.
%
% The frozen compact V170 classifier chooses the first complete-label
% action without truth. Every different-label V166 shortlist candidate is
% then rescored from the updated receiver posterior and evaluated as a
% conditional second action. Set-level cardinality summaries expose the
% MAP interaction that made two individually safe actions harmful in V172.

if nargin < 1 || isempty(options)
    options = struct();
end
v157 = getPositiveValueReferenceLabelV157Protocol();
v166Path = getField(options, 'v166DatasetPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v166', 'one_hop_action_value', ...
    'ONE_HOP_LABEL_ACTION_VALUE_DATASET_V166.mat'));
snapshotPath = getField(options, 'snapshotScreenPath', fullfile( ...
    v157.headroomOutputRoot, 'x36_t72_h8', 'screen', ...
    ['TRACKING_ALIGNED_X36_SCHEDULE_H8_', ...
     'X36_FORMATION_FOV_SEED211_T72_D8.mat']));
firstModelPath = getField(options, 'firstModelPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v170', 'cost_aware_feature_profile', ...
    'compact_scalar', ...
    'NONLINEAR_SAFE_ONE_HOP_ACTION_V168_MODEL.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v173', 'conditional_second_action'));
if exist(v166Path, 'file') ~= 2 || ...
        exist(snapshotPath, 'file') ~= 2 || ...
        exist(firstModelPath, 'file') ~= 2
    error('ConditionalSecondActionV173:MissingInput', ...
        'V166, the V157 snapshots, and the compact V170 model are required.');
end

v166Loaded = load(v166Path, 'dataset');
snapshotLoaded = load(snapshotPath, 'screen');
firstLoaded = load(firstModelPath, 'model');
v166 = v166Loaded.dataset;
screen = snapshotLoaded.screen;
firstModel = firstLoaded.model;
validateInputs(v166, firstModel);
outcome = outcomeByAction(screen, v157.candidateActionName);
inputs = generateDynamicTopologyScenarioInputs( ...
    screen.presetName, screen.seed);
model = inputs.model;
model.dynamicTopologyScenario.targetTrajectories = ...
    inputs.targetTrajectories;
groupIds = reshape(screen.sensorGroupIds, 1, []);
nodeCount = numel(groupIds);
activeThreshold = v166.activeExistenceThreshold;
rows = repmat(emptyRow(), 1, 0);
cells = repmat(emptyCell(), 1, 0);
featureNames = cell(1, 0);

for cellRecord166 = reshape(v166.cells, 1, [])
    cellId = cellRecord166.cellId;
    pageIdx = cellRecord166.page;
    currentTime = cellRecord166.time;
    receiverIdx = cellRecord166.receiver;
    fprintf('V173 conditional cell %d/%d: t=%d F=%d receiver=%d\n', ...
        cellId, numel(v166.cells), currentTime, ...
        cellRecord166.formation, receiverIdx);
    baseline = outcome. ...
        fusedPosteriorSnapshotsByTime{pageIdx}{receiverIdx};
    locals = outcome.localPosteriorSnapshotsByTime{pageIdx};
    physical = logical(inputs.graphData. ...
        physicalAdjacency(:, :, currentTime));
    physical = physical | physical';
    physical(1:nodeCount+1:end) = false;
    neighbors = reshape(find(physical(receiverIdx, :)), 1, []);
    sourceRows = v166.rows([v166.rows.cellId] == cellId);
    [firstRow, firstSafetyScore] = ...
        selectFirstAction(sourceRows, firstModel);
    firstObject = findLabelObject( ...
        locals{firstRow.source}, firstRow.label);
    if isempty(firstObject)
        error('ConditionalSecondActionV173:MissingFirstObject', ...
            'The selected first-action source label is missing.');
    end
    afterFirst = replaceLabelObject(baseline, firstObject);
    afterFirstEospa = evaluateLmbTopologyCurrentEospa( ...
        afterFirst, model, currentTime, struct());
    afterFirstRmse = currentPosteriorRmse( ...
        afterFirst, model, currentTime);
    receiverActiveCount = numel(activeObjects( ...
        afterFirst, activeThreshold));
    sourceActiveCount = cellfun(@(posterior) ...
        numel(activeObjects(posterior, activeThreshold)), locals);
    rowStart = numel(rows) + 1;
    for sourceRow = reshape(sourceRows, 1, [])
        if isequal(sourceRow.label, firstRow.label)
            continue;
        end
        sourceObject = findLabelObject( ...
            locals{sourceRow.source}, sourceRow.label);
        if isempty(sourceObject) || ...
                sourceObject.r <= activeThreshold || ...
                sourceObject.numberOfGmComponents <= 0
            continue;
        end
        receiverObject = findLabelObject( ...
            afterFirst, sourceRow.label);
        peers = peerObjectsForLabel( ...
            locals, neighbors, sourceRow.source, ...
            sourceRow.label, activeThreshold);
        context = struct( ...
            'receiverFormation', groupIds(receiverIdx), ...
            'sourceFormation', groupIds(sourceRow.source), ...
            'nodeCount', nodeCount, ...
            'physicalNeighborCount', numel(neighbors), ...
            'receiverActiveLabelCount', receiverActiveCount, ...
            'sourceActiveLabelCount', ...
                sourceActiveCount(sourceRow.source), ...
            'formationSize', nnz(groupIds == groupIds(receiverIdx)));
        firstAction = struct( ...
            'label', firstRow.label, ...
            'source', firstRow.source, ...
            'sourceObject', firstObject, ...
            'safetyScore', firstSafetyScore, ...
            'payloadBytes', firstRow.payloadBytes);
        context.activeExistenceThreshold = activeThreshold;
        [conditionalFeatures, currentFeatureNames, featureDetails] = ...
            computeObservableConditionalOneHopLabelActionFeatures( ...
                baseline, afterFirst, firstAction, sourceObject, peers, ...
                model, receiverIdx, sourceRow.source, currentTime, context);
        if isempty(featureNames)
            featureNames = currentFeatureNames;
        elseif ~isequal(featureNames, currentFeatureNames)
            error('ConditionalSecondActionV173:FeatureContractDrift', ...
                'Conditional feature names changed between candidates.');
        end
        trial = replaceLabelObject(afterFirst, sourceObject);
        row = emptyRow();
        row.cellId = cellId;
        row.split = cellRecord166.split;
        row.page = pageIdx;
        row.time = currentTime;
        row.formation = cellRecord166.formation;
        row.receiver = receiverIdx;
        row.source = sourceRow.source;
        row.label = sourceRow.label;
        row.firstSource = firstRow.source;
        row.firstLabel = firstRow.label;
        row.firstSafetyScore = firstSafetyScore;
        row.features = conditionalFeatures;
        row.eospaGain = afterFirstEospa - ...
            evaluateLmbTopologyCurrentEospa( ...
                trial, model, currentTime, struct());
        row.rmseGain = afterFirstRmse - ...
            currentPosteriorRmse(trial, model, currentTime);
        row.payloadBytes = featureDetails.payloadBytes;
        rows(end + 1) = row; %#ok<AGROW>
    end
    cellRows = rows(rowStart:end);
    cellRecord = emptyCell();
    cellRecord.cellId = cellId;
    cellRecord.split = cellRecord166.split;
    cellRecord.page = pageIdx;
    cellRecord.time = currentTime;
    cellRecord.formation = cellRecord166.formation;
    cellRecord.receiver = receiverIdx;
    cellRecord.firstSource = firstRow.source;
    cellRecord.firstLabel = firstRow.label;
    cellRecord.firstSafetyScore = firstSafetyScore;
    cellRecord.firstEospaGain = firstRow.eospaGain;
    cellRecord.firstRmseGain = firstRow.rmseGain;
    cellRecord.candidateCount = numel(cellRows);
    cellRecord.safeCandidateCount = nnz( ...
        [cellRows.eospaGain] > 0 & [cellRows.rmseGain] > 0);
    cellRecord.safeUniqueLabelCount = safeUniqueLabelCount(cellRows);
    cells(end + 1) = cellRecord; %#ok<AGROW>
end

dataset = struct();
dataset.contractVersion = ...
    'conditional-second-label-action-dataset-v173-v1';
dataset.featureContractVersion = ...
    'conditional-observable-one-hop-label-action-features-v173-v1';
dataset.presetName = screen.presetName;
dataset.seed = screen.seed;
dataset.v166DatasetPath = v166Path;
dataset.snapshotPath = snapshotPath;
dataset.firstModelPath = firstModelPath;
dataset.activeExistenceThreshold = activeThreshold;
dataset.featureNames = featureNames;
dataset.featureCount = numel(featureNames);
dataset.rows = rows;
dataset.cells = cells;
dataset.splitNames = v166.splitNames;
dataset.trainingCellIds = [cells([cells.split] == 1).cellId];
dataset.calibrationCellIds = [cells([cells.split] == 2).cellId];
dataset.heldoutCellIds = [cells([cells.split] == 3).cellId];
dataset.featuresUseTruth = false;
dataset.featuresUseFutureInformation = false;
dataset.numericLabelIdentifiersUsedAsFeatures = false;
dataset.targetsUseCurrentTruth = true;
dataset.targetsUseFutureOutcome = false;
dataset.firstActionUsesTruth = false;
dataset.policyTrained = false;
dataset.recursiveEvaluationRun = false;
dataset.deployable = false;
dataset.evidenceBoundary = [ ...
    'V173 is an opened X36 seed-211 conditional-action dataset. The ', ...
    'frozen compact V170 classifier selects a first complete label using ', ...
    'only present observable features. Candidate second actions use the ', ...
    'updated receiver posterior plus truth-free LMB cardinality, first-', ...
    'action and topology summaries; numeric label IDs are not features. ', ...
    'Current truth scores the incremental second-action E-OSPA and RMSE ', ...
    'gains. Splits remain grouped exactly as V166, so the same-seed ', ...
    'heldout screen is a learnability test rather than validation.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'CONDITIONAL_SECOND_LABEL_ACTION_DATASET_V173.mat');
reportPath = fullfile(outputRoot, ...
    'CONDITIONAL_SECOND_LABEL_ACTION_DATASET_V173.md');
dataset.matPath = matPath;
dataset.reportPath = reportPath;
save('-mat7-binary', matPath, 'dataset');
writeReport(reportPath, dataset);
fprintf('V173 conditional second-action dataset: %s\n', reportPath);
end

function validateInputs(dataset, model)
if ~strcmp(dataset.contractVersion, ...
        'one-hop-label-action-value-dataset-v166-v1') || ...
        dataset.featuresUseTruth || dataset.featuresUseFutureInformation || ...
        ~strcmp(model.contractVersion, ...
            'nonlinear-safe-one-hop-label-action-model-v168-v1') || ...
        ~isfield(model, 'heldoutSafetyGatePassed') || ...
        ~model.heldoutSafetyGatePassed || model.featuresUseTruth || ...
        model.numericLabelIdentifiersUsedAsFeatures
    error('ConditionalSecondActionV173:InputContract', ...
        'The V166 dataset or compact first-action model is invalid.');
end
end

function [row, score] = selectFirstAction(rows, model)
X = vertcat(rows.features);
if isfield(model, 'featureMask')
    X = X(:, logical(model.featureMask));
end
Z = bsxfun(@rdivide, bsxfun(@minus, X, model.featureMean), ...
    model.featureScale);
inSupport = all(bsxfun(@ge, Z, model.supportLower) & ...
    bsxfun(@le, Z, model.supportUpper), 2);
probability = zeros(numel(rows), 3, numel(model.members));
for memberIdx = 1:numel(model.members)
    parameters = model.members{memberIdx}.parameters;
    hidden = tanh(bsxfun(@plus, ...
        Z * parameters{1}, parameters{2}));
    logits = bsxfun(@plus, ...
        hidden * parameters{3}, parameters{4});
    probability(:, :, memberIdx) = stableSigmoid(logits);
end
lower = mean(probability, 3) - ...
    model.uncertaintyPenalty * std(probability, 0, 3);
scoreByRow = min(lower, [], 2);
eligible = find(inSupport & ...
    all(lower > model.probabilityThreshold, 2));
if isempty(eligible)
    error('ConditionalSecondActionV173:FirstActionAbstained', ...
        'The frozen first-action model abstained in a registered cell.');
end
labels = reshape([rows(eligible).label], 2, [])';
sources = [rows(eligible).source]';
[~, order] = sortrows( ...
    [-scoreByRow(eligible), labels, sources], [1, 2, 3, 4]);
selectedIdx = eligible(order(1));
row = rows(selectedIdx);
score = scoreByRow(selectedIdx);
end

function peers = peerObjectsForLabel( ...
        locals, neighbors, selectedSource, label, threshold)
peers = [];
for sourceIdx = neighbors
    if sourceIdx == selectedSource
        continue;
    end
    object = findLabelObject(locals{sourceIdx}, label);
    if ~isempty(object) && object.r > threshold && ...
            object.numberOfGmComponents > 0
        if isempty(peers)
            peers = object;
        else
            peers(end + 1) = object; %#ok<AGROW>
        end
    end
end
peers = reshape(peers, 1, []);
end

function objects = activeObjects(objects, threshold)
objects = reshape(objects, 1, []);
if isempty(objects)
    return;
end
objects = objects([objects.r] > threshold & ...
    [objects.numberOfGmComponents] > 0);
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

function value = currentPosteriorRmse(posterior, model, currentTime)
objects = activeObjects(posterior, model.existenceThreshold);
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

function outcome = outcomeByAction(screen, actionName)
idx = find(strcmp({screen.records.actionName}, actionName), 1);
if isempty(idx)
    error('ConditionalSecondActionV173:MissingAction', ...
        'The requested action is missing: %s', actionName);
end
outcome = screen.outcomes(idx);
end

function count = safeUniqueLabelCount(rows)
if isempty(rows)
    count = 0;
    return;
end
safe = rows([rows.eospaGain] > 0 & [rows.rmseGain] > 0);
if isempty(safe)
    count = 0;
else
    count = size(unique(reshape([safe.label], 2, [])', ...
        'rows', 'stable'), 1);
end
end

function row = emptyRow()
row = struct( ...
    'cellId', 0, 'split', 0, 'page', 0, 'time', 0, ...
    'formation', 0, 'receiver', 0, 'source', 0, ...
    'label', zeros(2, 1), 'firstSource', 0, ...
    'firstLabel', zeros(2, 1), 'firstSafetyScore', NaN, ...
    'features', zeros(1, 0), 'eospaGain', NaN, ...
    'rmseGain', NaN, 'payloadBytes', 0);
end

function cellRecord = emptyCell()
cellRecord = struct( ...
    'cellId', 0, 'split', 0, 'page', 0, 'time', 0, ...
    'formation', 0, 'receiver', 0, 'firstSource', 0, ...
    'firstLabel', zeros(2, 1), 'firstSafetyScore', NaN, ...
    'firstEospaGain', NaN, 'firstRmseGain', NaN, ...
    'candidateCount', 0, 'safeCandidateCount', 0, ...
    'safeUniqueLabelCount', 0);
end

function value = stableSigmoid(value)
positive = value >= 0;
negative = ~positive;
result = zeros(size(value));
result(positive) = 1 ./ (1 + exp(-value(positive)));
exponential = exp(value(negative));
result(negative) = exponential ./ (1 + exponential);
value = result;
end

function writeReport(path, dataset)
fid = fopen(path, 'w');
if fid < 0
    error('ConditionalSecondActionV173:ReportOpenFailed', ...
        'Could not open the V173 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V173 conditional second-label action dataset\n\n');
fprintf(fid, '- Preset / seed: `%s / %d`\n', ...
    dataset.presetName, dataset.seed);
fprintf(fid, '- Feature count: `%d`\n', dataset.featureCount);
fprintf(fid, '- Candidate rows / cells: `%d / %d`\n\n', ...
    numel(dataset.rows), numel(dataset.cells));
fprintf(fid, ['| Split | Cells | Rows | Joint-positive rows | ', ...
    'Cells with a safe second label |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|\n');
for splitIdx = 1:numel(dataset.splitNames)
    localCells = dataset.cells([dataset.cells.split] == splitIdx);
    localRows = dataset.rows([dataset.rows.split] == splitIdx);
    safeRows = nnz([localRows.eospaGain] > 0 & ...
        [localRows.rmseGain] > 0);
    fprintf(fid, '| %s | %d | %d | %d | %d |\n', ...
        dataset.splitNames{splitIdx}, numel(localCells), ...
        numel(localRows), safeRows, ...
        nnz([localCells.safeUniqueLabelCount] > 0));
end
fprintf(fid, ['\n| t | F | Receiver | First E/R | Candidates | ', ...
    'Safe | Safe labels |\n']);
fprintf(fid, '|--:|--:|--:|:--|--:|--:|--:|\n');
for cellRecord = dataset.cells
    fprintf(fid, '| %d | %d | %d | %+.3f/%+.3f | %d | %d | %d |\n', ...
        cellRecord.time, cellRecord.formation, cellRecord.receiver, ...
        cellRecord.firstEospaGain, cellRecord.firstRmseGain, ...
        cellRecord.candidateCount, cellRecord.safeCandidateCount, ...
        cellRecord.safeUniqueLabelCount);
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
