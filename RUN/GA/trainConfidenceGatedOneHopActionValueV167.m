function [modelPath, result] = ...
        trainConfidenceGatedOneHopActionValueV167(options)
% TRAINCONFIDENCEGATEDONEHOPACTIONVALUEV167 Two-target ridge learnability gate.
%
% Fit separate normalized E-OSPA and RMSE action-value heads on grouped V166
% training cells. A selected-action block calibration subtracts one shared
% overprediction correction from both heads. Inference may abstain and may
% select at most K unique labels. Heldout F5 cells are opened only after the
% lambda/K/calibration choice is frozen on training and F3 calibration cells.

if nargin < 1 || isempty(options)
    options = struct();
end
datasetPath = getField(options, 'datasetPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v166', 'one_hop_action_value', ...
    'ONE_HOP_LABEL_ACTION_VALUE_DATASET_V166.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v167', 'confidence_gated_action_value'));
ridgeGrid = reshape(getField(options, ...
    'ridgeGrid', [0.01, 0.1, 1, 10]), 1, []);
maximumEditsGrid = reshape(getField(options, ...
    'maximumEditsGrid', 1:4), 1, []);
confidenceLevel = getField(options, 'confidenceLevel', 0.80);
supportMargin = getField(options, 'supportMarginStd', 0.50);
if exist(datasetPath, 'file') ~= 2 || ...
        any(~isfinite(ridgeGrid) | ridgeGrid < 0) || ...
        any(maximumEditsGrid < 1 | ...
            maximumEditsGrid ~= round(maximumEditsGrid)) || ...
        confidenceLevel < 0.5 || confidenceLevel >= 1 || ...
        supportMargin < 0
    error('OneHopActionValueV167:InvalidOptions', ...
        'The V167 fit options are invalid.');
end
loaded = load(datasetPath, 'dataset');
dataset = loaded.dataset;
validateDataset(dataset);
rows = dataset.rows;
X = vertcat(rows.features);
Y = [[rows.eospaGain]', [rows.rmseGain]'];
cellIds = [rows.cellId]';
splits = [rows.split]';
trainMask = splits == 1;
calibrationMask = splits == 2;
heldoutMask = splits == 3;
featureMean = mean(X(trainMask, :), 1);
featureScale = std(X(trainMask, :), 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
Z = bsxfun(@rdivide, bsxfun(@minus, X, featureMean), featureScale);
targetScale = robustTargetScale(Y(trainMask, :));
normalizedTarget = bsxfun(@rdivide, Y, targetScale);
supportLower = min(Z(trainMask, :), [], 1) - supportMargin;
supportUpper = max(Z(trainMask, :), [], 1) + supportMargin;
inSupport = all(bsxfun(@ge, Z, supportLower) & ...
    bsxfun(@le, Z, supportUpper), 2);

configs = repmat(emptyConfig(), 1, 0);
for ridgeLambda = ridgeGrid
    [coefficient, intercept] = fitRidge( ...
        Z(trainMask, :), normalizedTarget(trainMask, :), ridgeLambda);
    prediction = bsxfun(@plus, Z * coefficient, intercept);
    for maximumEdits = maximumEditsGrid
        correction = selectedActionCorrection( ...
            prediction, normalizedTarget, rows, cellIds, ...
            calibrationMask, inSupport, maximumEdits, ...
            confidenceLevel);
        lowerBound = prediction - correction;
        lowerBound(~inSupport, :) = -inf;
        [calibrationReadout, calibrationSelected] = ...
            evaluateSelection(lowerBound, normalizedTarget, rows, ...
                cellIds, calibrationMask, maximumEdits, targetScale);
        if calibrationReadout.harmfulActionCount > 0
            correction = escalateCorrection( ...
                prediction, normalizedTarget, ...
                calibrationSelected, correction);
            lowerBound = prediction - correction;
            lowerBound(~inSupport, :) = -inf;
            [calibrationReadout, calibrationSelected] = ...
                evaluateSelection(lowerBound, normalizedTarget, rows, ...
                    cellIds, calibrationMask, maximumEdits, targetScale);
        end
        config = emptyConfig();
        config.ridgeLambda = ridgeLambda;
        config.maximumEdits = maximumEdits;
        config.correction = correction;
        config.coefficient = coefficient;
        config.intercept = intercept;
        config.trainingRmse = sqrt(mean((prediction(trainMask, :) - ...
            normalizedTarget(trainMask, :)) .^ 2, 1));
        config.calibration = calibrationReadout;
        config.calibrationPassed = ...
            calibrationReadout.selectedActionCount > 0 && ...
            calibrationReadout.harmfulActionCount == 0 && ...
            calibrationReadout.harmfulCellCount == 0 && ...
            all(calibrationReadout.totalGain > 0);
        configs(end + 1) = config; %#ok<AGROW>
    end
end
selectedConfigIdx = selectConfig(configs);
selected = configs(selectedConfigIdx);
prediction = bsxfun(@plus, ...
    Z * selected.coefficient, selected.intercept);
rawBound = prediction;
rawBound(~inSupport, :) = -inf;
[calibrationRaw, ~] = evaluateSelection( ...
    rawBound, normalizedTarget, rows, cellIds, calibrationMask, ...
    selected.maximumEdits, targetScale);
[heldoutRaw, ~] = evaluateSelection( ...
    rawBound, normalizedTarget, rows, cellIds, heldoutMask, ...
    selected.maximumEdits, targetScale);
lowerBound = prediction - selected.correction;
lowerBound(~inSupport, :) = -inf;
[heldout, heldoutSelected] = evaluateSelection( ...
    lowerBound, normalizedTarget, rows, cellIds, heldoutMask, ...
    selected.maximumEdits, targetScale);
heldoutGatePassed = selected.calibrationPassed && ...
    heldout.selectedActionCount > 0 && ...
    heldout.harmfulActionCount == 0 && ...
    heldout.harmfulCellCount == 0 && ...
    all(heldout.totalGain > 0) && ...
    heldout.actionCaptureFraction >= 0.50 && ...
    heldout.selectedCellFraction >= 0.50;

model = struct();
model.kind = 'confidence-gated-one-hop-label-action-value-v167';
model.contractVersion = ...
    'confidence-gated-one-hop-label-action-value-model-v167-v1';
model.datasetContractVersion = dataset.contractVersion;
model.featureContractVersion = dataset.featureContractVersion;
model.featureNames = dataset.featureNames;
model.featureMean = featureMean;
model.featureScale = featureScale;
model.supportLower = supportLower;
model.supportUpper = supportUpper;
model.targetNames = {'eospa_gain', 'rmse_gain'};
model.targetScale = targetScale;
model.coefficient = selected.coefficient;
model.intercept = selected.intercept;
model.ridgeLambda = selected.ridgeLambda;
model.maximumEdits = selected.maximumEdits;
model.confidenceLevel = confidenceLevel;
model.selectedActionCorrection = selected.correction;
model.minimumLowerBound = [0, 0];
model.uniqueLabelProjectionRequired = true;
model.abstentionEnabled = true;
model.numericLabelIdentifiersUsedAsFeatures = false;
model.featuresUseTruth = false;
model.trainingTargetsUseCurrentTruth = true;
model.heldoutGatePassed = heldoutGatePassed;
model.recursiveEvaluationRun = false;
model.deployable = false;
model.evidenceBoundary = [ ...
    'V167 is a same-seed grouped learnability gate. Two linear ridge ', ...
    'heads use truth-free V166 features and current-truth single-action ', ...
    'targets. Lambda, maximum edits, support box and selected-action ', ...
    'overprediction correction are frozen using t=76/78 training cells ', ...
    'and t=79 F3 calibration cells before t=79 F5 heldout rows are ', ...
    'evaluated. Label keys are used only to enforce a unique-label ', ...
    'projection, never as numeric features. Even a heldout pass only ', ...
    'authorizes a nonlinear/graph or recursive development probe; it is ', ...
    'not independent validation or generalization evidence.'];

result = struct();
result.contractVersion = ...
    'confidence-gated-one-hop-label-action-value-v167-result-v1';
result.datasetPath = datasetPath;
result.configs = stripCoefficients(configs);
result.selectedConfigIndex = selectedConfigIdx;
result.selectedConfig = stripCoefficients(selected);
result.calibrationRaw = calibrationRaw;
result.heldoutRaw = heldoutRaw;
result.calibration = selected.calibration;
result.heldout = heldout;
result.heldoutSelectedRowIndices = heldoutSelected;
result.heldoutGatePassed = heldoutGatePassed;
result.truthUsedForTrainingTargets = true;
result.truthUsedForInferenceFeatures = false;
result.recursiveEvaluationRun = false;
result.deployable = false;
result.evidenceBoundary = model.evidenceBoundary;

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
modelPath = fullfile(outputRoot, ...
    'CONFIDENCE_GATED_ONE_HOP_ACTION_VALUE_V167_MODEL.mat');
resultPath = fullfile(outputRoot, ...
    'CONFIDENCE_GATED_ONE_HOP_ACTION_VALUE_V167_RESULT.mat');
reportPath = fullfile(outputRoot, ...
    'CONFIDENCE_GATED_ONE_HOP_ACTION_VALUE_V167.md');
model.modelPath = modelPath;
result.modelPath = modelPath;
result.resultPath = resultPath;
result.reportPath = reportPath;
save('-mat7-binary', modelPath, 'model');
save('-mat7-binary', resultPath, 'result');
writeReport(reportPath, model, result);
fprintf('V167 confidence-gated action model: %s\n', reportPath);
end

function validateDataset(dataset)
required = {'contractVersion', 'featureContractVersion', ...
    'featureNames', 'rows', 'cells', 'splitNames', ...
    'featuresUseTruth', 'featuresUseFutureInformation', ...
    'numericLabelIdentifiersUsedAsFeatures', ...
    'targetsUseCurrentTruth', 'targetsUseFutureOutcome'};
if ~all(isfield(dataset, required)) || ...
        ~strcmp(dataset.contractVersion, ...
            'one-hop-label-action-value-dataset-v166-v1') || ...
        dataset.featuresUseTruth || ...
        dataset.featuresUseFutureInformation || ...
        dataset.numericLabelIdentifiersUsedAsFeatures || ...
        ~dataset.targetsUseCurrentTruth || ...
        dataset.targetsUseFutureOutcome || ...
        ~isequal(dataset.splitNames, ...
            {'training', 'calibration', 'heldout'})
    error('OneHopActionValueV167:DatasetContract', ...
        'The V166 dataset contract is invalid.');
end
end

function scale = robustTargetScale(Y)
scale = zeros(1, size(Y, 2));
for idx = 1:size(Y, 2)
    scale(idx) = std(Y(:, idx));
    if ~isfinite(scale(idx)) || scale(idx) <= eps
        centered = Y(:, idx) - median(Y(:, idx));
        scale(idx) = 1.4826 * median(abs(centered));
    end
    if ~isfinite(scale(idx)) || scale(idx) <= eps
        scale(idx) = 1;
    end
end
end

function [coefficient, intercept] = fitRidge(Z, Y, lambda)
intercept = mean(Y, 1);
centered = bsxfun(@minus, Y, intercept);
coefficient = (Z' * Z + lambda * eye(size(Z, 2))) \ ...
    (Z' * centered);
end

function correction = selectedActionCorrection( ...
        prediction, target, rows, cellIds, mask, inSupport, ...
        maximumEdits, confidenceLevel)
rawBound = prediction;
rawBound(~inSupport, :) = -inf;
[~, selected] = evaluateSelection( ...
    rawBound, target, rows, cellIds, mask, maximumEdits, [1, 1]);
groups = unique(cellIds(mask));
groupError = nan(numel(groups), 1);
for groupIdx = 1:numel(groups)
    indices = selected(cellIds(selected) == groups(groupIdx));
    if isempty(indices)
        continue;
    end
    error = prediction(indices, :) - target(indices, :);
    groupError(groupIdx) = max(error(:));
end
finiteError = groupError(isfinite(groupError));
if isempty(finiteError)
    correction = inf;
else
    finiteError = sort(finiteError);
    quantileIdx = min(numel(finiteError), max(1, ...
        ceil((numel(finiteError) + 1) * confidenceLevel)));
    correction = max(finiteError(quantileIdx), 0);
end
end

function correction = escalateCorrection( ...
        prediction, target, selected, correction)
if isempty(selected)
    return;
end
harmful = selected(any(target(selected, :) <= 0, 2));
if isempty(harmful)
    return;
end
error = prediction(harmful, :) - target(harmful, :);
required = max(error(:));
correction = max([correction, required + 1e-12]);
end

function [readout, selected] = evaluateSelection( ...
        lowerBound, normalizedTarget, rows, cellIds, mask, ...
        maximumEdits, targetScale)
selected = zeros(1, 0);
groups = unique(cellIds(mask));
cellGain = zeros(numel(groups), 2);
selectedCell = false(numel(groups), 1);
for groupIdx = 1:numel(groups)
    indices = find(mask & cellIds == groups(groupIdx));
    eligible = indices(all(lowerBound(indices, :) > 0, 2));
    if isempty(eligible)
        continue;
    end
    score = min(lowerBound(eligible, :), [], 2);
    source = [rows(eligible).source]';
    labels = reshape([rows(eligible).label], 2, [])';
    ranking = [-score, labels, source];
    [~, order] = sortrows(ranking, 1:size(ranking, 2));
    usedLabels = zeros(2, 0);
    localSelected = zeros(1, 0);
    for cursor = reshape(order, 1, [])
        idx = eligible(cursor);
        label = rows(idx).label;
        if ~isempty(usedLabels) && ...
                any(all(bsxfun(@eq, usedLabels, label), 1))
            continue;
        end
        localSelected(end + 1) = idx; %#ok<AGROW>
        usedLabels(:, end + 1) = label; %#ok<AGROW>
        if numel(localSelected) >= maximumEdits
            break;
        end
    end
    selected = [selected, localSelected]; %#ok<AGROW>
    if ~isempty(localSelected)
        selectedCell(groupIdx) = true;
        cellGain(groupIdx, :) = sum( ...
            normalizedTarget(localSelected, :), 1);
    end
end
actual = normalizedTarget(selected, :);
harmfulAction = any(actual <= 0, 2);
harmfulCell = any(cellGain < -1e-12, 2) & selectedCell;
[oracleGain, oracleCount] = independentOracle( ...
    normalizedTarget, rows, cellIds, mask, maximumEdits);
totalNormalizedGain = sum(actual, 1);
capture = min(totalNormalizedGain ./ max(oracleGain, eps));
readout = struct();
readout.cellCount = numel(groups);
readout.selectedCellCount = nnz(selectedCell);
readout.selectedCellFraction = nnz(selectedCell) / max(numel(groups), 1);
readout.selectedActionCount = numel(selected);
readout.harmfulActionCount = nnz(harmfulAction);
readout.harmfulCellCount = nnz(harmfulCell);
readout.totalNormalizedGain = totalNormalizedGain;
readout.totalGain = totalNormalizedGain .* targetScale;
readout.minimumSelectedCellNormalizedGain = ...
    minimumSelectedCellGain(cellGain, selectedCell);
readout.independentOracleActionCount = oracleCount;
readout.independentOracleNormalizedGain = oracleGain;
readout.actionCaptureFraction = capture;
end

function [gain, count] = independentOracle( ...
        target, rows, cellIds, mask, maximumEdits)
groups = unique(cellIds(mask));
gain = zeros(1, 2);
count = 0;
for group = reshape(groups, 1, [])
    indices = find(mask & cellIds == group & all(target > 0, 2));
    if isempty(indices)
        continue;
    end
    score = min(target(indices, :), [], 2);
    labels = reshape([rows(indices).label], 2, [])';
    source = [rows(indices).source]';
    [~, order] = sortrows([-score, labels, source], [1, 2, 3, 4]);
    used = zeros(2, 0);
    for cursor = reshape(order, 1, [])
        idx = indices(cursor);
        label = rows(idx).label;
        if ~isempty(used) && any(all(bsxfun(@eq, used, label), 1))
            continue;
        end
        gain = gain + target(idx, :);
        count = count + 1;
        used(:, end + 1) = label; %#ok<AGROW>
        if size(used, 2) >= maximumEdits
            break;
        end
    end
end
end

function value = minimumSelectedCellGain(cellGain, selectedCell)
if any(selectedCell)
    value = min(cellGain(selectedCell, :), [], 1);
else
    value = [0, 0];
end
end

function idx = selectConfig(configs)
passed = find([configs.calibrationPassed]);
if isempty(passed)
    harmful = arrayfun(@(item) ...
        item.calibration.harmfulActionCount, configs);
    minimumHarmful = min(harmful);
    candidates = find(harmful == minimumHarmful);
else
    candidates = passed;
end
capture = arrayfun(@(item) ...
    item.calibration.actionCaptureFraction, configs(candidates));
selectedCells = arrayfun(@(item) ...
    item.calibration.selectedCellFraction, configs(candidates));
edits = [configs(candidates).maximumEdits];
ranking = [-capture', -selectedCells', edits'];
[~, order] = sortrows(ranking, [1, 2, 3]);
idx = candidates(order(1));
end

function stripped = stripCoefficients(configs)
stripped = configs;
for idx = 1:numel(stripped)
    stripped(idx).coefficient = [];
    stripped(idx).intercept = [];
end
end

function config = emptyConfig()
config = struct( ...
    'ridgeLambda', NaN, 'maximumEdits', 0, 'correction', NaN, ...
    'coefficient', [], 'intercept', [], 'trainingRmse', [NaN, NaN], ...
    'calibration', struct(), 'calibrationPassed', false);
end

function writeReport(path, model, result)
fid = fopen(path, 'w');
if fid < 0
    error('OneHopActionValueV167:ReportOpenFailed', ...
        'Could not open the V167 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V167 confidence-gated one-hop action-value gate\n\n');
fprintf(fid, '- Model: `%s`\n', model.kind);
fprintf(fid, '- Ridge lambda / maximum edits: `%.4g / %d`\n', ...
    model.ridgeLambda, model.maximumEdits);
fprintf(fid, '- Selected-action correction: `%.6f`\n', ...
    model.selectedActionCorrection);
fprintf(fid, '- Heldout gate passed: `%d`\n\n', ...
    result.heldoutGatePassed);
fprintf(fid, ['| Split | Selected cells/actions | Harmful cells/actions | ', ...
    'E-OSPA / RMSE gain | Independent capture |\n']);
fprintf(fid, '|:--|:--|:--|:--|--:|\n');
writeReadout(fid, 'Calibration F3 t=79', result.calibration);
writeReadout(fid, 'Heldout F5 t=79', result.heldout);
fprintf(fid, '\nUncalibrated diagnostic (same frozen ridge/K; not a policy):\n\n');
fprintf(fid, ['| Split | Selected cells/actions | Harmful cells/actions | ', ...
    'E-OSPA / RMSE gain | Independent capture |\n']);
fprintf(fid, '|:--|:--|:--|:--|--:|\n');
writeReadout(fid, 'Calibration raw', result.calibrationRaw);
writeReadout(fid, 'Heldout raw', result.heldoutRaw);
fprintf(fid, '\n## Candidate configurations\n\n');
fprintf(fid, '| Lambda | K | Correction | Cal actions | Harmful | Capture | Pass |\n');
fprintf(fid, '|--:|--:|--:|--:|--:|--:|:--:|\n');
for config = result.configs
    fprintf(fid, '| %.4g | %d | %.5f | %d | %d | %.3f | %d |\n', ...
        config.ridgeLambda, config.maximumEdits, config.correction, ...
        config.calibration.selectedActionCount, ...
        config.calibration.harmfulActionCount, ...
        config.calibration.actionCaptureFraction, ...
        config.calibrationPassed);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    result.evidenceBoundary);
end

function writeReadout(fid, name, readout)
fprintf(fid, '| %s | %d/%d | %d/%d | %+.4f / %+.4f | %.3f |\n', ...
    name, readout.selectedCellCount, readout.selectedActionCount, ...
    readout.harmfulCellCount, readout.harmfulActionCount, ...
    readout.totalGain(1), readout.totalGain(2), ...
    readout.actionCaptureFraction);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
