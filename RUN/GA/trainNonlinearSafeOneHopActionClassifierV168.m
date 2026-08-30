function [modelPath, result] = ...
        trainNonlinearSafeOneHopActionClassifierV168(options)
% TRAINNONLINEARSAFEONEHOPACTIONCLASSIFIERV168 Nonlinear safety gate.
%
% The V166 immediate action targets are discontinuous because a label update
% may change the downstream MAP cardinality. V168 therefore learns the
% decision-relevant signs instead of regressing the two raw gains. A shared
% one-hidden-layer encoder predicts P(E-OSPA gain > 0), P(RMSE gain > 0),
% and P(both > 0). An initialization ensemble supplies an epistemic spread;
% calibration freezes the lower-confidence threshold before the F5 t=79
% heldout group is opened. Only one unique-label action is selected per
% receiver, so no additivity assumption is made from single-action targets.

if nargin < 1 || isempty(options)
    options = struct();
end
datasetPath = getField(options, 'datasetPath', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v166', 'one_hop_action_value', ...
    'ONE_HOP_LABEL_ACTION_VALUE_DATASET_V166.mat'));
outputRoot = getField(options, 'outputRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v168', 'nonlinear_safe_action'));
hiddenWidthGrid = reshape(getField(options, ...
    'hiddenWidthGrid', [16, 32]), 1, []);
weightDecayGrid = reshape(getField(options, ...
    'weightDecayGrid', [1e-4, 1e-3]), 1, []);
uncertaintyPenaltyGrid = reshape(getField(options, ...
    'uncertaintyPenaltyGrid', [0, 0.5, 1]), 1, []);
probabilityThresholdGrid = reshape(getField(options, ...
    'probabilityThresholdGrid', [0.5, 0.6, 0.7, 0.8]), 1, []);
ensembleSeeds = reshape(getField(options, ...
    'ensembleSeeds', [1701, 1709, 1721]), 1, []);
epochCount = getField(options, 'epochCount', 400);
learningRate = getField(options, 'learningRate', 5e-3);
supportMargin = getField(options, 'supportMarginStd', 0.50);
minimumCalibrationCells = getField(options, ...
    'minimumCalibrationCells', 3);
minimumHeldoutCells = getField(options, 'minimumHeldoutCells', 3);
if exist(datasetPath, 'file') ~= 2 || ...
        isempty(hiddenWidthGrid) || any(hiddenWidthGrid < 1) || ...
        any(hiddenWidthGrid ~= round(hiddenWidthGrid)) || ...
        isempty(weightDecayGrid) || any(weightDecayGrid < 0) || ...
        isempty(uncertaintyPenaltyGrid) || ...
        any(uncertaintyPenaltyGrid < 0) || ...
        isempty(probabilityThresholdGrid) || ...
        any(probabilityThresholdGrid < 0.5 | ...
            probabilityThresholdGrid >= 1) || ...
        isempty(ensembleSeeds) || epochCount < 1 || ...
        learningRate <= 0 || supportMargin < 0 || ...
        minimumCalibrationCells < 1 || minimumHeldoutCells < 1
    error('NonlinearSafeActionV168:InvalidOptions', ...
        'The V168 fit options are invalid.');
end

loaded = load(datasetPath, 'dataset');
dataset = loaded.dataset;
validateDataset(dataset);
rows = dataset.rows;
X = vertcat(rows.features);
gain = [[rows.eospaGain]', [rows.rmseGain]'];
cellIds = [rows.cellId]';
splits = [rows.split]';
trainMask = splits == 1;
calibrationMask = splits == 2;
heldoutMask = splits == 3;
targets = [gain(:, 1) > 0, gain(:, 2) > 0, ...
    all(gain > 0, 2)];
featureMean = mean(X(trainMask, :), 1);
featureScale = std(X(trainMask, :), 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
Z = bsxfun(@rdivide, bsxfun(@minus, X, featureMean), featureScale);
supportLower = min(Z(trainMask, :), [], 1) - supportMargin;
supportUpper = max(Z(trainMask, :), [], 1) + supportMargin;
inSupport = all(bsxfun(@ge, Z, supportLower) & ...
    bsxfun(@le, Z, supportUpper), 2);
gainScale = std(gain(trainMask, :), 0, 1);
gainScale(~isfinite(gainScale) | gainScale <= eps) = 1;

families = repmat(emptyFamily(), 1, 0);
configs = repmat(emptyConfig(), 1, 0);
familyIndex = 0;
for hiddenWidth = hiddenWidthGrid
    for weightDecay = weightDecayGrid
        familyIndex = familyIndex + 1;
        members = cell(1, numel(ensembleSeeds));
        probabilityCube = zeros(size(Z, 1), 3, numel(ensembleSeeds));
        for seedIdx = 1:numel(ensembleSeeds)
            members{seedIdx} = fitMlpClassifier( ...
                Z(trainMask, :), targets(trainMask, :), ...
                cellIds(trainMask), hiddenWidth, weightDecay, ...
                epochCount, learningRate, ensembleSeeds(seedIdx));
            probabilityCube(:, :, seedIdx) = ...
                predictMlpClassifier(members{seedIdx}, Z);
        end
        meanProbability = mean(probabilityCube, 3);
        probabilityStd = std(probabilityCube, 0, 3);
        family = emptyFamily();
        family.hiddenWidth = hiddenWidth;
        family.weightDecay = weightDecay;
        family.members = members;
        family.trainingLoss = cellfun(@(member) ...
            member.lossHistory(end), members);
        family.meanProbability = meanProbability;
        family.probabilityStd = probabilityStd;
        families(familyIndex) = family;
        for uncertaintyPenalty = uncertaintyPenaltyGrid
            lowerProbability = meanProbability - ...
                uncertaintyPenalty * probabilityStd;
            lowerProbability(~inSupport, :) = -inf;
            for probabilityThreshold = probabilityThresholdGrid
                [calibrationReadout, calibrationSelected] = ...
                    evaluateSelection(lowerProbability, ...
                        probabilityThreshold, gain, gainScale, ...
                        rows, cellIds, calibrationMask);
                config = emptyConfig();
                config.familyIndex = familyIndex;
                config.hiddenWidth = hiddenWidth;
                config.weightDecay = weightDecay;
                config.uncertaintyPenalty = uncertaintyPenalty;
                config.probabilityThreshold = probabilityThreshold;
                config.calibration = calibrationReadout;
                config.calibrationSelectedRowIndices = ...
                    calibrationSelected;
                config.calibrationPassed = ...
                    calibrationReadout.selectedCellCount >= ...
                        minimumCalibrationCells && ...
                    calibrationReadout.harmfulActionCount == 0 && ...
                    calibrationReadout.harmfulCellCount == 0 && ...
                    all(calibrationReadout.totalGain > 0);
                configs(end + 1) = config; %#ok<AGROW>
            end
        end
    end
end

selectedConfigIdx = selectConfig(configs);
selected = configs(selectedConfigIdx);
selectedFamily = families(selected.familyIndex);
selectedLowerProbability = selectedFamily.meanProbability - ...
    selected.uncertaintyPenalty * selectedFamily.probabilityStd;
selectedLowerProbability(~inSupport, :) = -inf;
[heldout, heldoutSelected] = evaluateSelection( ...
    selectedLowerProbability, selected.probabilityThreshold, ...
    gain, gainScale, rows, cellIds, heldoutMask);
[calibrationRaw, ~] = evaluateSelection( ...
    selectedFamily.meanProbability, 0.5, gain, gainScale, ...
    rows, cellIds, calibrationMask);
[heldoutRaw, ~] = evaluateSelection( ...
    selectedFamily.meanProbability, 0.5, gain, gainScale, ...
    rows, cellIds, heldoutMask);
heldoutGatePassed = selected.calibrationPassed && ...
    heldout.selectedCellCount >= minimumHeldoutCells && ...
    heldout.harmfulActionCount == 0 && ...
    heldout.harmfulCellCount == 0 && ...
    all(heldout.totalGain > 0) && ...
    heldout.jointUtilityCaptureFraction >= 0.25;

model = struct();
model.kind = 'nonlinear-safe-one-hop-label-action-v168';
model.contractVersion = ...
    'nonlinear-safe-one-hop-label-action-model-v168-v1';
model.datasetContractVersion = dataset.contractVersion;
model.featureContractVersion = dataset.featureContractVersion;
model.featureNames = dataset.featureNames;
model.featureMean = featureMean;
model.featureScale = featureScale;
model.supportLower = supportLower;
model.supportUpper = supportUpper;
model.members = selectedFamily.members;
model.hiddenWidth = selected.hiddenWidth;
model.weightDecay = selected.weightDecay;
model.ensembleSeeds = ensembleSeeds;
model.uncertaintyPenalty = selected.uncertaintyPenalty;
model.probabilityThreshold = selected.probabilityThreshold;
model.maximumEditsPerReceiver = 1;
model.outputNames = { ...
    'positive_eospa_gain_probability', ...
    'positive_rmse_gain_probability', ...
    'joint_positive_gain_probability'};
model.featuresUseTruth = false;
model.trainingTargetsUseCurrentTruth = true;
model.numericLabelIdentifiersUsedAsFeatures = false;
model.abstentionEnabled = true;
model.heldoutGatePassed = heldoutGatePassed;
model.recursiveEvaluationRun = false;
model.deployable = false;
model.evidenceBoundary = [ ...
    'V168 is a same-seed grouped nonlinear learnability gate. A shared ', ...
    'MLP classifier consumes the truth-free V166 action features and ', ...
    'learns only the signs of current-truth immediate E-OSPA and RMSE ', ...
    'gains. Width, weight decay, ensemble uncertainty penalty and ', ...
    'probability threshold are frozen with t=76/78 training cells and ', ...
    't=79 F3 calibration cells before t=79 F5 is opened. One action per ', ...
    'receiver avoids assuming that independently evaluated actions add. ', ...
    'A heldout pass authorizes only a recursive development experiment; ', ...
    'it is not independent-seed or cross-scenario evidence.'];

result = struct();
result.contractVersion = ...
    'nonlinear-safe-one-hop-label-action-result-v168-v1';
result.datasetPath = datasetPath;
result.configs = stripSelectedIndices(configs);
result.selectedConfigIndex = selectedConfigIdx;
result.selectedConfig = stripSelectedIndices(selected);
result.selectedFamilyTrainingLoss = selectedFamily.trainingLoss;
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
    'NONLINEAR_SAFE_ONE_HOP_ACTION_V168_MODEL.mat');
resultPath = fullfile(outputRoot, ...
    'NONLINEAR_SAFE_ONE_HOP_ACTION_V168_RESULT.mat');
reportPath = fullfile(outputRoot, ...
    'NONLINEAR_SAFE_ONE_HOP_ACTION_V168.md');
model.modelPath = modelPath;
result.modelPath = modelPath;
result.resultPath = resultPath;
result.reportPath = reportPath;
save('-mat7-binary', modelPath, 'model');
save('-mat7-binary', resultPath, 'result');
writeReport(reportPath, model, result);
fprintf('V168 nonlinear safe-action gate: %s\n', reportPath);
end

function validateDataset(dataset)
required = {'contractVersion', 'featureContractVersion', ...
    'featureNames', 'rows', 'splitNames', 'featuresUseTruth', ...
    'featuresUseFutureInformation', ...
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
    error('NonlinearSafeActionV168:DatasetContract', ...
        'The V166 dataset contract is invalid.');
end
end

function model = fitMlpClassifier( ...
        Z, targets, cellIds, hiddenWidth, weightDecay, ...
        epochCount, learningRate, initializationSeed)
if isempty(Z) || size(targets, 1) ~= size(Z, 1) || ...
        size(targets, 2) ~= 3 || numel(cellIds) ~= size(Z, 1)
    error('NonlinearSafeActionV168:TrainingShape', ...
        'The classifier training arrays are invalid.');
end
rng(initializationSeed);
inputWidth = size(Z, 2);
parameters = { ...
    randn(inputWidth, hiddenWidth) * sqrt(2 / ...
        max(inputWidth + hiddenWidth, 1)), ...
    zeros(1, hiddenWidth), ...
    randn(hiddenWidth, 3) * sqrt(2 / ...
        max(hiddenWidth + 3, 1)), ...
    zeros(1, 3)};
firstMoment = cellfun(@(value) zeros(size(value)), ...
    parameters, 'UniformOutput', false);
secondMoment = cellfun(@(value) zeros(size(value)), ...
    parameters, 'UniformOutput', false);
sampleWeight = buildBalancedCellWeights(targets, cellIds);
lossHistory = nan(1, epochCount);
for epoch = 1:epochCount
    hidden = tanh(bsxfun(@plus, ...
        Z * parameters{1}, parameters{2}));
    logits = bsxfun(@plus, hidden * parameters{3}, parameters{4});
    probability = stableSigmoid(logits);
    clipped = min(max(probability, 1e-9), 1 - 1e-9);
    dataLoss = sum(sum(sampleWeight .* ( ...
        -targets .* log(clipped) - ...
        (1 - targets) .* log(1 - clipped))));
    regularization = 0.5 * weightDecay * ( ...
        sum(parameters{1}(:).^2) + ...
        sum(parameters{3}(:).^2));
    lossHistory(epoch) = dataLoss + regularization;

    dLogits = sampleWeight .* (probability - targets);
    gradients = cell(1, 4);
    gradients{3} = hidden' * dLogits + ...
        weightDecay * parameters{3};
    gradients{4} = sum(dLogits, 1);
    dHidden = dLogits * parameters{3}';
    dActivation = dHidden .* (1 - hidden.^2);
    gradients{1} = Z' * dActivation + ...
        weightDecay * parameters{1};
    gradients{2} = sum(dActivation, 1);
    gradients = clipGradients(gradients, 10);
    [parameters, firstMoment, secondMoment] = adamStep( ...
        parameters, gradients, firstMoment, secondMoment, ...
        epoch, learningRate);
end
if any(~isfinite(lossHistory)) || ...
        any(cellfun(@(value) any(~isfinite(value(:))), parameters))
    error('NonlinearSafeActionV168:NonFiniteFit', ...
        'The nonlinear classifier fit became non-finite.');
end
model = struct();
model.parameters = parameters;
model.hiddenWidth = hiddenWidth;
model.weightDecay = weightDecay;
model.initializationSeed = initializationSeed;
model.lossHistory = lossHistory;
end

function weights = buildBalancedCellWeights(targets, cellIds)
base = zeros(size(cellIds));
groups = unique(cellIds);
for group = reshape(groups, 1, [])
    mask = cellIds == group;
    base(mask) = 1 / max(nnz(mask), 1);
end
base = base / max(sum(base), eps);
weights = zeros(size(targets));
for head = 1:size(targets, 2)
    positive = targets(:, head) == 1;
    negative = ~positive;
    positiveMass = sum(base(positive));
    negativeMass = sum(base(negative));
    if positiveMass <= 0 || negativeMass <= 0
        error('NonlinearSafeActionV168:DegenerateTarget', ...
            'A safety head has only one training class.');
    end
    weights(positive, head) = ...
        base(positive) / (2 * positiveMass * size(targets, 2));
    weights(negative, head) = ...
        base(negative) / (2 * negativeMass * size(targets, 2));
end
end

function probability = predictMlpClassifier(model, Z)
hidden = tanh(bsxfun(@plus, ...
    Z * model.parameters{1}, model.parameters{2}));
logits = bsxfun(@plus, ...
    hidden * model.parameters{3}, model.parameters{4});
probability = stableSigmoid(logits);
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

function gradients = clipGradients(gradients, maximumNorm)
totalSquared = 0;
for idx = 1:numel(gradients)
    totalSquared = totalSquared + sum(gradients{idx}(:).^2);
end
normValue = sqrt(totalSquared);
if normValue > maximumNorm
    scale = maximumNorm / normValue;
    for idx = 1:numel(gradients)
        gradients{idx} = gradients{idx} * scale;
    end
end
end

function [parameters, firstMoment, secondMoment] = adamStep( ...
        parameters, gradients, firstMoment, secondMoment, ...
        epoch, learningRate)
beta1 = 0.9;
beta2 = 0.999;
adamEpsilon = 1e-8;
for idx = 1:numel(parameters)
    firstMoment{idx} = beta1 * firstMoment{idx} + ...
        (1 - beta1) * gradients{idx};
    secondMoment{idx} = beta2 * secondMoment{idx} + ...
        (1 - beta2) * gradients{idx}.^2;
    correctedFirst = firstMoment{idx} / (1 - beta1^epoch);
    correctedSecond = secondMoment{idx} / (1 - beta2^epoch);
    parameters{idx} = parameters{idx} - learningRate * ...
        correctedFirst ./ (sqrt(correctedSecond) + adamEpsilon);
end
end

function [readout, selected] = evaluateSelection( ...
        probability, threshold, gain, gainScale, ...
        rows, cellIds, mask)
selected = zeros(1, 0);
groups = unique(cellIds(mask));
cellGain = zeros(numel(groups), 2);
selectedCell = false(numel(groups), 1);
for groupIdx = 1:numel(groups)
    indices = find(mask & cellIds == groups(groupIdx));
    eligible = indices(all(probability(indices, :) > threshold, 2));
    if isempty(eligible)
        continue;
    end
    score = min(probability(eligible, :), [], 2);
    labels = reshape([rows(eligible).label], 2, [])';
    source = [rows(eligible).source]';
    [~, order] = sortrows([-score, labels, source], [1, 2, 3, 4]);
    selectedIdx = eligible(order(1));
    selected(end + 1) = selectedIdx; %#ok<AGROW>
    selectedCell(groupIdx) = true;
    cellGain(groupIdx, :) = gain(selectedIdx, :);
end
actual = gain(selected, :);
harmfulAction = any(actual <= 0, 2);
harmfulCell = any(cellGain < -1e-12, 2) & selectedCell;
[oracleGain, oracleUtility, oracleCount] = independentOracle( ...
    gain, gainScale, rows, cellIds, mask);
totalGain = sum(actual, 1);
selectedUtility = sum(min(bsxfun(@rdivide, ...
    max(actual, 0), gainScale), [], 2));
readout = struct();
readout.cellCount = numel(groups);
readout.selectedCellCount = nnz(selectedCell);
readout.selectedCellFraction = nnz(selectedCell) / max(numel(groups), 1);
readout.selectedActionCount = numel(selected);
readout.harmfulActionCount = nnz(harmfulAction);
readout.harmfulCellCount = nnz(harmfulCell);
readout.totalGain = totalGain;
readout.minimumSelectedCellGain = ...
    minimumSelectedCellGain(cellGain, selectedCell);
readout.independentOracleActionCount = oracleCount;
readout.independentOracleGain = oracleGain;
readout.jointUtility = selectedUtility;
readout.independentOracleJointUtility = oracleUtility;
readout.jointUtilityCaptureFraction = ...
    selectedUtility / max(oracleUtility, eps);
end

function [gainTotal, utilityTotal, count] = independentOracle( ...
        gain, gainScale, rows, cellIds, mask)
groups = unique(cellIds(mask));
gainTotal = zeros(1, 2);
utilityTotal = 0;
count = 0;
for group = reshape(groups, 1, [])
    indices = find(mask & cellIds == group & all(gain > 0, 2));
    if isempty(indices)
        continue;
    end
    utility = min(bsxfun(@rdivide, gain(indices, :), gainScale), [], 2);
    labels = reshape([rows(indices).label], 2, [])';
    source = [rows(indices).source]';
    [~, order] = sortrows([-utility, labels, source], [1, 2, 3, 4]);
    selectedIdx = indices(order(1));
    gainTotal = gainTotal + gain(selectedIdx, :);
    utilityTotal = utilityTotal + utility(order(1));
    count = count + 1;
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
selectedCells = arrayfun(@(item) ...
    item.calibration.selectedCellCount, configs(candidates));
capture = arrayfun(@(item) ...
    item.calibration.jointUtilityCaptureFraction, configs(candidates));
minimumGain = arrayfun(@(item) min( ...
    item.calibration.minimumSelectedCellGain), configs(candidates));
width = [configs(candidates).hiddenWidth];
threshold = [configs(candidates).probabilityThreshold];
ranking = [-selectedCells', -capture', -minimumGain', ...
    width', threshold'];
[~, order] = sortrows(ranking, 1:size(ranking, 2));
idx = candidates(order(1));
end

function stripped = stripSelectedIndices(configs)
stripped = configs;
for idx = 1:numel(stripped)
    stripped(idx).calibrationSelectedRowIndices = [];
end
end

function family = emptyFamily()
family = struct( ...
    'hiddenWidth', 0, 'weightDecay', NaN, 'members', {{}}, ...
    'trainingLoss', [], 'meanProbability', [], ...
    'probabilityStd', []);
end

function config = emptyConfig()
config = struct( ...
    'familyIndex', 0, 'hiddenWidth', 0, 'weightDecay', NaN, ...
    'uncertaintyPenalty', NaN, 'probabilityThreshold', NaN, ...
    'calibration', struct(), ...
    'calibrationSelectedRowIndices', [], ...
    'calibrationPassed', false);
end

function writeReport(path, model, result)
fid = fopen(path, 'w');
if fid < 0
    error('NonlinearSafeActionV168:ReportOpenFailed', ...
        'Could not open the V168 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V168 nonlinear safe one-hop action gate\n\n');
fprintf(fid, '- Model: `%s`\n', model.kind);
fprintf(fid, '- Hidden width / weight decay: `%d / %.4g`\n', ...
    model.hiddenWidth, model.weightDecay);
fprintf(fid, '- Ensemble seeds: `%s`\n', mat2str(model.ensembleSeeds));
fprintf(fid, '- Uncertainty penalty / probability threshold: `%.3f / %.3f`\n', ...
    model.uncertaintyPenalty, model.probabilityThreshold);
fprintf(fid, '- Maximum edits per receiver: `%d`\n', ...
    model.maximumEditsPerReceiver);
fprintf(fid, '- Heldout gate passed: `%d`\n\n', ...
    result.heldoutGatePassed);
fprintf(fid, ['| Split | Selected cells/actions | Harmful cells/actions | ', ...
    'E-OSPA / RMSE gain | Joint-utility capture |\n']);
fprintf(fid, '|:--|:--|:--|:--|--:|\n');
writeReadout(fid, 'Calibration F3 t=79', result.calibration);
writeReadout(fid, 'Heldout F5 t=79', result.heldout);
fprintf(fid, '\nUncalibrated diagnostic (ensemble mean > 0.5; not a policy):\n\n');
fprintf(fid, ['| Split | Selected cells/actions | Harmful cells/actions | ', ...
    'E-OSPA / RMSE gain | Joint-utility capture |\n']);
fprintf(fid, '|:--|:--|:--|:--|--:|\n');
writeReadout(fid, 'Calibration raw', result.calibrationRaw);
writeReadout(fid, 'Heldout raw', result.heldoutRaw);
fprintf(fid, '\n## Candidate configurations\n\n');
fprintf(fid, ['| Width | Decay | Uncertainty | Threshold | ', ...
    'Cal cells | Harmful | Capture | Pass |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|:--:|\n');
for config = result.configs
    fprintf(fid, '| %d | %.4g | %.2f | %.2f | %d | %d | %.3f | %d |\n', ...
        config.hiddenWidth, config.weightDecay, ...
        config.uncertaintyPenalty, config.probabilityThreshold, ...
        config.calibration.selectedCellCount, ...
        config.calibration.harmfulActionCount, ...
        config.calibration.jointUtilityCaptureFraction, ...
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
    readout.jointUtilityCaptureFraction);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
