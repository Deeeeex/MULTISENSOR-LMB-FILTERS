function [reportPath, summary] = analyzeDecisionAwareGatewayV257(options)
% ANALYZEDECISIONAWAREGATEWAYV257 Cross loss and information content.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getDecisionAwareGatewayV257Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
datasetPath = absolutePath(getField(options, 'datasetPath', ...
    fullfile(repoRoot, protocol.outputRoot, 'feature_dataset', ...
        'DECISION_AWARE_GATEWAY_V257_TRAINING_DATASET.mat')), repoRoot);
if exist(datasetPath, 'file') ~= 2
    error('DecisionAwareGatewayV257:MissingDataset', ...
        'The V257 feature dataset is unavailable.');
end
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('DecisionAwareGatewayV257:DirtyAnalysisSource', ...
        'Official V257 analysis requires clean source.');
end
envelope = load(datasetPath, 'dataset');
if ~isfield(envelope, 'dataset')
    error('DecisionAwareGatewayV257:MissingDatasetEnvelope', ...
        'The V257 MAT lacks its dataset envelope.');
end
dataset = envelope.dataset;
validateDataset(dataset, protocol);

configurationCount = numel(protocol.featureModeNames) * ...
    numel(protocol.modelFamilyNames) * numel(protocol.ridgeLambdaGrid);
configurations = repmat(emptyConfiguration(), 1, configurationCount);
configurationIdx = 0;
for featureIdx = 1:numel(protocol.featureModeNames)
    featureMode = protocol.featureModeNames{featureIdx};
    flattened = flattenDataset(dataset, featureMode);
    for familyIdx = 1:numel(protocol.modelFamilyNames)
        family = protocol.modelFamilyNames{familyIdx};
        for lambda = protocol.ridgeLambdaGrid
            configurationIdx = configurationIdx + 1;
            evaluation = evaluateConfiguration( ...
                flattened, family, lambda, protocol);
            configuration = emptyConfiguration();
            configuration.featureMode = featureMode;
            configuration.featureCount = size(flattened.features, 2);
            configuration.modelFamily = family;
            configuration.ridgeLambda = lambda;
            configuration.evaluation = evaluation;
            configuration.diagnosticGatePassed = ...
                passesDiagnostic(evaluation, protocol);
            configurations(configurationIdx) = configuration;
        end
    end
end

compactMask = strcmp({configurations.featureMode}, ...
    protocol.featureModeNames{1});
richMask = strcmp({configurations.featureMode}, ...
    protocol.featureModeNames{2});
compactSignalPresent = any( ...
    [configurations(compactMask).diagnosticGatePassed]);
richSignalPresent = any( ...
    [configurations(richMask).diagnosticGatePassed]);
if compactSignalPresent
    nextDecision = ...
        'design-compact-decision-aware-successor-with-new-calibration-plan';
elseif richSignalPresent
    nextDecision = ...
        'design-and-cost-fixed-byte-posterior-compatibility-sketch';
else
    nextDecision = ...
        'stop-local-gateway-learning-and-change-action-scale-or-horizon';
end

summary = struct();
summary.contractVersion = ...
    'decision-aware-gateway-v257-summary-v1';
summary.protocol = protocol;
summary.analysisGitCommit = gitState.commit;
summary.datasetPath = datasetPath;
summary.datasetAssemblyGitCommit = dataset.assemblyGitCommit;
summary.trainingSeeds = protocol.trainingSeeds;
summary.recordCount = dataset.recordCount;
summary.actionRowCount = dataset.actionRowCount;
summary.safeActionCount = sum(cellfun( ...
    @(record) nnz(realizedSafe(record.targets, protocol)), ...
    dataset.records));
summary.configurations = configurations;
summary.compactSignalPresent = compactSignalPresent;
summary.richSignalPresent = richSignalPresent;
summary.nextDecision = nextDecision;
summary.calibrationAuthorized = false;
summary.holdoutAuthorized = false;
summary.gnnAuthorized = false;
summary.x36Authorized = false;
summary.validationClaimAllowed = false;
summary.developmentEvidenceOnly = true;
summary.completedAt = datestr(now, 31);
summary.evidenceBoundary = protocol.evidenceBoundary;

outputRoot = absolutePath(getField(options, 'outputRoot', ...
    fullfile(repoRoot, protocol.outputRoot, 'analysis')), repoRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'DECISION_AWARE_GATEWAY_V257.md');
matPath = fullfile(outputRoot, ...
    'DECISION_AWARE_GATEWAY_V257.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeReport(reportPath, summary);
fprintf(['V257 diagnostic: compact=%d rich=%d; next=%s\n'], ...
    compactSignalPresent, richSignalPresent, nextDecision);
fprintf('V257 report: %s\n', reportPath);
end

function validateDataset(dataset, protocol)
required = {'contractVersion', 'protocol', 'assemblyGitCommit', ...
    'seeds', 'compactFeatureNames', 'richFeatureNames', ...
    'outcomeNames', 'records', 'recordCount', 'actionRowCount', ...
    'actionRowsPerWindow', 'richPairwiseControlCosted', ...
    'truthUsedAsFeature', 'futureOutcomeUsedAsFeature'};
valid = isstruct(dataset) && isscalar(dataset) && ...
    all(isfield(dataset, required)) && ...
    strcmp(dataset.contractVersion, ...
        'decision-aware-gateway-v257-training-dataset-v1') && ...
    strcmp(dataset.protocol.id, protocol.id) && ...
    isequal(dataset.seeds, protocol.trainingSeeds) && ...
    numel(dataset.compactFeatureNames) == ...
        protocol.expectedFeatureCounts(1) && ...
    numel(dataset.richFeatureNames) == protocol.expectedFeatureCounts(2) && ...
    isequal(dataset.outcomeNames, protocol.outcomeNames) && ...
    dataset.recordCount == numel(dataset.records) && ...
    dataset.actionRowCount == sum(dataset.actionRowsPerWindow) && ...
    ~dataset.richPairwiseControlCosted && ...
    ~dataset.truthUsedAsFeature && ~dataset.futureOutcomeUsedAsFeature && ...
    all(cellfun(@validRecord, dataset.records));
if ~valid
    error('DecisionAwareGatewayV257:InvalidDataset', ...
        'The V257 training dataset violates its frozen contract.');
end
end

function valid = validRecord(record)
required = {'seed', 'anchorTime', 'compactFeatures', 'richFeatures', ...
    'targets', 'communicationProjection', 'truthUsedAsFeature', ...
    'futureOutcomeUsedAsFeature'};
valid = isstruct(record) && isscalar(record) && ...
    all(isfield(record, required)) && ...
    size(record.compactFeatures, 1) == size(record.targets, 1) && ...
    size(record.richFeatures, 1) == size(record.targets, 1) && ...
    numel(record.communicationProjection.candidateFeasible) == ...
        size(record.targets, 1) && ...
    all(isfinite(record.compactFeatures(:))) && ...
    all(isfinite(record.richFeatures(:))) && ...
    all(isfinite(record.targets(:))) && ...
    ~record.truthUsedAsFeature && ~record.futureOutcomeUsedAsFeature;
end

function data = flattenDataset(dataset, featureMode)
if strcmp(featureMode, 'compact-v256')
    featureField = 'compactFeatures';
    featureCount = numel(dataset.compactFeatureNames);
elseif strcmp(featureMode, 'rich-pairwise-information-upper-bound')
    featureField = 'richFeatures';
    featureCount = numel(dataset.richFeatureNames);
else
    error('DecisionAwareGatewayV257:UnknownFeatureMode', ...
        'The requested V257 feature mode is not registered.');
end
rowCount = dataset.actionRowCount;
data = struct();
data.features = zeros(rowCount, featureCount);
data.targets = zeros(rowCount, numel(dataset.outcomeNames));
data.seeds = zeros(rowCount, 1);
data.recordIndices = zeros(rowCount, 1);
data.communicationFeasible = false(rowCount, 1);
data.weights = zeros(rowCount, 1);
data.recordCount = dataset.recordCount;
data.seedList = dataset.seeds;
cursor = 0;
for recordIdx = 1:numel(dataset.records)
    record = dataset.records{recordIdx};
    current = record.(featureField);
    count = size(current, 1);
    rows = cursor + (1:count);
    data.features(rows, :) = current;
    data.targets(rows, :) = record.targets;
    data.seeds(rows) = record.seed;
    data.recordIndices(rows) = recordIdx;
    data.communicationFeasible(rows) = logical(reshape( ...
        record.communicationProjection.candidateFeasible, [], 1));
    data.weights(rows) = 1 / count;
    cursor = cursor + count;
end
if cursor ~= rowCount || any(~isfinite(data.features(:))) || ...
        any(~isfinite(data.targets(:))) || ...
        ~all(data.communicationFeasible)
    error('DecisionAwareGatewayV257:InvalidFlattenedData', ...
        'V257 rows are incomplete or outside deterministic support.');
end
end

function evaluation = evaluateConfiguration(data, family, lambda, protocol)
folds = repmat(emptyFold(), 1, numel(protocol.trainingSeeds));
for seedIdx = 1:numel(protocol.trainingSeeds)
    heldSeed = protocol.trainingSeeds(seedIdx);
    train = data.seeds ~= heldSeed;
    test = data.seeds == heldSeed;
    if strcmp(family, 'multi-output-ridge')
        model = fitRidge(data.features(train, :), ...
            data.targets(train, :), lambda, data.weights(train));
    elseif strcmp(family, 'joint-safe-ridge')
        labels = double(realizedSafe(data.targets, protocol));
        model = fitRidge(data.features(train, :), ...
            labels(train), lambda, data.weights(train));
    else
        error('DecisionAwareGatewayV257:UnknownModelFamily', ...
            'The requested V257 model family is not registered.');
    end
    predictions = predictRidge(model, data.features(test, :));
    folds(seedIdx) = evaluateHeldSeed( ...
        data, test, predictions, heldSeed, family, protocol);
end

topSafeCount = sum([folds.topActionSafeCount]);
windowCount = sum([folds.windowCount]);
selectedCount = sum([folds.gatedSelectionCount]);
safeCount = sum([folds.gatedSafeCount]);
chanceRate = mean([folds.uniformSafePrevalence]);
topSafeRate = topSafeCount / windowCount;
if selectedCount > 0
    precision = safeCount / selectedCount;
else
    precision = NaN;
end
selectedOutcomes = vertcat(folds.selectedOrReferenceMeanOutcomes);
evaluation = struct();
evaluation.folds = folds;
evaluation.windowCount = windowCount;
evaluation.topActionSafeCount = topSafeCount;
evaluation.topActionSafeRate = topSafeRate;
evaluation.uniformSafePrevalence = chanceRate;
evaluation.topActionSafeLift = topSafeRate - chanceRate;
evaluation.positiveLiftSeedCount = nnz([folds.safeLift] > 0);
evaluation.gatedSelectionCount = selectedCount;
evaluation.gatedSafeCount = safeCount;
evaluation.gatedUnsafeCount = selectedCount - safeCount;
evaluation.gatedSafePrecision = precision;
evaluation.selectedOrReferenceMeanOutcomes = mean(selectedOutcomes, 1);
evaluation.minimumSeedMeanOutcomes = min(selectedOutcomes, [], 1);
end

function fold = evaluateHeldSeed( ...
        data, test, predictions, heldSeed, family, protocol)
testRows = find(test);
recordIds = unique(data.recordIndices(test));
windowCount = numel(recordIds);
topSafe = false(1, windowCount);
chance = zeros(1, windowCount);
selected = false(1, windowCount);
selectedSafe = false(1, windowCount);
selectedOutcomes = zeros(windowCount, size(data.targets, 2));
for windowIdx = 1:windowCount
    recordId = recordIds(windowIdx);
    local = data.recordIndices(testRows) == recordId;
    rows = testRows(local);
    targets = data.targets(rows, :);
    safe = realizedSafe(targets, protocol);
    chance(windowIdx) = mean(safe);
    current = predictions(local, :);
    if strcmp(family, 'multi-output-ridge')
        rankingScore = current(:, 8);
        admissible = predictedSafe(current, protocol);
        gatedScore = rankingScore;
        gatedScore(~admissible) = -Inf;
    else
        rankingScore = current(:, 1);
        gatedScore = rankingScore;
        gatedScore(rankingScore < protocol.safetyThreshold) = -Inf;
    end
    [~, topIdx] = max(rankingScore);
    topSafe(windowIdx) = safe(topIdx);
    [bestScore, selectedIdx] = max(gatedScore);
    if isfinite(bestScore)
        selected(windowIdx) = true;
        selectedSafe(windowIdx) = safe(selectedIdx);
        selectedOutcomes(windowIdx, :) = targets(selectedIdx, :);
    end
end
fold = emptyFold();
fold.heldSeed = heldSeed;
fold.windowCount = windowCount;
fold.topActionSafeCount = nnz(topSafe);
fold.topActionSafeRate = mean(topSafe);
fold.uniformSafePrevalence = mean(chance);
fold.safeLift = fold.topActionSafeRate - fold.uniformSafePrevalence;
fold.gatedSelectionCount = nnz(selected);
fold.gatedSafeCount = nnz(selectedSafe & selected);
fold.gatedUnsafeCount = fold.gatedSelectionCount - fold.gatedSafeCount;
if fold.gatedSelectionCount > 0
    fold.gatedSafePrecision = ...
        fold.gatedSafeCount / fold.gatedSelectionCount;
else
    fold.gatedSafePrecision = NaN;
end
fold.selectedOrReferenceMeanOutcomes = mean(selectedOutcomes, 1);
end

function passed = passesDiagnostic(evaluation, protocol)
outcome = evaluation.selectedOrReferenceMeanOutcomes;
passed = evaluation.topActionSafeLift >= ...
        protocol.minimumTopActionSafeLift && ...
    evaluation.positiveLiftSeedCount >= ...
        protocol.minimumPositiveLiftSeedCount && ...
    evaluation.gatedSelectionCount >= ...
        protocol.minimumGatedSelectionCount && ...
    isfinite(evaluation.gatedSafePrecision) && ...
    evaluation.gatedSafePrecision >= ...
        protocol.minimumGatedSafePrecision && ...
    all(outcome(1:3) > 0) && ...
    outcome(5) >= -protocol.maximumFormationRegressionPercent && ...
    outcome(6) >= -protocol.maximumFormationRegressionPercent;
end

function safe = realizedSafe(outcomes, protocol)
safe = outcomes(:, 1) > 0 & outcomes(:, 2) > 0 & ...
    outcomes(:, 3) > 0 & ...
    outcomes(:, 5) >= -protocol.maximumFormationRegressionPercent & ...
    outcomes(:, 6) >= -protocol.maximumFormationRegressionPercent & ...
    outcomes(:, 7) >= protocol.receiverEospaMinimumPercent & ...
    outcomes(:, 8) > protocol.receiverRmseActivationPercent;
end

function safe = predictedSafe(predictions, protocol)
safe = predictions(:, 1) > 0 & predictions(:, 2) > 0 & ...
    predictions(:, 3) > 0 & ...
    predictions(:, 5) >= -protocol.maximumFormationRegressionPercent & ...
    predictions(:, 6) >= -protocol.maximumFormationRegressionPercent & ...
    predictions(:, 7) >= protocol.receiverEospaMinimumPercent & ...
    predictions(:, 8) > protocol.receiverRmseActivationPercent;
end

function model = fitRidge(features, targets, lambda, weights)
weights = reshape(weights, [], 1);
weights = weights * size(features, 1) / sum(weights);
model.featureMean = weightedMean(features, weights);
centered = bsxfun(@minus, features, model.featureMean);
model.featureScale = sqrt(weightedMean(centered .^ 2, weights));
model.activeFeatureMask = model.featureScale > 1e-9;
if ~any(model.activeFeatureMask)
    error('DecisionAwareGatewayV257:DegenerateFeatures', ...
        'A V257 training fold has no varying feature.');
end
model.featureScale(~model.activeFeatureMask) = 1;
standard = bsxfun(@rdivide, ...
    centered(:, model.activeFeatureMask), ...
    model.featureScale(model.activeFeatureMask));
model.targetMean = weightedMean(targets, weights);
centeredTargets = bsxfun(@minus, targets, model.targetMean);
rootWeight = sqrt(weights);
weightedFeatures = bsxfun(@times, standard, rootWeight);
weightedTargets = bsxfun(@times, centeredTargets, rootWeight);
model.coefficients = (weightedFeatures' * weightedFeatures + ...
    lambda * eye(nnz(model.activeFeatureMask))) \ ...
    (weightedFeatures' * weightedTargets);
end

function predictions = predictRidge(model, features)
centered = bsxfun(@minus, features, model.featureMean);
standard = bsxfun(@rdivide, ...
    centered(:, model.activeFeatureMask), ...
    model.featureScale(model.activeFeatureMask));
predictions = bsxfun(@plus, ...
    standard * model.coefficients, model.targetMean);
end

function value = weightedMean(values, weights)
value = (reshape(weights, 1, []) * values) / sum(weights);
end

function writeReport(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('DecisionAwareGatewayV257:ReportOpen', ...
        'Could not write the V257 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V257 decision-aware gateway diagnostic\n\n');
fprintf(fid, '- Analysis commit: `%s`\n', summary.analysisGitCommit);
fprintf(fid, '- Training seeds: `%s`\n', mat2str(summary.trainingSeeds));
fprintf(fid, '- Windows / actions / safe actions: `%d / %d / %d`\n', ...
    summary.recordCount, summary.actionRowCount, summary.safeActionCount);
fprintf(fid, '- Compact signal present: `%d`\n', ...
    summary.compactSignalPresent);
fprintf(fid, '- Rich information signal present: `%d`\n', ...
    summary.richSignalPresent);
fprintf(fid, '- Next decision: `%s`\n\n', summary.nextDecision);
fprintf(fid, ['| Features | Loss | Lambda | Top-safe | Chance | Lift | ', ...
    'Positive seeds | Selected | Precision | E | RMSE | Consistency | Pass |\n']);
fprintf(fid, '|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|\n');
for idx = 1:numel(summary.configurations)
    row = summary.configurations(idx);
    value = row.evaluation;
    outcome = value.selectedOrReferenceMeanOutcomes;
    fprintf(fid, ['| %s | %s | %.6g | %.3f | %.3f | %+.3f | ', ...
        '%d/7 | %d/42 | %.3f | %+.4f | %+.4f | %+.4f | %d |\n'], ...
        row.featureMode, row.modelFamily, row.ridgeLambda, ...
        value.topActionSafeRate, value.uniformSafePrevalence, ...
        value.topActionSafeLift, value.positiveLiftSeedCount, ...
        value.gatedSelectionCount, value.gatedSafePrecision, ...
        outcome(1), outcome(2), outcome(3), ...
        row.diagnosticGatePassed);
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', summary.evidenceBoundary);
end

function value = emptyConfiguration()
value = struct('featureMode', '', 'featureCount', 0, ...
    'modelFamily', '', 'ridgeLambda', NaN, 'evaluation', struct(), ...
    'diagnosticGatePassed', false);
end

function value = emptyFold()
value = struct('heldSeed', 0, 'windowCount', 0, ...
    'topActionSafeCount', 0, 'topActionSafeRate', NaN, ...
    'uniformSafePrevalence', NaN, 'safeLift', NaN, ...
    'gatedSelectionCount', 0, 'gatedSafeCount', 0, ...
    'gatedUnsafeCount', 0, 'gatedSafePrecision', NaN, ...
    'selectedOrReferenceMeanOutcomes', zeros(1, 8));
end

function path = absolutePath(path, repoRoot)
path = char(path);
if isempty(path) || ~(path(1) == '/' || ...
        ~isempty(regexp(path, '^[A-Za-z]:[\\/]', 'once')))
    path = fullfile(repoRoot, path);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
