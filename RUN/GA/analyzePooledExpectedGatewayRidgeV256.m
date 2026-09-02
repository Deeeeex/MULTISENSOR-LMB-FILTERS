function [reportPath, summary] = ...
        analyzePooledExpectedGatewayRidgeV256(options)
% ANALYZEPOOLEDEXPECTEDGATEWAYRIDGEV256 Select pooled expected-value ridge.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPooledExpectedGatewayV256Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
datasetPath = char(getField(options, 'datasetPath', fullfile( ...
    repoRoot, protocol.outputRoot, 'training_dataset', ...
    'POOLED_EXPECTED_GATEWAY_V256_TRAINING_DATASET.mat')));
if ~isAbsolutePath(datasetPath)
    datasetPath = fullfile(repoRoot, datasetPath);
end
if exist(datasetPath, 'file') ~= 2
    error('PooledExpectedGatewayV256:MissingDataset', ...
        'The frozen seven-seed V256 training dataset is unavailable.');
end
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('PooledExpectedGatewayV256:DirtyAnalysisSource', ...
        'Official V256 model selection requires clean source.');
end
envelope = load(datasetPath, 'dataset');
if ~isfield(envelope, 'dataset')
    error('PooledExpectedGatewayV256:MissingDatasetEnvelope', ...
        'The V256 dataset MAT has no dataset envelope.');
end
dataset = envelope.dataset;
validateDataset(dataset, protocol);
[features, targets, seedByRow, communicationFeasible, ...
    fitWeights, supportWeights] = ...
    flattenDataset(dataset);

configurations = repmat(emptyConfiguration(), ...
    1, numel(protocol.ridgeLambdaGrid));
bestIndex = 0;
bestKey = [-Inf, -Inf, -Inf];
for lambdaIdx = 1:numel(protocol.ridgeLambdaGrid)
    lambda = protocol.ridgeLambdaGrid(lambdaIdx);
    evaluation = evaluateLambda( ...
        features, targets, seedByRow, communicationFeasible, ...
        fitWeights, supportWeights, lambda, protocol);
    configuration = emptyConfiguration();
    configuration.ridgeLambda = lambda;
    configuration.evaluation = evaluation;
    configuration.selectionKey = [-evaluation.meanTrackingModelMse, ...
        -evaluation.primaryModelMse, log10(lambda)];
    configurations(lambdaIdx) = configuration;
    if lexicographicallyGreater(configuration.selectionKey, bestKey)
        bestIndex = lambdaIdx;
        bestKey = configuration.selectionKey;
    end
end
if bestIndex == 0
    error('PooledExpectedGatewayV256:ModelSelection', ...
        'No V256 ridge configuration was selected.');
end
selected = configurations(bestIndex);
model = fitRidge( ...
    features, targets, selected.ridgeLambda, fitWeights);
model.contractVersion = ...
    'pooled-expected-gateway-v256-ridge-model-v1';
model.protocolId = protocol.id;
model.featureNames = dataset.featureNames;
model.outcomeNames = dataset.outcomeNames;
model.trainingSeeds = protocol.trainingSeeds;
model.featureTransform = protocol.actionFeatureTransform;
model.modelSelectionMode = protocol.modelSelectionMode;
model.modelSelectionActionSupport = ...
    protocol.modelSelectionActionSupport;
model.modelSelectionSeedAggregation = ...
    protocol.modelSelectionSeedAggregation;
model.sampleWeighting = protocol.seedWeighting;
model.ridgeLambda = selected.ridgeLambda;
model.truthUsed = false;
model.futureInformationUsed = false;

evaluation = selected.evaluation;
calibrationAuthorized = ...
    evaluation.meanTrackingSkillGainPercent > ...
        protocol.minimumMeanCvSkillGainPercent && ...
    evaluation.primarySkillGainPercent > ...
        protocol.minimumPrimaryCvSkillGainPercent;
summary = struct();
summary.contractVersion = ...
    'pooled-expected-gateway-v256-model-selection-summary-v1';
summary.protocol = protocol;
summary.analysisGitCommit = gitState.commit;
summary.datasetPath = datasetPath;
summary.datasetAssemblyGitCommit = dataset.assemblyGitCommit;
summary.trainingSeeds = protocol.trainingSeeds;
summary.rowCount = size(features, 1);
summary.communicationFeasibleRowCount = nnz(communicationFeasible);
summary.minimumActionsPerWindowObserved = ...
    min(dataset.actionRowsPerWindow);
summary.maximumActionsPerWindowObserved = ...
    max(dataset.actionRowsPerWindow);
summary.featureCount = size(features, 2);
summary.outcomeCount = size(targets, 2);
summary.configurations = configurations;
summary.selectedConfiguration = selected;
summary.model = model;
summary.calibrationAuthorized = calibrationAuthorized;
summary.developmentHoldoutAuthorized = false;
summary.completeEpisodeAuthorized = false;
summary.gnnAuthorized = false;
summary.x36Authorized = false;
summary.validationClaimAllowed = false;
summary.developmentEvidenceOnly = true;
if calibrationAuthorized
    summary.nextDecision = ...
        'freeze-ridge-and-generate-v255-calibration-seeds-1311-1312';
else
    summary.nextDecision = ...
        'stop-feature-conditioned-v256-before-calibration';
end
summary.completedAt = datestr(now, 31);
summary.evidenceBoundary = protocol.evidenceBoundary;

outputRoot = char(getField(options, 'outputRoot', ...
    fullfile(repoRoot, protocol.outputRoot, 'ridge_model_selection')));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'V256_POOLED_EXPECTED_GATEWAY_RIDGE.md');
matPath = fullfile(outputRoot, ...
    'V256_POOLED_EXPECTED_GATEWAY_RIDGE.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
save('-mat7-binary', matPath, 'summary');
writeReportFile(reportPath, summary);
fprintf(['V256 pooled ridge: lambda %.6g; tracking skill %+.3f%%; ', ...
    'receiver-RMSE skill %+.3f%%; calibration=%d\n'], ...
    selected.ridgeLambda, evaluation.meanTrackingSkillGainPercent, ...
    evaluation.primarySkillGainPercent, calibrationAuthorized);
fprintf('V256 model report: %s\n', reportPath);
end

function validateDataset(dataset, protocol)
required = {'contractVersion', 'protocol', 'assemblyGitCommit', ...
    'trainingSeeds', 'featureNames', 'outcomeNames', 'records', ...
    'recordCount', 'actionRowCount', 'truthUsedAsFeature', ...
    'futureOutcomeUsedAsFeature', ...
    'communicationProjectionContractVersion', ...
    'communicationAdmissionMode', 'actionRowsPerWindow', ...
    'sampleWeighting'};
valid = isstruct(dataset) && isscalar(dataset) && ...
    all(isfield(dataset, required)) && ...
    strcmp(dataset.contractVersion, ...
        'pooled-expected-gateway-v256-training-dataset-v2') && ...
    strcmp(dataset.protocol.id, protocol.id) && ...
    isequal(sort(dataset.trainingSeeds), sort(protocol.trainingSeeds)) && ...
    numel(dataset.featureNames) == protocol.expectedFeatureCount && ...
    isequal(dataset.outcomeNames, protocol.outcomeNames) && ...
    dataset.recordCount == numel(dataset.records) && ...
    numel(dataset.actionRowsPerWindow) == dataset.recordCount && ...
    dataset.actionRowCount == sum(dataset.actionRowsPerWindow) && ...
    all(dataset.actionRowsPerWindow >= ...
        protocol.minimumActionsPerWindow) && ...
    all(dataset.actionRowsPerWindow <= ...
        protocol.maximumActionsPerWindow) && ...
    strcmp(dataset.sampleWeighting, protocol.seedWeighting) && ...
    strcmp(dataset.communicationProjectionContractVersion, ...
        'pooled-expected-gateway-v256-communication-projection-v1') && ...
    strcmp(dataset.communicationAdmissionMode, ...
        protocol.communicationAdmissionMode) && ...
    all(cellfun(@hasValidCommunicationProjection, ...
        dataset.records)) && ...
    ~dataset.truthUsedAsFeature && ~dataset.futureOutcomeUsedAsFeature;
if ~valid
    error('PooledExpectedGatewayV256:InvalidDataset', ...
        'The V256 training dataset violates its frozen contract.');
end

function valid = hasValidCommunicationProjection(record)
valid = isstruct(record) && isscalar(record) && ...
    isfield(record, 'communicationProjection') && ...
    isstruct(record.communicationProjection) && ...
    isscalar(record.communicationProjection) && ...
    isfield(record.communicationProjection, 'contractVersion') && ...
    strcmp(record.communicationProjection.contractVersion, ...
        'pooled-expected-gateway-v256-communication-projection-v1') && ...
    isfield(record.communicationProjection, 'candidateFeasible') && ...
    isfield(record, 'features') && ...
    numel(record.communicationProjection.candidateFeasible) == ...
        size(record.features, 1) && ...
    isfield(record, 'communicationAdmissionUsesRealizedBytes') && ...
    ~record.communicationAdmissionUsesRealizedBytes;
end
end

function [features, targets, seeds, communicationFeasible, ...
        fitWeights, supportWeights] = ...
        flattenDataset(dataset)
features = zeros(dataset.actionRowCount, numel(dataset.featureNames));
targets = zeros(dataset.actionRowCount, numel(dataset.outcomeNames));
seeds = zeros(dataset.actionRowCount, 1);
communicationFeasible = false(dataset.actionRowCount, 1);
fitWeights = zeros(dataset.actionRowCount, 1);
supportWeights = zeros(dataset.actionRowCount, 1);
cursor = 0;
for recordIdx = 1:numel(dataset.records)
    record = dataset.records{recordIdx};
    rowCount = size(record.features, 1);
    rows = cursor + (1:rowCount);
    features(rows, :) = record.features;
    targets(rows, :) = record.targets;
    seeds(rows) = record.seed;
    currentFeasible = reshape( ...
        logical(record.communicationProjection.candidateFeasible), [], 1);
    communicationFeasible(rows) = currentFeasible;
    fitWeights(rows) = 1 / rowCount;
    if any(currentFeasible)
        localSupport = zeros(rowCount, 1);
        localSupport(currentFeasible) = 1 / nnz(currentFeasible);
        supportWeights(rows) = localSupport;
    end
    cursor = cursor + rowCount;
end
if cursor ~= dataset.actionRowCount || ...
        any(~isfinite(features(:))) || any(~isfinite(targets(:)))
    error('PooledExpectedGatewayV256:InvalidRows', ...
        'The V256 training rows are incomplete or non-finite.');
end
end

function evaluation = evaluateLambda( ...
        features, targets, seeds, communicationFeasible, fitWeights, ...
        supportWeights, lambda, protocol)
trainingSeeds = protocol.trainingSeeds;
outcomeCount = size(targets, 2);
folds = repmat(emptyFold(), 1, numel(trainingSeeds));
for foldIdx = 1:numel(trainingSeeds)
    heldSeed = trainingSeeds(foldIdx);
    train = seeds ~= heldSeed;
    trainSupport = train & communicationFeasible;
    test = seeds == heldSeed & communicationFeasible;
    if ~any(train) || ~any(trainSupport) || ~any(test)
        error('PooledExpectedGatewayV256:FoldCoverage', ...
            ['A V256 leave-one-seed-out fold has no deterministic ', ...
             'communication-feasible support.']);
    end
    model = fitRidge(features(train, :), targets(train, :), lambda, ...
        fitWeights(train));
    prediction = predictRidge(model, features(test, :));
    baselineMean = weightedMean( ...
        targets(trainSupport, :), supportWeights(trainSupport));
    baseline = repmat(baselineMean, nnz(test), 1);
    trainCentered = bsxfun(@minus, ...
        targets(trainSupport, :), baselineMean);
    scale = sqrt(weightedMean( ...
        trainCentered .^ 2, supportWeights(trainSupport)));
    scale = max(scale, protocol.outputScaleFloor);
    modelError = bsxfun(@rdivide, ...
        prediction - targets(test, :), scale) .^ 2;
    baselineError = bsxfun(@rdivide, ...
        baseline - targets(test, :), scale) .^ 2;
    fold = emptyFold();
    fold.heldSeed = heldSeed;
    fold.rowCount = nnz(test);
    fold.modelMseByOutcome = weightedMean( ...
        modelError, supportWeights(test));
    fold.baselineMseByOutcome = weightedMean( ...
        baselineError, supportWeights(test));
    fold.meanTrackingModelMse = mean( ...
        fold.modelMseByOutcome(protocol.modelSelectionOutcomeIndices));
    fold.meanTrackingBaselineMse = mean( ...
        fold.baselineMseByOutcome(protocol.modelSelectionOutcomeIndices));
    fold.primaryModelMse = ...
        fold.modelMseByOutcome(protocol.primaryOutcomeIndex);
    fold.primaryBaselineMse = ...
        fold.baselineMseByOutcome(protocol.primaryOutcomeIndex);
    folds(foldIdx) = fold;
end
modelMse = mean(vertcat(folds.modelMseByOutcome), 1);
baselineMse = mean(vertcat(folds.baselineMseByOutcome), 1);
tracking = protocol.modelSelectionOutcomeIndices;
evaluation = struct();
evaluation.folds = folds;
evaluation.modelMseByOutcome = modelMse;
evaluation.baselineMseByOutcome = baselineMse;
evaluation.meanTrackingModelMse = mean(modelMse(tracking));
evaluation.meanTrackingBaselineMse = mean(baselineMse(tracking));
evaluation.primaryModelMse = modelMse(protocol.primaryOutcomeIndex);
evaluation.primaryBaselineMse = ...
    baselineMse(protocol.primaryOutcomeIndex);
evaluation.meanTrackingSkillGainPercent = lowerGain( ...
    evaluation.meanTrackingBaselineMse, ...
    evaluation.meanTrackingModelMse);
evaluation.primarySkillGainPercent = lowerGain( ...
    evaluation.primaryBaselineMse, evaluation.primaryModelMse);
evaluation.skillGainByOutcomePercent = lowerGainVector( ...
    baselineMse, modelMse);
end

function model = fitRidge(features, targets, lambda, weights)
weights = normalizeWeights(weights, size(features, 1));
xMean = weightedMean(features, weights);
xCentered = bsxfun(@minus, features, xMean);
xScale = sqrt(weightedMean(xCentered .^ 2, weights));
active = xScale > 1e-9;
if ~any(active)
    error('PooledExpectedGatewayV256:DegenerateFeatures', ...
        'The V256 training features contain no varying coordinate.');
end
xScale(~active) = 1;
standard = bsxfun(@rdivide, xCentered(:, active), xScale(active));
yMean = weightedMean(targets, weights);
yCentered = bsxfun(@minus, targets, yMean);
rootWeight = sqrt(weights);
weightedStandard = bsxfun(@times, standard, rootWeight);
weightedTargets = bsxfun(@times, yCentered, rootWeight);
coefficients = (weightedStandard' * weightedStandard + ...
    lambda * eye(nnz(active))) \ ...
    (weightedStandard' * weightedTargets);
model = struct();
model.featureMean = xMean;
model.featureScale = xScale;
model.activeFeatureMask = active;
model.targetMean = yMean;
model.coefficients = coefficients;
model.trainingRowCount = size(features, 1);
model.trainingWeightSum = sum(weights);
end

function weights = normalizeWeights(weights, expectedRows)
weights = reshape(weights, [], 1);
if numel(weights) ~= expectedRows || any(~isfinite(weights)) || ...
        any(weights < 0) || sum(weights) <= 0
    error('PooledExpectedGatewayV256:InvalidSampleWeights', ...
        'V256 ridge sample weights are malformed.');
end
weights = weights * expectedRows / sum(weights);
end

function value = weightedMean(values, weights)
weights = reshape(weights, [], 1);
if size(values, 1) ~= numel(weights) || sum(weights) <= 0
    error('PooledExpectedGatewayV256:InvalidWeightedMean', ...
        'V256 weighted aggregation has incompatible rows.');
end
value = (weights' * values) / sum(weights);
end

function predictions = predictRidge(model, features)
centered = bsxfun(@minus, features, model.featureMean);
standard = bsxfun(@rdivide, ...
    centered(:, model.activeFeatureMask), ...
    model.featureScale(model.activeFeatureMask));
predictions = bsxfun(@plus, ...
    standard * model.coefficients, model.targetMean);
end

function writeReportFile(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('PooledExpectedGatewayV256:ReportOpen', ...
        'Could not write the V256 model-selection report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
selected = summary.selectedConfiguration;
evaluation = selected.evaluation;
fprintf(fid, '# V256 pooled expected local gateway ridge\n\n');
fprintf(fid, '- Analysis commit: `%s`\n', summary.analysisGitCommit);
fprintf(fid, '- Training seeds: `%s`\n', mat2str(summary.trainingSeeds));
fprintf(fid, '- Rows / features / outcomes: `%d / %d / %d`\n', ...
    summary.rowCount, summary.featureCount, summary.outcomeCount);
fprintf(fid, '- Deterministic communication-feasible rows: `%d/%d`\n', ...
    summary.communicationFeasibleRowCount, summary.rowCount);
fprintf(fid, '- Actions per window, min / max: `%d / %d`\n', ...
    summary.minimumActionsPerWindowObserved, ...
    summary.maximumActionsPerWindowObserved);
fprintf(fid, '- Selected ridge lambda: `%.6g`\n', ...
    selected.ridgeLambda);
fprintf(fid, '- Mean tracking skill over seed-blind mean: `%+.3f%%`\n', ...
    evaluation.meanTrackingSkillGainPercent);
fprintf(fid, '- Receiver-RMSE skill over seed-blind mean: `%+.3f%%`\n', ...
    evaluation.primarySkillGainPercent);
fprintf(fid, '- Calibration authorized: `%d`\n', ...
    summary.calibrationAuthorized);
fprintf(fid, '- Next decision: `%s`\n\n', summary.nextDecision);

fprintf(fid, '| Lambda | Tracking normalized MSE | Baseline | ', ...
    'Skill | Receiver-RMSE MSE | Baseline | Skill |\n');
fprintf(fid, '|--:|--:|--:|--:|--:|--:|--:|\n');
for idx = 1:numel(summary.configurations)
    configuration = summary.configurations(idx);
    value = configuration.evaluation;
    fprintf(fid, ['| %.6g | %.5f | %.5f | %+.3f%% | ', ...
        '%.5f | %.5f | %+.3f%% |\n'], ...
        configuration.ridgeLambda, value.meanTrackingModelMse, ...
        value.meanTrackingBaselineMse, ...
        value.meanTrackingSkillGainPercent, value.primaryModelMse, ...
        value.primaryBaselineMse, value.primarySkillGainPercent);
end

fprintf(fid, '\n## Selected cross-seed outcome errors\n\n');
fprintf(fid, '| Outcome | Model MSE | Mean baseline MSE | Skill |\n');
fprintf(fid, '|:--|--:|--:|--:|\n');
for outcomeIdx = 1:numel(summary.protocol.outcomeNames)
    fprintf(fid, '| %s | %.5f | %.5f | %+.3f%% |\n', ...
        summary.protocol.outcomeNames{outcomeIdx}, ...
        evaluation.modelMseByOutcome(outcomeIdx), ...
        evaluation.baselineMseByOutcome(outcomeIdx), ...
        evaluation.skillGainByOutcomePercent(outcomeIdx));
end
fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function value = emptyConfiguration()
value = struct('ridgeLambda', NaN, 'evaluation', struct(), ...
    'selectionKey', zeros(1, 3));
end

function value = emptyFold()
value = struct('heldSeed', 0, 'rowCount', 0, ...
    'modelMseByOutcome', zeros(1, 0), ...
    'baselineMseByOutcome', zeros(1, 0), ...
    'meanTrackingModelMse', NaN, ...
    'meanTrackingBaselineMse', NaN, ...
    'primaryModelMse', NaN, 'primaryBaselineMse', NaN);
end

function greater = lexicographicallyGreater(left, right)
greater = false;
for idx = 1:numel(left)
    if left(idx) > right(idx) + 1e-12
        greater = true;
        return;
    elseif left(idx) < right(idx) - 1e-12
        return;
    end
end
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = lowerGainVector(reference, candidate)
value = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function absolute = isAbsolutePath(path)
absolute = ~isempty(path) && (path(1) == '/' || ...
    (~isempty(regexp(path, '^[A-Za-z]:[\\/]', 'once'))));
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
