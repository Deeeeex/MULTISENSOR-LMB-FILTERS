function [model, summary] = ...
    fitConfidenceGatedResidualRoutingModel( ...
        trainingDatasetPaths, calibrationDatasetPaths, options)
% FITCONFIDENCEGATEDRESIDUALROUTINGMODEL Support-gated joint-action model.
%
% Targets are expected-risk improvements relative to the registered
% reliability + equal-weight action for the same receiver. Training and
% calibration datasets must be disjoint at the scenario/seed/time level.
% Calibration nonconformity is aggregated by complete dataset block: the
% selected-action correction for one block is the worst receiver error in
% that block. With only one calibration block this is an empirical
% fail-closed check, not a population-level coverage guarantee.

if nargin < 3 || isempty(options)
    options = struct();
end
trainingDatasetPaths = normalizePaths(trainingDatasetPaths);
calibrationDatasetPaths = normalizePaths(calibrationDatasetPaths);
if isempty(trainingDatasetPaths) || isempty(calibrationDatasetPaths)
    error('Both training and calibration datasets are required.');
end
assertDisjointDatasetBlocks( ...
    trainingDatasetPaths, calibrationDatasetPaths);
[train, contract] = unpackDatasets(trainingDatasetPaths, []);
[calibration, calibrationContract] = ...
    unpackDatasets(calibrationDatasetPaths, contract);
if ~isequal(contract.featureNames, calibrationContract.featureNames) || ...
        ~isequal(contract.sourceWeightGrid, ...
            calibrationContract.sourceWeightGrid)
    error('Training and calibration action-feature contracts differ.');
end

featureMean = mean(train.X, 1);
featureScale = std(train.X, 0, 1);
featureScale(~isfinite(featureScale) | featureScale <= eps) = 1;
trainZ = standardize(train.X, featureMean, featureScale);
calibrationZ = standardize( ...
    calibration.X, featureMean, featureScale);
targetClip = max(getField(options, 'targetClip', 2.0), eps);
trainingTarget = min(max(train.y, -targetClip), targetClip);
ridgeLambda = max(getField(options, 'ridgeLambda', 1.0), 0);
intercept = mean(trainingTarget);
centeredTarget = trainingTarget - intercept;
coefficient = (trainZ' * trainZ + ...
    ridgeLambda * eye(size(trainZ, 2))) \ ...
    (trainZ' * centeredTarget);

trainingPrediction = intercept + trainZ * coefficient;
calibrationPrediction = intercept + ...
    calibrationZ * coefficient;
confidenceLevel = min(max(getField( ...
    options, 'confidenceLevel', 0.95), 0.50), 0.999);
simultaneousBlockOverestimate = maxErrorByGroup( ...
    calibrationPrediction - calibration.y, ...
    calibration.blockGroup);
minimumResidual = getField(options, 'minimumResidual', 0);

supportMargin = max(getField( ...
    options, 'supportMarginStd', 0.50), 0);
supportLower = min(trainZ, [], 1) - supportMargin;
supportUpper = max(trainZ, [], 1) + supportMargin;
calibrationInSupport = all(bsxfun(@ge, ...
    calibrationZ, supportLower) & bsxfun(@le, ...
    calibrationZ, supportUpper), 2);
backboneFeatureIdx = find(strcmp( ...
    contract.featureNames, 'backbone_action'), 1);
if isempty(backboneFeatureIdx)
    error('Residual-routing contract is missing backbone_action.');
end
calibrationAlternative = ...
    calibration.X(:, backboneFeatureIdx) < 0.5;
calibrationEligible = ...
    calibrationInSupport & calibrationAlternative;
calibrationMode = lower(strrep(char(getField( ...
    options, 'calibrationMode', 'selected-action')), '_', '-'));
switch calibrationMode
    case 'selected-action'
        blockOverestimate = selectedErrorByBlock( ...
            calibrationPrediction - calibration.y, ...
            calibrationPrediction, calibrationEligible, ...
            calibration.receiverGroup, ...
            calibration.blockGroup);
    case 'simultaneous-action'
        blockOverestimate = simultaneousBlockOverestimate;
    otherwise
        error('Unknown residual calibrationMode: %s', calibrationMode);
end
overestimateQuantile = max(conformalUpperQuantile( ...
    blockOverestimate, confidenceLevel), 0);
calibrationLowerBound = ...
    calibrationPrediction - overestimateQuantile;
calibrationLowerBound(~calibrationEligible) = -inf;
calibrationReadout = evaluateSelections( ...
    calibrationLowerBound, calibration.y, ...
    calibration.receiverGroup, minimumResidual, ...
    calibrationEligible);
maximumHarmfulOverrideFraction = max(getField(options, ...
    'maximumCalibrationHarmfulOverrideFraction', 0), 0);
safetyEscalated = false;
if calibrationReadout.harmfulOverrideFraction > ...
        maximumHarmfulOverrideFraction
    overestimateQuantile = max([ ...
        overestimateQuantile; ...
        blockOverestimate(isfinite(blockOverestimate)); ...
        0]);
    calibrationLowerBound = ...
        calibrationPrediction - overestimateQuantile;
    calibrationLowerBound(~calibrationEligible) = -inf;
    calibrationReadout = evaluateSelections( ...
        calibrationLowerBound, calibration.y, ...
        calibration.receiverGroup, minimumResidual, ...
        calibrationEligible);
    safetyEscalated = true;
end

model = struct();
model.kind = 'confidence-gated-residual-directed-kla';
model.featureNames = contract.featureNames;
model.sourceWeightGrid = contract.sourceWeightGrid;
model.backboneWeight = contract.backboneWeight;
model.featureMean = featureMean;
model.featureScale = featureScale;
model.coefficient = coefficient;
model.intercept = intercept;
model.ridgeLambda = ridgeLambda;
model.targetClip = targetClip;
model.confidenceLevel = confidenceLevel;
model.calibrationMode = calibrationMode;
model.calibrationUnit = 'scenario-seed-time-block-max';
model.overestimateQuantile = overestimateQuantile;
model.minimumResidual = minimumResidual;
model.maximumCalibrationHarmfulOverrideFraction = ...
    maximumHarmfulOverrideFraction;
model.calibrationSafetyEscalated = safetyEscalated;
model.supportLower = supportLower;
model.supportUpper = supportUpper;
model.trainingDatasetPaths = trainingDatasetPaths;
model.calibrationDatasetPaths = calibrationDatasetPaths;
model.maxMessagesPerReceiver = 1;

summary = struct();
summary.trainingExampleCount = numel(train.y);
summary.calibrationExampleCount = numel(calibration.y);
summary.trainingReceiverGroups = ...
    numel(unique(train.receiverGroup));
summary.calibrationReceiverGroups = ...
    numel(unique(calibration.receiverGroup));
summary.trainingBlockCount = ...
    numel(unique(train.blockGroup));
summary.calibrationBlockCount = ...
    numel(unique(calibration.blockGroup));
summary.featureCount = numel(contract.featureNames);
summary.ridgeLambda = ridgeLambda;
summary.trainingRmse = sqrt(mean( ...
    (trainingPrediction - train.y).^2));
summary.calibrationRmse = sqrt(mean( ...
    (calibrationPrediction - calibration.y).^2));
summary.overestimateQuantile = overestimateQuantile;
summary.simultaneousOverestimateQuantile = max(conformalUpperQuantile( ...
    simultaneousBlockOverestimate, confidenceLevel), 0);
summary.calibrationMode = calibrationMode;
summary.calibrationUnit = model.calibrationUnit;
summary.maximumCalibrationHarmfulOverrideFraction = ...
    maximumHarmfulOverrideFraction;
summary.calibrationSafetyEscalated = safetyEscalated;
summary.calibration = calibrationReadout;
summary.calibrationInSupportFraction = ...
    mean(calibrationInSupport);
summary.trainingTargetMean = mean(train.y);
summary.trainingPositiveResidualFraction = mean(train.y > 0);
summary.contract = contract;
end

function [dataset, contract] = unpackDatasets(paths, expectedContract)
X = zeros(0, 0);
y = zeros(0, 1);
receiverGroup = zeros(0, 1);
blockGroup = zeros(0, 1);
groupOffset = 0;
contract = struct();
for datasetIdx = 1:numel(paths)
    loaded = load(paths{datasetIdx}, ...
        'actionFeatures', 'actionFeatureNames', ...
        'actionFeatureMetadata', 'teacherDetails', ...
        'datasetMetadata');
    validateLoadedDataset(loaded, paths{datasetIdx});
    localContract = struct( ...
        'featureNames', ...
            {reshape(loaded.actionFeatureNames, 1, [])}, ...
        'sourceWeightGrid', reshape( ...
            loaded.actionFeatureMetadata.sourceWeightGrid, 1, []), ...
        'backboneWeight', ...
            loaded.actionFeatureMetadata.backboneWeight);
    if isempty(fieldnames(contract))
        contract = localContract;
        if ~isempty(expectedContract)
            assertContract(contract, expectedContract);
        end
    else
        assertContract(localContract, contract);
    end
    [localX, localY, localGroup] = unpackOne(loaded);
    if isempty(X)
        X = zeros(0, size(localX, 2));
    end
    X = [X; localX]; %#ok<AGROW>
    y = [y; localY]; %#ok<AGROW>
    receiverGroup = [receiverGroup; ...
        localGroup + groupOffset]; %#ok<AGROW>
    blockGroup = [blockGroup; ...
        datasetIdx * ones(size(localGroup))]; %#ok<AGROW>
    groupOffset = max(receiverGroup);
end
dataset = struct('X', X, 'y', y, ...
    'receiverGroup', receiverGroup, ...
    'blockGroup', blockGroup);
end

function [X, y, receiverGroup] = unpackOne(loaded)
features = loaded.actionFeatures;
expectedRisk = loaded.teacherDetails.firstStepExpectedRiskByWeight;
nodeRisk = reshape(loaded.teacherDetails.nodeRiskBefore, [], 1);
weightGrid = reshape( ...
    loaded.actionFeatureMetadata.sourceWeightGrid, 1, []);
[~, backboneWeightIdx] = min(abs(weightGrid - ...
    loaded.actionFeatureMetadata.backboneWeight));
backboneFeatureIdx = find(strcmp( ...
    reshape(loaded.actionFeatureNames, 1, []), ...
    'backbone_action'), 1);
if isempty(backboneFeatureIdx)
    error('Action features are missing backbone_action.');
end
nodeCount = size(features, 1);
featureCount = size(features, 4);
X = zeros(0, featureCount);
y = zeros(0, 1);
receiverGroup = zeros(0, 1);
for receiverIdx = 1:nodeCount
    backboneSlice = reshape(features( ...
        receiverIdx, :, backboneWeightIdx, ...
        backboneFeatureIdx), 1, []);
    backboneSender = find(backboneSlice > 0.5, 1);
    if isempty(backboneSender)
        continue;
    end
    backboneRisk = expectedRisk( ...
        receiverIdx, backboneSender, backboneWeightIdx);
    if ~isfinite(backboneRisk)
        continue;
    end
    for senderIdx = 1:nodeCount
        for weightIdx = 1:numel(weightGrid)
            row = reshape(features( ...
                receiverIdx, senderIdx, weightIdx, :), 1, []);
            candidateRisk = expectedRisk( ...
                receiverIdx, senderIdx, weightIdx);
            if any(~isfinite(row)) || ~isfinite(candidateRisk)
                continue;
            end
            X(end+1, :) = row; %#ok<AGROW>
            y(end+1, 1) = (backboneRisk - candidateRisk) / ...
                max(nodeRisk(receiverIdx), eps); %#ok<AGROW>
            receiverGroup(end+1, 1) = receiverIdx; %#ok<AGROW>
        end
    end
end
end

function validateLoadedDataset(loaded, path)
required = {'actionFeatures', 'actionFeatureNames', ...
    'actionFeatureMetadata', 'teacherDetails', 'datasetMetadata'};
if ~all(isfield(loaded, required))
    error('Residual-routing dataset is incomplete: %s', path);
end
if ~isfield(loaded.datasetMetadata, 'teacherLabelVersion') || ...
        ~strcmp(loaded.datasetMetadata.teacherLabelVersion, ...
            'joint-action-bernoulli-risk-v3-directed-physical') || ...
        ~isfield(loaded.datasetMetadata, ...
            'physicalAdjacencyConvention') || ...
        ~strcmp(loaded.datasetMetadata.physicalAdjacencyConvention, ...
            'receiver-row-sender-column-directed') || ...
        ~isfield(loaded.datasetMetadata, ...
            'scenarioConfigSnapshot') || ...
        ~isfield(loaded.datasetMetadata, ...
            'fusionConfigSnapshot') || ...
        ~isfield(loaded.teacherDetails, ...
            'firstStepExpectedRiskByWeight') || ...
        ~isfield(loaded.teacherDetails, 'deliveryExpectationMode') || ...
        ~strcmp(loaded.teacherDetails.deliveryExpectationMode, ...
            'bernoulli-risk')
    error('Residual-routing dataset uses incompatible labels: %s', path);
end
end

function assertContract(left, right)
if ~isequal(left.featureNames, right.featureNames) || ...
        ~isequal(left.sourceWeightGrid, right.sourceWeightGrid) || ...
        abs(left.backboneWeight - right.backboneWeight) > 1e-12
    error('Residual-routing dataset feature contracts differ.');
end
end

function values = standardize(X, featureMean, featureScale)
values = bsxfun(@rdivide, ...
    bsxfun(@minus, X, featureMean), featureScale);
end

function maxima = maxErrorByGroup(error, groups)
uniqueGroups = unique(groups);
maxima = zeros(numel(uniqueGroups), 1);
for groupIdx = 1:numel(uniqueGroups)
    maxima(groupIdx) = max(error(groups == uniqueGroups(groupIdx)));
end
end

function blockError = selectedErrorByBlock( ...
    error, prediction, eligible, receiverGroups, blockGroups)
uniqueBlocks = unique(blockGroups);
blockError = nan(numel(uniqueBlocks), 1);
for blockIdx = 1:numel(uniqueBlocks)
    blockMask = blockGroups == uniqueBlocks(blockIdx);
    localReceivers = unique(receiverGroups(blockMask));
    receiverError = nan(numel(localReceivers), 1);
    for receiverIdx = 1:numel(localReceivers)
        mask = blockMask & ...
            receiverGroups == localReceivers(receiverIdx) & ...
            eligible;
        if ~any(mask)
            continue;
        end
        localIndices = find(mask);
        [~, bestCursor] = max(prediction(localIndices));
        receiverError(receiverIdx) = ...
            error(localIndices(bestCursor));
    end
    finiteError = receiverError(isfinite(receiverError));
    if ~isempty(finiteError)
        blockError(blockIdx) = max(finiteError);
    end
end
end

function readout = evaluateSelections( ...
    lowerBound, actualResidual, groups, minimumResidual, eligible)
uniqueGroups = unique(groups);
selectedResidual = zeros(numel(uniqueGroups), 1);
selected = false(numel(uniqueGroups), 1);
oracleResidual = zeros(numel(uniqueGroups), 1);
for groupIdx = 1:numel(uniqueGroups)
    mask = groups == uniqueGroups(groupIdx) & eligible;
    if ~any(mask)
        continue;
    end
    localBound = lowerBound(mask);
    localActual = actualResidual(mask);
    [bestBound, bestIdx] = max(localBound);
    oracleResidual(groupIdx) = max([0; localActual]);
    if bestBound > minimumResidual
        selected(groupIdx) = true;
        selectedResidual(groupIdx) = localActual(bestIdx);
    end
end
readout = struct();
readout.receiverOverrideFraction = mean(selected);
if any(selected)
    readout.harmfulOverrideFraction = ...
        mean(selectedResidual(selected) < 0);
    readout.meanSelectedResidual = ...
        mean(selectedResidual(selected));
    readout.minimumSelectedResidual = ...
        min(selectedResidual(selected));
else
    readout.harmfulOverrideFraction = 0;
    readout.meanSelectedResidual = 0;
    readout.minimumSelectedResidual = 0;
end
readout.oracleCaptureFraction = ...
    sum(max(selectedResidual, 0)) / ...
    max(sum(oracleResidual), eps);
readout.selectedResidualByReceiver = selectedResidual;
end

function value = conformalUpperQuantile(values, probability)
values = sort(reshape(values(isfinite(values)), [], 1));
if isempty(values)
    value = inf;
    return;
end
idx = min(numel(values), max(1, ...
    ceil(probability * (numel(values) + 1))));
value = values(idx);
end

function paths = normalizePaths(paths)
if ischar(paths)
    paths = {paths};
end
paths = reshape(paths, 1, []);
for idx = 1:numel(paths)
    if ~ischar(paths{idx}) || ~exist(paths{idx}, 'file')
        error('Missing dataset path: %s', paths{idx});
    end
end
end

function assertDisjointDatasetBlocks(trainingPaths, calibrationPaths)
trainingBlocks = datasetBlockIdentities(trainingPaths);
calibrationBlocks = datasetBlockIdentities(calibrationPaths);
if numel(unique(trainingBlocks)) ~= numel(trainingBlocks)
    error('Training scenario/seed/time blocks contain duplicates.');
end
if numel(unique(calibrationBlocks)) ~= numel(calibrationBlocks)
    error('Calibration scenario/seed/time blocks contain duplicates.');
end
if ~isempty(intersect(trainingBlocks, calibrationBlocks))
    error(['Training and calibration scenario/seed/time blocks ', ...
        'must be disjoint.']);
end
end

function identities = datasetBlockIdentities(paths)
identities = cell(size(paths));
for pathIdx = 1:numel(paths)
    loaded = load(paths{pathIdx}, 'datasetMetadata');
    if ~isfield(loaded, 'datasetMetadata') || ...
            ~isfield(loaded.datasetMetadata, 'presetName') || ...
            ~isfield(loaded.datasetMetadata, 'seed') || ...
            ~isfield(loaded.datasetMetadata, 'snapshotTime')
        error('Dataset block identity is incomplete: %s', ...
            paths{pathIdx});
    end
    metadata = loaded.datasetMetadata;
    identities{pathIdx} = sprintf('%s|%d|%d', ...
        char(metadata.presetName), round(metadata.seed), ...
        round(metadata.snapshotTime));
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
