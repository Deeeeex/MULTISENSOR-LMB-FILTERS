function [reportPath, summary] = ...
        analyzeScaleEquivariantGatewayRidgeV254(options)
% ANALYZESCALEEQUIVARIANTGATEWAYRIDGEV254 Frozen compact additive screen.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getScaleEquivariantGatewayRidgeV254Protocol();
source = getIndependentM24GatewayTeacherV252Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
checkpointPaths = getField(options, 'checkpointPaths', ...
    defaultCheckpointPaths(repoRoot, source));
if ischar(checkpointPaths)
    checkpointPaths = {checkpointPaths};
end
if ~iscell(checkpointPaths) || ...
        numel(checkpointPaths) ~= numel(source.allowedSeeds)
    error('ScaleEquivariantGatewayV254:CheckpointList', ...
        'V254 requires one V252 checkpoint per registered teacher seed.');
end
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    repoRoot, protocol.outputRoot)));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
writeReport = logical(getField(options, 'writeReport', true));
writeMat = logical(getField(options, 'writeMat', true));
if ~isscalar(writeReport) || ~isscalar(writeMat)
    error('ScaleEquivariantGatewayV254:InvalidWriteRequest', ...
        'V254 write flags must be scalar logical values.');
end

gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('ScaleEquivariantGatewayV254:DirtySource', ...
        'Official V254 analysis requires clean source.');
end
[datasets, featureNames, featureDetails, sourceCommits] = ...
    loadDatasets(checkpointPaths, source, protocol);
compactMask = logical(featureDetails.compactTelemetryMask);
if nnz(compactMask) ~= 16
    error('ScaleEquivariantGatewayV254:CompactFeatureDrift', ...
        'The primary V254 telemetry contract must contain 16 features.');
end

configurations = repmat(emptyConfiguration(), 1, 0);
bestConfiguration = emptyConfiguration();
bestKey = -Inf(1, 11);
for lambda = protocol.ridgeLambdaGrid
    for threshold = protocol.activationThresholdGrid
        evaluation = evaluateTrainingCrossSeed( ...
            datasets, compactMask, lambda, threshold, protocol, ...
            featureNames);
        configuration = emptyConfiguration();
        configuration.featureSetName = protocol.featureSetName;
        configuration.featureMask = compactMask;
        configuration.featureCount = nnz(compactMask);
        configuration.ridgeLambda = lambda;
        configuration.activationThreshold = threshold;
        configuration.trainingEvaluation = evaluation;
        configuration.selectionKey = buildSelectionKey( ...
            evaluation, configuration);
        configurations(end + 1) = configuration; %#ok<AGROW>
        if lexicographicallyGreater( ...
                configuration.selectionKey, bestKey)
            bestConfiguration = configuration;
            bestKey = configuration.selectionKey;
        end
    end
end

trainingRecords = recordsForSeeds( ...
    datasets, protocol.trainingSeeds);
holdoutRecords = recordsForSeeds( ...
    datasets, protocol.developmentHoldoutSeeds);
model = fitUtilityRidge(trainingRecords, compactMask, ...
    bestConfiguration.ridgeLambda, featureNames, protocol);
model.activationThreshold = bestConfiguration.activationThreshold;
holdout = evaluateRecords(model, holdoutRecords, ...
    bestConfiguration.activationThreshold, protocol);
holdout.gatePassed = holdout.safePositiveCount >= ceil( ...
        protocol.minimumHoldoutSafeSelectionFraction * ...
        numel(holdoutRecords)) && ...
    holdout.aggregate.jointPositive;
projection = evaluateExactProjectionCoverage( ...
    model, holdoutRecords, protocol);

summary = struct();
summary.contractVersion = ...
    'scale-equivariant-gateway-ridge-v254-summary-v1';
summary.protocol = protocol;
summary.analysisGitCommit = gitState.commit;
summary.sourceGenerationGitCommits = sourceCommits;
summary.checkpointPaths = checkpointPaths;
summary.featureNames = featureNames;
summary.featureDetails = featureDetails;
summary.configurationCount = numel(configurations);
summary.configurations = configurations;
summary.selectedConfiguration = bestConfiguration;
summary.model = model;
summary.developmentHoldoutEvaluation = holdout;
summary.developmentHoldoutProjectionCoverage = projection;
summary.projectedH3EvaluationAuthorized = holdout.gatePassed;
summary.completeEpisodeM24Authorized = false;
summary.gnnAuthorized = false;
summary.x36Authorized = false;
summary.validationClaimAllowed = false;
summary.developmentEvidenceOnly = true;
if holdout.gatePassed
    summary.nextDecision = ...
        'freeze-model-and-score-exact-projected-actions-on-seed1304-h3';
else
    summary.nextDecision = ...
        'revise-additive-observable-representation-or-stop-before-gnn';
end
summary.completedAt = datestr(now, 31);
summary.evidenceBoundary = protocol.evidenceBoundary;

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'V254_COMPACT_ADDITIVE_GATEWAY_RIDGE.md');
matPath = fullfile(outputRoot, ...
    'V254_COMPACT_ADDITIVE_GATEWAY_RIDGE.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
if writeMat
    save('-mat7-binary', matPath, 'summary');
end
if writeReport
    writeReportFile(reportPath, summary);
end
fprintf(['V254 compact additive ridge complete: lambda %.6g ', ...
    'threshold %.3f; holdout pass=%d; applied projection bank coverage=%d/%d\n'], ...
    bestConfiguration.ridgeLambda, ...
    bestConfiguration.activationThreshold, ...
    holdout.gatePassed, projection.appliedBankMatchedCount, ...
    projection.appliedCount);
fprintf('V254 report: %s\n', reportPath);
end

function paths = defaultCheckpointPaths(repoRoot, source)
paths = cell(1, numel(source.allowedSeeds));
for seedIdx = 1:numel(source.allowedSeeds)
    seed = source.allowedSeeds(seedIdx);
    paths{seedIdx} = fullfile(repoRoot, source.outputRoot, ...
        sprintf('seed%d', seed), source.oracleMatName);
end
end

function [datasets, featureNames, featureDetails, commits] = ...
        loadDatasets(paths, source, protocol)
datasets = cell(1, numel(paths));
featureNames = {};
featureDetails = struct();
commits = cell(1, numel(paths));
seenSeeds = zeros(1, numel(paths));
for pathIdx = 1:numel(paths)
    path = char(paths{pathIdx});
    if exist(path, 'file') ~= 2
        error('ScaleEquivariantGatewayV254:MissingCheckpoint', ...
            'Missing V252 checkpoint: %s', path);
    end
    loaded = load(path, 'result');
    if ~isfield(loaded, 'result')
        error('ScaleEquivariantGatewayV254:MissingResult', ...
            'A V252 checkpoint lacks its result envelope.');
    end
    oracle = loaded.result;
    validateOracle(oracle, source);
    seenSeeds(pathIdx) = oracle.seed;
    commits{pathIdx} = oracle.generationGitCommit;
    cache = load(oracle.cachePath, 'behaviorBundle');
    if ~isfield(cache, 'behaviorBundle')
        error('ScaleEquivariantGatewayV254:MissingCache', ...
            'A V252 reference cache is unavailable.');
    end
    bundle = cache.behaviorBundle;
    inputs = generateDynamicTopologyScenarioInputs( ...
        oracle.presetName, oracle.seed);
    runtimeModel = removeRealizedTargetTruthFromDynamicTopologyModel( ...
        inputs.model);
    identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
    records = cell(1, numel(oracle.windows));
    for windowIdx = 1:numel(oracle.windows)
        window = oracle.windows{windowIdx};
        [posteriors, history] = extractBehaviorContinuationSnapshot( ...
            bundle, window.anchorTime, inputs.config.numberOfSensors);
        context = buildFeatureContext(inputs, runtimeModel, identity, ...
            posteriors, history, window.anchorTime);
        [features, currentNames, currentDetails] = ...
            computeScaleEquivariantGatewayAssignmentFeaturesV254( ...
                context, window.candidateAssignments, ...
                window.candidateAssignments{ ...
                    window.referenceCandidateIndex});
        if isempty(featureNames)
            featureNames = currentNames;
            featureDetails = stableFeatureDetails(currentDetails);
        elseif ~isequal(featureNames, currentNames) || ...
                ~isequal(featureDetails.compactTelemetryMask, ...
                    currentDetails.compactTelemetryMask)
            error('ScaleEquivariantGatewayV254:FeatureDrift', ...
                'The additive feature contract changed across windows.');
        end
        control = estimateScaleEquivariantGatewayControlBytesV254( ...
            context, currentDetails.referenceAssignment, ...
            protocol.controlSynopsisMode);
        controlWindowBytes = protocol.horizonSteps * ...
            control.totalAttemptedBytes;
        [utility, gains] = buildRobustUtility( ...
            window, controlWindowBytes, ...
            protocol.formationRegressionTolerancePercent);
        record = struct();
        record.seed = oracle.seed;
        record.anchorTime = window.anchorTime;
        record.context = context;
        record.features = features;
        record.utility = utility;
        record.gains = gains;
        record.referenceCandidateIndex = ...
            window.referenceCandidateIndex;
        record.changedArcCounts = changedArcCounts( ...
            window.candidateAssignments, ...
            window.candidateAssignments{ ...
                window.referenceCandidateIndex});
        record.controlAttemptedBytes = controlWindowBytes;
        record.window = window;
        records{windowIdx} = record;
    end
    datasets{pathIdx} = struct( ...
        'seed', oracle.seed, 'records', {records}, ...
        'oracle', oracle);
end
if ~isequal(sort(seenSeeds), sort(source.allowedSeeds))
    error('ScaleEquivariantGatewayV254:SeedCoverage', ...
        'The V252 checkpoints do not cover the frozen seed split.');
end
end

function details = stableFeatureDetails(source)
details = struct();
details.contractVersion = source.contractVersion;
details.featureNames = source.featureNames;
details.compactTelemetryMask = source.compactTelemetryMask;
details.compactTelemetryFeatureCount = ...
    source.compactTelemetryFeatureCount;
details.richPairwiseFeatureCount = source.richPairwiseFeatureCount;
details.pairwiseLabelFeatureNames = source.pairwiseLabelFeatureNames;
details.assignmentEmbeddingOperation = ...
    source.assignmentEmbeddingOperation;
details.sensorPermutationEquivariant = ...
    source.sensorPermutationEquivariant;
details.formationPermutationEquivariant = ...
    source.formationPermutationEquivariant;
details.truthUsed = source.truthUsed;
details.futureInformationUsed = source.futureInformationUsed;
end

function validateOracle(oracle, source)
valid = isstruct(oracle) && isscalar(oracle) && ...
    isfield(oracle, 'contractVersion') && ...
    strcmp(oracle.contractVersion, source.resultContractVersion) && ...
    isfield(oracle, 'protocolId') && ...
    strcmp(oracle.protocolId, source.id) && ...
    isfield(oracle, 'presetName') && ...
    ismember(oracle.presetName, source.allowedPresets) && ...
    isfield(oracle, 'seed') && ismember(oracle.seed, source.allowedSeeds) && ...
    isfield(oracle, 'protocol') && ...
    isequal(oracle.protocol.anchorTimes, source.anchorTimes) && ...
    isfield(oracle, 'windows') && ...
    numel(oracle.windows) == numel(source.anchorTimes) && ...
    isfield(oracle, 'generationGitCommit') && ...
    ischar(oracle.generationGitCommit);
if valid
    for windowIdx = 1:numel(oracle.windows)
        window = oracle.windows{windowIdx};
        valid = valid && isstruct(window) && ...
            window.anchorTime == source.anchorTimes(windowIdx) && ...
            numel(window.arms) == window.candidateCount && ...
            numel(window.comparisons) == window.candidateCount;
        for candidateIdx = 1:numel(window.arms)
            arm = window.arms{candidateIdx};
            valid = valid && isstruct(arm) && isscalar(arm) && ...
                isfield(arm, 'completed') && logical(arm.completed) && ...
                isstruct(window.comparisons{candidateIdx});
        end
    end
end
if ~valid
    error('ScaleEquivariantGatewayV254:InvalidOracle', ...
        'A V252 checkpoint is incomplete or belongs to another protocol.');
end
end

function context = buildFeatureContext( ...
        inputs, runtimeModel, identity, posteriors, history, currentTime)
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = runtimeModel;
context.physicalAdjacency = logical( ...
    inputs.graphData.physicalAdjacency(:, :, currentTime));
context.positions = inputs.graphData.positions(:, :, currentTime);
context.commConfig = struct('pDropByEdge', ...
    inputs.commConfig.pDropByEdge(:, :, currentTime));
context.currentTime = currentTime;
context.previousAdjacencyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.selectedDirectedEdgeHistory);
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
end

function [utility, gains] = ...
        buildRobustUtility(window, controlBytes, tolerance)
gains = zeros(window.candidateCount, 6);
utility = zeros(window.candidateCount, 1);
reference = window.arms{window.referenceCandidateIndex};
for candidateIdx = 1:window.candidateCount
    comparison = window.comparisons{candidateIdx};
    candidate = window.arms{candidateIdx};
    gains(candidateIdx, :) = [ ...
        comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, ...
        lowerGain(reference.attemptedPayloadBytes, ...
            candidate.attemptedPayloadBytes + controlBytes), ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent];
    utility(candidateIdx) = min([gains(candidateIdx, 1:4), ...
        gains(candidateIdx, 5:6) + tolerance]);
end
utility(window.referenceCandidateIndex) = 0;
end

function counts = changedArcCounts(assignments, reference)
reference = normalizeAssignment(reference);
counts = zeros(numel(assignments), 1);
for candidateIdx = 1:numel(assignments)
    candidate = normalizeAssignment(assignments{candidateIdx});
    counts(candidateIdx) = nnz(any( ...
        candidate(:, 3:4) ~= reference(:, 3:4), 2));
end
end

function records = recordsForSeeds(datasets, seeds)
records = cell(1, 0);
for datasetIdx = 1:numel(datasets)
    dataset = datasets{datasetIdx};
    if ismember(dataset.seed, seeds)
        records = [records, dataset.records]; %#ok<AGROW>
    end
end
expected = numel(seeds) * numel(datasets{1}.records);
if numel(records) ~= expected
    error('ScaleEquivariantGatewayV254:RecordCoverage', ...
        'The requested seed split has incomplete V252 records.');
end
end

function evaluation = evaluateTrainingCrossSeed( ...
        datasets, featureMask, lambda, threshold, protocol, featureNames)
folds = repmat(emptyEvaluation(), 1, numel(protocol.trainingSeeds));
allRows = repmat(emptyRow(), 1, 0);
selectedArms = cell(1, 0);
referenceArms = cell(1, 0);
selectedControlBytes = zeros(1, 0);
for foldIdx = 1:numel(protocol.trainingSeeds)
    heldSeed = protocol.trainingSeeds(foldIdx);
    trainSeeds = setdiff(protocol.trainingSeeds, heldSeed);
    trainRecords = recordsForSeeds(datasets, trainSeeds);
    testRecords = recordsForSeeds(datasets, heldSeed);
    model = fitUtilityRidge( ...
        trainRecords, featureMask, lambda, featureNames, protocol);
    fold = evaluateRecords(model, testRecords, threshold, protocol);
    fold.mode = sprintf('leave-seed-%d-out', heldSeed);
    folds(foldIdx) = fold;
    allRows = [allRows, fold.rows]; %#ok<AGROW>
    selectedArms = [selectedArms, fold.selectedArms]; %#ok<AGROW>
    referenceArms = [referenceArms, fold.referenceArms]; %#ok<AGROW>
    selectedControlBytes = [selectedControlBytes, ...
        fold.selectedControlBytes]; %#ok<AGROW>
end
aggregate = compareAggregate( ...
    aggregateArms(selectedArms, selectedControlBytes), ...
    aggregateArms(referenceArms, zeros(size(selectedControlBytes))), ...
    protocol);
evaluation = struct();
evaluation.mode = 'leave-one-training-seed-out';
evaluation.folds = folds;
evaluation.rows = allRows;
evaluation.selectedArms = selectedArms;
evaluation.referenceArms = referenceArms;
evaluation.selectedControlBytes = selectedControlBytes;
evaluation.aggregate = aggregate;
evaluation.safePositiveCount = sum([folds.safePositiveCount]);
evaluation.foldPassedCount = sum([folds.gatePassed]);
evaluation.minimumFoldJointScorePercent = min(arrayfun( ...
    @(value) value.aggregate.jointScorePercent, folds));
evaluation.meanSelectedUtility = mean([allRows.realizedUtility]);
evaluation.gatePassed = ...
    evaluation.foldPassedCount == numel(folds) && ...
    aggregate.jointPositive;
end

function model = fitUtilityRidge( ...
        records, featureMask, lambda, featureNames, protocol)
[features, targets] = stackRelativeTrainingRows(records, featureMask);
featureScale = sqrt(mean(features .^ 2, 1));
activeSelected = featureScale > 1e-9;
if ~any(activeSelected)
    error('ScaleEquivariantGatewayV254:DegenerateFeatures', ...
        'The training split has no varying compact additive feature.');
end
featureScale(~activeSelected) = 1;
standard = bsxfun(@rdivide, features(:, activeSelected), ...
    featureScale(activeSelected));
coefficients = (standard' * standard + ...
    lambda * eye(nnz(activeSelected))) \ (standard' * targets);
fullScale = ones(1, numel(featureNames));
fullScale(featureMask) = featureScale;
selectedIndices = find(featureMask);
fullActive = false(1, numel(featureNames));
fullActive(selectedIndices(activeSelected)) = true;
model = struct();
model.contractVersion = ...
    'scale-equivariant-gateway-v254-additive-ridge-model-v1';
model.featureNames = featureNames;
model.featureScale = fullScale;
model.activeFeatureMask = fullActive;
model.coefficients = coefficients;
model.ridgeLambda = lambda;
model.intercept = 0;
model.referenceMapsToZero = true;
model.activationThreshold = 0;
model.calibrationMargin = protocol.calibrationMargin;
model.switchingPenalty = protocol.switchingPenalty;
model.tieTolerance = protocol.tieTolerance;
model.controlSynopsisMode = protocol.controlSynopsisMode;
model.truthUsed = false;
model.futureInformationUsed = false;
end

function [features, targets] = ...
        stackRelativeTrainingRows(records, featureMask)
features = zeros(0, nnz(featureMask));
targets = zeros(0, 1);
for recordIdx = 1:numel(records)
    record = records{recordIdx};
    relative = bsxfun(@minus, record.features(:, featureMask), ...
        record.features(record.referenceCandidateIndex, featureMask));
    features = [features; relative]; %#ok<AGROW>
    targets = [targets; record.utility]; %#ok<AGROW>
end
end

function scores = predictUtility(model, record)
relative = bsxfun(@minus, record.features, ...
    record.features(record.referenceCandidateIndex, :));
active = logical(model.activeFeatureMask);
standard = bsxfun(@rdivide, relative(:, active), ...
    model.featureScale(active));
scores = standard * model.coefficients - ...
    model.switchingPenalty * record.changedArcCounts;
scores(record.referenceCandidateIndex) = 0;
end

function evaluation = evaluateRecords(model, records, threshold, protocol)
rows = repmat(emptyRow(), 1, numel(records));
selectedArms = cell(1, numel(records));
referenceArms = cell(1, numel(records));
selectedControlBytes = zeros(1, numel(records));
safePositiveCount = 0;
for recordIdx = 1:numel(records)
    record = records{recordIdx};
    scores = predictUtility(model, record);
    [bestScore, bestIdx] = max(scores);
    required = threshold + model.calibrationMargin;
    if bestScore <= required
        selectedIdx = record.referenceCandidateIndex;
    else
        selectedIdx = bestIdx;
    end
    realizedUtility = record.utility(selectedIdx);
    gains = record.gains(selectedIdx, :);
    safePositive = selectedIdx ~= record.referenceCandidateIndex && ...
        realizedUtility > 1e-9;
    row = emptyRow();
    row.seed = record.seed;
    row.anchorTime = record.anchorTime;
    row.selectedCandidateIndex = selectedIdx;
    row.candidateType = ...
        record.window.candidateTypes{selectedIdx};
    row.predictedUtility = scores(selectedIdx);
    row.realizedUtility = realizedUtility;
    row.activationMargin = bestScore - required;
    row.fallbackUsed = selectedIdx == record.referenceCandidateIndex;
    row.safePositive = safePositive;
    row.gains = gains;
    row.controlAttemptedBytes = record.controlAttemptedBytes;
    rows(recordIdx) = row;
    safePositiveCount = safePositiveCount + safePositive;
    selectedArms{recordIdx} = record.window.arms{selectedIdx};
    referenceArms{recordIdx} = record.window.arms{ ...
        record.referenceCandidateIndex};
    selectedControlBytes(recordIdx) = record.controlAttemptedBytes;
end
aggregate = compareAggregate( ...
    aggregateArms(selectedArms, selectedControlBytes), ...
    aggregateArms(referenceArms, zeros(size(selectedControlBytes))), ...
    protocol);
minimumSafe = ceil( ...
    protocol.minimumHoldoutSafeSelectionFraction * numel(records));
evaluation = emptyEvaluation();
evaluation.mode = 'fixed-model-candidate-bank';
evaluation.rows = rows;
evaluation.selectedArms = selectedArms;
evaluation.referenceArms = referenceArms;
evaluation.selectedControlBytes = selectedControlBytes;
evaluation.safePositiveCount = safePositiveCount;
evaluation.aggregate = aggregate;
evaluation.gatePassed = safePositiveCount >= minimumSafe && ...
    aggregate.jointPositive;
end

function projection = evaluateExactProjectionCoverage(model, records, protocol)
rows = repmat(emptyProjectionRow(), 1, numel(records));
bankMatchedCount = 0;
appliedBankMatchedCount = 0;
appliedCount = 0;
for recordIdx = 1:numel(records)
    record = records{recordIdx};
    reference = record.window.candidateAssignments{ ...
        record.referenceCandidateIndex};
    [edgeValues, ~] = scoreScaleEquivariantGatewayEdgesV254( ...
        record.context, reference, model);
    [assignment, details] = ...
        projectScaleEquivariantGatewayAssignmentV254( ...
            record.context, edgeValues, reference, struct( ...
                'switchingPenalty', model.switchingPenalty, ...
                'tieTolerance', model.tieTolerance));
    predicted = details.candidatePredictedAdvantage;
    apply = ~details.projectionFallbackUsed && ...
        ~isequal(normalizeAssignment(assignment), ...
            normalizeAssignment(reference)) && ...
        predicted > model.activationThreshold + ...
            model.calibrationMargin;
    if ~apply
        assignment = reference;
    end
    matched = findAssignment( ...
        record.window.candidateAssignments, assignment);
    row = emptyProjectionRow();
    row.seed = record.seed;
    row.anchorTime = record.anchorTime;
    row.applied = apply;
    row.predictedAdvantage = predicted;
    row.changedArcCount = details.changedArcCount;
    row.projectionFallbackUsed = details.projectionFallbackUsed;
    row.bankCandidateIndex = matched;
    row.bankMatched = matched > 0;
    rows(recordIdx) = row;
    bankMatchedCount = bankMatchedCount + row.bankMatched;
    appliedBankMatchedCount = appliedBankMatchedCount + ...
        (row.applied && row.bankMatched);
    appliedCount = appliedCount + apply;
end
projection = struct();
projection.contractVersion = ...
    'scale-equivariant-gateway-v254-projection-coverage-v1';
projection.mode = 'frozen-holdout-observable-projection-only';
projection.rows = rows;
projection.bankMatchedCount = bankMatchedCount;
projection.appliedBankMatchedCount = appliedBankMatchedCount;
projection.appliedCount = appliedCount;
projection.appliedOutOfBankCount = ...
    appliedCount - appliedBankMatchedCount;
projection.outcomeScored = false;
projection.requiresPairedH3Outcome = ...
    projection.appliedOutOfBankCount > 0;
projection.evidenceBoundary = [ ...
    'This diagnostic observes only which exact projected assignments ', ...
    'fall inside the frozen V252 bank. Out-of-bank actions have no ', ...
    'tracking outcome until a separate paired H=3 continuation is run.'];
end

function index = findAssignment(assignments, requested)
requested = normalizeAssignment(requested);
index = 0;
for candidateIdx = 1:numel(assignments)
    if isequal(normalizeAssignment(assignments{candidateIdx}), requested)
        index = candidateIdx;
        return;
    end
end
end

function key = buildSelectionKey(evaluation, configuration)
key = [ ...
    evaluation.foldPassedCount, ...
    evaluation.aggregate.jointPositive, ...
    evaluation.safePositiveCount, ...
    evaluation.minimumFoldJointScorePercent, ...
    evaluation.aggregate.jointScorePercent, ...
    evaluation.meanSelectedUtility, ...
    evaluation.aggregate.attemptedByteSavingPercent, ...
    evaluation.aggregate.minimumFormationEospaGainPercent, ...
    evaluation.aggregate.minimumFormationRmseGainPercent, ...
    configuration.activationThreshold, ...
    log10(configuration.ridgeLambda)];
end

function aggregate = aggregateArms(arms, controlBytes)
aggregate = struct();
aggregate.positionEospa = mean(cellfun( ...
    @(value) value.positionEospa, arms));
aggregate.positionRmse = mean(cellfun( ...
    @(value) value.positionRmse, arms));
aggregate.interFormationPositionOspa = mean(cellfun( ...
    @(value) value.interFormationPositionOspa, arms));
aggregate.posteriorAttemptedBytes = sum(cellfun( ...
    @(value) value.attemptedPayloadBytes, arms));
aggregate.controlAttemptedBytes = sum(controlBytes);
aggregate.attemptedPayloadBytes = ...
    aggregate.posteriorAttemptedBytes + aggregate.controlAttemptedBytes;
formationCount = numel(arms{1}.perFormationPositionEospa);
formationEospa = zeros(numel(arms), formationCount);
formationRmse = zeros(numel(arms), formationCount);
for armIdx = 1:numel(arms)
    formationEospa(armIdx, :) = ...
        arms{armIdx}.perFormationPositionEospa;
    formationRmse(armIdx, :) = ...
        arms{armIdx}.perFormationPositionRmse;
end
aggregate.perFormationPositionEospa = mean(formationEospa, 1);
aggregate.perFormationPositionRmse = mean(formationRmse, 1);
end

function comparison = compareAggregate(candidate, reference, protocol)
comparison = struct();
comparison.eospaGainPercent = lowerGain( ...
    reference.positionEospa, candidate.positionEospa);
comparison.rmseGainPercent = lowerGain( ...
    reference.positionRmse, candidate.positionRmse);
comparison.consistencyGainPercent = lowerGain( ...
    reference.interFormationPositionOspa, ...
    candidate.interFormationPositionOspa);
comparison.attemptedByteSavingPercent = lowerGain( ...
    reference.attemptedPayloadBytes, candidate.attemptedPayloadBytes);
comparison.candidatePosteriorAttemptedBytes = ...
    candidate.posteriorAttemptedBytes;
comparison.candidateControlAttemptedBytes = ...
    candidate.controlAttemptedBytes;
comparison.candidateTotalAttemptedBytes = ...
    candidate.attemptedPayloadBytes;
comparison.referenceAttemptedBytes = reference.attemptedPayloadBytes;
comparison.formationEospaGainPercent = lowerGainVector( ...
    reference.perFormationPositionEospa, ...
    candidate.perFormationPositionEospa);
comparison.formationRmseGainPercent = lowerGainVector( ...
    reference.perFormationPositionRmse, ...
    candidate.perFormationPositionRmse);
comparison.minimumFormationEospaGainPercent = ...
    min(comparison.formationEospaGainPercent);
comparison.minimumFormationRmseGainPercent = ...
    min(comparison.formationRmseGainPercent);
comparison.jointScorePercent = min([ ...
    comparison.eospaGainPercent, comparison.rmseGainPercent, ...
    comparison.consistencyGainPercent]);
comparison.jointPositive = comparison.jointScorePercent > 0 && ...
    comparison.attemptedByteSavingPercent >= 0 && ...
    comparison.minimumFormationEospaGainPercent >= ...
        -protocol.formationRegressionTolerancePercent && ...
    comparison.minimumFormationRmseGainPercent >= ...
        -protocol.formationRegressionTolerancePercent;
end

function writeReportFile(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('ScaleEquivariantGatewayV254:ReportOpen', ...
        'Could not write the V254 report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
selected = summary.selectedConfiguration;
training = selected.trainingEvaluation;
holdout = summary.developmentHoldoutEvaluation;
projection = summary.developmentHoldoutProjectionCoverage;
fprintf(fid, '# V254 compact additive gateway ridge\n\n');
fprintf(fid, '- Analysis source commit: `%s`\n', ...
    summary.analysisGitCommit);
fprintf(fid, '- V252 source commits: `%s`\n', ...
    strjoin(unique(summary.sourceGenerationGitCommits), ', '));
fprintf(fid, '- Feature set: `%s` (%d features)\n', ...
    selected.featureSetName, selected.featureCount);
fprintf(fid, '- Control synopsis: `%s`\n', ...
    summary.protocol.controlSynopsisMode);
fprintf(fid, '- Lambda / activation threshold: `%.6g / %.3f`\n', ...
    selected.ridgeLambda, selected.activationThreshold);
fprintf(fid, '- Candidate-bank holdout pass: `%d`\n', ...
    holdout.gatePassed);
fprintf(fid, '- Applied exact-projection bank coverage: `%d/%d`\n', ...
    projection.appliedBankMatchedCount, projection.appliedCount);
fprintf(fid, '- Next decision: `%s`\n\n', summary.nextDecision);

fprintf(fid, '## Deployable objective and accounting\n\n');
fprintf(fid, ['The ridge predicts candidate-minus-V242 robust utility from ', ...
    'the sum of 16 compact edge contributions. Each evaluated policy ', ...
    'window adds its fixed 32-byte-per-sensor synopsis and route command ', ...
    'to attempted bytes, including windows that abstain. The V242 ', ...
    'comparator pays no learned-policy control overhead.\n\n']);

fprintf(fid, '## Training-seed model selection\n\n');
fprintf(fid, ['| Fold | Safe positive | E | RMSE | Consistency | Total ', ...
    'bytes | Weakest formation E / R | Pass |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|:--|:--:|\n');
for foldIdx = 1:numel(training.folds)
    fold = training.folds(foldIdx);
    gain = fold.aggregate;
    fprintf(fid, ['| %s | %d/%d | %+.3f%% | %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% | %+.3f%% / %+.3f%% | %d |\n'], ...
        fold.mode, fold.safePositiveCount, numel(fold.rows), ...
        gain.eospaGainPercent, gain.rmseGainPercent, ...
        gain.consistencyGainPercent, ...
        gain.attemptedByteSavingPercent, ...
        gain.minimumFormationEospaGainPercent, ...
        gain.minimumFormationRmseGainPercent, fold.gatePassed);
end

fprintf(fid, '\n## Seed-1304 frozen candidate-bank holdout\n\n');
fprintf(fid, ['| Anchor | Selected | Type | Predicted / realized utility | ', ...
    'E / RMSE / C / total bytes | Control B | Safe positive |\n']);
fprintf(fid, '|--:|--:|:--|:--|:--|--:|:--:|\n');
for rowIdx = 1:numel(holdout.rows)
    row = holdout.rows(rowIdx);
    gain = row.gains;
    fprintf(fid, ['| %d | %d | %s | %+.3f / %+.3f | ', ...
        '%+.3f%% / %+.3f%% / %+.3f%% / %+.3f%% | %d | %d |\n'], ...
        row.anchorTime, row.selectedCandidateIndex, row.candidateType, ...
        row.predictedUtility, row.realizedUtility, gain(1), gain(2), ...
        gain(3), gain(4), row.controlAttemptedBytes, row.safePositive);
end
gain = holdout.aggregate;
fprintf(fid, ['\nHoldout aggregate: E `%+.3f%%`, RMSE `%+.3f%%`, ', ...
    'consistency `%+.3f%%`, total-byte saving `%+.3f%%`, weakest ', ...
    'formation E/R `%+.3f%% / %+.3f%%`; posterior/control/total ', ...
    'attempted bytes `%d / %d / %d`; safe-positive selections `%d/%d`; ', ...
    'gate `%d`.\n\n'], gain.eospaGainPercent, gain.rmseGainPercent, ...
    gain.consistencyGainPercent, gain.attemptedByteSavingPercent, ...
    gain.minimumFormationEospaGainPercent, ...
    gain.minimumFormationRmseGainPercent, ...
    gain.candidatePosteriorAttemptedBytes, ...
    gain.candidateControlAttemptedBytes, ...
    gain.candidateTotalAttemptedBytes, holdout.safePositiveCount, ...
    numel(holdout.rows), holdout.gatePassed);

fprintf(fid, '## Exact projection coverage\n\n');
fprintf(fid, ['| Anchor | Applied | Predicted advantage | Changed arcs | ', ...
    'Projection fallback | V252 bank index |\n']);
fprintf(fid, '|--:|:--:|--:|--:|:--:|--:|\n');
for rowIdx = 1:numel(projection.rows)
    row = projection.rows(rowIdx);
    fprintf(fid, '| %d | %d | %+.3f | %d | %d | %d |\n', ...
        row.anchorTime, row.applied, row.predictedAdvantage, ...
        row.changedArcCount, row.projectionFallbackUsed, ...
        row.bankCandidateIndex);
end
fprintf(fid, '\n%s\n\n', projection.evidenceBoundary);
fprintf(fid, '## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function value = emptyConfiguration()
value = struct('featureSetName', '', 'featureMask', false(1, 0), ...
    'featureCount', 0, 'ridgeLambda', 0, ...
    'activationThreshold', 0, 'trainingEvaluation', struct(), ...
    'selectionKey', zeros(1, 11));
end

function value = emptyEvaluation()
value = struct('mode', '', 'rows', repmat(emptyRow(), 1, 0), ...
    'selectedArms', {cell(1, 0)}, 'referenceArms', {cell(1, 0)}, ...
    'selectedControlBytes', zeros(1, 0), ...
    'safePositiveCount', 0, 'aggregate', struct(), ...
    'gatePassed', false);
end

function value = emptyRow()
value = struct('seed', 0, 'anchorTime', 0, ...
    'selectedCandidateIndex', 0, 'candidateType', '', ...
    'predictedUtility', 0, 'realizedUtility', 0, ...
    'activationMargin', 0, 'fallbackUsed', false, ...
    'safePositive', false, 'gains', zeros(1, 6), ...
    'controlAttemptedBytes', 0);
end

function value = emptyProjectionRow()
value = struct('seed', 0, 'anchorTime', 0, 'applied', false, ...
    'predictedAdvantage', 0, 'changedArcCount', 0, ...
    'projectionFallbackUsed', false, 'bankCandidateIndex', 0, ...
    'bankMatched', false);
end

function greater = lexicographicallyGreater(left, right)
greater = false;
for valueIdx = 1:numel(left)
    if left(valueIdx) > right(valueIdx) + 1e-12
        greater = true;
        return;
    end
    if left(valueIdx) < right(valueIdx) - 1e-12
        return;
    end
end
end

function assignment = normalizeAssignment(assignment)
if ~isnumeric(assignment) || isempty(assignment) || ...
        size(assignment, 2) ~= 4 || any(~isfinite(assignment(:)))
    error('ScaleEquivariantGatewayV254:InvalidTrainingAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
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
