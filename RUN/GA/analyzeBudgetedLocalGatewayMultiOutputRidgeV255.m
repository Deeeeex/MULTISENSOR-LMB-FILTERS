function [reportPath, summary] = ...
        analyzeBudgetedLocalGatewayMultiOutputRidgeV255(options)
% ANALYZEBUDGETEDLOCALGATEWAYMULTIOUTPUTRIDGEV255 Seed-robust vector model.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getBudgetedLocalGatewayRepairV255Protocol();
source = getIndependentM24GatewayTeacherV252Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
checkpointPaths = getField(options, 'checkpointPaths', ...
    defaultCheckpointPaths(repoRoot, source, protocol.trainingSeeds));
if ischar(checkpointPaths)
    checkpointPaths = {checkpointPaths};
end
if ~iscell(checkpointPaths) || ...
        numel(checkpointPaths) ~= numel(protocol.trainingSeeds)
    error('BudgetedLocalGatewayRepairV255:ModelCheckpointList', ...
        'V255 model selection requires all three development seeds.');
end
outputRoot = char(getField(options, 'outputRoot', ...
    fullfile(repoRoot, protocol.outputRoot, 'multi_output_ridge')));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
writeReport = logical(getField(options, 'writeReport', true));
writeMat = logical(getField(options, 'writeMat', true));

gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('BudgetedLocalGatewayRepairV255:DirtyModelSource', ...
        'Official V255 model selection requires clean source.');
end
[datasets, featureNames, sourceCommits] = ...
    loadDatasets(checkpointPaths, source, protocol);

configurations = repmat(emptyConfiguration(), 1, 0);
best = emptyConfiguration();
bestKey = -Inf(1, 11);
for lambda = protocol.ridgeLambdaGrid
    for threshold = protocol.receiverRmseActivationThresholdGrid
        evaluation = evaluateCrossSeed( ...
            datasets, lambda, threshold, featureNames, protocol);
        config = emptyConfiguration();
        config.ridgeLambda = lambda;
        config.receiverRmseActivationThreshold = threshold;
        config.trainingEvaluation = evaluation;
        config.selectionKey = buildSelectionKey(evaluation, config);
        configurations(end + 1) = config; %#ok<AGROW>
        if lexicographicallyGreater(config.selectionKey, bestKey)
            best = config;
            bestKey = config.selectionKey;
        end
    end
end

allRecords = recordsForSeeds(datasets, protocol.trainingSeeds);
model = fitSeedEnsemble(allRecords, protocol.trainingSeeds, ...
    best.ridgeLambda, featureNames, protocol);
model.receiverRmseActivationThreshold = ...
    best.receiverRmseActivationThreshold;

summary = struct();
summary.contractVersion = ...
    'budgeted-local-gateway-multi-output-ridge-v255-summary-v1';
summary.protocol = protocol;
summary.analysisGitCommit = gitState.commit;
summary.sourceGenerationGitCommits = sourceCommits;
summary.checkpointPaths = checkpointPaths;
summary.featureNames = featureNames;
summary.outcomeNames = protocol.predictedOutcomeNames;
summary.configurationCount = numel(configurations);
summary.configurations = configurations;
summary.selectedConfiguration = best;
summary.model = model;
summary.seed1306TeacherAuthorized = ...
    best.trainingEvaluation.gatePassed;
summary.seed1306ModelEvaluationAuthorized = ...
    best.trainingEvaluation.gatePassed;
summary.completeEpisodeM24Authorized = false;
summary.gnnAuthorized = false;
summary.x36Authorized = false;
summary.validationClaimAllowed = false;
summary.developmentEvidenceOnly = true;
if summary.seed1306TeacherAuthorized
    summary.nextDecision = ...
        'freeze-model-and-generate-single-arc-seed1306-h3-holdout';
else
    summary.nextDecision = ...
        'revise-local-observable-features-before-new-holdout';
end
summary.completedAt = datestr(now, 31);
summary.evidenceBoundary = protocol.evidenceBoundary;

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'V255_MULTI_OUTPUT_RIDGE_MODEL_SELECTION.md');
matPath = fullfile(outputRoot, ...
    'V255_MULTI_OUTPUT_RIDGE_MODEL_SELECTION.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
if writeMat
    save('-mat7-binary', matPath, 'summary');
end
if writeReport
    writeReportFile(reportPath, summary);
end
fprintf(['V255 multi-output ridge complete: lambda %.6g threshold %.3f; ', ...
    'folds passed %d/%d; gate=%d; next=%s\n'], ...
    best.ridgeLambda, best.receiverRmseActivationThreshold, ...
    best.trainingEvaluation.foldPassedCount, ...
    numel(best.trainingEvaluation.folds), ...
    best.trainingEvaluation.gatePassed, summary.nextDecision);
fprintf('V255 report: %s\n', reportPath);
end

function paths = defaultCheckpointPaths(repoRoot, source, seeds)
paths = cell(1, numel(seeds));
for seedIdx = 1:numel(seeds)
    paths{seedIdx} = fullfile(repoRoot, source.outputRoot, ...
        sprintf('seed%d', seeds(seedIdx)), source.oracleMatName);
end
end

function [datasets, featureNames, commits] = ...
        loadDatasets(paths, source, protocol)
datasets = cell(1, numel(paths));
featureNames = {};
commits = cell(1, numel(paths));
seenSeeds = zeros(1, numel(paths));
for pathIdx = 1:numel(paths)
    loaded = load(char(paths{pathIdx}), 'result');
    if ~isfield(loaded, 'result')
        error('BudgetedLocalGatewayRepairV255:MissingModelResult', ...
            'A V252 model checkpoint has no result envelope.');
    end
    oracle = loaded.result;
    validateOracle(oracle, source, protocol.trainingSeeds);
    seenSeeds(pathIdx) = oracle.seed;
    commits{pathIdx} = oracle.generationGitCommit;
    cache = load(oracle.cachePath, 'behaviorBundle');
    bundle = cache.behaviorBundle;
    sensorsPerFormation = bundle.configSnapshot.sensorsPerFormation;
    controlBytes = localControlCost(protocol, sensorsPerFormation);
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
        [candidateIndices, receiverFormationIndices] = ...
            selectSingleArcCandidates(window, identity);
        assignments = window.candidateAssignments(candidateIndices);
        [features, currentNames, featureDetails] = ...
            computeBudgetedLocalGatewayActionFeaturesV255( ...
                context, assignments, ...
                window.candidateAssignments{ ...
                    window.referenceCandidateIndex});
        if isempty(featureNames)
            featureNames = currentNames;
        elseif ~isequal(featureNames, currentNames)
            error('BudgetedLocalGatewayRepairV255:ActionFeatureDrift', ...
                'The local action feature contract changed across windows.');
        end
        if featureDetails.featureCount ~= 32
            error('BudgetedLocalGatewayRepairV255:ActionFeatureCount', ...
                'The compact candidate/incumbent action pair must have 32 features.');
        end
        targets = buildTargets(window, candidateIndices, ...
            receiverFormationIndices, controlBytes);
        record = struct();
        record.seed = oracle.seed;
        record.anchorTime = window.anchorTime;
        record.features = features;
        record.targets = targets;
        record.candidateIndices = candidateIndices;
        record.receiverFormationIndices = receiverFormationIndices;
        record.controlAttemptedBytes = controlBytes;
        record.window = window;
        records{windowIdx} = record;
    end
    datasets{pathIdx} = struct( ...
        'seed', oracle.seed, 'records', {records});
end
if ~isequal(sort(seenSeeds), sort(protocol.trainingSeeds))
    error('BudgetedLocalGatewayRepairV255:ModelSeedCoverage', ...
        'The V255 model data do not cover the frozen development seeds.');
end
end

function validateOracle(oracle, source, seeds)
valid = isstruct(oracle) && isscalar(oracle) && ...
    isfield(oracle, 'contractVersion') && ...
    strcmp(oracle.contractVersion, source.resultContractVersion) && ...
    isfield(oracle, 'protocolId') && strcmp(oracle.protocolId, source.id) && ...
    isfield(oracle, 'seed') && ismember(oracle.seed, seeds) && ...
    isfield(oracle, 'windows') && ...
    numel(oracle.windows) == numel(source.anchorTimes) && ...
    isfield(oracle, 'generationGitCommit') && ...
    ischar(oracle.generationGitCommit);
if ~valid
    error('BudgetedLocalGatewayRepairV255:InvalidModelOracle', ...
        'A V255 development checkpoint is incomplete or incompatible.');
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

function [indices, receiverFormationIndices] = ...
        selectSingleArcCandidates(window, identity)
reference = normalizeAssignment( ...
    window.candidateAssignments{window.referenceCandidateIndex});
indices = zeros(1, 0);
receiverFormationIndices = zeros(1, 0);
for candidateIdx = 1:window.candidateCount
    if candidateIdx == window.referenceCandidateIndex
        continue;
    end
    candidate = normalizeAssignment( ...
        window.candidateAssignments{candidateIdx});
    changed = find(any(candidate(:, 3:4) ~= reference(:, 3:4), 2));
    if numel(changed) ~= 1
        continue;
    end
    receiverUid = candidate(changed, 2);
    receiverFormationIdx = find( ...
        identity.formationPhysicalUids == receiverUid, 1);
    if isempty(receiverFormationIdx)
        error('BudgetedLocalGatewayRepairV255:UnknownReceiverFormation', ...
            'A local action references an unknown receiving formation.');
    end
    indices(end + 1) = candidateIdx; %#ok<AGROW>
    receiverFormationIndices(end + 1) = ...
        receiverFormationIdx; %#ok<AGROW>
end
if isempty(indices)
    error('BudgetedLocalGatewayRepairV255:NoSingleArcCandidates', ...
        'A development window has no one-arc gateway replacement.');
end
end

function targets = buildTargets( ...
        window, candidateIndices, receiverFormationIndices, controlBytes)
targets = zeros(numel(candidateIndices), 8);
reference = window.arms{window.referenceCandidateIndex};
for localIdx = 1:numel(candidateIndices)
    candidateIdx = candidateIndices(localIdx);
    comparison = window.comparisons{candidateIdx};
    candidate = window.arms{candidateIdx};
    receiverFormationIdx = receiverFormationIndices(localIdx);
    targets(localIdx, :) = [ ...
        comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, ...
        lowerGain(reference.attemptedPayloadBytes, ...
            candidate.attemptedPayloadBytes + controlBytes), ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent, ...
        comparison.formationEospaGainPercent(receiverFormationIdx), ...
        comparison.formationRmseGainPercent(receiverFormationIdx)];
end
end

function evaluation = evaluateCrossSeed( ...
        datasets, lambda, threshold, featureNames, protocol)
folds = repmat(emptyEvaluation(), 1, numel(protocol.trainingSeeds));
allSelected = cell(1, 0);
allReference = cell(1, 0);
allControl = zeros(1, 0);
safeCount = 0;
for foldIdx = 1:numel(protocol.trainingSeeds)
    heldSeed = protocol.trainingSeeds(foldIdx);
    trainSeeds = setdiff(protocol.trainingSeeds, heldSeed);
    trainRecords = recordsForSeeds(datasets, trainSeeds);
    testRecords = recordsForSeeds(datasets, heldSeed);
    model = fitSeedEnsemble( ...
        trainRecords, trainSeeds, lambda, featureNames, protocol);
    fold = evaluateRecords(model, testRecords, threshold, protocol);
    fold.mode = sprintf('leave-seed-%d-out', heldSeed);
    folds(foldIdx) = fold;
    allSelected = [allSelected, fold.selectedArms]; %#ok<AGROW>
    allReference = [allReference, fold.referenceArms]; %#ok<AGROW>
    allControl = [allControl, fold.controlBytes]; %#ok<AGROW>
    safeCount = safeCount + fold.safePositiveCount;
end
aggregate = compareAggregate( ...
    aggregateArms(allSelected, allControl), ...
    aggregateArms(allReference, zeros(size(allControl))), protocol);
evaluation = struct();
evaluation.mode = 'leave-one-development-seed-out';
evaluation.folds = folds;
evaluation.foldPassedCount = sum([folds.gatePassed]);
evaluation.safePositiveCount = safeCount;
evaluation.aggregate = aggregate;
evaluation.minimumFoldJointScorePercent = min(arrayfun( ...
    @(value) value.aggregate.jointScorePercent, folds));
evaluation.meanFoldReceiverRmseGainPercent = mean(arrayfun( ...
    @(value) value.meanSelectedReceiverRmseGainPercent, folds));
evaluation.gatePassed = evaluation.foldPassedCount >= ...
        protocol.minimumPassingTrainingFoldCount && ...
    aggregate.jointPositive;
end

function model = fitSeedEnsemble( ...
        records, seeds, lambda, featureNames, protocol)
members = repmat(emptyRidgeMember(), 1, numel(seeds));
for seedIdx = 1:numel(seeds)
    seedRecords = selectRecordsBySeed(records, seeds(seedIdx));
    [features, targets] = stackRows(seedRecords);
    members(seedIdx) = fitRidgeMember( ...
        features, targets, seeds(seedIdx), lambda);
end
model = struct();
model.contractVersion = ...
    'budgeted-local-gateway-multi-output-ridge-v255-model-v1';
model.featureNames = featureNames;
model.outcomeNames = protocol.predictedOutcomeNames;
model.members = members;
model.memberTrainingSeeds = seeds;
model.ridgeLambda = lambda;
model.receiverRmseActivationThreshold = 0;
model.maximumChangedDirectedGatewayArcs = 1;
model.communicationCreditFraction = protocol.communicationCreditFraction;
model.predictionAggregation = 'coordinate-wise-minimum-across-seed-models';
model.primaryObjectiveName = protocol.primaryObjectiveName;
model.truthUsed = false;
model.futureInformationUsed = false;
end

function member = fitRidgeMember(features, targets, seed, lambda)
xMean = mean(features, 1);
centered = bsxfun(@minus, features, xMean);
xScale = sqrt(mean(centered .^ 2, 1));
active = xScale > 1e-9;
if ~any(active)
    error('BudgetedLocalGatewayRepairV255:DegenerateActionFeatures', ...
        'A seed-specific V255 model has no varying action feature.');
end
xScale(~active) = 1;
standard = bsxfun(@rdivide, centered(:, active), xScale(active));
yMean = mean(targets, 1);
centeredTargets = bsxfun(@minus, targets, yMean);
coefficients = (standard' * standard + ...
    lambda * eye(nnz(active))) \ (standard' * centeredTargets);
member = emptyRidgeMember();
member.trainingSeed = seed;
member.featureMean = xMean;
member.featureScale = xScale;
member.activeFeatureMask = active;
member.targetMean = yMean;
member.coefficients = coefficients;
member.trainingRowCount = size(features, 1);
end

function predictions = predictEnsemble(model, features)
memberCount = numel(model.members);
outcomeCount = numel(model.outcomeNames);
predictions = zeros(size(features, 1), outcomeCount, memberCount);
for memberIdx = 1:memberCount
    member = model.members(memberIdx);
    centered = bsxfun(@minus, features, member.featureMean);
    standard = bsxfun(@rdivide, ...
        centered(:, member.activeFeatureMask), ...
        member.featureScale(member.activeFeatureMask));
    predictions(:, :, memberIdx) = bsxfun(@plus, ...
        standard * member.coefficients, member.targetMean);
end
end

function evaluation = evaluateRecords(model, records, threshold, protocol)
rows = repmat(emptySelectionRow(), 1, numel(records));
selectedArms = cell(1, numel(records));
referenceArms = cell(1, numel(records));
controlBytes = zeros(1, numel(records));
safePositiveCount = 0;
receiverRmseGains = zeros(1, numel(records));
for recordIdx = 1:numel(records)
    record = records{recordIdx};
    predictions = predictEnsemble(model, record.features);
    lower = min(predictions, [], 3);
    admissible = lower(:, 1) > 0 & lower(:, 2) > 0 & ...
        lower(:, 3) > 0 & ...
        lower(:, 4) >= ...
            -protocol.developmentIncrementalByteCapPercentRelativeV242 & ...
        lower(:, 5) >= -protocol.maximumFormationRegressionPercent & ...
        lower(:, 6) >= -protocol.maximumFormationRegressionPercent & ...
        lower(:, 7) >= -protocol.maximumFormationRegressionPercent & ...
        lower(:, 8) > threshold;
    scores = lower(:, 8);
    scores(~admissible) = -Inf;
    [bestScore, localIdx] = max(scores);
    if ~isfinite(bestScore)
        selectedCandidateIdx = record.window.referenceCandidateIndex;
        selectedLocalIdx = 0;
        selectedTarget = zeros(1, 8);
        fallback = true;
    else
        selectedCandidateIdx = record.candidateIndices(localIdx);
        selectedLocalIdx = localIdx;
        selectedTarget = record.targets(localIdx, :);
        fallback = false;
    end
    realizedSafe = ~fallback && isOutcomeSafe( ...
        selectedTarget, protocol, threshold);
    row = emptySelectionRow();
    row.seed = record.seed;
    row.anchorTime = record.anchorTime;
    row.selectedCandidateIndex = selectedCandidateIdx;
    row.selectedLocalCandidateIndex = selectedLocalIdx;
    row.fallbackUsed = fallback;
    row.predictedLowerReceiverRmseGainPercent = ...
        finiteOrZero(bestScore);
    row.realizedOutcomes = selectedTarget;
    row.safePositive = realizedSafe;
    rows(recordIdx) = row;
    safePositiveCount = safePositiveCount + realizedSafe;
    receiverRmseGains(recordIdx) = selectedTarget(8);
    selectedArms{recordIdx} = ...
        record.window.arms{selectedCandidateIdx};
    referenceArms{recordIdx} = record.window.arms{ ...
        record.window.referenceCandidateIndex};
    controlBytes(recordIdx) = record.controlAttemptedBytes;
end
aggregate = compareAggregate( ...
    aggregateArms(selectedArms, controlBytes), ...
    aggregateArms(referenceArms, zeros(size(controlBytes))), protocol);
evaluation = emptyEvaluation();
evaluation.mode = 'fixed-seed-ensemble';
evaluation.rows = rows;
evaluation.selectedArms = selectedArms;
evaluation.referenceArms = referenceArms;
evaluation.controlBytes = controlBytes;
evaluation.safePositiveCount = safePositiveCount;
evaluation.meanSelectedReceiverRmseGainPercent = ...
    mean(receiverRmseGains);
evaluation.aggregate = aggregate;
evaluation.gatePassed = safePositiveCount >= ...
        protocol.minimumSafeSelectionsPerFold && ...
    aggregate.jointPositive;
end

function safe = isOutcomeSafe(outcome, protocol, threshold)
safe = outcome(1) > 0 && outcome(2) > 0 && outcome(3) > 0 && ...
    outcome(4) >= ...
        -protocol.developmentIncrementalByteCapPercentRelativeV242 && ...
    outcome(5) >= -protocol.maximumFormationRegressionPercent && ...
    outcome(6) >= -protocol.maximumFormationRegressionPercent && ...
    outcome(7) >= -protocol.maximumFormationRegressionPercent && ...
    outcome(8) > threshold;
end

function records = recordsForSeeds(datasets, seeds)
records = cell(1, 0);
for datasetIdx = 1:numel(datasets)
    dataset = datasets{datasetIdx};
    if ismember(dataset.seed, seeds)
        records = [records, dataset.records]; %#ok<AGROW>
    end
end
end

function selected = selectRecordsBySeed(records, seed)
selected = cell(1, 0);
for recordIdx = 1:numel(records)
    if records{recordIdx}.seed == seed
        selected{end + 1} = records{recordIdx}; %#ok<AGROW>
    end
end
if isempty(selected)
    error('BudgetedLocalGatewayRepairV255:MissingSeedRecords', ...
        'A seed-specific ridge member has no training records.');
end
end

function [features, targets] = stackRows(records)
features = zeros(0, size(records{1}.features, 2));
targets = zeros(0, size(records{1}.targets, 2));
for recordIdx = 1:numel(records)
    features = [features; records{recordIdx}.features]; %#ok<AGROW>
    targets = [targets; records{recordIdx}.targets]; %#ok<AGROW>
end
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
aggregate.totalAttemptedBytes = ...
    aggregate.posteriorAttemptedBytes + aggregate.controlAttemptedBytes;
formationCount = numel(arms{1}.perFormationPositionEospa);
formationEospa = zeros(numel(arms), formationCount);
formationRmse = zeros(numel(arms), formationCount);
for armIdx = 1:numel(arms)
    formationEospa(armIdx, :) = arms{armIdx}.perFormationPositionEospa;
    formationRmse(armIdx, :) = arms{armIdx}.perFormationPositionRmse;
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
comparison.totalByteSavingPercent = lowerGain( ...
    reference.totalAttemptedBytes, candidate.totalAttemptedBytes);
comparison.minimumFormationEospaGainPercent = min(lowerGainVector( ...
    reference.perFormationPositionEospa, ...
    candidate.perFormationPositionEospa));
comparison.minimumFormationRmseGainPercent = min(lowerGainVector( ...
    reference.perFormationPositionRmse, ...
    candidate.perFormationPositionRmse));
comparison.jointScorePercent = min([comparison.eospaGainPercent, ...
    comparison.rmseGainPercent, comparison.consistencyGainPercent]);
comparison.candidatePosteriorAttemptedBytes = ...
    candidate.posteriorAttemptedBytes;
comparison.candidateControlAttemptedBytes = ...
    candidate.controlAttemptedBytes;
comparison.candidateTotalAttemptedBytes = candidate.totalAttemptedBytes;
comparison.referenceAttemptedBytes = reference.totalAttemptedBytes;
comparison.jointPositive = comparison.jointScorePercent > 0 && ...
    comparison.totalByteSavingPercent >= ...
        -protocol.developmentIncrementalByteCapPercentRelativeV242 && ...
    comparison.minimumFormationEospaGainPercent >= ...
        -protocol.maximumFormationRegressionPercent && ...
    comparison.minimumFormationRmseGainPercent >= ...
        -protocol.maximumFormationRegressionPercent;
end

function key = buildSelectionKey(evaluation, config)
key = [evaluation.foldPassedCount, evaluation.gatePassed, ...
    evaluation.safePositiveCount, ...
    evaluation.minimumFoldJointScorePercent, ...
    evaluation.aggregate.jointScorePercent, ...
    evaluation.aggregate.rmseGainPercent, ...
    evaluation.meanFoldReceiverRmseGainPercent, ...
    evaluation.aggregate.totalByteSavingPercent, ...
    evaluation.aggregate.minimumFormationRmseGainPercent, ...
    config.receiverRmseActivationThreshold, ...
    log10(config.ridgeLambda)];
end

function writeReportFile(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('BudgetedLocalGatewayRepairV255:ModelReportOpen', ...
        'Could not write the V255 model-selection report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
selected = summary.selectedConfiguration;
evaluation = selected.trainingEvaluation;
fprintf(fid, '# V255 multi-output ridge model selection\n\n');
fprintf(fid, '- Analysis source commit: `%s`\n', ...
    summary.analysisGitCommit);
fprintf(fid, '- V252 source commits: `%s`\n', ...
    strjoin(unique(summary.sourceGenerationGitCommits), ', '));
fprintf(fid, '- Action features / outcomes: `%d / %d`\n', ...
    numel(summary.featureNames), numel(summary.outcomeNames));
fprintf(fid, '- Ridge lambda: `%.6g`\n', selected.ridgeLambda);
fprintf(fid, '- Receiver-RMSE activation threshold: `%.3f%%`\n', ...
    selected.receiverRmseActivationThreshold);
fprintf(fid, '- Passing held-seed folds: `%d/%d`\n', ...
    evaluation.foldPassedCount, numel(evaluation.folds));
fprintf(fid, '- Cross-seed gate: `%d`\n', evaluation.gatePassed);
fprintf(fid, '- Next decision: `%s`\n\n', summary.nextDecision);

fprintf(fid, ['The model contains one ridge member per training seed. ', ...
    'Each outcome coordinate uses the minimum prediction across members; ', ...
    'an action is accepted only when all registered lower-envelope ', ...
    'constraints pass. The selected objective is receiving-formation ', ...
    'RMSE gain, while network RMSE remains a positive constraint.\n\n']);

fprintf(fid, ['| Fold | Safe positive | E | RMSE | Consistency | ', ...
    'Total bytes | Weakest formation E / R | Mean receiver RMSE | Pass |\n']);
fprintf(fid, '|:--|--:|--:|--:|--:|--:|:--|--:|:--:|\n');
for foldIdx = 1:numel(evaluation.folds)
    fold = evaluation.folds(foldIdx);
    gain = fold.aggregate;
    fprintf(fid, ['| %s | %d/%d | %+.3f%% | %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% | %+.3f%% / %+.3f%% | %+.3f%% | %d |\n'], ...
        fold.mode, fold.safePositiveCount, numel(fold.rows), ...
        gain.eospaGainPercent, gain.rmseGainPercent, ...
        gain.consistencyGainPercent, gain.totalByteSavingPercent, ...
        gain.minimumFormationEospaGainPercent, ...
        gain.minimumFormationRmseGainPercent, ...
        fold.meanSelectedReceiverRmseGainPercent, fold.gatePassed);
end
gain = evaluation.aggregate;
fprintf(fid, ['\nCross-seed aggregate: E `%+.3f%%`, RMSE `%+.3f%%`, ', ...
    'consistency `%+.3f%%`, total-byte saving `%+.3f%%`, weakest ', ...
    'formation E/R `%+.3f%% / %+.3f%%`; safe-positive selections ', ...
    '`%d/%d`; gate `%d`.\n\n'], gain.eospaGainPercent, ...
    gain.rmseGainPercent, gain.consistencyGainPercent, ...
    gain.totalByteSavingPercent, gain.minimumFormationEospaGainPercent, ...
    gain.minimumFormationRmseGainPercent, ...
    evaluation.safePositiveCount, ...
    numel(evaluation.folds) * numel(evaluation.folds(1).rows), ...
    evaluation.gatePassed);

fprintf(fid, '## Per-window held-seed decisions\n\n');
fprintf(fid, ['| Fold | Anchor | Selected | Lower receiver RMSE | ', ...
    'Realized E / R / C / B / receiver R | Safe |\n']);
fprintf(fid, '|:--|--:|--:|--:|:--|:--:|\n');
for foldIdx = 1:numel(evaluation.folds)
    fold = evaluation.folds(foldIdx);
    for rowIdx = 1:numel(fold.rows)
        row = fold.rows(rowIdx);
        y = row.realizedOutcomes;
        fprintf(fid, ['| %s | %d | %d | %+.3f%% | ', ...
            '%+.3f / %+.3f / %+.3f / %+.3f / %+.3f | %d |\n'], ...
            fold.mode, row.anchorTime, row.selectedCandidateIndex, ...
            row.predictedLowerReceiverRmseGainPercent, ...
            y(1), y(2), y(3), y(4), y(8), row.safePositive);
    end
end

fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function value = emptyConfiguration()
value = struct('ridgeLambda', 0, ...
    'receiverRmseActivationThreshold', 0, ...
    'trainingEvaluation', struct(), ...
    'selectionKey', zeros(1, 11));
end

function value = emptyEvaluation()
value = struct('mode', '', ...
    'rows', repmat(emptySelectionRow(), 1, 0), ...
    'selectedArms', {cell(1, 0)}, ...
    'referenceArms', {cell(1, 0)}, ...
    'controlBytes', zeros(1, 0), ...
    'safePositiveCount', 0, ...
    'meanSelectedReceiverRmseGainPercent', 0, ...
    'aggregate', struct(), 'gatePassed', false);
end

function value = emptySelectionRow()
value = struct('seed', 0, 'anchorTime', 0, ...
    'selectedCandidateIndex', 0, ...
    'selectedLocalCandidateIndex', 0, ...
    'fallbackUsed', false, ...
    'predictedLowerReceiverRmseGainPercent', 0, ...
    'realizedOutcomes', zeros(1, 8), ...
    'safePositive', false);
end

function value = emptyRidgeMember()
value = struct('trainingSeed', 0, 'featureMean', zeros(1, 0), ...
    'featureScale', zeros(1, 0), ...
    'activeFeatureMask', false(1, 0), ...
    'targetMean', zeros(1, 0), 'coefficients', zeros(0), ...
    'trainingRowCount', 0);
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
    error('BudgetedLocalGatewayRepairV255:InvalidModelAssignment', ...
        'Gateway assignments must be finite nonempty K-by-4 matrices.');
end
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function bytes = localControlCost(protocol, sensorsPerFormation)
bytes = 2 * sensorsPerFormation * ...
    protocol.compactNodeBytesPerSensor + ...
    protocol.routeCommandHeaderBytes + ...
    protocol.routeCommandBytesPerChangedArc;
end

function value = finiteOrZero(value)
if ~isfinite(value)
    value = 0;
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
