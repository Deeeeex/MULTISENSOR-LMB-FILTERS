function [reportPath, summary] = ...
        analyzeCausalGatewayEmbeddingRidgeLearnabilityV251(options)
% ANALYZECAUSALGATEWAYEMBEDDINGRIDGELEARNABILITYV251
% Leave-one-anchor-out ridge screen for the V250 paired H=3 arms.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getCausalGatewayEmbeddingV250Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
checkpointPath = char(getField(options, 'checkpointPath', fullfile( ...
    repoRoot, protocol.oracleOutputRoot, ...
    'CAUSAL_GATEWAY_EMBEDDING_V250_H3_ORACLE.mat')));
if ~isAbsolutePath(checkpointPath)
    checkpointPath = fullfile(repoRoot, checkpointPath);
end
outputRoot = char(getField(options, 'outputRoot', fullfile( ...
    repoRoot, 'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v251', 'ridge_learnability', ...
    'm24_seed1301')));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
writeReport = logical(getField(options, 'writeReport', true));
writeMat = logical(getField(options, 'writeMat', true));
ridgeLambda = getField(options, 'ridgeLambda', 1e-6);
if ~isscalar(writeReport) || ~isscalar(writeMat) || ...
        ~isscalar(ridgeLambda) || ~isfinite(ridgeLambda) || ...
        ridgeLambda <= 0
    error('CausalGatewayEmbeddingV251:InvalidAnalysisRequest', ...
        'The V251 ridge analysis request is invalid.');
end
if exist(checkpointPath, 'file') ~= 2
    error('CausalGatewayEmbeddingV251:MissingCheckpoint', ...
        'The completed V250 H=3 checkpoint is required.');
end

loaded = load(checkpointPath, 'result');
if ~isfield(loaded, 'result')
    error('CausalGatewayEmbeddingV251:MissingOracleResult', ...
        'The V250 checkpoint does not contain result.');
end
oracle = loaded.result;
validateOracle(oracle, protocol);
cache = load(oracle.cachePath, 'behaviorBundle');
if ~isfield(cache, 'behaviorBundle')
    error('CausalGatewayEmbeddingV251:MissingReferenceCache', ...
        'The V250 behavior cache is unavailable.');
end
bundle = cache.behaviorBundle;
inputs = generateDynamicTopologyScenarioInputs( ...
    oracle.presetName, oracle.seed);
runtimeModel = removeRealizedTargetTruthFromDynamicTopologyModel( ...
    inputs.model);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
gitState = resolveResearchGitState();

targetNames = { ...
    'network_eospa_gain', ...
    'network_rmse_gain', ...
    'network_consistency_gain', ...
    'attempted_byte_saving', ...
    'minimum_formation_eospa_gain', ...
    'minimum_formation_rmse_gain', ...
    'target_formation_eospa_gain', ...
    'target_formation_rmse_gain'};
windowRecords = cell(1, numel(oracle.windows));
featureNames = {};
featureDetails = struct();
for windowIdx = 1:numel(oracle.windows)
    window = oracle.windows{windowIdx};
    [posteriors, history] = extractBehaviorContinuationSnapshot( ...
        bundle, window.anchorTime, inputs.config.numberOfSensors);
    context = buildFeatureContext( ...
        inputs, runtimeModel, posteriors, history, ...
        window.anchorTime);
    [currentFeatures, currentNames, currentDetails] = ...
        computeCausalGatewayEmbeddingCandidateFeaturesV251( ...
            context, window.candidateAssignments, ...
            window.candidateAssignments{ ...
                window.referenceCandidateIndex}, ...
            identity, struct('candidateTypes', ...
                {window.candidateTypes}));
    if isempty(featureNames)
        featureNames = currentNames;
        featureDetails = currentDetails;
    elseif ~isequal(featureNames, currentNames) || ...
            ~isequal(featureDetails.linkOnlyMask, ...
                currentDetails.linkOnlyMask)
        error('CausalGatewayEmbeddingV251:FeatureDrift', ...
            'The causal feature contract changed across anchors.');
    end
    [targets, targetFormationIdx] = buildTargets(window);
    riskProxy = computeFormationRiskProxy( ...
        posteriors, inputs.config.sensorGroupIds, runtimeModel, ...
        inputs.config.ospaPositionCutoff, targetFormationIdx);
    teacherIdx = selectTeacherCandidate( ...
        window, targetFormationIdx);
    record = struct();
    record.anchorTime = window.anchorTime;
    record.referenceCandidateIndex = window.referenceCandidateIndex;
    record.targetFormationIndex = targetFormationIdx;
    record.teacherCandidateIndex = teacherIdx;
    record.riskProxy = riskProxy;
    record.features = currentFeatures;
    record.targets = targets;
    windowRecords{windowIdx} = record;
end

subsetDefinitions = struct( ...
    'name', {'link-only', 'posterior-rich'}, ...
    'mask', {featureDetails.linkOnlyMask, ...
        true(1, numel(featureNames))});
subsetResults = repmat(emptySubsetResult(), ...
    1, numel(subsetDefinitions));
for subsetIdx = 1:numel(subsetDefinitions)
    subset = subsetDefinitions(subsetIdx);
    if ~any(subset.mask)
        error('CausalGatewayEmbeddingV251:EmptyFeatureSubset', ...
            'A registered feature subset is empty.');
    end
    result = emptySubsetResult();
    result.name = subset.name;
    result.featureCount = nnz(subset.mask);
    result.featureNames = featureNames(subset.mask);
    result.inSample = evaluateFitMode( ...
        oracle, windowRecords, subset.mask, targetNames, ...
        ridgeLambda, 'in-sample');
    result.leaveOneAnchorOut = evaluateFitMode( ...
        oracle, windowRecords, subset.mask, targetNames, ...
        ridgeLambda, 'leave-one-anchor-out');
    result.pairwiseInSample = evaluatePairwiseMode( ...
        oracle, windowRecords, subset.mask, ridgeLambda, ...
        'in-sample');
    result.pairwiseLeaveOneAnchorOut = evaluatePairwiseMode( ...
        oracle, windowRecords, subset.mask, ridgeLambda, ...
        'leave-one-anchor-out');
    subsetResults(subsetIdx) = result;
end

rich = subsetResults(strcmp({subsetResults.name}, 'posterior-rich'));
if rich.pairwiseLeaveOneAnchorOut.gatePassed
    nextDecision = ...
        'expand-frozen-m24-training-windows-before-online-policy';
elseif rich.pairwiseInSample.gatePassed
    nextDecision = ...
        'pairwise-representation-fits-but-more-independent-anchors-are-required';
else
    nextDecision = ...
        'revise-causal-feature-representation-before-more-h3-compute';
end

summary = struct();
summary.contractVersion = ...
    'causal-gateway-embedding-v251-ridge-learnability-v1';
summary.analysisGitCommit = gitState.commit;
summary.oracleGenerationGitCommit = oracle.generationGitCommit;
summary.checkpointPath = checkpointPath;
summary.presetName = oracle.presetName;
summary.seed = oracle.seed;
summary.anchorTimes = protocol.anchorTimes;
summary.ridgeLambda = ridgeLambda;
summary.featureNames = featureNames;
summary.targetNames = targetNames;
summary.featureDetails = featureDetails;
summary.windowRecords = windowRecords;
summary.riskProxySummary = summarizeRiskProxies(windowRecords);
summary.subsetResults = subsetResults;
summary.nextDecision = nextDecision;
summary.ridgeOnlinePolicyAuthorized = false;
summary.trainingWindowExpansionAuthorized = ...
    rich.pairwiseInSample.gatePassed;
summary.gnnAuthorized = false;
summary.truthUsedForTeacherTargets = true;
summary.truthUsedByFeatures = false;
summary.futureInformationUsedByFeatures = false;
summary.developmentEvidenceOnly = true;
summary.validationClaimAllowed = false;
summary.completedAt = datestr(now, 31);
summary.evidenceBoundary = [ ...
    'V251 reuses all completed paired V250 H=3 arms from one M24 ', ...
    'development seed. Ridge inputs contain only current directed-edge ', ...
    'posterior summaries, current link reliability and geometry, past ', ...
    'selected-edge persistence, action type and change fraction. Numeric ', ...
    'sensor/formation identities, truth, future measurements, future ', ...
    'physical pages and alternative-arm outcomes are not features. Truth ', ...
    'defines eight offline outcome heads. In-sample fit tests representation ', ...
    'capacity; leave-one-anchor-out is only a three-group development ', ...
    'screen. Neither result is an online, full-episode, X36, validation or ', ...
    'paper-level claim.'];

if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
reportPath = fullfile(outputRoot, ...
    'V251_CAUSAL_RIDGE_LEARNABILITY.md');
matPath = fullfile(outputRoot, ...
    'V251_CAUSAL_RIDGE_LEARNABILITY.mat');
summary.reportPath = reportPath;
summary.matPath = matPath;
if writeMat
    save('-mat7-binary', matPath, 'summary');
end
if writeReport
    writeReportFile(reportPath, summary);
end
fprintf(['V251 ridge learnability complete: rich in/LOAO pass %d/%d; ', ...
    'next=%s\n'], rich.pairwiseInSample.gatePassed, ...
    rich.pairwiseLeaveOneAnchorOut.gatePassed, nextDecision);
fprintf('V251 ridge report: %s\n', reportPath);
end

function context = buildFeatureContext( ...
        inputs, runtimeModel, posteriors, history, currentTime)
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
end

function [targets, targetFormationIdx] = buildTargets(window)
reference = window.arms{window.referenceCandidateIndex};
[~, targetFormationIdx] = max( ...
    reference.perFormationPositionRmse);
targets = zeros(window.candidateCount, 8);
for candidateIdx = 1:window.candidateCount
    comparison = window.comparisons{candidateIdx};
    targets(candidateIdx, :) = [ ...
        comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, ...
        comparison.attemptedByteSavingPercent, ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent, ...
        comparison.formationEospaGainPercent(targetFormationIdx), ...
        comparison.formationRmseGainPercent(targetFormationIdx)];
end
end

function proxy = computeFormationRiskProxy( ...
        posteriors, groupIds, model, positionCutoff, truthWorstFormation)
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
formationCount = numel(groups);
bayesRisk = zeros(1, formationCount);
positionUncertainty = zeros(1, formationCount);
withinSpatialDisagreement = zeros(1, formationCount);
for formationIdx = 1:formationCount
    members = find(groupIds == groups(formationIdx));
    summaries = cell(1, numel(members));
    labelUniverse = zeros(2, 0);
    for memberIdx = 1:numel(members)
        summaries{memberIdx} = ...
            summarizeLmbPosteriorForDisagreement( ...
                posteriors{members(memberIdx)}, model);
        labelUniverse = [labelUniverse, ... %#ok<AGROW>
            summaries{memberIdx}.labels];
    end
    if ~isempty(labelUniverse)
        labelUniverse = unique(labelUniverse', 'rows')';
    end
    memberRisk = zeros(1, numel(members));
    uncertainty = zeros(1, 0);
    for memberIdx = 1:numel(members)
        [memberRisk(memberIdx), ~] = ...
            computeLmbPosteriorSummaryBayesRisk( ...
                summaries{memberIdx}, struct( ...
                    'positionCutoff', positionCutoff, ...
                    'labelUniverse', labelUniverse));
        summary = summaries{memberIdx};
        for labelIdx = 1:numel(summary.existence)
            covariance = summary.positionCovariance(:, :, labelIdx);
            uncertainty(end + 1) = ... %#ok<AGROW>
                summary.existence(labelIdx) * min( ...
                    trace(covariance) / positionCutoff^2, 1);
        end
    end
    sortedRisk = sort(memberRisk, 'descend');
    tailCount = max(1, ceil(0.34 * numel(sortedRisk)));
    bayesRisk(formationIdx) = 0.5 * mean(memberRisk) + ...
        0.5 * mean(sortedRisk(1:tailCount));
    if isempty(uncertainty)
        positionUncertainty(formationIdx) = 0;
    else
        positionUncertainty(formationIdx) = mean(uncertainty);
    end
    disagreement = computeLmbPosteriorNetworkDisagreement( ...
        posteriors(members), model);
    withinSpatialDisagreement(formationIdx) = disagreement.spatial;
end
[~, bayesWorst] = max(bayesRisk);
[~, uncertaintyWorst] = max(positionUncertainty);
[~, disagreementWorst] = max(withinSpatialDisagreement);
proxy = struct();
proxy.truthWorstFormationIndex = truthWorstFormation;
proxy.bayesRisk = bayesRisk;
proxy.positionUncertainty = positionUncertainty;
proxy.withinSpatialDisagreement = withinSpatialDisagreement;
proxy.bayesWorstFormationIndex = bayesWorst;
proxy.uncertaintyWorstFormationIndex = uncertaintyWorst;
proxy.disagreementWorstFormationIndex = disagreementWorst;
proxy.bayesMatched = bayesWorst == truthWorstFormation;
proxy.uncertaintyMatched = uncertaintyWorst == truthWorstFormation;
proxy.disagreementMatched = disagreementWorst == truthWorstFormation;
proxy.truthUsedOnlyForMatchAudit = true;
proxy.featuresTruthFree = true;
end

function result = summarizeRiskProxies(windows)
result = struct();
result.bayesMatchCount = sum(cellfun(@(value) ...
    value.riskProxy.bayesMatched, windows));
result.uncertaintyMatchCount = sum(cellfun(@(value) ...
    value.riskProxy.uncertaintyMatched, windows));
result.disagreementMatchCount = sum(cellfun(@(value) ...
    value.riskProxy.disagreementMatched, windows));
result.anchorCount = numel(windows);
end

function teacherIdx = selectTeacherCandidate(window, targetFormationIdx)
teacherIdx = window.referenceCandidateIndex;
bestKey = [-Inf, -Inf, -Inf, -Inf, -Inf];
for candidateIdx = 1:window.candidateCount
    if candidateIdx == window.referenceCandidateIndex
        continue;
    end
    comparison = window.comparisons{candidateIdx};
    if ~isTailAligned(comparison, targetFormationIdx)
        continue;
    end
    targetRmse = comparison.formationRmseGainPercent( ...
        targetFormationIdx);
    targetEospa = comparison.formationEospaGainPercent( ...
        targetFormationIdx);
    minimumCore = min([comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent]);
    key = [targetRmse, targetEospa, minimumCore, ...
        comparison.attemptedByteSavingPercent, -candidateIdx];
    if lexicographicallyGreater(key, bestKey)
        teacherIdx = candidateIdx;
        bestKey = key;
    end
end
end

function evaluation = evaluateFitMode( ...
        oracle, windows, featureMask, targetNames, lambda, mode)
windowCount = numel(windows);
predictions = cell(1, windowCount);
switch mode
    case 'in-sample'
        [trainFeatures, trainTargets] = stackWindows( ...
            windows, 1:windowCount, featureMask);
        model = fitRidge(trainFeatures, trainTargets, lambda);
        for windowIdx = 1:windowCount
            predictions{windowIdx} = predictRidge(model, ...
                windows{windowIdx}.features(:, featureMask));
        end
    case 'leave-one-anchor-out'
        for windowIdx = 1:windowCount
            trainingWindows = setdiff(1:windowCount, windowIdx);
            [trainFeatures, trainTargets] = stackWindows( ...
                windows, trainingWindows, featureMask);
            model = fitRidge(trainFeatures, trainTargets, lambda);
            predictions{windowIdx} = predictRidge(model, ...
                windows{windowIdx}.features(:, featureMask));
        end
    otherwise
        error('CausalGatewayEmbeddingV251:UnknownFitMode', ...
            'Unknown V251 ridge fit mode: %s', mode);
end

selectedArms = cell(1, windowCount);
referenceArms = cell(1, windowCount);
rows = repmat(emptyEvaluationRow(), 1, windowCount);
teacherRecall = 0;
tailAlignedCount = 0;
rankOnlyTailAlignedCount = 0;
oracleSafeTeacherRecallCount = 0;
for windowIdx = 1:windowCount
    record = windows{windowIdx};
    window = oracle.windows{windowIdx};
    prediction = anchorPrediction( ...
        predictions{windowIdx}, record.referenceCandidateIndex);
    [selectedIdx, admissibleCount] = selectPredictedCandidate( ...
        prediction, record.referenceCandidateIndex);
    rankOnlyIdx = selectRankOnlyCandidate( ...
        prediction, record.referenceCandidateIndex);
    oracleSafeRankIdx = selectOracleSafeRankCandidate( ...
        prediction, window, record.targetFormationIndex, ...
        record.referenceCandidateIndex);
    comparison = window.comparisons{selectedIdx};
    aligned = selectedIdx ~= record.referenceCandidateIndex && ...
        isTailAligned(comparison, record.targetFormationIndex);
    row = emptyEvaluationRow();
    row.anchorTime = record.anchorTime;
    row.teacherCandidateIndex = record.teacherCandidateIndex;
    row.selectedCandidateIndex = selectedIdx;
    row.teacherRecalled = selectedIdx == record.teacherCandidateIndex;
    row.tailAligned = aligned;
    row.predictedAdmissibleCandidateCount = admissibleCount;
    row.rankOnlyCandidateIndex = rankOnlyIdx;
    row.rankOnlyTailAligned = rankOnlyIdx ~= ...
        record.referenceCandidateIndex && isTailAligned( ...
            window.comparisons{rankOnlyIdx}, ...
            record.targetFormationIndex);
    row.oracleSafeRankCandidateIndex = oracleSafeRankIdx;
    row.oracleSafeTeacherRecalled = ...
        oracleSafeRankIdx == record.teacherCandidateIndex;
    row.realizedTargets = record.targets(selectedIdx, :);
    row.predictedTargets = prediction(selectedIdx, :);
    rows(windowIdx) = row;
    teacherRecall = teacherRecall + row.teacherRecalled;
    tailAlignedCount = tailAlignedCount + aligned;
    rankOnlyTailAlignedCount = rankOnlyTailAlignedCount + ...
        row.rankOnlyTailAligned;
    oracleSafeTeacherRecallCount = ...
        oracleSafeTeacherRecallCount + ...
        row.oracleSafeTeacherRecalled;
    selectedArms{windowIdx} = window.arms{selectedIdx};
    referenceArms{windowIdx} = ...
        window.arms{record.referenceCandidateIndex};
end
aggregate = compareAggregate( ...
    aggregateArms(selectedArms), aggregateArms(referenceArms), ...
    oracle.protocol);
evaluation = struct();
evaluation.mode = mode;
evaluation.targetNames = targetNames;
evaluation.rows = rows;
evaluation.teacherRecallCount = teacherRecall;
evaluation.tailAlignedAnchorCount = tailAlignedCount;
evaluation.rankOnlyTailAlignedAnchorCount = ...
    rankOnlyTailAlignedCount;
evaluation.oracleSafeTeacherRecallCount = ...
    oracleSafeTeacherRecallCount;
evaluation.aggregate = aggregate;
evaluation.gatePassed = tailAlignedCount >= ...
        oracle.protocol.minimumJointPositiveAnchorCount && ...
    aggregate.jointPositive;
end

function [features, targets] = stackWindows(windows, indices, mask)
features = zeros(0, nnz(mask));
targets = zeros(0, size(windows{1}.targets, 2));
for windowIdx = reshape(indices, 1, [])
    features = [features; ... %#ok<AGROW>
        windows{windowIdx}.features(:, mask)];
    targets = [targets; windows{windowIdx}.targets]; %#ok<AGROW>
end
end

function evaluation = evaluatePairwiseMode( ...
        oracle, windows, featureMask, lambda, mode)
windowCount = numel(windows);
scores = cell(1, windowCount);
switch mode
    case 'in-sample'
        model = fitPairwiseRidge( ...
            windows, 1:windowCount, featureMask, lambda);
        for windowIdx = 1:windowCount
            scores{windowIdx} = predictPairwiseRidge(model, ...
                windows{windowIdx}.features(:, featureMask));
        end
    case 'leave-one-anchor-out'
        for windowIdx = 1:windowCount
            trainingWindows = setdiff(1:windowCount, windowIdx);
            model = fitPairwiseRidge( ...
                windows, trainingWindows, featureMask, lambda);
            scores{windowIdx} = predictPairwiseRidge(model, ...
                windows{windowIdx}.features(:, featureMask));
        end
    otherwise
        error('CausalGatewayEmbeddingV251:UnknownPairwiseMode', ...
            'Unknown V251 pairwise fit mode: %s', mode);
end

selectedArms = cell(1, windowCount);
referenceArms = cell(1, windowCount);
rows = repmat(emptyPairwiseRow(), 1, windowCount);
topOneRecall = 0;
topThreeRecall = 0;
tailAlignedCount = 0;
for windowIdx = 1:windowCount
    record = windows{windowIdx};
    window = oracle.windows{windowIdx};
    [orderedScores, order] = sort( ...
        reshape(scores{windowIdx}, 1, []), 'descend');
    selectedIdx = order(1);
    teacherRank = find(order == record.teacherCandidateIndex, 1);
    comparison = window.comparisons{selectedIdx};
    aligned = selectedIdx ~= record.referenceCandidateIndex && ...
        isTailAligned(comparison, record.targetFormationIndex);
    row = emptyPairwiseRow();
    row.anchorTime = record.anchorTime;
    row.teacherCandidateIndex = record.teacherCandidateIndex;
    row.selectedCandidateIndex = selectedIdx;
    row.teacherRank = teacherRank;
    row.topOneRecalled = teacherRank == 1;
    row.topThreeRecalled = teacherRank <= min(3, numel(order));
    row.tailAligned = aligned;
    row.selectionMargin = orderedScores(1) - ...
        orderedScores(min(2, numel(orderedScores)));
    row.realizedTargets = record.targets(selectedIdx, :);
    rows(windowIdx) = row;
    topOneRecall = topOneRecall + row.topOneRecalled;
    topThreeRecall = topThreeRecall + row.topThreeRecalled;
    tailAlignedCount = tailAlignedCount + aligned;
    selectedArms{windowIdx} = window.arms{selectedIdx};
    referenceArms{windowIdx} = ...
        window.arms{record.referenceCandidateIndex};
end
aggregate = compareAggregate( ...
    aggregateArms(selectedArms), aggregateArms(referenceArms), ...
    oracle.protocol);
evaluation = struct();
evaluation.mode = ['pairwise-', mode];
evaluation.rows = rows;
evaluation.topOneTeacherRecallCount = topOneRecall;
evaluation.topThreeTeacherRecallCount = topThreeRecall;
evaluation.tailAlignedAnchorCount = tailAlignedCount;
evaluation.aggregate = aggregate;
evaluation.gatePassed = tailAlignedCount >= ...
        oracle.protocol.minimumJointPositiveAnchorCount && ...
    aggregate.jointPositive;
end

function model = fitPairwiseRidge(windows, indices, featureMask, lambda)
[candidateFeatures, ~] = stackWindows( ...
    windows, indices, featureMask);
featureMean = mean(candidateFeatures, 1);
featureScale = std(candidateFeatures, 0, 1);
active = featureScale > 1e-9;
if ~any(active)
    error('CausalGatewayEmbeddingV251:DegeneratePairwiseFeatures', ...
        'The pairwise ridge training set has no varying feature.');
end
featureScale(~active) = 1;
pairFeatures = zeros(0, nnz(active));
pairTargets = zeros(0, 1);
for windowIdx = reshape(indices, 1, [])
    record = windows{windowIdx};
    current = bsxfun(@rdivide, bsxfun(@minus, ...
        record.features(:, featureMask), featureMean), featureScale);
    current = current(:, active);
    teacher = current(record.teacherCandidateIndex, :);
    alternatives = setdiff(1:size(current, 1), ...
        record.teacherCandidateIndex);
    positive = bsxfun(@minus, teacher, current(alternatives, :));
    pairFeatures = [pairFeatures; positive; -positive]; %#ok<AGROW>
    pairTargets = [pairTargets; ... %#ok<AGROW>
        ones(numel(alternatives), 1); ...
        -ones(numel(alternatives), 1)];
end
coefficients = (pairFeatures' * pairFeatures + ...
    lambda * eye(nnz(active))) \ ...
    (pairFeatures' * pairTargets);
model = struct();
model.featureMean = featureMean;
model.featureScale = featureScale;
model.activeFeatureMask = active;
model.coefficients = coefficients;
end

function scores = predictPairwiseRidge(model, features)
standard = bsxfun(@rdivide, bsxfun(@minus, ...
    features, model.featureMean), model.featureScale);
scores = standard(:, model.activeFeatureMask) * model.coefficients;
end

function model = fitRidge(features, targets, lambda)
featureMean = mean(features, 1);
featureScale = std(features, 0, 1);
active = featureScale > 1e-9;
if ~any(active)
    error('CausalGatewayEmbeddingV251:DegenerateRidgeFeatures', ...
        'The V251 ridge training set has no varying feature.');
end
featureScale(~active) = 1;
targetMean = mean(targets, 1);
targetScale = std(targets, 0, 1);
targetScale(targetScale <= 1e-12) = 1;
standardFeatures = bsxfun(@rdivide, ...
    bsxfun(@minus, features(:, active), featureMean(active)), ...
    featureScale(active));
standardTargets = bsxfun(@rdivide, ...
    bsxfun(@minus, targets, targetMean), targetScale);
coefficients = (standardFeatures' * standardFeatures + ...
    lambda * eye(nnz(active))) \ ...
    (standardFeatures' * standardTargets);
model = struct();
model.featureMean = featureMean;
model.featureScale = featureScale;
model.activeFeatureMask = active;
model.targetMean = targetMean;
model.targetScale = targetScale;
model.coefficients = coefficients;
end

function targets = predictRidge(model, features)
standardFeatures = bsxfun(@rdivide, bsxfun(@minus, ...
    features(:, model.activeFeatureMask), ...
    model.featureMean(model.activeFeatureMask)), ...
    model.featureScale(model.activeFeatureMask));
standardTargets = standardFeatures * model.coefficients;
targets = bsxfun(@plus, bsxfun(@times, ...
    standardTargets, model.targetScale), model.targetMean);
end

function prediction = anchorPrediction(prediction, referenceIdx)
prediction = bsxfun(@minus, prediction, prediction(referenceIdx, :));
prediction(referenceIdx, :) = 0;
end

function [selectedIdx, admissibleCount] = selectPredictedCandidate( ...
        prediction, referenceIdx)
selectedIdx = referenceIdx;
admissibleCount = 0;
bestKey = [-Inf, -Inf, -Inf, -Inf, -Inf];
for candidateIdx = 1:size(prediction, 1)
    if candidateIdx == referenceIdx
        continue;
    end
    value = prediction(candidateIdx, :);
    minimumCore = min(value(1:3));
    admissible = minimumCore > 0 && value(4) >= 0 && ...
        value(5) >= -2 && value(6) >= -2 && ...
        value(7) >= 0 && value(8) > 0;
    if ~admissible
        continue;
    end
    admissibleCount = admissibleCount + 1;
    key = [value(8), value(7), minimumCore, value(4), -candidateIdx];
    if lexicographicallyGreater(key, bestKey)
        selectedIdx = candidateIdx;
        bestKey = key;
    end
end
end

function selectedIdx = selectRankOnlyCandidate(prediction, referenceIdx)
selectedIdx = referenceIdx;
bestKey = [-Inf, -Inf, -Inf, -Inf];
for candidateIdx = 1:size(prediction, 1)
    if candidateIdx == referenceIdx
        continue;
    end
    value = prediction(candidateIdx, :);
    key = [value(8), value(7), min(value(1:3)), -candidateIdx];
    if lexicographicallyGreater(key, bestKey)
        selectedIdx = candidateIdx;
        bestKey = key;
    end
end
end

function selectedIdx = selectOracleSafeRankCandidate( ...
        prediction, window, targetFormationIdx, referenceIdx)
selectedIdx = referenceIdx;
bestKey = [-Inf, -Inf, -Inf, -Inf];
for candidateIdx = 1:size(prediction, 1)
    if candidateIdx == referenceIdx || ...
            ~isTailAligned(window.comparisons{candidateIdx}, ...
                targetFormationIdx)
        continue;
    end
    value = prediction(candidateIdx, :);
    key = [value(8), value(7), min(value(1:3)), -candidateIdx];
    if lexicographicallyGreater(key, bestKey)
        selectedIdx = candidateIdx;
        bestKey = key;
    end
end
end

function aligned = isTailAligned(comparison, targetFormationIdx)
aligned = comparison.jointPositive && ...
    comparison.formationEospaGainPercent(targetFormationIdx) >= ...
        -1e-6 && ...
    comparison.formationRmseGainPercent(targetFormationIdx) > 1e-6;
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

function aggregate = aggregateArms(arms)
aggregate = struct();
aggregate.positionEospa = mean(cellfun( ...
    @(value) value.positionEospa, arms));
aggregate.positionRmse = mean(cellfun( ...
    @(value) value.positionRmse, arms));
aggregate.interFormationPositionOspa = mean(cellfun( ...
    @(value) value.interFormationPositionOspa, arms));
aggregate.attemptedPayloadBytes = sum(cellfun( ...
    @(value) value.attemptedPayloadBytes, arms));
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
comparison.jointPositive = ...
    comparison.jointScorePercent > ...
        protocol.minimumAggregateCoreGainPercent && ...
    comparison.attemptedByteSavingPercent >= ...
        protocol.minimumAggregateByteSavingPercent && ...
    comparison.minimumFormationEospaGainPercent >= ...
        -protocol.maximumFormationRegressionPercent && ...
    comparison.minimumFormationRmseGainPercent >= ...
        -protocol.maximumFormationRegressionPercent;
end

function writeReportFile(path, summary)
fid = fopen(path, 'w');
if fid < 0
    error('CausalGatewayEmbeddingV251:ReportOpen', ...
        'Could not write the V251 ridge report.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '# V251 causal ridge learnability\n\n');
fprintf(fid, '- Scene / seed: `%s / %d`\n', ...
    summary.presetName, summary.seed);
fprintf(fid, '- Oracle source commit: `%s`\n', ...
    summary.oracleGenerationGitCommit);
fprintf(fid, '- Analysis source commit: `%s`\n', ...
    summary.analysisGitCommit);
fprintf(fid, '- Anchors: `%s`\n', mat2str(summary.anchorTimes));
fprintf(fid, '- Frozen ridge lambda: `%.6g`\n', summary.ridgeLambda);
fprintf(fid, '- Next decision: `%s`\n\n', summary.nextDecision);

fprintf(fid, '## Method decision\n\n');
fprintf(fid, ['A formation-level observable risk proxy is not used as a ', ...
    'deployment target because confident spatial bias need not appear as ', ...
    'large covariance or within-formation disagreement. V251 instead ', ...
    'predicts eight candidate outcome heads from current directed-edge ', ...
    'features. The deterministic selector applies the V250 core, byte and ', ...
    'formation-regression constraints to the predictions, then maximizes ', ...
    'the predicted RMSE gain of the offline target formation. The latter ', ...
    'is a teacher target, never a feature.\n\n']);

fprintf(fid, '## Why formation-level risk is not the selector\n\n');
fprintf(fid, ['| Anchor | Truth-worst | Bayes-risk worst | ', ...
    'Uncertainty worst | Within-formation disagreement worst |\n']);
fprintf(fid, '|--:|--:|--:|--:|--:|\n');
for windowIdx = 1:numel(summary.windowRecords)
    record = summary.windowRecords{windowIdx};
    proxy = record.riskProxy;
    fprintf(fid, '| %d | F%d | F%d | F%d | F%d |\n', ...
        record.anchorTime, proxy.truthWorstFormationIndex, ...
        proxy.bayesWorstFormationIndex, ...
        proxy.uncertaintyWorstFormationIndex, ...
        proxy.disagreementWorstFormationIndex);
end
proxy = summary.riskProxySummary;
fprintf(fid, ['\nMatch counts are Bayes risk `%d/3`, uncertainty `%d/3`, ', ...
    'and within-formation spatial disagreement `%d/3`. A confident but ', ...
    'biased formation can therefore look internally certain and coherent; ', ...
    'candidate-level cross-formation complementarity is required.\n\n'], ...
    proxy.bayesMatchCount, proxy.uncertaintyMatchCount, ...
    proxy.disagreementMatchCount);

fprintf(fid, '| Feature set | Features | In-sample recall / aligned / pass | ');
fprintf(fid, 'LOAO recall / aligned / pass | Oracle-safe rank recall in / LOAO |\n');
fprintf(fid, '|:--|--:|:--|:--|:--|\n');
for subsetIdx = 1:numel(summary.subsetResults)
    subset = summary.subsetResults(subsetIdx);
    fprintf(fid, ['| %s | %d | %d/3 / %d/3 / %d | ', ...
        '%d/3 / %d/3 / %d | %d/3 / %d/3 |\n'], ...
        subset.name, subset.featureCount, ...
        subset.inSample.teacherRecallCount, ...
        subset.inSample.tailAlignedAnchorCount, ...
        subset.inSample.gatePassed, ...
        subset.leaveOneAnchorOut.teacherRecallCount, ...
        subset.leaveOneAnchorOut.tailAlignedAnchorCount, ...
        subset.leaveOneAnchorOut.gatePassed, ...
        subset.inSample.oracleSafeTeacherRecallCount, ...
        subset.leaveOneAnchorOut.oracleSafeTeacherRecallCount);
end

fprintf(fid, '\nPairwise ridge is the actual ranking baseline; the table above ');
fprintf(fid, ['keeps the multi-head outcome predictor as a diagnostic. ', ...
    'Pairwise results follow.\n\n']);
fprintf(fid, '| Feature set | In-sample top-1 / top-3 / aligned / pass | ');
fprintf(fid, 'LOAO top-1 / top-3 / aligned / pass |\n');
fprintf(fid, '|:--|:--|:--|\n');
for subsetIdx = 1:numel(summary.subsetResults)
    subset = summary.subsetResults(subsetIdx);
    left = subset.pairwiseInSample;
    right = subset.pairwiseLeaveOneAnchorOut;
    fprintf(fid, ['| %s | %d/3 / %d/3 / %d/3 / %d | ', ...
        '%d/3 / %d/3 / %d/3 / %d |\n'], ...
        subset.name, left.topOneTeacherRecallCount, ...
        left.topThreeTeacherRecallCount, ...
        left.tailAlignedAnchorCount, left.gatePassed, ...
        right.topOneTeacherRecallCount, ...
        right.topThreeTeacherRecallCount, ...
        right.tailAlignedAnchorCount, right.gatePassed);
end

for subsetIdx = 1:numel(summary.subsetResults)
    subset = summary.subsetResults(subsetIdx);
    fprintf(fid, '\n## %s\n\n', subset.name);
    writeEvaluation(fid, subset.inSample);
    writeEvaluation(fid, subset.leaveOneAnchorOut);
    writePairwiseEvaluation(fid, subset.pairwiseInSample);
    writePairwiseEvaluation(fid, subset.pairwiseLeaveOneAnchorOut);
end

fprintf(fid, '\n## Evidence boundary\n\n%s\n', ...
    summary.evidenceBoundary);
end

function writePairwiseEvaluation(fid, evaluation)
fprintf(fid, '### %s\n\n', evaluation.mode);
fprintf(fid, ['| Anchor | Teacher | Selected | Teacher rank | Top-3 | ', ...
    'Tail-aligned | Score margin | Realized E / R / C | ', ...
    'Byte saving | Target E / R | Minimum formation E / R |\n']);
fprintf(fid, '|--:|--:|--:|--:|:--:|:--:|--:|:--|--:|:--|:--|\n');
for rowIdx = 1:numel(evaluation.rows)
    row = evaluation.rows(rowIdx);
    value = row.realizedTargets;
    fprintf(fid, ['| %d | %d | %d | %d | %d | %d | %.6f | ', ...
        '%+.3f%% / %+.3f%% / %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% / %+.3f%% | %+.3f%% / %+.3f%% |\n'], ...
        row.anchorTime, row.teacherCandidateIndex, ...
        row.selectedCandidateIndex, row.teacherRank, ...
        row.topThreeRecalled, row.tailAligned, row.selectionMargin, ...
        value(1), value(2), value(3), value(4), ...
        value(7), value(8), value(5), value(6));
end
gain = evaluation.aggregate;
fprintf(fid, ['\nAggregate pairwise selections: E `%+.3f%%`, ', ...
    'RMSE `%+.3f%%`, consistency `%+.3f%%`, attempted-byte saving ', ...
    '`%+.3f%%`, weakest formation E/R `%+.3f%% / %+.3f%%`; gate `%d`.\n'], ...
    gain.eospaGainPercent, gain.rmseGainPercent, ...
    gain.consistencyGainPercent, gain.attemptedByteSavingPercent, ...
    gain.minimumFormationEospaGainPercent, ...
    gain.minimumFormationRmseGainPercent, evaluation.gatePassed);
end

function writeEvaluation(fid, evaluation)
fprintf(fid, '### %s\n\n', evaluation.mode);
fprintf(fid, ['| Anchor | Teacher | Selected | Recalled | Tail-aligned | ', ...
    'Admissible | Rank-only / safe-rank | Realized E / R / C | Byte saving | Target E / R | ', ...
    'Minimum formation E / R |\n']);
fprintf(fid, '|--:|--:|--:|:--:|:--:|--:|:--|:--|--:|:--|:--|\n');
for rowIdx = 1:numel(evaluation.rows)
    row = evaluation.rows(rowIdx);
    value = row.realizedTargets;
    fprintf(fid, ['| %d | %d | %d | %d | %d | ', ...
        '%d | %d / %d | %+.3f%% / %+.3f%% / %+.3f%% | %+.3f%% | ', ...
        '%+.3f%% / %+.3f%% | %+.3f%% / %+.3f%% |\n'], ...
        row.anchorTime, row.teacherCandidateIndex, ...
        row.selectedCandidateIndex, row.teacherRecalled, ...
        row.tailAligned, row.predictedAdmissibleCandidateCount, ...
        row.rankOnlyCandidateIndex, row.oracleSafeRankCandidateIndex, ...
        value(1), value(2), value(3), ...
        value(4), value(7), value(8), value(5), value(6));
end
gain = evaluation.aggregate;
fprintf(fid, ['\nAggregate selected actions: E `%+.3f%%`, ', ...
    'RMSE `%+.3f%%`, consistency `%+.3f%%`, attempted-byte saving ', ...
    '`%+.3f%%`, weakest formation E/R `%+.3f%% / %+.3f%%`; ', ...
    'gate `%d`.\n'], ...
    gain.eospaGainPercent, gain.rmseGainPercent, ...
    gain.consistencyGainPercent, ...
    gain.attemptedByteSavingPercent, ...
    gain.minimumFormationEospaGainPercent, ...
    gain.minimumFormationRmseGainPercent, ...
    evaluation.gatePassed);
end

function value = emptySubsetResult()
value = struct('name', '', 'featureCount', 0, ...
    'featureNames', {{}}, 'inSample', struct(), ...
    'leaveOneAnchorOut', struct(), ...
    'pairwiseInSample', struct(), ...
    'pairwiseLeaveOneAnchorOut', struct());
end

function value = emptyEvaluationRow()
value = struct('anchorTime', 0, 'teacherCandidateIndex', 0, ...
    'selectedCandidateIndex', 0, 'teacherRecalled', false, ...
    'tailAligned', false, ...
    'predictedAdmissibleCandidateCount', 0, ...
    'rankOnlyCandidateIndex', 0, 'rankOnlyTailAligned', false, ...
    'oracleSafeRankCandidateIndex', 0, ...
    'oracleSafeTeacherRecalled', false, ...
    'realizedTargets', zeros(1, 8), ...
    'predictedTargets', zeros(1, 8));
end

function value = emptyPairwiseRow()
value = struct('anchorTime', 0, 'teacherCandidateIndex', 0, ...
    'selectedCandidateIndex', 0, 'teacherRank', 0, ...
    'topOneRecalled', false, 'topThreeRecalled', false, ...
    'tailAligned', false, 'selectionMargin', 0, ...
    'realizedTargets', zeros(1, 8));
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
end

function value = lowerGainVector(reference, candidate)
value = 100 * (reference - candidate) ./ max(abs(reference), eps);
end

function validateOracle(oracle, protocol)
valid = isstruct(oracle) && isscalar(oracle) && ...
    isfield(oracle, 'contractVersion') && ...
    strcmp(oracle.contractVersion, ...
        'causal-gateway-embedding-v250-h3-oracle-result-v1') && ...
    isfield(oracle, 'protocolId') && ...
    strcmp(oracle.protocolId, protocol.id) && ...
    isfield(oracle, 'completedAt') && ~isempty(oracle.completedAt) && ...
    isfield(oracle, 'windows') && ...
    numel(oracle.windows) == numel(protocol.anchorTimes) && ...
    isfield(oracle, 'cachePath') && exist(oracle.cachePath, 'file') == 2;
if ~valid
    error('CausalGatewayEmbeddingV251:InvalidOracle', ...
        'The V250 oracle checkpoint is incomplete or incompatible.');
end
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
