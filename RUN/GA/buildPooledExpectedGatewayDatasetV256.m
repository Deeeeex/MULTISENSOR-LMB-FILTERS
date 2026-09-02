function [matPath, dataset] = ...
        buildPooledExpectedGatewayDatasetV256(options)
% BUILDPOOLEDEXPECTEDGATEWAYDATASETV256 Align seven local-action seeds.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getPooledExpectedGatewayV256Protocol();
legacy = getIndependentM24GatewayTeacherV252Protocol();
teacher = getBudgetedLocalGatewayTeacherV255Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
legacyPaths = getField(options, 'legacyCheckpointPaths', ...
    defaultCheckpointPaths(repoRoot, legacy.outputRoot, ...
        protocol.preliminaryDevelopmentSeeds, legacy.oracleMatName));
teacherPaths = getField(options, 'teacherCheckpointPaths', ...
    defaultCheckpointPaths(repoRoot, teacher.outputRoot, ...
        protocol.additionalTrainingSeeds, teacher.oracleMatName));
legacyPaths = normalizePaths(legacyPaths, ...
    numel(protocol.preliminaryDevelopmentSeeds), 'legacy');
teacherPaths = normalizePaths(teacherPaths, ...
    numel(protocol.additionalTrainingSeeds), 'teacher');
allPaths = [legacyPaths, teacherPaths];

gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('PooledExpectedGatewayV256:DirtyDatasetSource', ...
        'Official V256 dataset assembly requires clean source.');
end

records = cell(1, 0);
featureNames = {};
sourceCommits = cell(1, numel(allPaths));
sourceProtocolIds = cell(1, numel(allPaths));
observedSeeds = zeros(1, numel(allPaths));
for pathIdx = 1:numel(allPaths)
    checkpointPath = char(allPaths{pathIdx});
    if exist(checkpointPath, 'file') ~= 2
        error('PooledExpectedGatewayV256:MissingCheckpoint', ...
            'Missing V256 training checkpoint: %s', checkpointPath);
    end
    envelope = load(checkpointPath, 'result');
    if ~isfield(envelope, 'result')
        error('PooledExpectedGatewayV256:MissingResult', ...
            'A V256 source checkpoint lacks its result envelope.');
    end
    oracle = envelope.result;
    sourceKind = validateOracle(oracle, legacy, teacher, protocol);
    observedSeeds(pathIdx) = oracle.seed;
    sourceCommits{pathIdx} = oracle.generationGitCommit;
    sourceProtocolIds{pathIdx} = oracle.protocolId;

    cacheEnvelope = load(oracle.cachePath, 'behaviorBundle');
    if ~isfield(cacheEnvelope, 'behaviorBundle')
        error('PooledExpectedGatewayV256:MissingBehaviorCache', ...
            'A V256 source checkpoint lacks its continuation cache.');
    end
    bundle = cacheEnvelope.behaviorBundle;
    validateCache(bundle, oracle);
    inputs = generateDynamicTopologyScenarioInputs( ...
        oracle.presetName, oracle.seed);
    runtimeModel = removeRealizedTargetTruthFromDynamicTopologyModel( ...
        inputs.model);
    identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
    controlBytes = localControlCost( ...
        teacher, inputs.config.sensorsPerFormation);

    for windowIdx = 1:numel(oracle.windows)
        window = oracle.windows{windowIdx};
        [posteriors, history] = extractBehaviorContinuationSnapshot( ...
            bundle, window.anchorTime, inputs.config.numberOfSensors);
        context = buildFeatureContext(inputs, runtimeModel, identity, ...
            posteriors, history, window.anchorTime);
        desiredBank = buildRestrictedBank(context, inputs, teacher);
        desiredAssignments = { ...
            desiredBank.candidates(2:end).gatewayAssignment};
        candidateIndices = matchAssignments(window, desiredAssignments);
        [baseFeatures, baseNames, featureDetails] = ...
            computeBudgetedLocalGatewayActionFeaturesV255( ...
                context, desiredAssignments, ...
                window.candidateAssignments{ ...
                    window.referenceCandidateIndex});
        [features, currentNames] = transformFeatures( ...
            baseFeatures, baseNames, protocol);
        if isempty(featureNames)
            featureNames = currentNames;
        elseif ~isequal(featureNames, currentNames)
            error('PooledExpectedGatewayV256:FeatureDrift', ...
                'The V256 action features changed across seeds.');
        end
        if featureDetails.featureCount ~= ...
                protocol.expectedFeatureCount || ...
                size(features, 2) ~= protocol.expectedFeatureCount
            error('PooledExpectedGatewayV256:FeatureCount', ...
                'The V256 action representation must have 32 features.');
        end
        [targets, receiverFormationIndices, candidates] = ...
            buildTargets(window, candidateIndices, desiredAssignments, ...
                identity, controlBytes, sourceKind);
        communicationProjection = buildCommunicationProjection( ...
            context, desiredAssignments, ...
            window.candidateAssignments{ ...
                window.referenceCandidateIndex}, ...
            identity, controlBytes, protocol);
        record = struct();
        record.contractVersion = ...
            'pooled-expected-gateway-v256-window-record-v1';
        record.seed = oracle.seed;
        record.sourceKind = sourceKind;
        record.sourceProtocolId = oracle.protocolId;
        record.anchorTime = window.anchorTime;
        record.endTime = window.endTime;
        record.features = features;
        record.targets = targets;
        record.candidateIndices = candidateIndices;
        record.candidateAssignments = desiredAssignments;
        record.receiverFormationIndices = receiverFormationIndices;
        record.reference = summarizeArm( ...
            window.arms{window.referenceCandidateIndex}, 0, false);
        record.candidates = candidates;
        record.controlAttemptedBytesPerAction = controlBytes;
        record.communicationProjection = communicationProjection;
        record.communicationAdmissionUsesRealizedBytes = false;
        record.truthUsedAsFeature = false;
        record.futureOutcomeUsedAsFeature = false;
        records{end + 1} = record; %#ok<AGROW>
    end
end

if ~isequal(sort(observedSeeds), sort(protocol.trainingSeeds))
    error('PooledExpectedGatewayV256:TrainingSeedCoverage', ...
        'The V256 dataset does not cover its seven frozen training seeds.');
end
expectedRecordCount = numel(protocol.trainingSeeds) * ...
    numel(protocol.anchorTimes);
actionRowsPerWindow = cellfun( ...
    @(value) size(value.features, 1), records);
if numel(records) ~= expectedRecordCount || ...
        any(actionRowsPerWindow < protocol.minimumActionsPerWindow) || ...
        any(actionRowsPerWindow > protocol.maximumActionsPerWindow)
    error('PooledExpectedGatewayV256:DatasetShape', ...
        ['Every M24 training window must retain one or two physical ', ...
         'alternatives for each directed formation-tree slot.']);
end

dataset = struct();
dataset.contractVersion = ...
    'pooled-expected-gateway-v256-training-dataset-v2';
dataset.protocol = protocol;
dataset.assemblyGitCommit = gitState.commit;
dataset.checkpointPaths = allPaths;
dataset.sourceGenerationGitCommits = sourceCommits;
dataset.sourceProtocolIds = sourceProtocolIds;
dataset.trainingSeeds = observedSeeds;
dataset.featureNames = featureNames;
dataset.outcomeNames = protocol.outcomeNames;
dataset.records = records;
dataset.recordCount = numel(records);
dataset.actionRowCount = sum(cellfun( ...
    @(value) size(value.features, 1), records));
dataset.actionRowsPerWindow = actionRowsPerWindow;
dataset.minimumActionsPerWindow = min(actionRowsPerWindow);
dataset.maximumActionsPerWindow = max(actionRowsPerWindow);
dataset.sampleWeighting = protocol.seedWeighting;
dataset.communicationProjectionContractVersion = ...
    'pooled-expected-gateway-v256-communication-projection-v1';
dataset.communicationAdmissionMode = ...
    protocol.communicationAdmissionMode;
dataset.truthUsedAsFeature = false;
dataset.futureOutcomeUsedAsFeature = false;
dataset.completedAt = datestr(now, 31);
dataset.developmentEvidenceOnly = true;
dataset.validationClaimAllowed = false;
dataset.evidenceBoundary = protocol.evidenceBoundary;

outputRoot = char(getField(options, 'outputRoot', ...
    fullfile(repoRoot, protocol.outputRoot, 'training_dataset')));
if ~isAbsolutePath(outputRoot)
    outputRoot = fullfile(repoRoot, outputRoot);
end
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'POOLED_EXPECTED_GATEWAY_V256_TRAINING_DATASET.mat');
save('-mat7-binary', matPath, 'dataset');
fprintf('V256 dataset: %d seeds, %d windows, %d action rows\n', ...
    numel(unique(observedSeeds)), dataset.recordCount, ...
    dataset.actionRowCount);
fprintf('V256 dataset MAT: %s\n', matPath);
end

function projection = buildCommunicationProjection( ...
        context, candidateAssignments, referenceAssignment, identity, ...
        controlBytes, protocol)
nodeCount = numel(context.localPosteriorBySensor);
formationCount = numel(unique( ...
    context.formationPhysicalUidsBySensor));
routeContext = context;
routeContext.directedMessageBudget = ...
    nodeCount + 2 * (formationCount - 1);
[v242Adjacency, routeDetails] = ...
    selectCausalMinimumFormationBackboneV242Policy(routeContext);
nodePayloadBytes = zeros(1, nodeCount);
for sensorIdx = 1:nodeCount
    estimate = estimateLmbPayloadSize( ...
        context.localPosteriorBySensor{sensorIdx}, ...
        context.model, 2, struct());
    nodePayloadBytes(sensorIdx) = estimate.estimatedBytes;
end
projection = projectPooledExpectedGatewayCommunicationV256( ...
    nodePayloadBytes, routeDetails.referenceAdjacency, v242Adjacency, ...
    referenceAssignment, candidateAssignments, ...
    identity.sensorPhysicalUids, context.physicalAdjacency, ...
    controlBytes, protocol.horizonSteps, protocol);
end

function paths = defaultCheckpointPaths(repoRoot, outputRoot, seeds, name)
paths = cell(1, numel(seeds));
for seedIdx = 1:numel(seeds)
    paths{seedIdx} = fullfile(repoRoot, outputRoot, ...
        sprintf('seed%d', seeds(seedIdx)), name);
end
end

function paths = normalizePaths(paths, expectedCount, label)
if ischar(paths)
    paths = {paths};
end
if ~iscell(paths) || numel(paths) ~= expectedCount
    error('PooledExpectedGatewayV256:CheckpointList', ...
        'The V256 %s checkpoint list has the wrong length.', label);
end
paths = reshape(paths, 1, []);
end

function sourceKind = validateOracle(oracle, legacy, teacher, protocol)
required = {'contractVersion', 'protocolId', 'generationGitCommit', ...
    'presetName', 'seed', 'cachePath', 'windows', 'completedAt'};
valid = isstruct(oracle) && isscalar(oracle) && ...
    all(isfield(oracle, required)) && ischar(oracle.generationGitCommit) && ...
    ischar(oracle.completedAt) && ~isempty(oracle.completedAt) && ...
    ismember(oracle.presetName, protocol.allowedPresets) && ...
    numel(oracle.windows) == numel(protocol.anchorTimes);
if ~valid
    error('PooledExpectedGatewayV256:IncompleteOracle', ...
        'A V256 source oracle is incomplete or malformed.');
end
if strcmp(oracle.protocolId, legacy.id) && ...
        strcmp(oracle.contractVersion, legacy.resultContractVersion) && ...
        ismember(oracle.seed, protocol.preliminaryDevelopmentSeeds)
    sourceKind = 'legacy-v252-full-bank';
elseif strcmp(oracle.protocolId, teacher.id) && ...
        strcmp(oracle.contractVersion, teacher.resultContractVersion) && ...
        ismember(oracle.seed, protocol.additionalTrainingSeeds)
    sourceKind = 'v255-restricted-local-bank';
else
    error('PooledExpectedGatewayV256:OracleProvenance', ...
        'A V256 source oracle has an unregistered protocol or seed.');
end
for windowIdx = 1:numel(oracle.windows)
    window = oracle.windows{windowIdx};
    if ~isstruct(window) || ~isscalar(window) || ...
            ~isfield(window, 'arms') || ...
            any(~cellfun(@armComplete, window.arms)) || ...
            ~isfield(window, 'comparisons') || ...
            numel(window.comparisons) ~= numel(window.arms)
        error('PooledExpectedGatewayV256:IncompleteWindow', ...
            'A V256 training window has incomplete paired arms.');
    end
end
end

function validateCache(bundle, oracle)
required = {'protocolId', 'presetName', 'seed', 'snapshotTimes', ...
    'posteriorSnapshots', 'preDecisionTopologyHistoryByTime'};
if ~isstruct(bundle) || ~isscalar(bundle) || ...
        ~all(isfield(bundle, required)) || ...
        ~strcmp(bundle.protocolId, oracle.protocolId) || ...
        ~strcmp(bundle.presetName, oracle.presetName) || ...
        bundle.seed ~= oracle.seed
    error('PooledExpectedGatewayV256:CacheProvenance', ...
        'A V256 continuation cache does not match its oracle.');
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
context.previousAdjacencyHistoryCount = ...
    size(context.previousAdjacencyHistory, 3);
context.previousAdjacencyHistoryTimes = history.times;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
end

function bank = buildRestrictedBank(context, inputs, teacher)
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
formationCount = numel(unique(inputs.config.sensorGroupIds));
context.directedMessageBudget = inputs.config.numberOfSensors + ...
    2 * (formationCount - 1);
options = struct('singleArcOnly', true, ...
    'singleArcAlternativesPerSlot', ...
        teacher.maximumCandidatesPerDirectedSlot, ...
    'minimumCandidateCount', teacher.minimumCandidateCount, ...
    'minimumReceiverCoveragePerFormation', 1);
bank = buildCausalGatewayEmbeddingCandidateBankV250( ...
    context, teacher.maximumRestrictedCandidateCount, options);
bank = restrictBudgetedLocalGatewayTeacherCandidateBankV255( ...
    bank, teacher);
end

function indices = matchAssignments(window, desiredAssignments)
indices = zeros(1, numel(desiredAssignments));
for desiredIdx = 1:numel(desiredAssignments)
    desired = normalizeAssignment(desiredAssignments{desiredIdx});
    matches = zeros(1, 0);
    for candidateIdx = 1:window.candidateCount
        if isequal(normalizeAssignment( ...
                window.candidateAssignments{candidateIdx}), desired)
            matches(end + 1) = candidateIdx; %#ok<AGROW>
        end
    end
    if numel(matches) ~= 1
        error('PooledExpectedGatewayV256:ActionAlignment', ...
            'A restricted V255 action is missing or duplicated in its source.');
    end
    indices(desiredIdx) = matches;
end
end

function [features, names] = transformFeatures(base, baseNames, protocol)
if size(base, 2) ~= protocol.expectedFeatureCount || ...
        mod(size(base, 2), 2) ~= 0
    error('PooledExpectedGatewayV256:BaseFeatureShape', ...
        'V256 requires paired candidate/incumbent feature halves.');
end
half = size(base, 2) / 2;
candidate = base(:, 1:half);
incumbent = base(:, half + 1:end);
features = [candidate - incumbent, incumbent];
names = cell(1, size(base, 2));
for idx = 1:half
    baseName = strrep(baseNames{idx}, 'candidate_', '');
    incumbentName = strrep(baseNames{half + idx}, 'incumbent_', '');
    if ~strcmp(baseName, incumbentName)
        error('PooledExpectedGatewayV256:FeaturePairing', ...
            'Candidate and incumbent feature names are not aligned.');
    end
    names{idx} = ['delta_', baseName];
    names{half + idx} = ['incumbent_', incumbentName];
end
end

function [targets, receiverIndices, candidates] = buildTargets( ...
        window, candidateIndices, assignments, identity, controlBytes, ...
        sourceKind)
targetCount = numel(candidateIndices);
targets = zeros(targetCount, 8);
receiverIndices = zeros(1, targetCount);
candidates = repmat(emptyArmSummary(), 1, targetCount);
reference = window.arms{window.referenceCandidateIndex};
referenceAssignment = normalizeAssignment( ...
    window.candidateAssignments{window.referenceCandidateIndex});
for localIdx = 1:targetCount
    candidateIdx = candidateIndices(localIdx);
    assignment = normalizeAssignment(assignments{localIdx});
    changed = find(any( ...
        assignment(:, 3:4) ~= referenceAssignment(:, 3:4), 2));
    if numel(changed) ~= 1
        error('PooledExpectedGatewayV256:ChangedArcCount', ...
            'A V256 training action must change one directed arc.');
    end
    receiverUid = assignment(changed, 2);
    receiverIdx = find( ...
        identity.formationPhysicalUids == receiverUid, 1);
    if isempty(receiverIdx)
        error('PooledExpectedGatewayV256:ReceiverFormation', ...
            'A V256 action references an unknown receiving formation.');
    end
    comparison = window.comparisons{candidateIdx};
    candidate = window.arms{candidateIdx};
    if strcmp(sourceKind, 'legacy-v252-full-bank')
        byteSaving = lowerGain(reference.attemptedPayloadBytes, ...
            candidate.attemptedPayloadBytes + controlBytes);
        chargedControl = controlBytes;
    else
        byteSaving = comparison.attemptedByteSavingPercent;
        chargedControl = getField(candidate, ...
            'controlAttemptedBytes', NaN);
        if chargedControl ~= controlBytes
            error('PooledExpectedGatewayV256:ControlChargeDrift', ...
                'A V255 teacher arm has the wrong control charge.');
        end
    end
    receiverIndices(localIdx) = receiverIdx;
    targets(localIdx, :) = [comparison.eospaGainPercent, ...
        comparison.rmseGainPercent, ...
        comparison.consistencyGainPercent, byteSaving, ...
        comparison.minimumFormationEospaGainPercent, ...
        comparison.minimumFormationRmseGainPercent, ...
        comparison.formationEospaGainPercent(receiverIdx), ...
        comparison.formationRmseGainPercent(receiverIdx)];
    candidates(localIdx) = summarizeArm( ...
        candidate, chargedControl, ...
        strcmp(sourceKind, 'legacy-v252-full-bank'));
    candidates(localIdx).candidateIndex = candidateIdx;
    candidates(localIdx).receiverFormationIndex = receiverIdx;
end
end

function summary = summarizeArm(arm, extraControlBytes, addControl)
summary = emptyArmSummary();
summary.positionEospa = arm.positionEospa;
summary.positionRmse = arm.positionRmse;
summary.interFormationPositionOspa = arm.interFormationPositionOspa;
summary.perFormationPositionEospa = arm.perFormationPositionEospa;
summary.perFormationPositionRmse = arm.perFormationPositionRmse;
summary.posteriorAttemptedPayloadBytes = getField(arm, ...
    'posteriorAttemptedPayloadBytes', arm.attemptedPayloadBytes);
if addControl
    summary.controlAttemptedBytes = extraControlBytes;
    summary.totalAttemptedBytes = ...
        arm.attemptedPayloadBytes + extraControlBytes;
else
    summary.controlAttemptedBytes = getField(arm, ...
        'controlAttemptedBytes', extraControlBytes);
    summary.totalAttemptedBytes = arm.attemptedPayloadBytes;
end
end

function summary = emptyArmSummary()
summary = struct('candidateIndex', 0, ...
    'receiverFormationIndex', 0, ...
    'positionEospa', NaN, 'positionRmse', NaN, ...
    'interFormationPositionOspa', NaN, ...
    'perFormationPositionEospa', zeros(1, 0), ...
    'perFormationPositionRmse', zeros(1, 0), ...
    'posteriorAttemptedPayloadBytes', NaN, ...
    'controlAttemptedBytes', 0, 'totalAttemptedBytes', NaN);
end

function bytes = localControlCost(teacher, sensorsPerFormation)
bytes = teacher.controlCollectionCountPerHold * ...
    2 * sensorsPerFormation * teacher.compactNodeBytesPerSensor + ...
    teacher.routeCommandHeaderBytes + ...
    teacher.maximumChangedDirectedGatewayArcs * ...
        teacher.routeCommandBytesPerChangedArc;
if bytes ~= 408
    error('PooledExpectedGatewayV256:ControlCost', ...
        'The registered M24 V256 control cost must be 408 B.');
end
end

function complete = armComplete(arm)
complete = isstruct(arm) && isscalar(arm) && ...
    isfield(arm, 'completed') && logical(arm.completed);
end

function assignment = normalizeAssignment(assignment)
assignment = sortrows(assignment, [2, 1, 4, 3]);
end

function value = lowerGain(reference, candidate)
value = 100 * (reference - candidate) / max(abs(reference), eps);
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
