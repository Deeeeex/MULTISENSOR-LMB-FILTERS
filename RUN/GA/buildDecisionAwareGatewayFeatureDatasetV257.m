function [matPath, dataset] = ...
        buildDecisionAwareGatewayFeatureDatasetV257(options)
% BUILDDECISIONAWAREGATEWAYFEATUREDATASETV257 Add rich current information.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getDecisionAwareGatewayV257Protocol();
sourceProtocol = getPooledExpectedGatewayV256Protocol();
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
sourcePath = absolutePath(getField(options, 'sourceDatasetPath', ...
    fullfile(repoRoot, sourceProtocol.outputRoot, ...
        'training_dataset', ...
        'POOLED_EXPECTED_GATEWAY_V256_TRAINING_DATASET.mat')), repoRoot);
if exist(sourcePath, 'file') ~= 2
    error('DecisionAwareGatewayV257:MissingSourceDataset', ...
        'The frozen V256 training dataset is unavailable.');
end
gitState = resolveResearchGitState();
if gitState.trackedWorktreeDirty || ...
        ~isempty(gitState.untrackedSourceFiles)
    error('DecisionAwareGatewayV257:DirtyDatasetSource', ...
        'Official V257 dataset assembly requires clean source.');
end
envelope = load(sourcePath, 'dataset');
if ~isfield(envelope, 'dataset')
    error('DecisionAwareGatewayV257:MissingDatasetEnvelope', ...
        'The V256 source lacks its dataset envelope.');
end
source = envelope.dataset;
validateSource(source, protocol);

records = source.records;
richNames = {};
compactNames = {};
for seedIdx = 1:numel(source.seeds)
    seed = source.seeds(seedIdx);
    checkpoint = load(source.checkpointPaths{seedIdx}, 'result');
    oracle = checkpoint.result;
    cache = load(oracle.cachePath, 'behaviorBundle');
    bundle = cache.behaviorBundle;
    inputs = generateDynamicTopologyScenarioInputs( ...
        oracle.presetName, oracle.seed);
    runtimeModel = removeRealizedTargetTruthFromDynamicTopologyModel( ...
        inputs.model);
    identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
    seedRecordIndices = find(cellfun( ...
        @(record) record.seed == seed, records));
    for cursor = 1:numel(seedRecordIndices)
        recordIdx = seedRecordIndices(cursor);
        record = records{recordIdx};
        windowIdx = find(cellfun(@(window) ...
            window.anchorTime == record.anchorTime, oracle.windows), 1);
        if isempty(windowIdx)
            error('DecisionAwareGatewayV257:MissingSourceWindow', ...
                'A V257 record has no matching teacher window.');
        end
        window = oracle.windows{windowIdx};
        [posteriors, history] = extractBehaviorContinuationSnapshot( ...
            bundle, record.anchorTime, inputs.config.numberOfSensors);
        context = buildFeatureContext(inputs, runtimeModel, identity, ...
            posteriors, history, record.anchorTime);
        referenceAssignment = window.candidateAssignments{ ...
            window.referenceCandidateIndex};
        [rich, currentRichNames, details] = ...
            computeDecisionAwareGatewayActionFeaturesV257( ...
                context, record.candidateAssignments, referenceAssignment);
        if isempty(richNames)
            richNames = currentRichNames;
            compactNames = details.compactFeatureNames;
        elseif ~isequal(richNames, currentRichNames) || ...
                ~isequal(compactNames, details.compactFeatureNames)
            error('DecisionAwareGatewayV257:FeatureDrift', ...
                'V257 feature names changed across windows.');
        end
        if ~isequal(compactNames, source.featureNames) || ...
                max(abs(details.compactFeatures(:) - ...
                    record.features(:))) > 1e-10
            error('DecisionAwareGatewayV257:CompactProjectionDrift', ...
                'The V257 rich representation does not reproduce V256.');
        end
        record.compactFeatures = record.features;
        record.richFeatures = rich;
        record.featureContractVersion = details.contractVersion;
        record.richPairwiseControlCosted = false;
        records{recordIdx} = record;
    end
    fprintf('V257 feature dataset seed %d complete: %d windows\n', ...
        seed, numel(seedRecordIndices));
end

dataset = struct();
dataset.contractVersion = ...
    'decision-aware-gateway-v257-training-dataset-v1';
dataset.protocol = protocol;
dataset.assemblyGitCommit = gitState.commit;
dataset.sourceDatasetPath = sourcePath;
dataset.sourceDatasetAssemblyGitCommit = source.assemblyGitCommit;
dataset.sourceCheckpointPaths = source.checkpointPaths;
dataset.seeds = source.seeds;
dataset.anchorTimes = protocol.anchorTimes;
dataset.compactFeatureNames = compactNames;
dataset.richFeatureNames = richNames;
dataset.outcomeNames = source.outcomeNames;
dataset.records = records;
dataset.recordCount = numel(records);
dataset.actionRowCount = source.actionRowCount;
dataset.actionRowsPerWindow = source.actionRowsPerWindow;
dataset.richPairwiseControlCosted = false;
dataset.truthUsedAsFeature = false;
dataset.futureOutcomeUsedAsFeature = false;
dataset.completedAt = datestr(now, 31);
dataset.developmentEvidenceOnly = true;
dataset.validationClaimAllowed = false;
dataset.evidenceBoundary = protocol.evidenceBoundary;

outputRoot = absolutePath(getField(options, 'outputRoot', ...
    fullfile(repoRoot, protocol.outputRoot, 'feature_dataset')), repoRoot);
if exist(outputRoot, 'dir') ~= 7
    mkdir(outputRoot);
end
matPath = fullfile(outputRoot, ...
    'DECISION_AWARE_GATEWAY_V257_TRAINING_DATASET.mat');
save('-mat7-binary', matPath, 'dataset');
fprintf('V257 feature dataset: %d seeds, %d windows, %d action rows\n', ...
    numel(dataset.seeds), dataset.recordCount, dataset.actionRowCount);
fprintf('V257 dataset MAT: %s\n', matPath);
end

function validateSource(dataset, protocol)
required = {'contractVersion', 'datasetRole', 'assemblyGitCommit', ...
    'checkpointPaths', 'seeds', 'featureNames', 'outcomeNames', ...
    'records', 'recordCount', 'actionRowCount', 'actionRowsPerWindow', ...
    'truthUsedAsFeature', 'futureOutcomeUsedAsFeature'};
valid = isstruct(dataset) && isscalar(dataset) && ...
    all(isfield(dataset, required)) && ...
    strcmp(dataset.contractVersion, ...
        'pooled-expected-gateway-v256-dataset-v3') && ...
    strcmp(dataset.datasetRole, 'training') && ...
    isequal(dataset.seeds, protocol.trainingSeeds) && ...
    numel(dataset.checkpointPaths) == numel(dataset.seeds) && ...
    dataset.recordCount == numel(dataset.records) && ...
    dataset.actionRowCount == sum(dataset.actionRowsPerWindow) && ...
    numel(dataset.featureNames) == protocol.expectedFeatureCounts(1) && ...
    ~dataset.truthUsedAsFeature && ~dataset.futureOutcomeUsedAsFeature;
if ~valid
    error('DecisionAwareGatewayV257:InvalidSourceDataset', ...
        'The V257 source is not the frozen V256 training dataset.');
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
