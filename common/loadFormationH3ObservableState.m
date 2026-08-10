function state = loadFormationH3ObservableState( ...
        presetName, seed, currentTime, options)
% LOADFORMATIONH3OBSERVABLESTATE Rebuild a deployable H=3 feature state.
%
% The returned context contains the current local LMB posteriors, current
% link probabilities, current sensor positions, and two past topology pages.
% Explicit target trajectories, future link uniforms, and future link-probability
% pages are removed before the context is exposed to a feature extractor.

if nargin < 4 || isempty(options)
    options = struct();
end
if ~ischar(presetName) || ~isscalar(seed) || ~isfinite(seed) || ...
        seed ~= round(seed) || seed < 1 || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime ~= round(currentTime) || currentTime < 1
    error('Formation H=3 observable-state request is invalid.');
end
cacheRoot = getField(options, 'cacheRoot', fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'posterior_risk_v10', 'opened_state_audit', 'cache'));
% Event scans load many snapshots from one deterministic scenario.  Keep
% the generated inputs within the current Octave process so each cache read
% does not rebuild identical measurements and geometry.
persistent cachedInputKey cachedInputs
inputKey = sprintf('%s::%d', presetName, seed);
if isempty(cachedInputKey) || ~strcmp(cachedInputKey, inputKey)
    cachedInputs = generateDynamicTopologyScenarioInputs(presetName, seed);
    cachedInputKey = inputKey;
end
inputs = cachedInputs;
cachePath = fullfile(cacheRoot, sprintf( ...
    '%s_seed%d_n1_sig%d.mat', strrep(presetName, '-', '_'), ...
    seed, currentTime));
if exist(cachePath, 'file') ~= 2
    error('Missing formation H=3 observable-state cache: %s', cachePath);
end
loaded = load(cachePath, 'behaviorBundle');
if ~isfield(loaded, 'behaviorBundle')
    error('Formation H=3 cache lacks behaviorBundle.');
end
bundle = loaded.behaviorBundle;
required = { ...
    'presetName', 'seed', 'snapshotTimes', 'configSnapshot', ...
    'posteriorSnapshots', 'preDecisionTopologyHistoryByTime', ...
    'openedDevelopmentEvidenceOnly', 'validationClaimAllowed'};
if ~all(isfield(bundle, required)) || ...
        ~strcmp(bundle.presetName, presetName) || ...
        bundle.seed ~= seed || ...
        ~isequal(bundle.snapshotTimes, currentTime) || ...
        ~isequaln(bundle.configSnapshot, inputs.config) || ...
        ~bundle.openedDevelopmentEvidenceOnly || ...
        bundle.validationClaimAllowed || ...
        numel(bundle.posteriorSnapshots) < currentTime || ...
        isempty(bundle.posteriorSnapshots{currentTime}) || ...
        numel(bundle.preDecisionTopologyHistoryByTime) < currentTime || ...
        isempty(bundle.preDecisionTopologyHistoryByTime{currentTime})
    error('Formation H=3 observable-state cache provenance mismatch.');
end
history = bundle.preDecisionTopologyHistoryByTime{currentTime};
if ~isfield(history, 'selectedDirectedEdgeHistory') || ...
        ~isfield(history, 'deliveredDirectedEdgeHistory') || ...
        ~isfield(history, 'times') || ...
        size(history.selectedDirectedEdgeHistory, 3) ~= 2 || ...
        size(history.deliveredDirectedEdgeHistory, 3) ~= 2 || ...
        numel(history.times) ~= 2 || ...
        any(history.times >= currentTime)
    error('Formation H=3 observable state lacks past topology history.');
end

model = removeExplicitTargetTruth(inputs.model);
commConfig = currentLinkConfiguration( ...
    inputs.commConfig, currentTime);
selectedHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.selectedDirectedEdgeHistory);
context = struct();
context.localPosteriorBySensor = ...
    bundle.posteriorSnapshots{currentTime};
context.model = model;
context.commConfig = commConfig;
context.currentTime = currentTime;
context.previousAdjacencyHistory = selectedHistory;
context.previousAdjacencyHistoryCount = size(selectedHistory, 3);
context.previousAdjacencyHistoryTimes = history.times;
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
context.previousAdjacencyHistorySource = ...
    'opened-reference-continuation-cache-v1';
context.previousAdjacency = selectedHistory(:, :, end);
context.baseAdjacency = inputs.graphData.staticAdjacency;
context.physicalAdjacency = logical( ...
    inputs.graphData.physicalAdjacency(:, :, currentTime));
context.edgeBudget = inputs.config.edgeBudget;
context.directedMessageBudget = ...
    2 * inputs.config.numberOfSensors;
context.positions = ...
    inputs.graphData.positions(:, :, currentTime);

state = struct();
state.contractVersion = ...
    'formation-h3-observable-state-v1';
state.presetName = presetName;
state.seed = seed;
state.currentTime = currentTime;
state.context = context;
state.groupIds = reshape( ...
    inputs.config.sensorGroupIds, 1, []);
state.cachePath = cachePath;
state.cacheSha256 = computeFileSha256(cachePath);
state.truthUsed = false;
state.futureMeasurementsUsed = false;
state.futureLinkUniformsAvailable = false;
state.futureLinkProbabilitiesAvailable = false;
state.numericSeedUsedAsFeature = false;
state.openedDevelopmentEvidenceOnly = true;
state.validationClaimAllowed = false;
end

function model = removeExplicitTargetTruth(model)
if isfield(model, 'explicitTargetTrajectories')
    model = rmfield(model, 'explicitTargetTrajectories');
end
if isfield(model, 'dynamicTopologyScenario') && ...
        isstruct(model.dynamicTopologyScenario) && ...
        isscalar(model.dynamicTopologyScenario)
    removableFields = intersect( ...
        {'targetTrajectories', 'target'}, ...
        fieldnames(model.dynamicTopologyScenario), 'stable');
    if ~isempty(removableFields)
        model.dynamicTopologyScenario = rmfield( ...
            model.dynamicTopologyScenario, removableFields);
    end
end
end

function config = currentLinkConfiguration(config, currentTime)
if isfield(config, 'linkUniforms')
    config = rmfield(config, 'linkUniforms');
end
if isfield(config, 'pDropByEdge') && ...
        ndims(config.pDropByEdge) >= 3
    if currentTime > size(config.pDropByEdge, 3)
        error('Current link-probability page is unavailable.');
    end
    config.pDropByEdge = ...
        config.pDropByEdge(:, :, currentTime);
end
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
