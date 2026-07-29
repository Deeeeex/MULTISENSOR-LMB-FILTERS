function [inputs, context, sourceState] = ...
    loadBackboneResidualReturnHeadroomState(options)
% LOADBACKBONERESIDUALRETURNHEADROOMSTATE Reconstruct frozen M24 t=75 state.

if nargin < 1 || isempty(options)
    options = struct();
end
protocol = getBackboneResidualReturnHeadroomProtocol();
inputs = generateDynamicTopologyScenarioInputs( ...
    protocol.presetName, protocol.seed);
if ~inputs.validation.isValid || ...
        inputs.config.numberOfSensors ~= 24
    error('Frozen return-headroom scenario is invalid.');
end
cachePath = getField(options, ...
    'behaviorCachePath', fullfile( ...
        'RUN', 'GA', 'dynamic_topology', 'cache', ...
        'm24_hard_seed7_n1_sig75.mat'));
if exist(cachePath, 'file') ~= 2
    error('Missing frozen M24 continuation cache: %s', cachePath);
end
loaded = load(cachePath, 'behaviorBundle');
if ~isfield(loaded, 'behaviorBundle')
    error('Frozen continuation cache lacks behaviorBundle.');
end
bundle = loaded.behaviorBundle;
expectedBehaviorConfig = ...
    buildMixtureAwareKlaReferenceConfig();
expectedBehaviorConfig.dynamicTopologyEnabled = false;
expectedBehaviorConfig.topologyPosteriorCaptureTimes = ...
    protocol.currentTime;
required = { ...
    'presetName', 'seed', 'configSnapshot', ...
    'behaviorConfigSnapshot', 'snapshotTimes'};
if ~all(isfield(bundle, required)) || ...
        ~strcmp(bundle.presetName, protocol.presetName) || ...
        bundle.seed ~= protocol.seed || ...
        ~isequaln(bundle.configSnapshot, inputs.config) || ...
        ~isequaln(bundle.behaviorConfigSnapshot, ...
            expectedBehaviorConfig) || ...
        ~isequal(bundle.snapshotTimes, ...
            protocol.currentTime)
    error('Frozen continuation cache metadata differs from protocol.');
end
[localPosteriors, history] = ...
    extractBehaviorContinuationSnapshot( ...
        bundle, protocol.currentTime, ...
        inputs.config.numberOfSensors);
policyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.selectedDirectedEdgeHistory);

nodeCount = inputs.config.numberOfSensors;
positions = zeros(2, nodeCount);
for sensorIdx = 1:nodeCount
    positions(:, sensorIdx) = ...
        inputs.sensorTrajectories{sensorIdx}( ...
            1:2, protocol.currentTime);
end
commRange = getField(inputs.model, ...
    'sensorCommRange', inf);
physical = false(nodeCount);
for leftIdx = 1:(nodeCount - 1)
    for rightIdx = (leftIdx + 1):nodeCount
        if ~isfinite(commRange) || ...
                norm(positions(:, leftIdx) - ...
                    positions(:, rightIdx)) <= commRange
            physical(leftIdx, rightIdx) = true;
            physical(rightIdx, leftIdx) = true;
        end
    end
end

context = struct();
context.localPosteriorBySensor = localPosteriors;
context.model = inputs.model;
context.sensorTrajectories = inputs.sensorTrajectories;
context.commConfig = inputs.commConfig;
context.triggerConfig = expectedBehaviorConfig;
context.currentTime = protocol.currentTime;
context.previousAdjacencyHistory = policyHistory;
context.previousAdjacencyHistoryCount = ...
    size(policyHistory, 3);
context.previousAdjacencyHistoryTimes = ...
    reshape(history.times, 1, []);
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
if isempty(policyHistory)
    context.previousAdjacency = false(nodeCount);
else
    context.previousAdjacency = ...
        policyHistory(:, :, end);
end
context.previousAdjacencyHistorySource = ...
    'behavior-filter-diagnostics-cache-v2';
context.baseAdjacency = neighborMapToAdjacency( ...
    inputs.neighborMap, nodeCount);
context.physicalAdjacency = physical;
context.edgeScores = double(physical);
context.edgeBudget = inputs.config.edgeBudget;
context.directedMessageBudget = 2 * nodeCount;
context.positions = positions;

sourceState = struct();
sourceState.contractVersion = ...
    'backbone-residual-return-source-state-v1';
sourceState.cachePath = cachePath;
sourceState.cacheSha256 = computeFileSha256(cachePath);
sourceState.localPosteriors = localPosteriors;
sourceState.previousAdjacencyHistory = policyHistory;
sourceState.previousDeliveredAdjacencyHistory = ...
    convertDiagnosticEdgeHistoryToPolicyAdjacencyHistory( ...
        history.deliveredDirectedEdgeHistory);
sourceState.previousAdjacencyHistoryTimes = ...
    reshape(history.times, 1, []);
sourceState.groupIds = reshape( ...
    inputs.config.sensorGroupIds, 1, []);
sourceState.physicalAdjacency = physical;
sourceState.currentTime = protocol.currentTime;
end

function adjacency = ...
    neighborMapToAdjacency(neighborMap, nodeCount)
adjacency = false(nodeCount);
for receiverIdx = 1:nodeCount
    senders = reshape(neighborMap{receiverIdx}, 1, []);
    senders = senders(senders ~= receiverIdx);
    adjacency(receiverIdx, senders) = true;
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
