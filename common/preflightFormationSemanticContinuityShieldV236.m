function result = preflightFormationSemanticContinuityShieldV236(inputs)
% PREFLIGHTFORMATIONSEMANTICCONTINUITYSHIELDV236 Structural teacher probe.

protocol = getFormationSemanticContinuityShieldV236Protocol();
if ~isstruct(inputs) || ~isscalar(inputs) || ...
        ~all(isfield(inputs, {'config', 'model', 'graphData', ...
            'commConfig'})) || ...
        ~strcmp(inputs.config.presetName, protocol.presetName) || ...
        inputs.seed ~= protocol.seed
    error('FormationSemanticContinuityV236:InvalidPreflightInput', ...
        'V236 preflight is frozen to the V235 M24 teacher case.');
end

base = preflightCorrectedStaticRoutingBaselineV227(inputs);
registration = base.staticRegistration;
nodeCount = inputs.config.numberOfSensors;
timeCount = inputs.config.simulationLength;
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
context.directedMessageBudget = ...
    protocol.directedInputsPerReceiver * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;

pages = false(nodeCount, nodeCount, timeCount);
weights = cell(1, timeCount);
sensorStrong = false(1, timeCount);
formationStrong = false(1, timeCount);
shieldApplied = false(1, timeCount);
physicalPassed = false(1, timeCount);
messageParityPassed = false(1, timeCount);
weightPassed = false(1, timeCount);
for currentTime = 1:timeCount
    context.currentTime = currentTime;
    context.positions = inputs.graphData.positions(:, :, currentTime);
    context.physicalAdjacency = logical( ...
        inputs.graphData.physicalAdjacency(:, :, currentTime));
    context.commConfig = struct('pDropByEdge', ...
        inputs.commConfig.pDropByEdge(:, :, currentTime));
    [pages(:, :, currentTime), details] = ...
        selectFormationSemanticContinuityShieldV236Policy( ...
            context, registration);
    weights{currentTime} = details.fusionWeightMatrix;
    sensorStrong(currentTime) = ...
        details.instantaneousSensorStrongConnected;
    formationStrong(currentTime) = ...
        details.instantaneousFormationStrongConnected;
    shieldApplied(currentTime) = details.shieldApplied;
    physicalPassed(currentTime) = ~any(any( ...
        pages(:, :, currentTime) & ~context.physicalAdjacency));
    messageParityPassed(currentTime) = ...
        nnz(pages(:, :, currentTime)) == ...
            protocol.directedInputsPerReceiver * nodeCount;
    support = pages(:, :, currentTime) | logical(eye(nodeCount));
    weightPassed(currentTime) = all(abs(sum(weights{currentTime}, 2) - 1) ...
        <= 1e-12) && ~any(weights{currentTime}(:) < 0) && ...
        ~any(weights{currentTime}(:) > 0 & ~support(:));
end

groupIds = reshape(inputs.config.sensorGroupIds, 1, []);
members = find(groupIds == protocol.targetFormationId);
dynamicBefore = true;
staticTargetAfter = true;
for currentTime = 1:min(protocol.shieldStartTime - 1, timeCount)
    [dynamicPage, ~] = reconstructDynamicPage(inputs, identity, ...
        currentTime, protocol);
    dynamicBefore = dynamicBefore && ...
        isequal(pages(:, :, currentTime), dynamicPage);
end
for currentTime = protocol.shieldStartTime:timeCount
    staticTargetAfter = staticTargetAfter && isequal( ...
        pages(members, :, currentTime), ...
        registration.adjacency(members, :));
end

result = struct();
result.contractVersion = ...
    'formation-semantic-continuity-shield-v236-preflight-v1';
result.protocolId = protocol.id;
result.presetName = inputs.config.presetName;
result.seed = inputs.seed;
result.staticRegistration = registration;
result.pages = pages;
result.fusionWeightMatrixByTime = weights;
result.uniqueTopologyCount = uniquePageCount(pages);
result.topologyChangeCount = pageChangeCount(pages);
result.messageCountPerRound = ...
    protocol.directedInputsPerReceiver * nodeCount;
result.shieldAppliedPageCount = nnz(shieldApplied);
result.firstShieldPage = find(shieldApplied, 1);
result.allPhysicalPassed = all(physicalPassed);
result.allMessageParityPassed = all(messageParityPassed);
result.allWeightPassed = all(weightPassed);
result.allSensorStrongConnected = all(sensorStrong);
result.allFormationStrongConnected = all(formationStrong);
result.sensorDisconnectedTimes = find(~sensorStrong);
result.formationDisconnectedTimes = find(~formationStrong);
result.dynamicBeforeShieldPassed = dynamicBefore;
result.staticTargetRowsAfterShieldPassed = staticTargetAfter;
result.runtimeTruthUsed = false;
result.runtimeFutureOutcomeUsed = false;
result.teacherScheduleOutcomeDerived = true;
result.structuralPass = result.allPhysicalPassed && ...
    result.allMessageParityPassed && result.allWeightPassed && ...
    result.allSensorStrongConnected && ...
    result.allFormationStrongConnected && ...
    result.dynamicBeforeShieldPassed && ...
    result.staticTargetRowsAfterShieldPassed && ...
    result.firstShieldPage == protocol.shieldStartTime;
end

function [page, details] = reconstructDynamicPage( ...
        inputs, identity, currentTime, protocol)
nodeCount = inputs.config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.baseAdjacency = logical(inputs.graphData.staticAdjacency);
context.directedMessageBudget = ...
    protocol.directedInputsPerReceiver * nodeCount;
context.sensorPhysicalUids = identity.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
context.currentTime = currentTime;
context.positions = inputs.graphData.positions(:, :, currentTime);
context.physicalAdjacency = logical( ...
    inputs.graphData.physicalAdjacency(:, :, currentTime));
context.commConfig = struct('pDropByEdge', ...
    inputs.commConfig.pDropByEdge(:, :, currentTime));
[page, details] = ...
    selectCorrectedDynamicFormationBackboneV227Policy(context);
end

function count = uniquePageCount(pages)
flat = reshape(logical(pages), [], size(pages, 3))';
count = size(unique(flat, 'rows'), 1);
end

function count = pageChangeCount(pages)
count = 0;
for timeIndex = 2:size(pages, 3)
    count = count + ~isequal( ...
        pages(:, :, timeIndex), pages(:, :, timeIndex - 1));
end
end
