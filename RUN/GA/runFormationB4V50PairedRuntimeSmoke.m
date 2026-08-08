function result = runFormationB4V50PairedRuntimeSmoke()
% RUNFORMATIONB4V50PAIREDRUNTIMESMOKE Minimal real-filter sanity check.

protocol = getFormationB4V50RuntimeProtocol();
[model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, identity] = buildControlledCase();
nodeCount = model.numberOfSensors;
timeCount = model.simulationLength;
[commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        49008, identity.sensorPhysicalUids, timeCount);

records = cell(1, 2);
for armIdx = 1:2
    armId = protocol.primaryArms{armIdx};
    if armIdx == 1
        triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
            armId, nodeCount);
    else
        triggerConfig = buildFormationB4V50FixedTriggerConfig( ...
            armId, nodeCount);
    end
    if ~strcmpi(triggerConfig.eventPolicy, 'alwaysHeavy') || ...
            triggerConfig.linkGateEnabled || ...
            ~isempty(commConfig.outageSchedule)
        error('FormationB4V50Smoke:InvalidComparison', ...
            'The controlled comparison requires no gating or outage.');
    end
    [~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, triggerConfig);
    records{armIdx} = summarizeArm( ...
        armId, diagnostics, commConfig, nodeCount, timeCount, protocol);
end

reference = records{1};
candidate = records{2};
expectedCounts = repmat([2 * nodeCount, nodeCount, ...
    nodeCount, nodeCount], 1, timeCount / protocol.period);
commonAttempted = reference.attempted & candidate.attempted;
phaseOneTimes = 1:protocol.period:timeCount;
nonburstTimes = setdiff(1:timeCount, phaseOneTimes);
phaseOneDecisionConsistent = all(arrayfun(@(time) ...
    candidate.cycleSelectedByTime(time) == ...
    ~isequal(reference.adjacencyByTime(:, :, time), ...
        candidate.adjacencyByTime(:, :, time)), phaseOneTimes));
nonburstRoutesMatch = all(arrayfun(@(time) ...
    isequal(reference.adjacencyByTime(:, :, time), ...
        candidate.adjacencyByTime(:, :, time)) && ...
    isequal(reference.weightsByTime{time}, ...
        candidate.weightsByTime{time}), nonburstTimes));

passed = ...
    isequal(reference.messageCountByTime, expectedCounts) && ...
    isequal(candidate.messageCountByTime, expectedCounts) && ...
    isequal(reference.delivered(commonAttempted), ...
        candidate.delivered(commonAttempted)) && ...
    phaseOneDecisionConsistent && nonburstRoutesMatch && ...
    ~any(candidate.cycleSelectedByTime(nonburstTimes));
if ~passed
    error('FormationB4V50Smoke:PairedComparisonFailed', ...
        'The controlled V46/V50 runtime comparison failed.');
end

result = struct();
result.reference = reference;
result.candidate = candidate;
result.expectedMessageCountByTime = expectedCounts;
result.commonAttemptedMessageCount = nnz(commonAttempted);
result.commonDeliveryMatched = true;
result.phaseOneDecisionConsistent = phaseOneDecisionConsistent;
result.nonburstRoutesAndWeightsMatch = nonburstRoutesMatch;
result.measurementsEmpty = all(cellfun(@isempty, measurements(:)));
result.trackingOutcomeScored = false;
fprintf(['V50 runtime smoke: attempts=%d/%d common=%d ', ...
    'cycles=%d/%d delivered-strong(v46/v50)=%d/%d\n'], ...
    reference.totalAttemptedMessageCount, ...
    candidate.totalAttemptedMessageCount, ...
    result.commonAttemptedMessageCount, ...
    nnz(candidate.cycleSelectedByTime), numel(phaseOneTimes), ...
    reference.deliveredStrongWindowCount, ...
    candidate.deliveredStrongWindowCount);
end

function record = summarizeArm(armId, diagnostics, commConfig, ...
        nodeCount, timeCount, protocol)
attempted = logical(diagnostics.attempted);
delivered = logical(diagnostics.delivered);
expectedDelivered = attempted & ...
    (commConfig.linkUniforms >= commConfig.pDropByEdge);
messageCountByTime = reshape( ...
    sum(sum(attempted, 1), 2), 1, []);
if ~isequal(delivered, expectedDelivered)
    error('FormationB4V50Smoke:DeliveryMismatch', ...
        'Delivered messages do not match the controlled loss draws.');
end

adjacencyByTime = false(nodeCount, nodeCount, timeCount);
weightsByTime = cell(1, timeCount);
cycleSelectedByTime = false(1, timeCount);
for currentTime = 1:timeCount
    adjacencyByTime(:, :, currentTime) = logical( ...
        diagnostics.topologyActiveEdge(:, :, currentTime)');
    weightsByTime{currentTime} = ...
        diagnostics.topologyPolicyFusionWeightMatrix{currentTime};
    if strcmp(armId, protocol.candidateArmId)
        schedule = diagnostics. ...
            topologyPolicyScheduleCertificate{currentTime};
        cycleSelectedByTime(currentTime) = schedule.cycleSelected;
    end
end

windowEndTimes = protocol.period:timeCount;
attemptedStrong = false(size(windowEndTimes));
deliveredStrong = false(size(windowEndTimes));
for windowIdx = 1:numel(windowEndTimes)
    windowEnd = windowEndTimes(windowIdx);
    window = windowEnd-protocol.period+1:windowEnd;
    attemptedStrong(windowIdx) = isStronglyConnected( ...
        any(attempted(:, :, window), 3));
    deliveredStrong(windowIdx) = isStronglyConnected( ...
        any(delivered(:, :, window), 3));
end
if ~all(attemptedStrong)
    error('FormationB4V50Smoke:AttemptedRouteDisconnected', ...
        'A controlled attempted-route B4 window is disconnected.');
end

record = struct();
record.armId = armId;
record.messageCountByTime = messageCountByTime;
record.totalAttemptedMessageCount = nnz(attempted);
record.totalDeliveredMessageCount = nnz(delivered);
record.attempted = attempted;
record.delivered = delivered;
record.adjacencyByTime = adjacencyByTime;
record.weightsByTime = weightsByTime;
record.cycleSelectedByTime = cycleSelectedByTime;
record.attemptedStrongWindowCount = nnz(attemptedStrong);
record.deliveredStrongWindowCount = nnz(deliveredStrong);
record.rollingWindowCount = numel(windowEndTimes);
end

function [model, measurements, sensorTrajectories, neighborMap, ...
        commConfig, identity] = buildControlledCase()
formationCount = 4;
sensorsPerFormation = 3;
nodeCount = formationCount * sensorsPerFormation;
timeCount = 8;
model = generateMultisensorModel( ...
    nodeCount, zeros(1, nodeCount), ...
    0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
model.simulationLength = timeCount;
model.sensorCommRange = 82;

sceneConfig = struct();
sceneConfig.formationCount = formationCount;
sceneConfig.sensorsPerFormation = sensorsPerFormation;
sceneConfig.numberOfSensors = nodeCount;
sceneConfig.sensorCenterWaypoints = { ...
    [-35; -35], [35; -35], [35; 35], [-35; 35]};
sceneConfig.sensorGroupIds = repelem( ...
    1:formationCount, sensorsPerFormation);
identity = buildDynamicTopologyPhysicalIdentityRegistry(sceneConfig);
sceneConfig.sensorPhysicalUids = identity.sensorPhysicalUids;
sceneConfig.formationPhysicalUidsBySensor = ...
    identity.formationPhysicalUidsBySensor;
sceneConfig.sensorLocalRoleUidsByFormation = ...
    identity.sensorLocalRoleUidsByFormation;
sceneConfig.physicalIdentityRegistryCanonicalSha256 = ...
    identity.canonicalSha256;
model.dynamicTopologyScenario = struct( ...
    'config', sceneConfig, 'sensor', struct( ...
        'sensorPhysicalUids', identity.sensorPhysicalUids, ...
        'formationPhysicalUidsBySensor', ...
            identity.formationPhysicalUidsBySensor, ...
        'physicalIdentityRegistryCanonicalSha256', ...
            identity.canonicalSha256));

centers = [-35, 35, 35, -35; -35, -35, 35, 35];
localOffsets = [-4, 2, 2; 0, 3, -3];
sensorTrajectories = cell(1, nodeCount);
for formationIdx = 1:formationCount
    for localIdx = 1:sensorsPerFormation
        sensorIdx = (formationIdx - 1) * ...
            sensorsPerFormation + localIdx;
        positions = repmat(centers(:, formationIdx) + ...
            localOffsets(:, localIdx), 1, timeCount);
        sensorTrajectories{sensorIdx} = [ ...
            positions; zeros(2, timeCount)];
    end
end
measurements = repmat({{}}, nodeCount, timeCount);

baseAdjacency = false(nodeCount);
for formationIdx = 1:formationCount
    members = (formationIdx - 1) * sensorsPerFormation + ...
        (1:sensorsPerFormation);
    for localIdx = 1:sensorsPerFormation
        left = members(localIdx);
        right = members(mod(localIdx, sensorsPerFormation) + 1);
        baseAdjacency(left, right) = true;
        baseAdjacency(right, left) = true;
    end
end
for pair = [1, 4; 4, 7; 7, 10]'
    baseAdjacency(pair(1), pair(2)) = true;
    baseAdjacency(pair(2), pair(1)) = true;
end
neighborMap = cell(1, nodeCount);
for receiver = 1:nodeCount
    neighborMap{receiver} = unique([ ...
        receiver, find(baseAdjacency(receiver, :))]);
end

pDropByEdge = zeros(nodeCount, nodeCount, timeCount);
uids = identity.sensorPhysicalUids;
for currentTime = 1:timeCount
    for sender = 1:nodeCount
        for receiver = 1:nodeCount
            pDropByEdge(sender, receiver, currentTime) = ...
                0.05 + mod(3 * uids(sender) + ...
                    7 * uids(receiver) + currentTime, 20) / 100;
        end
    end
    page = pDropByEdge(:, :, currentTime);
    page(1:nodeCount+1:end) = 1;
    pDropByEdge(:, :, currentTime) = page;
end
commConfig = struct( ...
    'forceDelivery', false, ...
    'outageSchedule', [], ...
    'pDropByEdge', pDropByEdge, ...
    'linkUniforms', zeros(nodeCount, nodeCount, timeCount));
end

function connected = isStronglyConnected(adjacency)
connected = reachableAll(adjacency) && reachableAll(adjacency');
end

function passed = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end
