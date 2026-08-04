function [model, measurements, sensorTrajectories, neighborMap, ...
    commConfig, identity] = buildFormationB4RuntimeSmokeCase()
% BUILDFORMATIONB4RUNTIMESMOKECASE Small deterministic repair/runtime case.

formationCount = 3;
sensorsPerFormation = 3;
nodeCount = formationCount * sensorsPerFormation;
timeCount = 8;
model = generateMultisensorModel( ...
    nodeCount, zeros(1, nodeCount), ...
    0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
model.simulationLength = timeCount;
model.sensorCommRange = 60;

sceneConfig = struct();
sceneConfig.formationCount = formationCount;
sceneConfig.sensorsPerFormation = sensorsPerFormation;
sceneConfig.numberOfSensors = nodeCount;
sceneConfig.sensorCenterWaypoints = { ...
    [-50; 0], [0; 0], [50; 0]};
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
sensorMetadata = struct( ...
    'sensorPhysicalUids', identity.sensorPhysicalUids, ...
    'formationPhysicalUidsBySensor', ...
        identity.formationPhysicalUidsBySensor, ...
    'physicalIdentityRegistryCanonicalSha256', ...
        identity.canonicalSha256);
model.dynamicTopologyScenario = struct( ...
    'config', sceneConfig, 'sensor', sensorMetadata);

centers = zeros(2, formationCount, timeCount);
centers(:, :, 1:4) = repmat( ...
    reshape([-50, 0, 50; 0, 0, 0], 2, formationCount, 1), ...
    1, 1, 4);
centers(:, :, 5:8) = repmat( ...
    reshape([0, -50, 50; 0, 0, 0], 2, formationCount, 1), ...
    1, 1, 4);
localOffsets = [-4, 2, 2; 0, 3, -3];
sensorTrajectories = cell(1, nodeCount);
for formationIdx = 1:formationCount
    for localIdx = 1:sensorsPerFormation
        sensorIdx = (formationIdx - 1) * ...
            sensorsPerFormation + localIdx;
        positions = squeeze(centers(:, formationIdx, :));
        positions = positions + repmat( ...
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
crossPlaceholders = [1, 4; 4, 7];
for edgeIdx = 1:size(crossPlaceholders, 1)
    left = crossPlaceholders(edgeIdx, 1);
    right = crossPlaceholders(edgeIdx, 2);
    baseAdjacency(left, right) = true;
    baseAdjacency(right, left) = true;
end
neighborMap = cell(1, nodeCount);
for receiverIdx = 1:nodeCount
    neighborMap{receiverIdx} = unique([ ...
        receiverIdx, find(baseAdjacency(receiverIdx, :))]);
end

pDropByEdge = zeros(nodeCount, nodeCount, timeCount);
uids = identity.sensorPhysicalUids;
for currentTime = 1:timeCount
    for senderIdx = 1:nodeCount
        for receiverIdx = 1:nodeCount
            pDropByEdge(senderIdx, receiverIdx, currentTime) = ...
                0.15 + mod(5 * uids(senderIdx) + ...
                    7 * uids(receiverIdx) + currentTime, 30) / 100;
        end
    end
    currentDropPage = pDropByEdge(:, :, currentTime);
    currentDropPage(1:nodeCount+1:end) = 1;
    pDropByEdge(:, :, currentTime) = currentDropPage;
end
commConfig = struct( ...
    'forceDelivery', false, ...
    'pDropByEdge', pDropByEdge, ...
    'linkUniforms', zeros(nodeCount, nodeCount, timeCount));
end
