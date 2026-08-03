function [sensorTrajectories, metadata] = ...
    generateMultiFormationTrajectories(config)
% GENERATEMULTIFORMATIONTRAJECTORIES Generate extensible sensor formations.
%
% Formation centres follow preset waypoints. Individual sensors preserve a
% rotated ring around the centre, with one episode-level radius/rotation
% perturbation. The output uses the repository's [x;y;vx;vy] convention.

timeCount = config.simulationLength;
timeValues = 1:timeCount;
sensorCount = config.numberOfSensors;
physicalIdentity = buildDynamicTopologyPhysicalIdentityRegistry(config);
if isfield(config, 'physicalIdentityRegistryCanonicalSha256') && ...
        ~strcmp(config.physicalIdentityRegistryCanonicalSha256, ...
            physicalIdentity.canonicalSha256)
    error('DynamicTopologyPhysicalIdentity:RegistryDrift', ...
        'The trajectory generator physical identity registry drifted.');
end
sensorTrajectories = cell(1, sensorCount);
centerStates = cell(1, config.formationCount);
formationRadii = zeros(1, config.formationCount);
formationRotations = zeros(1, config.formationCount);
radiusJitters = zeros(1, config.formationCount);
rotationJitters = zeros(1, config.formationCount);
[~, canonicalFormationOrder] = sort( ...
    physicalIdentity.formationStochasticRoleUidsByFormation);
for canonicalIdx = 1:config.formationCount
    formationIdx = canonicalFormationOrder(canonicalIdx);
    radiusJitters(formationIdx) = ...
        config.formationRadiusJitterFraction * (2 * rand - 1);
    rotationJitters(formationIdx) = ...
        deg2rad(config.formationRotationJitterDeg) * (2 * rand - 1);
end

sensorCursor = 0;
for groupIdx = 1:config.formationCount
    waypoints = config.sensorCenterWaypoints{groupIdx};
    waypointTimes = config.sensorWaypointTimes;
    if size(waypoints, 1) ~= 2 || ...
            size(waypoints, 2) ~= numel(waypointTimes)
        error(['Each sensor centre waypoint matrix must be 2-by-', ...
            'numel(sensorWaypointTimes).']);
    end
    centerPosition = interpolateWaypoints( ...
        waypointTimes, waypoints, timeValues);
    centerVelocity = finiteDifference(centerPosition, config.samplingPeriod);
    centerStates{groupIdx} = [centerPosition; centerVelocity];

    radiusJitter = radiusJitters(groupIdx);
    radius = config.formationRadius * (1 + radiusJitter);
    rotationJitter = rotationJitters(groupIdx);
    formationRadii(groupIdx) = radius;
    formationRotations(groupIdx) = rotationJitter;

    baseAngles = 2 * pi * (0:config.sensorsPerFormation-1) / ...
        config.sensorsPerFormation;
    if isfield(config, 'formationHeadingMode') && ...
            strcmpi(config.formationHeadingMode, 'fixed')
        fixedHeading = atan2(-centerPosition(2, 1), ...
            -centerPosition(1, 1));
        heading = fixedHeading * ones(1, timeCount);
    else
        heading = resolveFormationHeading(centerPosition, centerVelocity);
    end
    localRoleUids = physicalIdentity. ...
        sensorLocalRoleUidsByFormation{groupIdx};
    for localStorageIdx = 1:config.sensorsPerFormation
        sensorCursor = sensorCursor + 1;
        localRoleUid = localRoleUids(localStorageIdx);
        theta = heading + baseAngles(localRoleUid) + rotationJitter;
        offset = radius * [cos(theta); sin(theta)];
        position = centerPosition + offset;
        velocity = finiteDifference(position, config.samplingPeriod);
        sensorTrajectories{sensorCursor} = [position; velocity];
    end
end

metadata = struct();
metadata.centerStates = centerStates;
metadata.formationRadii = formationRadii;
metadata.formationRotations = formationRotations;
metadata.formationRadiusJittersByPhysicalEntity = radiusJitters;
metadata.formationRotationJittersByPhysicalEntity = rotationJitters;
metadata.formationStochasticDrawOrder = ...
    physicalIdentity.formationStochasticRoleUidsByFormation( ...
        canonicalFormationOrder);
metadata.formationStochasticRoleUidsByFormation = ...
    physicalIdentity.formationStochasticRoleUidsByFormation;
metadata.sensorGroupIds = config.sensorGroupIds;
metadata.physicalIdentityRegistryContractVersion = ...
    physicalIdentity.contractVersion;
metadata.physicalIdentityRegistryCanonicalSha256 = ...
    physicalIdentity.canonicalSha256;
metadata.sensorPhysicalUids = physicalIdentity.sensorPhysicalUids;
metadata.formationPhysicalUidsBySensor = ...
    physicalIdentity.formationPhysicalUidsBySensor;
metadata.sensorLocalRoleUidsBySensor = ...
    physicalIdentity.sensorLocalRoleUidsBySensor;
end

function values = interpolateWaypoints(times, waypoints, queryTimes)
if numel(times) == 1
    values = repmat(waypoints(:, 1), 1, numel(queryTimes));
    return;
end
values = zeros(size(waypoints, 1), numel(queryTimes));
for dimensionIdx = 1:size(waypoints, 1)
    if all(abs(diff(waypoints(dimensionIdx, :))) < eps)
        values(dimensionIdx, :) = waypoints(dimensionIdx, 1);
    else
        values(dimensionIdx, :) = interp1( ...
            times, waypoints(dimensionIdx, :), queryTimes, 'pchip');
    end
end
end

function derivative = finiteDifference(values, sampleTime)
derivative = zeros(size(values));
if size(values, 2) <= 1
    return;
end
derivative(:, 1) = (values(:, 2) - values(:, 1)) / sampleTime;
derivative(:, end) = ...
    (values(:, end) - values(:, end-1)) / sampleTime;
if size(values, 2) > 2
    derivative(:, 2:end-1) = ...
        (values(:, 3:end) - values(:, 1:end-2)) / (2 * sampleTime);
end
end

function heading = resolveFormationHeading(position, velocity)
timeCount = size(position, 2);
heading = zeros(1, timeCount);
speed = sqrt(sum(velocity.^2, 1));
moving = speed > 1e-6;
heading(moving) = atan2(velocity(2, moving), velocity(1, moving));
if any(moving)
    firstMoving = find(moving, 1, 'first');
    heading(1:firstMoving-1) = heading(firstMoving);
    for timeIdx = firstMoving+1:timeCount
        if ~moving(timeIdx)
            heading(timeIdx) = heading(timeIdx-1);
        end
    end
else
    % A stationary diagnostic formation faces the monitored centre.
    heading = atan2(-position(2, :), -position(1, :));
end
heading = unwrap(heading);
end
