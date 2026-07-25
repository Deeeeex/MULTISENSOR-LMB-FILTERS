function [targetTrajectories, metadata] = ...
    generateCorridorTargetTrajectories(config)
% GENERATECORRIDORTARGETTRAJECTORIES Generate labelled corridor crossings.
%
% Each target group follows a shared centre route. Group members receive
% fixed cross-track offsets so handover is caused by finite sensing support,
% not by labels appearing at different spatial templates.

timeCount = config.simulationLength;
targetCount = config.numberOfTargets;
targetTrajectories = cell(1, targetCount);
birthTimes = zeros(1, targetCount);
deathTimes = zeros(1, targetCount);
groupIds = zeros(1, targetCount);
targetCursor = 0;

for groupIdx = 1:config.targetGroupCount
    birthTime = round(config.targetBirthTimesByGroup(groupIdx));
    deathTime = round(config.targetDeathTimesByGroup(groupIdx));
    if birthTime < 1 || deathTime > timeCount || deathTime < birthTime
        error('Invalid birth/death interval for target group %d.', groupIdx);
    end
    activeTimes = birthTime:deathTime;
    route = config.targetRoutes{groupIdx};
    if size(route, 1) ~= 2 || size(route, 2) < 2
        error('Each target route must contain at least two 2-D waypoints.');
    end
    if isfield(config, 'normalizeTargetRouteDuration') && ...
            config.normalizeTargetRouteDuration
        durations = config.targetDeathTimesByGroup - ...
            config.targetBirthTimesByGroup + 1;
        durationScale = numel(activeTimes) / max(durations);
        anchor = route(:, ceil(size(route, 2) / 2));
        route = anchor + durationScale * (route - anchor);
    end
    routeTimes = linspace(birthTime, deathTime, size(route, 2));
    centerPosition = interpolateRoute(routeTimes, route, activeTimes);
    tangent = finiteDifference(centerPosition, config.samplingPeriod);
    normal = normalizedNormal(tangent);
    offsets = centeredOffsets( ...
        config.targetsPerTargetGroup, config.targetCrossTrackSpacing);

    for localIdx = 1:config.targetsPerTargetGroup
        targetCursor = targetCursor + 1;
        position = centerPosition + offsets(localIdx) * normal;
        velocity = finiteDifference(position, config.samplingPeriod);
        trajectory = nan(4, timeCount);
        trajectory(:, activeTimes) = [position; velocity];
        targetTrajectories{targetCursor} = trajectory;
        birthTimes(targetCursor) = birthTime;
        deathTimes(targetCursor) = deathTime;
        groupIds(targetCursor) = groupIdx;
    end
end

metadata = struct();
metadata.birthTimes = birthTimes;
metadata.deathTimes = deathTimes;
metadata.targetGroupIds = groupIds;
end

function values = interpolateRoute(times, waypoints, queryTimes)
values = zeros(2, numel(queryTimes));
for dimensionIdx = 1:2
    values(dimensionIdx, :) = interp1( ...
        times, waypoints(dimensionIdx, :), queryTimes, 'pchip');
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

function normal = normalizedNormal(tangent)
normal = [-tangent(2, :); tangent(1, :)];
norms = sqrt(sum(normal.^2, 1));
for timeIdx = 1:size(normal, 2)
    if norms(timeIdx) > 1e-9
        normal(:, timeIdx) = normal(:, timeIdx) / norms(timeIdx);
    elseif timeIdx > 1
        normal(:, timeIdx) = normal(:, timeIdx-1);
    else
        normal(:, timeIdx) = [0; 1];
    end
end
end

function offsets = centeredOffsets(count, spacing)
if count <= 1
    offsets = 0;
else
    offsets = ((1:count) - (count + 1) / 2) * spacing;
end
end
