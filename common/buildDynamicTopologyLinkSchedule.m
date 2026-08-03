function [pDropByEdge, metadata] = ...
    buildDynamicTopologyLinkSchedule(config, graphData)
% BUILDDYNAMICTOPOLOGYLINKSCHEDULE Time-varying directed drop probabilities.

sensorCount = config.numberOfSensors;
timeCount = config.simulationLength;
pDropByEdge = ones(sensorCount, sensorCount, timeCount);
groupIds = config.sensorGroupIds;
resolvedBlockageWindows = ...
    resolveDynamicTopologyBlockageWindows(config, graphData);

for timeIdx = 1:timeCount
    physical = graphData.physicalAdjacency(:, :, timeIdx);
    distances = graphData.distanceByTime(:, :, timeIdx);
    for leftIdx = 1:sensorCount-1
        for rightIdx = leftIdx+1:sensorCount
            if ~physical(leftIdx, rightIdx)
                probability = 1;
            elseif strcmpi(config.linkMode, 'ideal')
                probability = 0;
            elseif groupIds(leftIdx) == groupIds(rightIdx)
                probability = config.intraFormationDropProbability;
            else
                normalizedDistance = 0;
                if isfinite(config.commRange) && config.commRange > 0
                    normalizedDistance = min( ...
                        distances(leftIdx, rightIdx) / config.commRange, 1);
                end
                probability = config.interFormationDropMinimum + ...
                    config.interFormationDropScale * normalizedDistance^2;
                probability = probability + blockagePenalty( ...
                    resolvedBlockageWindows, groupIds(leftIdx), ...
                    groupIds(rightIdx), timeIdx);
            end
            probability = min(max(probability, 0), 0.95);
            pDropByEdge(leftIdx, rightIdx, timeIdx) = probability;
            pDropByEdge(rightIdx, leftIdx, timeIdx) = probability;
        end
    end
end

metadata = struct();
metadata.meanPhysicalDropProbability = mean( ...
    pDropByEdge(graphData.physicalAdjacency));
metadata.blockageWindows = resolvedBlockageWindows;
metadata.blockageScheduleMode = getField( ...
    config, 'blockageScheduleMode', 'explicit');
end

function penalty = blockagePenalty(windows, leftGroup, rightGroup, timeIdx)
penalty = 0;
for windowIdx = 1:size(windows, 1)
    pairMatches = ...
        (windows(windowIdx, 1) == leftGroup && ...
         windows(windowIdx, 2) == rightGroup) || ...
        (windows(windowIdx, 1) == rightGroup && ...
         windows(windowIdx, 2) == leftGroup);
    inWindow = timeIdx >= windows(windowIdx, 3) && ...
        timeIdx <= windows(windowIdx, 4);
    if pairMatches && inWindow
        penalty = max(penalty, 0.65);
    end
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
