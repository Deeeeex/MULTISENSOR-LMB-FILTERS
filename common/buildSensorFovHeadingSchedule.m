function headings = buildSensorFovHeadingSchedule( ...
    config, sensorTrajectories)
% BUILDSENSORFOVHEADINGSCHEDULE Decouple sensor boresight from platform speed.
%
% Rows correspond to sensors and columns to time. NaN means that the legacy
% velocity-derived heading should be used. A separate boresight schedule is
% important for mobile surveillance platforms, whose sensor may keep looking
% at the monitored region while the platform moves tangentially.

sensorCount = numel(sensorTrajectories);
timeCount = config.simulationLength;
headings = nan(sensorCount, timeCount);
mode = lower(getField(config, ...
    'sensorFovHeadingMode', 'velocity'));

switch mode
    case {'velocity', 'legacy'}
        return;
    case {'omnidirectional', 'none'}
        return;
    case {'scene-center', 'inward'}
        targetPoint = reshape(getField( ...
            config, 'sensorFovPointingCenter', [0; 0]), 2, 1);
        for sensorIdx = 1:sensorCount
            positions = sensorTrajectories{sensorIdx}(1:2, :);
            relative = targetPoint - positions;
            headings(sensorIdx, :) = atan2(relative(2, :), relative(1, :));
        end
    case {'formation-shared-scene-center', ...
            'formation-shared-inward', 'formation-center'}
        groupIds = validateGroupIds(config, sensorCount);
        targetPoint = reshape(getField( ...
            config, 'sensorFovPointingCenter', [0; 0]), 2, 1);
        for groupId = unique(groupIds, 'stable')
            groupSensors = find(groupIds == groupId);
            centerPositions = zeros(2, timeCount);
            for sensorIdx = reshape(groupSensors, 1, [])
                centerPositions = centerPositions + ...
                    sensorTrajectories{sensorIdx}(1:2, :);
            end
            centerPositions = centerPositions / numel(groupSensors);
            relative = targetPoint - centerPositions;
            sharedHeading = atan2(relative(2, :), relative(1, :));
            headings(groupSensors, :) = repmat( ...
                sharedHeading, numel(groupSensors), 1);
        end
    otherwise
        error('Unknown sensorFovHeadingMode: %s', mode);
end
end

function groupIds = validateGroupIds(config, sensorCount)
if ~isstruct(config) || ~isfield(config, 'sensorGroupIds')
    error(['Formation-shared FoV headings require one sensorGroupIds ', ...
        'entry per sensor.']);
end
groupIds = reshape(config.sensorGroupIds, 1, []);
if numel(groupIds) ~= sensorCount || any(~isfinite(groupIds)) || ...
        any(groupIds < 1) || any(groupIds ~= round(groupIds))
    error(['Formation-shared FoV headings require one positive integer ', ...
        'sensorGroupIds entry per sensor.']);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
