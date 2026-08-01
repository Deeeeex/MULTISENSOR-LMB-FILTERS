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
    case {'formation-shared-velocity', ...
            'formation-shared-motion', 'formation-forward'}
        groupIds = validateGroupIds(config, sensorCount);
        for groupId = unique(groupIds, 'stable')
            groupSensors = find(groupIds == groupId);
            centerVelocity = zeros(2, timeCount);
            for sensorIdx = reshape(groupSensors, 1, [])
                centerVelocity = centerVelocity + ...
                    sensorTrajectories{sensorIdx}(3:4, :);
            end
            centerVelocity = centerVelocity / numel(groupSensors);
            sharedHeading = resolveVelocityHeading(centerVelocity);
            headings(groupSensors, :) = repmat( ...
                sharedHeading, numel(groupSensors), 1);
        end
    case {'formation-shared-fixed', 'formation-fixed'}
        groupIds = validateGroupIds(config, sensorCount);
        if ~isfield(config, 'sensorFovFixedHeadingRadByFormation')
            error(['Formation-fixed FoV headings require ', ...
                'sensorFovFixedHeadingRadByFormation.']);
        end
        fixedHeadings = reshape( ...
            config.sensorFovFixedHeadingRadByFormation, 1, []);
        groups = unique(groupIds, 'stable');
        if isscalar(fixedHeadings)
            fixedHeadings = repmat(fixedHeadings, 1, numel(groups));
        end
        if numel(fixedHeadings) ~= numel(groups) || ...
                any(~isfinite(fixedHeadings))
            error(['Formation-fixed FoV headings require one finite ', ...
                'angle per formation.']);
        end
        for groupIdx = 1:numel(groups)
            groupSensors = find(groupIds == groups(groupIdx));
            headings(groupSensors, :) = fixedHeadings(groupIdx);
        end
    otherwise
        error('Unknown sensorFovHeadingMode: %s', mode);
end
end

function heading = resolveVelocityHeading(velocity)
speed = sqrt(sum(velocity.^2, 1));
moving = speed > 1e-8;
if ~any(moving)
    error('Formation-shared velocity heading requires platform motion.');
end
heading = nan(1, size(velocity, 2));
heading(moving) = atan2(velocity(2, moving), velocity(1, moving));
firstMoving = find(moving, 1, 'first');
heading(1:firstMoving-1) = heading(firstMoving);
for timeIdx = firstMoving+1:numel(heading)
    if ~isfinite(heading(timeIdx))
        heading(timeIdx) = heading(timeIdx-1);
    end
end
heading = unwrap(heading);
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
