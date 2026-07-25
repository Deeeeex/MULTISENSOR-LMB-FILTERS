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
    otherwise
        error('Unknown sensorFovHeadingMode: %s', mode);
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
