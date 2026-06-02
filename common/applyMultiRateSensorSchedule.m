function [measurementsScheduled, samplingStats] = applyMultiRateSensorSchedule(measurements, samplingPeriods, phaseOffsets)
% APPLYMULTIRATESENSORSCHEDULE - Gate measurements by per-sensor sampling period.
%   Non-sampling instants are marked separately from scheduled scans with no
%   detections so multi-rate experiments can propagate stale posteriors
%   without treating every skipped scan as a missed detection.
%   File guide:
%       Pre-filter scheduler for asynchronous sensors. It removes scans at
%       unscheduled time steps and returns age/mask diagnostics consumed by
%       freshness and information-decay weighting experiments.

if nargin < 2 || isempty(samplingPeriods)
    measurementsScheduled = measurements;
    samplingStats = struct();
    return;
end

if ~iscell(measurements)
    error('measurements must be a cell array.');
end

numSensors = size(measurements, 1);
numSteps = size(measurements, 2);
samplingPeriods = round(reshape(samplingPeriods, 1, []));
if numel(samplingPeriods) ~= numSensors
    error('samplingPeriods must have one entry per sensor.');
end
samplingPeriods = max(samplingPeriods, 1);

if nargin < 3 || isempty(phaseOffsets)
    phaseOffsets = zeros(1, numSensors);
end
phaseOffsets = round(reshape(phaseOffsets, 1, []));
if numel(phaseOffsets) ~= numSensors
    error('phaseOffsets must have one entry per sensor.');
end
phaseOffsets = mod(phaseOffsets, samplingPeriods);

measurementsScheduled = measurements;
sensorSampleMask = false(numSensors, numSteps);
sensorSampleAge = zeros(numSensors, numSteps);
droppedBySchedule = zeros(numSensors, numSteps);

for s = 1:numSensors
    lastSampleStep = 1;
    for t = 1:numSteps
        isSampleStep = mod((t - 1) - phaseOffsets(s), samplingPeriods(s)) == 0;
        sensorSampleMask(s, t) = isSampleStep;
        if isSampleStep
            lastSampleStep = t;
            sensorSampleAge(s, t) = 0;
        else
            droppedBySchedule(s, t) = numel(measurements{s, t});
            measurementsScheduled{s, t} = {};
            sensorSampleAge(s, t) = t - lastSampleStep;
        end
    end
end

samplingStats = struct();
samplingStats.sensorSampleMask = sensorSampleMask;
samplingStats.sensorSampleAge = sensorSampleAge;
samplingStats.droppedBySchedule = droppedBySchedule;
samplingStats.samplingPeriods = samplingPeriods;
samplingStats.samplingPhaseOffsets = phaseOffsets;
end
