function [currentAdjacency, nextAdjacency, details] = ...
    buildRollingReserveSchedule(groupCount, currentTime, options)
% BUILDROLLINGRESERVESCHEDULE Registered formation-level B=3 reserve.
%
% Adjacency uses sender-row, receiver-column orientation.  Keeping this
% builder public lets execution and attribution independently reconstruct
% the same pre-registered schedule instead of trusting an arm name.

if nargin < 3 || isempty(options)
    options = struct();
end
if ~isscalar(groupCount) || ~isfinite(groupCount) || ...
        groupCount < 2 || mod(groupCount, 1) ~= 0
    error('Rolling reserve groupCount must be an integer above one.');
end
if ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        mod(currentTime, 1) ~= 0
    error('Rolling reserve currentTime must be a finite integer.');
end

anchorTime = round(getField(options, 'anchorTime', 1));
scheduleType = lower(char(getField( ...
    options, 'scheduleType', 'burst')));
temporalPhase = mod(round(getField( ...
    options, 'temporalPhase', 0)), 3);
rootFormation = 1 + mod(round(getField( ...
    options, 'rootFormation', 1)) - 1, groupCount);
orientation = lower(char(getField( ...
    options, 'orientation', 'clockwise')));
cycleAdjacency = false(groupCount);
cycleSenders = zeros(1, groupCount);
cycleReceivers = zeros(1, groupCount);
for senderFormation = 1:groupCount
    if strcmp(orientation, 'clockwise')
        receiverFormation = 1 + mod(senderFormation, groupCount);
    elseif strcmp(orientation, 'counter-clockwise') || ...
            strcmp(orientation, 'counterclockwise')
        orientation = 'counter-clockwise';
        receiverFormation = ...
            1 + mod(senderFormation - 2, groupCount);
    else
        error('Unknown rolling reserve orientation: %s', orientation);
    end
    cycleAdjacency(senderFormation, receiverFormation) = true;
    cycleSenders(senderFormation) = senderFormation;
    cycleReceivers(senderFormation) = receiverFormation;
end

if strcmp(scheduleType, 'burst')
    incomingSender = find(cycleAdjacency(:, rootFormation), 1);
    missingEdge = false(groupCount);
    missingEdge(incomingSender, rootFormation) = true;
    tree = cycleAdjacency & ~missingEdge;
    slot = mod(currentTime - anchorTime + temporalPhase, 3);
    nextSlot = mod(slot + 1, 3);
    currentAdjacency = reserveSlot(slot, tree, missingEdge);
    nextAdjacency = reserveSlot(nextSlot, tree, missingEdge);
    quota = NaN;
    formationPhase = NaN;
elseif strcmp(scheduleType, 'cyclic-chunk')
    quota = round(getField(options, 'quota', groupCount - 1));
    minimumQuota = ceil(groupCount / 3);
    if quota < minimumQuota || quota > groupCount - 1
        error(['Cyclic-chunk quota must lie between ceil(G/3) ', ...
            'and G-1.']);
    end
    formationPhase = mod(round(getField( ...
        options, 'formationPhase', 0)), groupCount);
    currentAdjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime, anchorTime, groupCount);
    nextAdjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime + 1, anchorTime, groupCount);
    tree = false(groupCount);
    missingEdge = false(groupCount);
    slot = NaN;
    nextSlot = NaN;
else
    error('Unknown rolling reserve schedule type: %s', scheduleType);
end

details = struct( ...
    'scheduleType', scheduleType, ...
    'anchorTime', anchorTime, ...
    'temporalPhase', temporalPhase, ...
    'rootFormation', rootFormation, ...
    'orientation', orientation, ...
    'slot', slot, ...
    'nextSlot', nextSlot, ...
    'quota', quota, ...
    'formationPhase', formationPhase, ...
    'cycleAdjacency', cycleAdjacency, ...
    'treeAdjacency', tree, ...
    'missingEdgeAdjacency', missingEdge);
end

function adjacency = chunkScheduleAtTime( ...
        cycleSenders, cycleReceivers, quota, formationPhase, ...
        currentTime, anchorTime, groupCount)
startPosition = mod( ...
    formationPhase + quota * (currentTime - anchorTime), ...
    groupCount);
positions = 1 + mod( ...
    startPosition + (0:(quota - 1)), groupCount);
adjacency = false(groupCount);
for position = positions
    adjacency(cycleSenders(position), ...
        cycleReceivers(position)) = true;
end
end

function adjacency = reserveSlot(slot, tree, missingEdge)
switch slot
    case 0
        adjacency = tree;
    case 1
        adjacency = missingEdge;
    otherwise
        adjacency = false(size(tree));
end
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
