function audit = auditRollingScheduleExecution( ...
        armMode, diagnostics, analysisTimes, groupIds)
% AUDITROLLINGSCHEDULEEXECUTION Independently verify registered behavior.
%
% The policy emits its proposed formation schedule and metadata.  This
% audit rebuilds the proposal from the canonical arm mode, compares every
% executed time step, and derives the selected-route signature directly
% from validated sensor-level topology diagnostics.

mode = lower(strrep(char(armMode), '_', '-'));
analysisTimes = reshape(analysisTimes, 1, []);
groupIds = reshape(groupIds, 1, []);
groups = unique(groupIds, 'stable');
groupCount = numel(groups);
audit = emptyAudit();

[registered, scheduled, scheduleOptions] = ...
    parseRollingMode(mode, groupCount);
if ~registered || isempty(analysisTimes)
    return;
end
audit.available = true;
audit.scheduled = scheduled;

activeEdges = diagnostics.topologyActiveEdge(:, :, analysisTimes);
selectedTrace = formationTraceFromSensorEdges( ...
    activeEdges, groupIds, groups);
audit.selectedRouteSignature = traceSignature(selectedTrace);
audit.maximumActualCrossEdgeCount = maximumCrossEdgeCount( ...
    activeEdges, groupIds);
audit.formationPairCapPassFraction = ...
    formationPairCapPassFraction(activeEdges, groupIds, groups);

maximumCrossEdges = getNumericDiagnostic( ...
    diagnostics, 'topologyPolicyMaximumCrossEdges', analysisTimes);
actualCrossCounts = crossEdgeCountByTime(activeEdges, groupIds);
audit.registeredCrossCapExactFraction = mean(double( ...
    isfinite(maximumCrossEdges) & ...
    maximumCrossEdges == groupCount - 1 & ...
    actualCrossCounts <= groupCount - 1));

if ~scheduled
    return;
end

metadataMatches = false(1, numel(analysisTimes));
proposalMatches = false(1, numel(analysisTimes));
expectedTrace = false(groupCount, groupCount, numel(analysisTimes));
observedTrace = false(groupCount, groupCount, numel(analysisTimes));
observedPresent = false(1, numel(analysisTimes));
for cursor = 1:numel(analysisTimes)
    timeIdx = analysisTimes(cursor);
    [expectedProposal, ~, expectedDetails] = ...
        buildRollingReserveSchedule( ...
            groupCount, timeIdx, scheduleOptions);
    expectedTrace(:, :, cursor) = expectedProposal;

    observedDetails = getCellDiagnostic( ...
        diagnostics, 'topologyPolicyReserveSchedule', timeIdx, struct());
    metadataMatches(cursor) = ...
        isstruct(observedDetails) && ...
        isequaln(observedDetails, expectedDetails);

    observedProposal = getCellDiagnostic( ...
        diagnostics, ...
        'topologyPolicyReserveProposalFormationAdjacency', ...
        timeIdx, []);
    if isequal(size(observedProposal), [groupCount, groupCount])
        observedPresent(cursor) = true;
        observedTrace(:, :, cursor) = logical(observedProposal);
        proposalMatches(cursor) = ...
            isequal(logical(observedProposal), expectedProposal);
    end
end
audit.registeredScheduleMetadataMatchFraction = ...
    mean(double(metadataMatches));
audit.registeredProposalMatchFraction = ...
    mean(double(proposalMatches));
audit.expectedProposalSignature = traceSignature(expectedTrace);
if all(observedPresent)
    audit.observedProposalSignature = traceSignature(observedTrace);
else
    audit.observedProposalSignature = 'missing';
end
end

function audit = emptyAudit()
audit = struct( ...
    'available', false, ...
    'scheduled', false, ...
    'registeredScheduleMetadataMatchFraction', NaN, ...
    'registeredProposalMatchFraction', NaN, ...
    'registeredCrossCapExactFraction', NaN, ...
    'formationPairCapPassFraction', NaN, ...
    'maximumActualCrossEdgeCount', NaN, ...
    'expectedProposalSignature', '', ...
    'observedProposalSignature', '', ...
    'selectedRouteSignature', '');
end

function [registered, scheduled, options] = ...
        parseRollingMode(mode, groupCount)
registered = false;
scheduled = false;
options = struct();
chunk = regexp(mode, ...
    ['^directed-rolling-chunk-q([0-9]+)-', ...
     'd(cw|ccw)-c([0-9]+)-w([0-9]+)$'], ...
    'tokens', 'once');
if ~isempty(chunk)
    registered = true;
    scheduled = true;
    options = struct( ...
        'scheduleType', 'cyclic-chunk', ...
        'quota', str2double(chunk{1}), ...
        'orientation', expandOrientation(chunk{2}), ...
        'formationPhase', str2double(chunk{3}));
    return;
end
burst = regexp(mode, ...
    ['^directed-rolling-burst-r([0-9]+)-', ...
     'd(cw|ccw)-p([0-2])-w([0-9]+)$'], ...
    'tokens', 'once');
if ~isempty(burst)
    registered = true;
    scheduled = true;
    options = struct( ...
        'scheduleType', 'burst', ...
        'rootFormation', str2double(burst{1}), ...
        'orientation', expandOrientation(burst{2}), ...
        'temporalPhase', str2double(burst{3}));
    return;
end
registered = ~isempty(regexp(mode, ...
    ['^(rolling-safe-analytic|', ...
     'directed-link-aware-rolling-b3|', ...
     'oracle-rolling-safe-current|', ...
     'oracle-rolling-safe-minimax|', ...
     'oracle-rolling-safe-risk-c[0-9]+|', ...
     'oracle-rolling-safe-h[0-9]+)-w[0-9]+$'], ...
    'once')) || ...
    ~isempty(regexp(mode, ...
        ['^oracle-rolling-safe-(anchor|tailcap)-r[0-9]+-', ...
         'd(cw|ccw)-p[0-2]-w[0-9]+$'], 'once')) || ...
    ~isempty(regexp(mode, ...
        ['^oracle-rolling-safe-hybrid-t[0-9]+-m[01]+-', ...
         'r[0-9]+-d(cw|ccw)-p[0-2]-w[0-9]+$'], 'once')) || ...
    ~isempty(regexp(mode, ...
        ['^oracle-rolling-safe-sequence-t[0-9]+-', ...
         's[0-9]+-w[0-9]+$'], 'once'));
if registered && groupCount < 2
    error('Rolling schedule audit needs at least two formations.');
end
end

function orientation = expandOrientation(token)
if strcmp(token, 'cw')
    orientation = 'clockwise';
else
    orientation = 'counter-clockwise';
end
end

function trace = formationTraceFromSensorEdges( ...
        edges, groupIds, groups)
trace = false(numel(groups), numel(groups), size(edges, 3));
for timeCursor = 1:size(edges, 3)
    [senders, receivers] = find(edges(:, :, timeCursor));
    for edgeCursor = 1:numel(senders)
        senderGroup = find( ...
            groups == groupIds(senders(edgeCursor)), 1);
        receiverGroup = find( ...
            groups == groupIds(receivers(edgeCursor)), 1);
        if senderGroup ~= receiverGroup
            trace(senderGroup, receiverGroup, timeCursor) = true;
        end
    end
end
end

function fraction = formationPairCapPassFraction( ...
        edges, groupIds, groups)
passed = false(1, size(edges, 3));
for timeCursor = 1:size(edges, 3)
    counts = zeros(numel(groups));
    [senders, receivers] = find(edges(:, :, timeCursor));
    for edgeCursor = 1:numel(senders)
        senderGroup = find( ...
            groups == groupIds(senders(edgeCursor)), 1);
        receiverGroup = find( ...
            groups == groupIds(receivers(edgeCursor)), 1);
        if senderGroup ~= receiverGroup
            counts(senderGroup, receiverGroup) = ...
                counts(senderGroup, receiverGroup) + 1;
        end
    end
    passed(timeCursor) = all(counts(:) <= 1);
end
fraction = mean(double(passed));
end

function counts = crossEdgeCountByTime(edges, groupIds)
counts = zeros(1, size(edges, 3));
for timeCursor = 1:size(edges, 3)
    [senders, receivers] = find(edges(:, :, timeCursor));
    counts(timeCursor) = nnz( ...
        groupIds(senders) ~= groupIds(receivers));
end
end

function value = maximumCrossEdgeCount(edges, groupIds)
counts = crossEdgeCountByTime(edges, groupIds);
if isempty(counts)
    value = NaN;
else
    value = max(counts);
end
end

function signature = traceSignature(trace)
groupCount = size(trace, 1);
parts = cell(1, size(trace, 3));
for timeCursor = 1:size(trace, 3)
    bits = reshape(logical(trace(:, :, timeCursor)), 1, []);
    chunkCount = ceil(numel(bits) / 60);
    chunks = cell(1, chunkCount);
    for chunkIdx = 1:chunkCount
        firstBit = 1 + 60 * (chunkIdx - 1);
        lastBit = min(numel(bits), firstBit + 59);
        code = uint64(0);
        for bitIdx = firstBit:lastBit
            if bits(bitIdx)
                code = bitset(code, bitIdx - firstBit + 1, 1);
            end
        end
        chunks{chunkIdx} = upper(dec2hex( ...
            code, ceil((lastBit - firstBit + 1) / 4)));
    end
    parts{timeCursor} = strjoin(chunks, '.');
end
signature = strjoin(parts, '-');
end

function values = getNumericDiagnostic( ...
        diagnostics, fieldName, times)
if ~isfield(diagnostics, fieldName) || ...
        numel(diagnostics.(fieldName)) < max(times)
    values = nan(size(times));
else
    values = reshape(diagnostics.(fieldName)(times), 1, []);
end
end

function value = getCellDiagnostic( ...
        diagnostics, fieldName, timeIdx, defaultValue)
if ~isfield(diagnostics, fieldName) || ...
        numel(diagnostics.(fieldName)) < timeIdx || ...
        isempty(diagnostics.(fieldName){timeIdx})
    value = defaultValue;
else
    value = diagnostics.(fieldName){timeIdx};
end
end
