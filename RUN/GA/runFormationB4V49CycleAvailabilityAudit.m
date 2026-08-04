function result = runFormationB4V49CycleAvailabilityAudit(options)
% RUNFORMATIONB4V49CYCLEAVAILABILITYAUDIT Full-route cheap eligibility gate.

if nargin < 1 || isempty(options)
    options = struct();
end
allowed = {'presets', 'seeds'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowed))
    error('FormationB4V49Availability:InvalidOptions', ...
        'The cycle-availability options are malformed.');
end
presets = getField(options, 'presets', { ...
    'm24-formation-fov', 'm24-formation-fov-convoy', ...
    'm24-formation-fov-relay', 'm24-formation-fov-crossing', ...
    'x36-formation-fov', 'x36-formation-fov-convoy', ...
    'x36-formation-fov-relay', 'x36-formation-fov-crossing'});
seeds = reshape(getField(options, 'seeds', 41), 1, []);
if ~iscell(presets) || isempty(presets) || ...
        ~isnumeric(seeds) || isempty(seeds) || ...
        any(~isfinite(seeds)) || any(seeds ~= round(seeds))
    error('FormationB4V49Availability:InvalidOptions', ...
        'The preset or seed list is invalid.');
end

records = repmat(emptyRecord(), 1, numel(presets) * numel(seeds));
cursor = 0;
for presetIdx = 1:numel(presets)
    for seedIdx = 1:numel(seeds)
        cursor = cursor + 1;
        presetName = presets{presetIdx};
        seed = seeds(seedIdx);
        fprintf('V49 cycle availability %d/%d: %s seed=%d\n', ...
            cursor, numel(records), presetName, seed);
        records(cursor) = auditCase(presetName, seed);
        item = records(cursor);
        fprintf(['  eligible=%d/%d (%.2f%%) cycles[min/med/max]=', ...
            '[%d/%.1f/%d] uidFirstChurn=%d noCycleRun=%d\n'], ...
            item.eligiblePageCount, item.pageCount, ...
            100 * item.eligiblePageFraction, ...
            item.minimumCycleCount, item.medianCycleCount, ...
            item.maximumCycleCount, ...
            item.uidFirstCycleEdgeSetChangeCount, ...
            item.longestNoCycleRunLength);
    end
end

payload = struct();
payload.contractVersion = ...
    'formation-b4-v49-cycle-availability-audit-v1';
payload.presets = presets;
payload.seeds = seeds;
payload.records = records;
payload.caseCount = numel(records);
payload.totalPageCount = sum([records.pageCount]);
payload.totalEligiblePageCount = sum([records.eligiblePageCount]);
payload.overallEligiblePageFraction = ...
    payload.totalEligiblePageCount / payload.totalPageCount;
payload.allPagesEligible = all([records.allPagesEligible]);
payload.anyCaseRequiresFallback = any( ...
    [records.fallbackPageCount] > 0);
payload.fullPlannedSensorGeometryMaterialized = true;
payload.perPagePolicyFutureGeometryUsed = false;
payload.posteriorUsed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.linkProbabilityUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.trackingOutcomeScored = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;
payload.canonicalSha256 = computeCanonicalValueSha256(payload);
result = payload;
fprintf(['V49 cycle availability complete: eligible=%d/%d ', ...
    '(%.2f%%) all=%d sha256=%s\n'], ...
    result.totalEligiblePageCount, result.totalPageCount, ...
    100 * result.overallEligiblePageFraction, ...
    result.allPagesEligible, result.canonicalSha256);
end

function item = auditCase(presetName, seed)
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[sensorTrajectories, trajectoryMetadata] = ...
    generateMultiFormationTrajectories(config);
graph = buildDynamicTopologyGraphs(config, sensorTrajectories);
identity = buildDynamicTopologyPhysicalIdentityRegistry(config);
if ~isequal(identity.sensorPhysicalUids, ...
        trajectoryMetadata.sensorPhysicalUids) || ...
        ~isequal(identity.formationPhysicalUidsBySensor, ...
            trajectoryMetadata.formationPhysicalUidsBySensor)
    error('FormationB4V49Availability:IdentityDrift', ...
        'The scene and trajectory physical identities disagree.');
end

formationUidsBySensor = identity.formationPhysicalUidsBySensor;
formationUids = sort(unique(formationUidsBySensor));
pageCount = size(graph.physicalAdjacency, 3);
cycleCountByTime = zeros(1, pageCount);
uidFirstCycleOrderByTime = nan(pageCount, numel(formationUids));
uidFirstCycleEdgeHashByTime = repmat({''}, 1, pageCount);
uidFirstCycleEdgeSetChangeByTime = false(1, pageCount);
uidFirstCycleEdgeSymmetricDifferenceByTime = zeros(1, pageCount);
minimumPhysicalFormationDegreeByTime = zeros(1, pageCount);
previousEdges = zeros(0, 2);
previousEligible = false;
for currentTime = 1:pageCount
    formationAdjacency = collapsePhysicalToFormations( ...
        graph.physicalAdjacency(:, :, currentTime), ...
        formationUidsBySensor, formationUids);
    minimumPhysicalFormationDegreeByTime(currentTime) = ...
        min(sum(formationAdjacency, 2));
    orders = enumerateCycles(formationAdjacency, formationUids);
    cycleCountByTime(currentTime) = size(orders, 1);
    eligible = ~isempty(orders);
    currentEdges = zeros(0, 2);
    if eligible
        uidFirstCycleOrderByTime(currentTime, :) = orders(1, :);
        currentEdges = cycleEdges(orders(1, :));
        uidFirstCycleEdgeHashByTime{currentTime} = ...
            computeCanonicalValueSha256(struct( ...
                'contractVersion', ...
                    'formation-b4-v49-uid-first-cycle-edges-v1', ...
                'formationPairPhysicalUids', currentEdges));
    end
    if currentTime > 1
        changed = eligible ~= previousEligible;
        differenceCount = 0;
        if eligible && previousEligible
            differenceCount = size(setxor( ...
                currentEdges, previousEdges, 'rows'), 1);
            changed = differenceCount > 0;
        elseif changed
            differenceCount = size(currentEdges, 1) + ...
                size(previousEdges, 1);
        end
        uidFirstCycleEdgeSetChangeByTime(currentTime) = changed;
        uidFirstCycleEdgeSymmetricDifferenceByTime(currentTime) = ...
            differenceCount;
    end
    previousEdges = currentEdges;
    previousEligible = eligible;
end

eligibleMask = cycleCountByTime > 0;
item = emptyRecord();
item.caseId = sprintf('%s__seed%d__all-pages', presetName, seed);
item.presetName = presetName;
item.seed = seed;
item.nodeCount = config.numberOfSensors;
item.formationCount = config.formationCount;
item.pageCount = pageCount;
item.cycleCountByTime = cycleCountByTime;
item.minimumCycleCount = min(cycleCountByTime);
item.medianCycleCount = median(cycleCountByTime);
item.maximumCycleCount = max(cycleCountByTime);
item.eligiblePageMask = eligibleMask;
item.eligiblePageCount = nnz(eligibleMask);
item.eligiblePageFraction = nnz(eligibleMask) / pageCount;
item.fallbackPageCount = pageCount - nnz(eligibleMask);
item.allPagesEligible = all(eligibleMask);
item.longestNoCycleRunLength = longestTrueRun(~eligibleMask);
item.minimumPhysicalFormationDegreeByTime = ...
    minimumPhysicalFormationDegreeByTime;
item.uidFirstCycleOrderByTime = uidFirstCycleOrderByTime;
item.uidFirstCycleEdgeHashByTime = uidFirstCycleEdgeHashByTime;
item.uidFirstCycleEdgeSetChangeByTime = ...
    uidFirstCycleEdgeSetChangeByTime;
item.uidFirstCycleEdgeSetChangeCount = nnz( ...
    uidFirstCycleEdgeSetChangeByTime);
item.uidFirstCycleEdgeSymmetricDifferenceByTime = ...
    uidFirstCycleEdgeSymmetricDifferenceByTime;
item.fullPlannedSensorGeometryMaterialized = true;
item.perPagePolicyFutureGeometryUsed = false;
item.posteriorUsed = false;
item.truthUsed = false;
item.measurementUsed = false;
item.linkProbabilityUsed = false;
item.realizedDeliveryUniformsUsed = false;
item.trackingOutcomeScored = false;
item.developmentEvidenceOnly = true;
item.canonicalSha256 = computeCanonicalValueSha256( ...
    rmfield(item, 'canonicalSha256'));
end

function adjacency = collapsePhysicalToFormations( ...
        physical, formationUidsBySensor, formationUids)
formationCount = numel(formationUids);
adjacency = false(formationCount);
for receiverIdx = 1:formationCount
    receivers = formationUidsBySensor == formationUids(receiverIdx);
    for senderIdx = 1:formationCount
        if receiverIdx ~= senderIdx
            senders = formationUidsBySensor == ...
                formationUids(senderIdx);
            adjacency(receiverIdx, senderIdx) = ...
                any(any(physical(receivers, senders)));
        end
    end
end
adjacency = adjacency & adjacency';
end

function orders = enumerateCycles(adjacency, formationUids)
formationCount = numel(formationUids);
tailOrders = perms(2:formationCount);
orders = zeros(0, formationCount);
for rowIdx = 1:size(tailOrders, 1)
    order = [1, tailOrders(rowIdx, :)];
    if formationUids(order(2)) > formationUids(order(end))
        continue;
    end
    valid = true;
    for position = 1:formationCount
        valid = valid && adjacency(order(position), ...
            order(mod(position, formationCount) + 1));
    end
    if valid
        orders(end + 1, :) = formationUids(order); %#ok<AGROW>
    end
end
if ~isempty(orders)
    orders = sortrows(orders, 1:formationCount);
end
end

function edges = cycleEdges(order)
count = numel(order);
edges = zeros(count, 2);
for position = 1:count
    pair = sort([order(position), order(mod(position, count) + 1)]);
    edges(position, :) = pair;
end
edges = sortrows(edges, [1, 2]);
end

function length = longestTrueRun(mask)
length = 0;
current = 0;
for idx = 1:numel(mask)
    if mask(idx)
        current = current + 1;
        length = max(length, current);
    else
        current = 0;
    end
end
end

function item = emptyRecord()
item = struct('caseId', '', 'presetName', '', 'seed', NaN, ...
    'nodeCount', NaN, 'formationCount', NaN, 'pageCount', NaN, ...
    'cycleCountByTime', zeros(1, 0), ...
    'minimumCycleCount', NaN, 'medianCycleCount', NaN, ...
    'maximumCycleCount', NaN, ...
    'eligiblePageMask', false(1, 0), ...
    'eligiblePageCount', NaN, 'eligiblePageFraction', NaN, ...
    'fallbackPageCount', NaN, 'allPagesEligible', false, ...
    'longestNoCycleRunLength', NaN, ...
    'minimumPhysicalFormationDegreeByTime', zeros(1, 0), ...
    'uidFirstCycleOrderByTime', zeros(0, 0), ...
    'uidFirstCycleEdgeHashByTime', {cell(1, 0)}, ...
    'uidFirstCycleEdgeSetChangeByTime', false(1, 0), ...
    'uidFirstCycleEdgeSetChangeCount', NaN, ...
    'uidFirstCycleEdgeSymmetricDifferenceByTime', zeros(1, 0), ...
    'fullPlannedSensorGeometryMaterialized', false, ...
    'perPagePolicyFutureGeometryUsed', false, ...
    'posteriorUsed', false, 'truthUsed', false, ...
    'measurementUsed', false, 'linkProbabilityUsed', false, ...
    'realizedDeliveryUniformsUsed', false, ...
    'trackingOutcomeScored', false, ...
    'developmentEvidenceOnly', true, 'canonicalSha256', '');
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
