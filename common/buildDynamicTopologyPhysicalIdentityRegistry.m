function registry = buildDynamicTopologyPhysicalIdentityRegistry(config)
% BUILDDYNAMICTOPOLOGYPHYSICALIDENTITYREGISTRY Stable scene identities.
%
% Formation identity is assigned from the registered initial physical
% centre, never from the current storage position or numeric group label.
% Sensor identity combines that formation identity with an explicit local
% ring-role ID.  A caller may reorder storage by supplying
% sensorLocalRoleUidsByFormation; the same physical role then retains its
% UID.  These IDs are static scene metadata and are used only as the final
% deterministic tie-break after reliability and distance.

required = {'formationCount', 'sensorsPerFormation', ...
    'sensorCenterWaypoints'};
if ~isstruct(config) || ~isscalar(config) || ...
        ~all(isfield(config, required)) || ...
        ~isscalar(config.formationCount) || ...
        config.formationCount < 1 || ...
        config.formationCount ~= round(config.formationCount) || ...
        ~isscalar(config.sensorsPerFormation) || ...
        config.sensorsPerFormation < 1 || ...
        config.sensorsPerFormation ~= round(config.sensorsPerFormation) || ...
        ~iscell(config.sensorCenterWaypoints) || ...
        numel(config.sensorCenterWaypoints) ~= config.formationCount
    error('DynamicTopologyPhysicalIdentity:InvalidConfig', ...
        'The scene cannot define a physical identity registry.');
end

formationCount = config.formationCount;
sensorsPerFormation = config.sensorsPerFormation;
initialCenters = zeros(formationCount, 2);
for formationIdx = 1:formationCount
    waypoints = config.sensorCenterWaypoints{formationIdx};
    if ~isnumeric(waypoints) || ~isreal(waypoints) || ...
            size(waypoints, 1) ~= 2 || isempty(waypoints) || ...
            any(~isfinite(waypoints(:)))
        error('DynamicTopologyPhysicalIdentity:InvalidConfig', ...
            'Each formation needs a finite registered initial centre.');
    end
    initialCenters(formationIdx, :) = waypoints(:, 1)';
end
[sortedCenters, canonicalOrder] = sortrows(initialCenters, [1, 2]);
if formationCount > 1 && any(sqrt(sum(diff(sortedCenters, 1, 1).^2, 2)) ...
        <= 1e-9)
    error('DynamicTopologyPhysicalIdentity:AmbiguousFormation', ...
        ['Two formations share the same registered initial centre; ', ...
         'an explicit exogenous formation UID registry is required.']);
end
formationUids = zeros(1, formationCount);
uidPool = 700000 + 101 * (1:formationCount);
formationUids(canonicalOrder) = uidPool;

if isfield(config, 'sensorLocalRoleUidsByFormation')
    localRolesByFormation = config.sensorLocalRoleUidsByFormation;
else
    localRolesByFormation = repmat( ...
        {1:sensorsPerFormation}, 1, formationCount);
end

if isfield(config, 'formationStochasticRoleUidsByFormation')
    stochasticRoleUids = reshape( ...
        config.formationStochasticRoleUidsByFormation, 1, []);
else
    stochasticRoleUids = 1:formationCount;
end
if ~isnumeric(stochasticRoleUids) || ~isreal(stochasticRoleUids) || ...
        numel(stochasticRoleUids) ~= formationCount || ...
        any(~isfinite(stochasticRoleUids)) || ...
        any(stochasticRoleUids ~= round(stochasticRoleUids)) || ...
        ~isequal(sort(stochasticRoleUids), 1:formationCount)
    error('DynamicTopologyPhysicalIdentity:InvalidStochasticRoles', ...
        'Every formation must carry one persistent stochastic role UID.');
end
if ~iscell(localRolesByFormation) || ...
        numel(localRolesByFormation) ~= formationCount
    error('DynamicTopologyPhysicalIdentity:InvalidLocalRoles', ...
        'Local physical role metadata must contain one entry per formation.');
end

nodeCount = formationCount * sensorsPerFormation;
sensorUids = zeros(1, nodeCount);
formationUidsBySensor = zeros(1, nodeCount);
localRoleUidsBySensor = zeros(1, nodeCount);
sensorCursor = 0;
for formationIdx = 1:formationCount
    roles = reshape(localRolesByFormation{formationIdx}, 1, []);
    if ~isnumeric(roles) || ~isreal(roles) || ...
            numel(roles) ~= sensorsPerFormation || ...
            any(~isfinite(roles)) || any(roles ~= round(roles)) || ...
            ~isequal(sort(roles), 1:sensorsPerFormation)
        error('DynamicTopologyPhysicalIdentity:InvalidLocalRoles', ...
            'Each formation must use every registered local role once.');
    end
    localRolesByFormation{formationIdx} = roles;
    indices = sensorCursor + (1:sensorsPerFormation);
    sensorCursor = sensorCursor + sensorsPerFormation;
    formationUid = formationUids(formationIdx);
    formationUidsBySensor(indices) = formationUid;
    localRoleUidsBySensor(indices) = roles;
    sensorUids(indices) = formationUid * 100 + roles;
end
if numel(unique(sensorUids)) ~= nodeCount
    error('DynamicTopologyPhysicalIdentity:UidCollision', ...
        'The physical sensor UID registry contains a collision.');
end

payload = struct();
payload.contractVersion = ...
    'dynamic-topology-physical-identity-registry-v1';
payload.formationIdentitySource = ...
    'lexicographic-registered-initial-physical-centre';
payload.sensorIdentitySource = ...
    'formation-physical-uid-plus-explicit-local-ring-role';
payload.formationStochasticIdentitySource = ...
    'explicit-persistent-legacy-random-draw-role';
payload.registeredInitialCenters = initialCenters;
payload.formationPhysicalUids = formationUids;
payload.formationStochasticRoleUidsByFormation = stochasticRoleUids;
payload.sensorLocalRoleUidsByFormation = localRolesByFormation;
payload.sensorLocalRoleUidsBySensor = localRoleUidsBySensor;
payload.formationPhysicalUidsBySensor = formationUidsBySensor;
payload.sensorPhysicalUids = sensorUids;
payload.arrayIndexUsedAsIdentity = false;
payload.groupNumericLabelUsedAsIdentity = false;
registry = payload;
registry.canonicalSha256 = computeCanonicalValueSha256(payload);
end
