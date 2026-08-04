function testFormationB4V47RuntimePolicies()
% Real M24/X36 route pages exercise the V47 wrapper and exact B4 budget.

protocol = getFormationGatewayDebtV47Protocol();
presets = {'m24-formation-fov-convoy', 'x36-formation-fov'};
for presetIdx = 1:numel(presets)
    [context, metadata] = ...
        buildFormationIndexEquivariantGeometryDevelopmentContext( ...
            presets{presetIdx}, 41);
    context = rmfield(context, 'auditBoundary');
    context.commConfig = struct( ...
        'pDropByEdge', context.commConfig.pDropByEdge);
    context.physicalIdentityRegistryCanonicalSha256 = ...
        metadata.physicalUidAssignmentCanonicalSha256;
    nodeCount = numel(context.localPosteriorBySensor);
    context.triggerConfig = buildFormationB4V47FixedTriggerConfig( ...
        protocol.primaryArms{2}, nodeCount);
    startTime = context.currentTime;
    context.previousAdjacencyHistory = ...
        false(nodeCount, nodeCount, 0);
    context.previousAdjacencyHistoryTimes = zeros(1, 0);
    context.previousDeliveryHistory = ...
        false(nodeCount, nodeCount, 0);
    context.previousDeliveryHistoryTimes = zeros(1, 0);
    context.observableInputContract = observableContract(protocol);

    messages = zeros(1, protocol.period);
    selectedHistory = false(nodeCount, nodeCount, 0);
    deliveryHistory = false(nodeCount, nodeCount, 0);
    times = zeros(1, 0);
    lastDetails = struct();
    for offset = 0:(protocol.period - 1)
        context.currentTime = startTime + offset;
        context.previousAdjacencyHistory = selectedHistory;
        context.previousAdjacencyHistoryTimes = times;
        context.previousDeliveryHistory = deliveryHistory;
        context.previousDeliveryHistoryTimes = times;
        [adjacency, details] = ...
            selectFormationB4V47GatewayDebtRuntimePolicy(context);
        messages(offset + 1) = nnz(adjacency);
        assert(strcmp(details.contractVersion, ...
            'formation-b4-v47-fixed-runtime-policy-v1'));
        assert(strcmp(details.armId, protocol.primaryArms{2}));
        assert(details.currentPosteriorUsed);
        assert(details.pastSuccessfulDeliveryUsed);
        assert(~details.truthUsed && ~details.futureOutcomeUsed);
        selectedHistory(:, :, end + 1) = adjacency; %#ok<AGROW>
        deliveryHistory(:, :, end + 1) = adjacency; %#ok<AGROW>
        times(end + 1) = context.currentTime; %#ok<AGROW>
        lastDetails = details;
    end
    assert(sum(messages) == 5 * nodeCount);
    assert(~lastDetails.referenceFallbackUsed);
    assert(lastDetails.rollingCrossServiceMature);
    assert(lastDetails.rollingFormationStrong);
end

[referenceConfig, referenceDetails] = ...
    buildFormationB4V47FixedTriggerConfig( ...
        protocol.primaryArms{1}, 24);
[candidateConfig, candidateDetails] = ...
    buildFormationB4V47FixedTriggerConfig( ...
        protocol.primaryArms{2}, 24);
assert(referenceConfig.topologyPolicyHistoryDepth == 8);
assert(candidateConfig.topologyPolicyHistoryDepth == 8);
assert(~getField(referenceConfig, ...
    'topologyPolicyDeliveryHistoryEnabled', false));
assert(candidateConfig.topologyPolicyDeliveryHistoryEnabled);
assert(strcmp(referenceDetails.protocolId, protocol.id));
assert(strcmp(candidateDetails.protocolId, protocol.id));

fprintf('PASS: FormationB4V47 runtime policy tests\n');
end

function contract = observableContract(protocol)
contract = struct( ...
    'contractVersion', protocol.observableContractVersion, ...
    'passed', true, 'deliveryHistoryPresent', true, ...
    'pastDeliveryHistoryOnly', true, ...
    'deliveryHistoryAlignedToTopologyHistory', true, ...
    'deliveryHistoryAttemptSubset', true);
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
