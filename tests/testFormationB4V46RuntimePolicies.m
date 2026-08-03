function testFormationB4V46RuntimePolicies()
% No-op parity, repaired crossing and causal-boundary tests for V46.

protocol = getFormationCausalMinimalEditV46Protocol();
assert(isequal(protocol.primaryArms, { ...
    'v46-repaired-reference-a70-e05', ...
    'v46-repaired-sync-all-b4-e20-mc'}));

presets = {'m24-formation-fov-convoy', 'x36-formation-fov'};
for presetIdx = 1:numel(presets)
    context = buildNoOpContext(presets{presetIdx}, 41);
    registeredBefore = context.baseAdjacency;
    staticBefore = context.model.dynamicTopologyScenario.staticAdjacency;

    context.triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
        protocol.primaryArms{1}, numel(context.localPosteriorBySensor));
    [v45Reference, v45ReferenceDetails] = ...
        selectFormationB4V45ReferenceRuntimePolicy(context);
    [v46Reference, v46ReferenceDetails] = ...
        selectFormationB4V46ReferenceRuntimePolicy(context);
    assert(isequal(v46Reference, v45Reference));
    assert(isequal(v46ReferenceDetails.fusionWeightMatrix, ...
        v45ReferenceDetails.fusionWeightMatrix));
    assertNoOpParity(v46ReferenceDetails, v45Reference, ...
        v45ReferenceDetails.fusionWeightMatrix, 1);

    context.triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
        protocol.primaryArms{2}, numel(context.localPosteriorBySensor));
    [v45Sync, v45SyncDetails] = ...
        selectFormationB4V45SynchronizedRuntimePolicy(context);
    [v46Sync, v46SyncDetails] = ...
        selectFormationB4V46SynchronizedRuntimePolicy(context);
    assert(isequal(v46Sync, v45Sync));
    assert(isequal(v46SyncDetails.fusionWeightMatrix, ...
        v45SyncDetails.fusionWeightMatrix));
    assertNoOpParity(v46SyncDetails, v45Sync, ...
        v45SyncDetails.fusionWeightMatrix, 2);

    assert(isequal(context.baseAdjacency, registeredBefore));
    assert(isequal(context.model.dynamicTopologyScenario. ...
        staticAdjacency, staticBefore));
end

repairContext = buildRepairContext();
assertErrorId(@() selectFormationB4V45ReferenceRuntimePolicy( ...
    repairContext), 'IndexEquivariantFormationRoute:NoCrossAssignment');
nodeCount = numel(repairContext.localPosteriorBySensor);

repairContext.triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
    protocol.primaryArms{1}, nodeCount);
[reference, referenceDetails] = ...
    selectFormationB4V46ReferenceRuntimePolicy(repairContext);
assert(referenceDetails.repairTriggered);
assert(nnz(reference) == 72 && nodeCount == 36);
assert(isStronglyConnected(reference));
assertRuntimePage(repairContext, reference, referenceDetails, reference);
assertRepairDetails(referenceDetails, 1);

repairContext.triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
    protocol.primaryArms{2}, nodeCount);
[synchronized, synchronizedDetails] = ...
    selectFormationB4V46SynchronizedRuntimePolicy(repairContext);
assert(synchronizedDetails.repairTriggered);
assert(synchronizedDetails.currentAbsolutePhase == 2);
assert(isequal(synchronizedDetails.messageCountsByPhase, ...
    [72, 36, 36, 36]));
assert(nnz(synchronized) == 36);
assertRuntimePage(repairContext, synchronized, ...
    synchronizedDetails, synchronizedDetails.referenceAdjacency);
assertRepairDetails(synchronizedDetails, 2);
assert(strcmp(referenceDetails. ...
        projectedBaseAdjacencyExecutionViewSha256, ...
    synchronizedDetails. ...
        projectedBaseAdjacencyExecutionViewSha256));
assert(strcmp(referenceDetails. ...
        projectedBaseGraphPhysicalUidCanonicalSha256, ...
    synchronizedDetails. ...
        projectedBaseGraphPhysicalUidCanonicalSha256));
assert(strcmp(referenceDetails.projectionCertificateCanonicalSha256, ...
    synchronizedDetails.projectionCertificateCanonicalSha256));

invalid = repairContext;
invalid.observableInputContract.contractVersion = ...
    'topology-policy-observable-input-v2';
assertErrorId(@() selectFormationB4V46ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = repairContext;
invalid.triggerConfig.topologyDirectedMessageBudget = 71;
assertErrorId(@() selectFormationB4V46ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidDirectedRuntime');
invalid = repairContext;
invalid.futureOutcome = 1;
assertErrorId(@() selectFormationB4V46ReferenceRuntimePolicy(invalid), ...
    'FormationB4V45:InvalidObservableBoundary');
invalid = repairContext;
invalid.commConfig = rmfield(invalid.commConfig, 'pDropByEdge');
assertErrorId(@() selectFormationB4V46ReferenceRuntimePolicy(invalid), ...
    'FormationB4V46:InvalidContext');
invalid = repairContext;
invalid.model.dynamicTopologyScenario.staticAdjacency(1, 2) = ...
    ~invalid.model.dynamicTopologyScenario.staticAdjacency(1, 2);
assertErrorId(@() selectFormationB4V46ReferenceRuntimePolicy(invalid), ...
    'FormationB4V46:StaticBaseMismatch');
assertErrorId(@() buildFormationB4V46FixedRuntimeArm( ...
    repairContext, 'v44-formation-all-b4-e20-mc'), ...
    'FormationB4V46:InvalidArm');

fprintf('PASS: FormationB4V46 runtime policy tests\n');
end

function context = buildNoOpContext(presetName, seed)
[context, metadata] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        presetName, seed);
context = rmfield(context, 'auditBoundary');
context.commConfig = struct( ...
    'pDropByEdge', context.commConfig.pDropByEdge);
context.physicalIdentityRegistryCanonicalSha256 = ...
    metadata.physicalUidAssignmentCanonicalSha256;
nodeCount = numel(context.localPosteriorBySensor);
context.observableInputContract = observableContract( ...
    context.currentTime, nodeCount, ...
    context.physicalIdentityRegistryCanonicalSha256);
end

function context = buildRepairContext()
presetName = 'x36-formation-fov-crossing';
seed = 41;
currentTime = 158;
rng(seed, 'twister');
config = buildDynamicTopologyScenarioConfig(presetName);
[trajectories, metadata] = generateMultiFormationTrajectories(config);
graph = buildDynamicTopologyGraphs(config, trajectories);
[pDrop, ~] = buildDynamicTopologyLinkSchedule(config, graph);
registry = FormationB4V45BuildCausalRegisteredBaseGraph( ...
    config.formationBackboneMode, config.sensorGroupIds, ...
    metadata.sensorPhysicalUids, ...
    metadata.formationPhysicalUidsBySensor, ...
    graph.positions(:, :, 1), graph.physicalAdjacency(:, :, 1));
nodeCount = config.numberOfSensors;
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.currentTime = currentTime;
context.model = struct('dynamicTopologyScenario', struct( ...
    'config', struct('sensorGroupIds', config.sensorGroupIds), ...
    'staticAdjacency', logical(registry.baseAdjacency)));
context.baseAdjacency = logical(registry.baseAdjacency);
context.physicalAdjacency = logical( ...
    graph.physicalAdjacency(:, :, currentTime));
context.positions = graph.positions(:, :, currentTime);
context.sensorPhysicalUids = metadata.sensorPhysicalUids;
context.formationPhysicalUidsBySensor = ...
    metadata.formationPhysicalUidsBySensor;
context.physicalIdentityRegistryCanonicalSha256 = ...
    metadata.physicalIdentityRegistryCanonicalSha256;
context.commConfig = struct('pDropByEdge', ...
    pDrop(:, :, currentTime));
context.triggerConfig = buildFormationB4V46FixedTriggerConfig( ...
    'v46-repaired-reference-a70-e05', nodeCount);
context.observableInputContract = observableContract( ...
    currentTime, nodeCount, ...
    context.physicalIdentityRegistryCanonicalSha256);
end

function contract = observableContract(time, nodeCount, registrySha)
contract = struct( ...
    'contractVersion', ...
        'topology-policy-observable-input-v3-physical-uid', ...
    'enforced', true, 'passed', true, 'currentTime', time, ...
    'targetTruthAbsent', true, 'linkUniformsAbsent', true, ...
    'futurePDropPagesAbsent', true, ...
    'physicalIdentityPresent', true, ...
    'physicalIdentitySchemaRestricted', true, ...
    'directedTopologyRuntimeSemanticsPresent', true, ...
    'topologyDirectedEnabled', true, ...
    'topologyDirectedMessageBudget', 2 * nodeCount, ...
    'physicalIdentityRegistryCanonicalSha256', registrySha);
end

function assertNoOpParity(details, adjacency, weights, ordinal)
assert(~details.repairTriggered);
assert(details.armOrdinal == ordinal);
assert(strcmp(details.contractVersion, ...
    'formation-b4-v46-fixed-runtime-policy-v1'));
assert(strcmp(details.parentRouteAndWeightCanonicalSha256, ...
    computeCanonicalValueSha256(struct( ...
        'adjacency', logical(adjacency), ...
        'fusionWeightMatrix', weights))));
assert(strcmp(details.routeAndWeightCanonicalSha256, ...
    details.parentRouteAndWeightCanonicalSha256));
assert(details.staticAdjacencyWasPresent);
assert(details.staticAdjacencyReplacedWhenPresent);
assert(strcmp(details. ...
        registeredBaseGraphPhysicalUidCanonicalSha256, ...
    details.projectedBaseGraphPhysicalUidCanonicalSha256));
assertObjectiveSchema(details, ordinal);
assertBoundary(details);
end

function assertRepairDetails(details, ordinal)
certificate = details.projectionCertificate;
assert(details.armOrdinal == ordinal && details.repairTriggered);
assert(certificate.registeredRemovalCount >= 1);
assert(certificate.nonregisteredAdditionCount >= 1);
assert(details.baseAdjacencyReplacedInClone);
assert(details.staticAdjacencyWasPresent && ...
    details.staticAdjacencyReplacedWhenPresent);
assert(~strcmp(details. ...
        registeredBaseGraphPhysicalUidCanonicalSha256, ...
    details.projectedBaseGraphPhysicalUidCanonicalSha256));
assertObjectiveSchema(details, ordinal);
assertBoundary(details);
end

function assertObjectiveSchema(details, ordinal)
assert(isa(details.objective, 'double') && ...
    isreal(details.objective) && isscalar(details.objective));
assert(details.objectiveDiagnosticsScalar);
if ordinal == 1
    assert(isequal(size(details.parentRouteObjectiveVector), [1, 2]));
    assert(all(isfinite(details.parentRouteObjectiveVector)));
    assert(isfinite(details.objective));
    assert(details.objective == ...
        min(details.parentRouteObjectiveVector));
    assert(strcmp(details.objectiveDefinition, ...
        ['minimum-of-v43-dominant-and-local-residual-', ...
         'bottleneck-reliability']));
    assert(strcmp(details.objectiveSource, ...
        'v43-reference-objective-vector-minimum'));
    assert(details.objectiveFiniteScalarRequired);
else
    assert(isempty(details.parentRouteObjectiveVector));
    assert(isnan(details.objective));
    assert(strcmp(details.objectiveDefinition, ...
        'not-defined-for-synchronized-duty-cycle-parent'));
    assert(strcmp(details.objectiveSource, 'not-applicable'));
    assert(~details.objectiveFiniteScalarRequired);
end
end

function assertBoundary(details)
assert(details.routeReceivesCurrentPageOnly);
assert(details.projectionReceivesCurrentPageOnly);
assert(details.observableContextOnly);
assert(~details.policyOptionsAccepted);
assert(~details.posteriorUsedByProjection);
assert(~details.truthUsedByProjection);
assert(~details.measurementUsedByProjection);
assert(~details.futurePageUsedByProjection);
assert(~details.realizedDeliveryUniformsUsedByProjection);
assert(~details.perEdgeMassEquivalenceClaimAllowed);
assert(details.dynamicSenderIdentityMayChange);
assert(details. ...
    receiverPeriodResidualMassMatchedToCurrentReferenceWeight);
assert(~isfield(details, 'residualMassMatchedToReference'));
assert(details.staticAdjacencyReplacementRuleEnforced);
assert(details.projectionCertificateCrossBackboneOnly);
assert(details.fullV43RouteCompositionPassed);
assert(~details.futureOutcomeUsed);
assert(~details.trackingOutcomeScored);
assert(~details.validationClaimAllowed);
assert(details.developmentEvidenceOnly);
certificate = details.projectionCertificate;
canonicalPayload = rmfield(certificate, { ...
    'canonicalSha256', ...
    'arrayOrderExecutionCanonicalSha256'});
assert(strcmp(computeCanonicalValueSha256(canonicalPayload), ...
    certificate.canonicalSha256));
assert(isa(certificate.registeredRemovalCount, 'double') && ...
    isscalar(certificate.registeredRemovalCount));
assert(islogical(certificate.registeredBaseGraphUsed) && ...
    isscalar(certificate.registeredBaseGraphUsed));
end

function assertRuntimePage(context, adjacency, details, reference)
nodeCount = numel(context.localPosteriorBySensor);
physical = logical(context.physicalAdjacency);
weights = details.fusionWeightMatrix;
positive = weights > 1e-12;
positive(1:nodeCount+1:end) = false;
assert(islogical(adjacency));
assert(~any(diag(adjacency)));
assert(~any(adjacency(:) & ~physical(:)));
assert(~any(adjacency(:) & ~reference(:)));
assert(isequal(positive, adjacency));
assert(all(abs(sum(weights, 2) - 1) <= 1e-12));
assert(~any(weights(:) < -1e-12));
end

function connected = isStronglyConnected(adjacency)
connected = reachableAll(adjacency') && reachableAll(adjacency);
end

function passed = reachableAll(adjacency)
visited = false(1, size(adjacency, 1));
frontier = 1;
while ~isempty(frontier)
    node = frontier(end);
    frontier(end) = [];
    if visited(node)
        continue;
    end
    visited(node) = true;
    frontier = [frontier, reshape(find( ...
        adjacency(node, :) & ~visited), 1, [])]; %#ok<AGROW>
end
passed = all(visited);
end

function assertErrorId(handle, expected)
failed = false;
try
    handle();
catch errorInfo
    failed = strcmp(errorInfo.identifier, expected);
end
assert(failed);
end
