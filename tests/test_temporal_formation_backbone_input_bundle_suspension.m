function test_temporal_formation_backbone_input_bundle_suspension()
% TEST_TEMPORALFORMATIONINPUTBUNDLESUSPENSION Structural v38 action tests.

context = buildContext();
[referenceAdjacency, reference] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy(context, []);
assert(strcmp(reference.contractVersion, ...
    'temporal-formation-input-bundle-suspension-v1'));
assert(reference.messageSavingCount == 0);
assert(reference.referenceCrossFormationMessageCount == 6);
assert(all(reference.rollingB3SensorPass));
assert(all(reference.rollingB3FormationPass));

[candidateAdjacency, candidate] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy(context, [2, 3]);
expectedSaving = reference.tour.backboneDegrees(2) + ...
    reference.tour.backboneDegrees(3);
assert(candidate.messageSavingCount == expectedSaving);
assert(candidate.suspendedCrossEdgeCount == expectedSaving);
assert(nnz(candidateAdjacency) == ...
    nnz(referenceAdjacency) - expectedSaving);
assert(all(abs(sum(candidate.fusionWeightMatrix, 2) - 1) < 1e-12));
assert(~any(candidateAdjacency(:) & ~context.physicalAdjacency(:)));
assert(all(candidate.rollingB3SensorPass));
assert(all(candidate.rollingB3FormationPass));
assert(~candidate.truthUsed && ~candidate.futureOutcomeUsed);

config = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
assert(strcmp(config.contractVersion, ...
    'formation-backbone-bundle-staggered-recovery-policy-config-v1'));
assert(~config.sceneSpecificParametersAllowed);
assert(numel(config.canonicalSha256) == 64);
assertRejectsPolicyDrift(context, config);
assertRejectsUnknownOptions(context, config);
fprintf('PASS: temporal formation input-bundle suspension tests\n');
end

function assertRejectsPolicyDrift(context, config)
drifted = config;
drifted.protectionScoreOnFraction = ...
    drifted.protectionScoreOnFraction + 0.01;
failed = false;
try
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [], struct('policyConfig', drifted));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBundleSuspension:InvalidPolicyConfig');
end
assert(failed);
end

function assertRejectsUnknownOptions(context, config)
failed = false;
try
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [], struct( ...
            'policyConfig', config, 'sceneOverride', true));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBundleSuspension:InvalidOptions');
end
assert(failed);
end

function context = buildContext()
formationCount = 4;
sensorsPerFormation = 6;
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
staticAdjacency = buildStatic(groupIds, [1, 2; 2, 3; 3, 4]);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', staticAdjacency);
context = struct();
context.localPosteriorBySensor = cell(1, nodeCount);
context.model = model;
context.currentTime = 10;
context.physicalAdjacency = logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = staticAdjacency;
context.directedMessageBudget = 2 * nodeCount;
[referenceAdjacency, ~] = ...
    selectFormationBackboneResidualTourPolicy(context);
context.previousAdjacency = referenceAdjacency;
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
end

function adjacency = buildStatic(groupIds, treeEdges)
nodeCount = numel(groupIds);
adjacency = false(nodeCount);
for left = 1:nodeCount-1
    for right = left+1:nodeCount
        pair = sort([groupIds(left), groupIds(right)]);
        same = pair(1) == pair(2);
        linked = any(all(treeEdges == pair, 2));
        if same || linked
            adjacency(left, right) = true;
            adjacency(right, left) = true;
        end
    end
end
end
