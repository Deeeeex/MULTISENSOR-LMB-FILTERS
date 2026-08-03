function test_formation_index_equivariant_adaptive_window_route_probe()
% The exact structural certificate must inherit v43 route equivariance.

[context, ~] = ...
    buildFormationIndexEquivariantGeometryDevelopmentContext( ...
        'm24-formation-fov', 41);
baseline = buildFormationIndexEquivariantAdaptiveWindowRouteProbe( ...
    context);
assert(baseline.calibrationPassed);
assert(baseline.evaluationHorizon >= 1);
assert(baseline.referenceSquaredContractionFactor <= 0.90 + 1e-12);
assert(baseline.referenceRouteIndexEquivarianceDesigned);
assert(baseline.fullPlannedSensorGeometryMaterialized);
assert(baseline.fullPlannedLinkProbabilityScheduleMaterialized);
assert(~baseline.formalRuntimeObservableBoundaryPassed);
assert(~baseline.runtimePhysicalUidRegistryIntegrated);
assert(~baseline.trackingOutcomeAuthorized);
assert(~baseline.validationClaimAllowed);
assert(baseline.leastHarmfulCandidateFactorDelta > ...
    baseline.improvementTolerance);
assert(baseline.improvingCandidateCount == 0);
assert(~baseline.sparseActionAuthorized);
assert(strcmp(baseline.selectedActionName, 'reference'));
assert(isnan(baseline.selectedActionPhysicalFormationUid));
assert(baseline.referenceFallbackUsed);

reversed = buildFormationIndexEquivariantAdaptiveWindowRouteProbe( ...
    context, struct('candidateFormationPhysicalUids', ...
        fliplr(baseline.formationPhysicalUids)));
assert(reversed.leastHarmfulCandidatePhysicalFormationUid == ...
    baseline.leastHarmfulCandidatePhysicalFormationUid);
assert(abs(reversed.leastHarmfulCandidateFactorDelta - ...
    baseline.leastHarmfulCandidateFactorDelta) < 1e-12);
assert(strcmp(reversed.selectedActionName, ...
    baseline.selectedActionName));
assert(~baseline.posteriorContentsRead && ~baseline.truthUsed && ...
    ~baseline.futureOutcomeUsed && ~baseline.trackingOutcomeScored);

groupIds = context.model.dynamicTopologyScenario.config.sensorGroupIds;
order = [find(groupIds == 4), find(groupIds == 2), ...
    find(groupIds == 1), find(groupIds == 3)];
[permuted, ~] = permuteFormationIndexEquivariantContext( ...
    context, order);
actual = buildFormationIndexEquivariantAdaptiveWindowRouteProbe( ...
    permuted);
assert(actual.evaluationHorizon == baseline.evaluationHorizon);
assert(abs(actual.referenceSquaredContractionFactor - ...
    baseline.referenceSquaredContractionFactor) < 1e-12);
assert(actual.leastHarmfulCandidatePhysicalFormationUid == ...
    baseline.leastHarmfulCandidatePhysicalFormationUid);
assert(abs(actual.leastHarmfulCandidateFactorDelta - ...
    baseline.leastHarmfulCandidateFactorDelta) < 1e-12);
assert(actual.sparseActionAuthorized == ...
    baseline.sparseActionAuthorized);
assert(strcmp(actual.selectedActionName, baseline.selectedActionName));

for baselineIdx = 1:numel(baseline.actions)
    uid = baseline.actions(baselineIdx).formationPhysicalUid;
    if isnan(uid)
        actualIdx = 1;
    else
        actualIdx = find([actual.actions.formationPhysicalUid] == uid);
    end
    assert(numel(actualIdx) == 1);
    assert(actual.actions(actualIdx).available == ...
        baseline.actions(baselineIdx).available);
    if ~baseline.actions(baselineIdx).available
        continue;
    end
    restoredAdjacency = false(size(actual.actions(actualIdx).adjacency));
    restoredAdjacency(order, order) = ...
        actual.actions(actualIdx).adjacency;
    restoredWeights = zeros(size(actual.actions(actualIdx).fusionWeights));
    restoredWeights(order, order) = ...
        actual.actions(actualIdx).fusionWeights;
    assert(isequal(restoredAdjacency, ...
        baseline.actions(baselineIdx).adjacency));
    assert(isequal(restoredWeights, ...
        baseline.actions(baselineIdx).fusionWeights));
    assert(abs(actual.actions(actualIdx).squaredContractionFactor - ...
        baseline.actions(baselineIdx).squaredContractionFactor) < 1e-12);
end

fprintf('PASS: v43 adaptive-window route-probe tests\n');
end
