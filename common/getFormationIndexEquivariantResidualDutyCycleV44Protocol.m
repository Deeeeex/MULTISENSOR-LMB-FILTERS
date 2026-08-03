function protocol = ...
    getFormationIndexEquivariantResidualDutyCycleV44Protocol()
% Frozen multiscene development screen for periodic V43 residual routes.

parent = getFormationIndexEquivariantRouteV43DevelopmentProtocol();
payload = struct();
payload.id = ...
    'formation-index-equivariant-residual-duty-cycle-v44-development-v1';
payload.contractVersion = ...
    'formation-index-equivariant-residual-duty-cycle-v44-protocol-v1';
payload.methodCommit = ...
    'd5b0f85ca1e667c60946d237935f357cc43b5267';
payload.parentProtocolId = parent.id;
payload.parentProtocolCanonicalSha256 = parent.canonicalSha256;
payload.parentEvidenceRelativePath = fullfile('RUN', 'GA', ...
    'dynamic_topology', 'evidence', 'formation_value_v43', ...
    'index_equivariant_route_development_v1', ...
    'FORMATION_INDEX_EQUIVARIANT_ROUTE_V43_DEVELOPMENT_V1.mat');
payload.parentEvidenceRawSha256 = ...
    'c46552b5f42363faa5491df3f720f748b9505ba976c84355cfe64fb3f9c00914';
payload.registeredPresets = parent.registeredPresets;
payload.registeredSeeds = parent.registeredSeeds;
payload.registeredCases = parent.registeredCases;
payload.registeredCartesianCaseCount = ...
    parent.registeredCartesianCaseCount;
payload.candidates = buildCandidates();
payload.candidateCount = numel(payload.candidates);
payload.targetSquaredContractionFactor = 0.90;
payload.maximumHorizonMultiplier = 4;
payload.fixedHorizonRule = 'node-count-minus-one';
payload.maximumIncomingCount = 2;
payload.missingNeighborWeightMode = 'renormalize';
payload.dominantWeight = 0.70;
payload.referenceResidualWeight = 0.05;
payload.periodGrid = [2, 3, 4];
payload.minimumActiveSelfWeight = 0.10;
payload.everyStartPhaseRequired = true;
payload.everyRollingPeriodUnionMustEqualV43 = true;
payload.referenceReproductionRequired = true;
payload.maximumWorstPhaseTargetMessageRatio = 1.00;
payload.maximumScaleMedianTargetMessageRatio = 0.85;
payload.minimumPeriodMessageSavingFraction = 0.25;
payload.everyCaseFixedHorizonStrictContractionRequired = true;
payload.crossingIncludedInAdvanceGate = true;
payload.selectionRule = [ ...
    'minimize-maximum-target-message-ratio-then-', ...
    'mean-target-message-ratio-then-candidate-id'];
payload.runnerSelfAuthorizationAllowed = false;
payload.postHocExecutableFreezeAuditRequired = true;
payload.formalRuntimeObservableBoundaryPassed = false;
payload.runtimePhysicalUidRegistryIntegrated = false;
payload.posteriorUsed = false;
payload.truthUsed = false;
payload.measurementUsed = false;
payload.futureOutcomeUsed = false;
payload.realizedDeliveryUniformsUsed = false;
payload.trackingOutcomeScored = false;
payload.trackingOutcomeAuthorized = false;
payload.validationClaimAllowed = false;
payload.developmentEvidenceOnly = true;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '6b26fcec9238df591a6c721229911e9328daf7ad822315539604cdcb7f2a2339';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationResidualDutyCycleV44:UnregisteredDrift', ...
        ['The v44 protocol changed without a version update: ', ...
         'actual=%s expected=%s.'], actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end

function candidates = buildCandidates()
rows = { ...
    'sync-all-b2-e05-nc', 2, 'all', 'synchronized', 0.05, 'uncompensated'; ...
    'sync-all-b2-e10-mc', 2, 'all', 'synchronized', 0.10, 'mass-matched'; ...
    'sync-all-b2-e20-sp', 2, 'all', 'synchronized', 0.20, 'strong-pulse'; ...
    'formation-all-b2-e10-mc', 2, 'all', 'formation-staggered', 0.10, 'mass-matched'; ...
    'formation-all-b2-e20-sp', 2, 'all', 'formation-staggered', 0.20, 'strong-pulse'; ...
    'sync-all-b3-e15-mc', 3, 'all', 'synchronized', 0.15, 'mass-matched'; ...
    'sync-all-b3-e20-sp', 3, 'all', 'synchronized', 0.20, 'strong-pulse'; ...
    'formation-all-b3-e15-mc', 3, 'all', 'formation-staggered', 0.15, 'mass-matched'; ...
    'formation-all-b3-e20-sp', 3, 'all', 'formation-staggered', 0.20, 'strong-pulse'; ...
    'sync-all-b4-e20-mc', 4, 'all', 'synchronized', 0.20, 'mass-matched'; ...
    'formation-all-b4-e20-mc', 4, 'all', 'formation-staggered', 0.20, 'mass-matched'};
template = struct('ordinal', NaN, 'id', '', 'period', NaN, ...
    'dutyLayer', '', 'phasePattern', '', ...
    'activeResidualWeight', NaN, 'weightMode', '');
candidates = repmat(template, 1, size(rows, 1));
for rowIdx = 1:size(rows, 1)
    candidates(rowIdx).ordinal = rowIdx;
    candidates(rowIdx).id = rows{rowIdx, 1};
    candidates(rowIdx).period = rows{rowIdx, 2};
    candidates(rowIdx).dutyLayer = rows{rowIdx, 3};
    candidates(rowIdx).phasePattern = rows{rowIdx, 4};
    candidates(rowIdx).activeResidualWeight = rows{rowIdx, 5};
    candidates(rowIdx).weightMode = rows{rowIdx, 6};
end
end
