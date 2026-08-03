function test_formation_backbone_bundle_staggered_recovery_control()
% TEST_FORMATIONBACKBONEBUNDLESTAGGEREDRECOVERYCONTROL v38 contracts.

policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
assertReferenceConstrainedParetoSelection();
schedule = buildFormationProtectionCoverageReleaseSchedule( ...
    1:4, [2, 3, 4], [0.01, 0.05, 0.012, 0.049], ...
    [0, 1, 1, 1], policy.minimumSuspensionAgeSteps, ...
    policy.minimumRetainedProtectionCoverageFraction);
assert(strcmp(schedule.contractVersion, ...
    'formation-protection-coverage-release-schedule-v1'));
assert(isequal(schedule.releaseOrder, [3, 4, 2]));
assert(schedule.candidateCount == 1);
assert(isequal(schedule.candidateReleasedFormationIds{1}, 3));
assert(isequal(schedule.candidateRetainedFormationIds{1}, [2, 4]));

context = buildContext();
base = buildFormationBackboneBundleProtectionControl(context);
control = buildFormationBackboneBundleStaggeredRecoveryControl(context);
assert(strcmp(base.contractVersion, ...
    'formation-backbone-bundle-protection-control-v1'));
assert(strcmp(control.contractVersion, ...
    'formation-backbone-bundle-staggered-recovery-control-v1'));
assert(strcmp(control.policyConfigSha256, policy.canonicalSha256));
assert(control.nodeCount == 24 && control.formationCount == 4);
assert(control.referenceFallbackUsed);
assert(isempty(control.selectedFormationIds));
assert(control.selectedScore.safe);
assert(all(control.selectedRollingB3SensorPass));
assert(all(control.selectedRollingB3FormationPass));
assert(control.selectionUsesCurrentPosterior);
assert(control.selectionUsesCurrentLinkReliability);
assert(~control.truthUsed && ~control.futureOutcomeUsed);

[adjacency, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy(context);
assert(isequal(adjacency, control.selectedAdjacency));
assert(strcmp(details.contractVersion, ...
    'formation-backbone-bundle-staggered-recovery-runtime-policy-v1'));
assert(isfield(details, 'protectionScoreByFormation'));
assert(isequaln(details.protectionScoreByFormation, ...
    details.retentionDebtFractionByFormation));
assert(details.posteriorUsed && details.currentLinkReliabilityUsed);
assert(~details.truthUsed && ~details.futureOutcomeUsed);
assert(isfinite(details.selectionSeconds) && ...
    details.selectionSeconds >= 0);
assert(isfinite(details.objective) && ...
    isfinite(details.taskRisk) && ...
    isfinite(details.baselineTaskRisk) && ...
    isfinite(details.taskAdvantage));
assert(~details.oneStepJointProjectionUsed);
assert(strcmp(details.validCandidateCountSemantics, ...
    'safe-route-evaluations-not-deduplicated'));

observable = buildObservableTopologyPolicyContext(context);
[observableAdjacency, observableDetails] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy(observable);
assert(isequal(observableAdjacency, adjacency));
assert(isequaln(observableDetails.protectionScoreByFormation, ...
    details.protectionScoreByFormation));
assert(observable.observableInputContract.passed);

[paretoAdjacency, paretoDetails] = ...
    selectFormationBackboneBundleParetoRiskRuntimePolicy(observable);
assert(nnz(paretoAdjacency) <= nnz(control.referenceAdjacency));
assert(paretoDetails.taskRisk <= paretoDetails.baselineTaskRisk);
assert(paretoDetails.taskAdvantage >= 0);
assert(strcmp(paretoDetails.contractVersion, ...
    'formation-backbone-bundle-pareto-risk-runtime-policy-v1'));
assert(~paretoDetails.truthUsed && ~paretoDetails.futureOutcomeUsed);
assert(isfield(paretoDetails, 'paretoRiskGatePassed'));
assert(paretoDetails.referenceSelected);
assert(paretoDetails.noSparseProposal);
assert(~paretoDetails.referenceFallbackUsed);
assert(~paretoDetails.paretoRiskRejected);
assert(paretoDetails.consensusProjectionFallbackCount == 0);
assert(strcmp(paretoDetails.selectionState, 'no-sparse-proposal'));
assert(strcmp(paretoDetails.taskRiskMetricId, ...
    'exact-one-round-expected-lmb-summary-disagreement-v1'));
assert(~paretoDetails.taskRiskIsTrackingOutcome);
assert(~paretoDetails.trackingRiskNondegradationClaimed);

assertJointProjectionAndRuntimeMetadata();
assertStaggeredRelease();
assertIncrementalExpectedDisagreementExactness(context);
assertIncrementalExpectedDisagreementPropertyMatrix();
assertRejectsRepeatedB3Violation(context);
fprintf('PASS: formation-backbone input-bundle recovery tests\n');
end

function assertReferenceConstrainedParetoSelection()
[selected, details] = ...
    selectReferenceConstrainedParetoCandidateIndex( ...
        0, 1, 0, 1, 1, 1);
assert(selected == 1);
assert(~any(details.sparseMask));

% A higher-saving route wins even when a lower-saving route has lower risk.
selected = selectReferenceConstrainedParetoCandidateIndex( ...
    [0, 2, 4], [1, 0.2, 0.9], [0, 0, 0], 1:3, 1, 1);
assert(selected == 3);

% At equal saving, lower risk wins.
selected = selectReferenceConstrainedParetoCandidateIndex( ...
    [0, 4, 4], [1, 0.9, 0.8], [0, 0, 0], 1:3, 1, 1);
assert(selected == 3);

% At equal saving and risk, fewer selected-history switches win.
selected = selectReferenceConstrainedParetoCandidateIndex( ...
    [0, 4, 4], [1, 0.8, 0.8], [0, 3, 2], 1:3, 1, 1);
assert(selected == 3);

% A complete tie is resolved by stable ordinal, not input position.
selected = selectReferenceConstrainedParetoCandidateIndex( ...
    [0, 4, 4], [1, 0.8, 0.8], [0, 2, 2], [1, 3, 2], 1, 1);
assert(selected == 3);

% A route above the reference is excluded even if it saves most messages.
[selected, details] = ...
    selectReferenceConstrainedParetoCandidateIndex( ...
        [0, 2, 8], [1, 0.9, 1 + eps], [0, 1, 0], ...
        1:3, 1, 1);
assert(selected == 2);
assert(details.riskRejectedSparseMask(3));
end

function assertIncrementalExpectedDisagreementExactness(context)
policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
[referenceAdjacency, referenceDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
[candidateAdjacency, candidateDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
baseOptions = struct( ...
    'maximumIncomingCount', ...
        policy.maximumIncomingCountForOutcomeEnumeration);
[referenceRisk, referenceNetworkDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, referenceAdjacency, ...
        referenceDetails.fusionWeightMatrix, baseOptions);
[fullRisk, fullDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, ...
        candidateDetails.fusionWeightMatrix, baseOptions);
incrementalOptions = baseOptions;
incrementalOptions.referenceNetworkDetails = ...
    referenceNetworkDetails;
incrementalOptions.referenceAdjacency = referenceAdjacency;
incrementalOptions.referenceFusionWeights = ...
    referenceDetails.fusionWeightMatrix;
[incrementalRisk, incrementalDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, ...
        candidateDetails.fusionWeightMatrix, incrementalOptions);

assert(isfinite(referenceRisk));
assert(abs(fullRisk - incrementalRisk) <= ...
    1e-12 * max(1, abs(fullRisk)));
assert(isequaln(fullDetails.expectedPairRisk, ...
    incrementalDetails.expectedPairRisk));
assert(isequaln(fullDetails.receiverOutcomeCounts, ...
    incrementalDetails.receiverOutcomeCounts));
assert(isequaln(fullDetails.receiverDistributions, ...
    incrementalDetails.receiverDistributions));
assert(incrementalDetails.incrementalReuseApplied);
expectedChanged = find(any( ...
    candidateAdjacency ~= referenceAdjacency, 2) | ...
    any(candidateDetails.fusionWeightMatrix ~= ...
        referenceDetails.fusionWeightMatrix, 2));
assert(isequal(reshape( ...
    incrementalDetails.recomputedReceiverIndices, [], 1), ...
    reshape(expectedChanged, [], 1)));
assert(numel(incrementalDetails.reusedReceiverIndices) + ...
    numel(expectedChanged) == context.nodeCount);

% A representable sub-1e-15 weight transfer must not be rounded into an
% unchanged receiver.  Exact row comparison is required by the reuse proof.
tinyWeights = referenceDetails.fusionWeightMatrix;
tinyReceiver = find(any(referenceAdjacency, 2), 1, 'first');
tinySender = find(referenceAdjacency(tinyReceiver, :), 1, 'first');
tinyDelta = 2^-51;
tinyWeights(tinyReceiver, tinyReceiver) = ...
    tinyWeights(tinyReceiver, tinyReceiver) + tinyDelta;
tinyWeights(tinyReceiver, tinySender) = ...
    tinyWeights(tinyReceiver, tinySender) - tinyDelta;
assert(tinyWeights(tinyReceiver, tinyReceiver) ~= ...
    referenceDetails.fusionWeightMatrix(tinyReceiver, tinyReceiver));
assert(tinyWeights(tinyReceiver, tinySender) ~= ...
    referenceDetails.fusionWeightMatrix(tinyReceiver, tinySender));
[tinyFullRisk, tinyFullDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, referenceAdjacency, tinyWeights, baseOptions);
[tinyIncrementalRisk, tinyIncrementalDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, referenceAdjacency, tinyWeights, incrementalOptions);
assert(isequaln(tinyFullRisk, tinyIncrementalRisk));
assert(isequaln(tinyFullDetails.expectedPairRisk, ...
    tinyIncrementalDetails.expectedPairRisk));
assert(isequaln(tinyFullDetails.receiverOutcomeCounts, ...
    tinyIncrementalDetails.receiverOutcomeCounts));
assert(isequaln(tinyFullDetails.receiverDistributions, ...
    tinyIncrementalDetails.receiverDistributions));
assert(isequal(tinyIncrementalDetails.recomputedReceiverIndices, ...
    tinyReceiver));

% The reusable reference is valid only for the exact same observable input.
posteriorDrift = context;
posteriorDrift.localPosteriorBySensor{1}(1).r = ...
    posteriorDrift.localPosteriorBySensor{1}(1).r - 1e-6;
assertIncrementalInputDriftRejected( ...
    posteriorDrift, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, incrementalOptions);
linkDrift = context;
linkDrift.commConfig.forceDelivery = false;
linkDrift.commConfig.pDropByEdge(1, 2) = 0.125;
assertIncrementalInputDriftRejected( ...
    linkDrift, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, incrementalOptions);
timeDrift = context;
timeDrift.currentTime = timeDrift.currentTime + 1;
assertIncrementalInputDriftRejected( ...
    timeDrift, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, incrementalOptions);
triggerDrift = context;
triggerDrift.triggerConfig.payloadExistenceThreshold = ...
    triggerDrift.triggerConfig.payloadExistenceThreshold + 1e-6;
assertIncrementalInputDriftRejected( ...
    triggerDrift, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, incrementalOptions);

% Irrelevant truth/future/runtime fields are excluded from the semantic
% fingerprint, so the causal attestation does not itself inspect them.
irrelevantDrift = context;
irrelevantDrift.model.groundTruth = struct('sentinel', 123);
irrelevantDrift.model.topologyPolicyFcn = @sin;
irrelevantDrift.commConfig.linkUniforms = ...
    rand(context.nodeCount, context.nodeCount, context.currentTime + 5);
for sensorIdx = 1:context.nodeCount
    for objectIdx = 1:numel( ...
            irrelevantDrift.localPosteriorBySensor{sensorIdx})
        irrelevantDrift.localPosteriorBySensor{sensorIdx}( ...
            objectIdx).groundTruthSentinel = 123;
    end
end
[irrelevantRisk, irrelevantDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        irrelevantDrift, referenceAdjacency, ...
        referenceDetails.fusionWeightMatrix, incrementalOptions);
assert(isequaln(irrelevantRisk, referenceRisk));
assert(isempty(irrelevantDetails.recomputedReceiverIndices));
assert(numel(irrelevantDetails.reusedReceiverIndices) == context.nodeCount);

invalidCountOptions = baseOptions;
invalidCountOptions.maximumIncomingCount = 1.6;
failed = false;
try
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, referenceAdjacency, ...
        referenceDetails.fusionWeightMatrix, invalidCountOptions);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ExpectedDirectedDisagreement:InvalidMaximumIncomingCount');
end
assert(failed);

malformedOptions = incrementalOptions;
malformedOptions.referenceNetworkDetails.receiverOutcomeCounts(:) = nan;
assertMalformedIncrementalReferenceRejected( ...
    context, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, malformedOptions);
malformedOptions = incrementalOptions;
malformedOptions.referenceNetworkDetails.expectedPairRisk(1, 2) = -1;
malformedOptions.referenceNetworkDetails.expectedPairRisk(2, 1) = -1;
assertMalformedIncrementalReferenceRejected( ...
    context, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, malformedOptions);
cacheDriftOptions = incrementalOptions;
cacheDriftOptions.referenceNetworkDetails. ...
    receiverDistributions{1}.senderReliabilities(1) = 0.75;
assertIncrementalCacheDriftRejected( ...
    context, referenceAdjacency, ...
    referenceDetails.fusionWeightMatrix, cacheDriftOptions);

partialOptions = baseOptions;
partialOptions.referenceNetworkDetails = referenceNetworkDetails;
failed = false;
try
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, ...
        candidateDetails.fusionWeightMatrix, partialOptions);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ExpectedDirectedDisagreement:IncompleteIncrementalReference');
end
assert(failed);
end

function assertMalformedIncrementalReferenceRejected( ...
        context, adjacency, weights, incrementalOptions)
failed = false;
try
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, adjacency, weights, incrementalOptions);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ExpectedDirectedDisagreement:MalformedIncrementalReference');
end
assert(failed);
end

function assertIncrementalCacheDriftRejected( ...
        context, adjacency, weights, incrementalOptions)
failed = false;
try
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, adjacency, weights, incrementalOptions);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ExpectedDirectedDisagreement:IncrementalCacheDrift');
end
assert(failed);
end

function assertIncrementalInputDriftRejected( ...
        driftedContext, adjacency, weights, incrementalOptions)
failed = false;
try
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        driftedContext, adjacency, weights, incrementalOptions);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'ExpectedDirectedDisagreement:IncrementalInputDrift');
end
assert(failed);
end

function assertIncrementalExpectedDisagreementPropertyMatrix()
policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
context = buildContext(3, [1, 2; 2, 3; 3, 4], 2, ...
    [0.15, 0.35, 0.65, 0.85]);
context.commConfig.forceDelivery = false;
[senderGrid, receiverGrid] = ndgrid( ...
    1:context.nodeCount, 1:context.nodeCount);
context.commConfig.pDropByEdge = ...
    mod(7 * senderGrid + 3 * receiverGrid, 9) / 20;
context.commConfig.pDropByEdge( ...
    1:context.nodeCount+1:end) = 0;
[referenceAdjacency, referenceDetails] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
referenceWeights = referenceDetails.fusionWeightMatrix;

missingModes = {'renormalize', 'self'};
aggregationModes = {'mean', 'tail'};
for missingIdx = 1:numel(missingModes)
    variantContext = context;
    variantContext.triggerConfig.missingNeighborWeightMode = ...
        missingModes{missingIdx};
    for aggregationIdx = 1:numel(aggregationModes)
        baseOptions = struct( ...
            'maximumIncomingCount', ...
                policy.maximumIncomingCountForOutcomeEnumeration, ...
            'aggregationMode', aggregationModes{aggregationIdx}, ...
            'tailFraction', 0.4);
        [~, referenceNetworkDetails] = ...
            computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
                variantContext, referenceAdjacency, ...
                referenceWeights, baseOptions);
        if missingIdx == 1 && aggregationIdx == 1
            driftContext = variantContext;
            driftContext.localPosteriorBySensor{1}(1).r = ...
                driftContext.localPosteriorBySensor{1}(1).r - 1e-5;
            [~, driftNetworkDetails] = ...
                computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
                    driftContext, referenceAdjacency, ...
                    referenceWeights, baseOptions);
            mixedEnvelopeOptions = baseOptions;
            mixedEnvelopeOptions.referenceNetworkDetails = ...
                referenceNetworkDetails;
            mixedEnvelopeOptions.referenceNetworkDetails. ...
                reuseInputCanonicalSha256 = ...
                    driftNetworkDetails.reuseInputCanonicalSha256;
            mixedEnvelopeOptions.referenceAdjacency = referenceAdjacency;
            mixedEnvelopeOptions.referenceFusionWeights = ...
                referenceWeights;
            assertIncrementalCacheDriftRejected( ...
                driftContext, referenceAdjacency, referenceWeights, ...
                mixedEnvelopeOptions);
        end

        % No change.
        assertIncrementalMatchesFull( ...
            variantContext, referenceAdjacency, referenceWeights, ...
            referenceAdjacency, referenceWeights, ...
            referenceNetworkDetails, baseOptions);

        % Adjacency-only change: enumerate a zero-weight physical sender.
        adjacencyOnly = referenceAdjacency;
        [receiverIdx, senderIdx] = find( ...
            variantContext.physicalAdjacency & ~referenceAdjacency, ...
            1, 'first');
        adjacencyOnly(receiverIdx, senderIdx) = true;
        assertIncrementalMatchesFull( ...
            variantContext, adjacencyOnly, referenceWeights, ...
            referenceAdjacency, referenceWeights, ...
            referenceNetworkDetails, baseOptions);

        % Weight-only change on an existing route row.
        weightOnly = referenceWeights;
        receiverIdx = find(any(referenceAdjacency, 2), 1, 'first');
        senderIdx = find(referenceAdjacency(receiverIdx, :), 1, 'first');
        delta = min(0.01, weightOnly(receiverIdx, senderIdx) / 4);
        weightOnly(receiverIdx, receiverIdx) = ...
            weightOnly(receiverIdx, receiverIdx) + delta;
        weightOnly(receiverIdx, senderIdx) = ...
            weightOnly(receiverIdx, senderIdx) - delta;
        assertIncrementalMatchesFull( ...
            variantContext, referenceAdjacency, weightOnly, ...
            referenceAdjacency, referenceWeights, ...
            referenceNetworkDetails, baseOptions);

        % Joint adjacency and weight change by removing one sender.
        jointAdjacency = referenceAdjacency;
        jointWeights = referenceWeights;
        jointAdjacency(receiverIdx, senderIdx) = false;
        jointWeights(receiverIdx, receiverIdx) = ...
            jointWeights(receiverIdx, receiverIdx) + ...
            jointWeights(receiverIdx, senderIdx);
        jointWeights(receiverIdx, senderIdx) = 0;
        assertIncrementalMatchesFull( ...
            variantContext, jointAdjacency, jointWeights, ...
            referenceAdjacency, referenceWeights, ...
            referenceNetworkDetails, baseOptions);

        % Fixed-seed randomized one-row route replacements.
        rng(7300 + 10 * missingIdx + aggregationIdx, 'twister');
        for trialIdx = 1:6
            randomAdjacency = referenceAdjacency;
            randomWeights = referenceWeights;
            receiverIdx = randi(variantContext.nodeCount);
            availableSenders = find( ...
                variantContext.physicalAdjacency(receiverIdx, :));
            order = randperm(numel(availableSenders));
            incomingCount = min(1 + mod(trialIdx, 2), ...
                numel(availableSenders));
            selectedSenders = availableSenders( ...
                order(1:incomingCount));
            randomAdjacency(receiverIdx, :) = false;
            randomAdjacency(receiverIdx, selectedSenders) = true;
            randomWeights(receiverIdx, :) = 0;
            randomWeights(receiverIdx, receiverIdx) = 0.5;
            randomWeights(receiverIdx, selectedSenders) = ...
                0.5 / incomingCount;
            assertIncrementalMatchesFull( ...
                variantContext, randomAdjacency, randomWeights, ...
                referenceAdjacency, referenceWeights, ...
                referenceNetworkDetails, baseOptions);
        end
    end
end
end

function assertIncrementalMatchesFull( ...
        context, candidateAdjacency, candidateWeights, ...
        referenceAdjacency, referenceWeights, ...
        referenceNetworkDetails, baseOptions)
[fullRisk, fullDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, candidateWeights, baseOptions);
incrementalOptions = baseOptions;
incrementalOptions.referenceNetworkDetails = referenceNetworkDetails;
incrementalOptions.referenceAdjacency = referenceAdjacency;
incrementalOptions.referenceFusionWeights = referenceWeights;
[incrementalRisk, incrementalDetails] = ...
    computeExpectedLmbNetworkDisagreementForDirectedRouting( ...
        context, candidateAdjacency, candidateWeights, ...
        incrementalOptions);
assert(isequaln(fullRisk, incrementalRisk));
assert(isequaln(fullDetails.expectedPairRisk, ...
    incrementalDetails.expectedPairRisk));
assert(isequaln(fullDetails.receiverOutcomeCounts, ...
    incrementalDetails.receiverOutcomeCounts));
assert(isequaln(fullDetails.receiverDistributions, ...
    incrementalDetails.receiverDistributions));
expectedChanged = find(any( ...
    candidateAdjacency ~= referenceAdjacency, 2) | ...
    any(candidateWeights ~= referenceWeights, 2));
assert(isequal(reshape( ...
    incrementalDetails.recomputedReceiverIndices, [], 1), ...
    reshape(expectedChanged, [], 1)));
end

function assertJointProjectionAndRuntimeMetadata()
ringEdges = [1, 2; 2, 3; 3, 4; 1, 4];
context = buildContext(4, ringEdges, 4, ...
    [0.011, 0.011, 0.30, 0.30]);
[referenceAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
[jointAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [3, 4]);
historyPage = jointAdjacency;
groupIds = context.model.dynamicTopologyScenario.config.sensorGroupIds;
[receivers, senders] = find(referenceAdjacency);
restoredMask = ...
    (groupIds(receivers) == 3 & groupIds(senders) == 4) | ...
    (groupIds(receivers) == 4 & groupIds(senders) == 3);
for edgeIdx = reshape(find(restoredMask), 1, [])
    historyPage(receivers(edgeIdx), senders(edgeIdx)) = true;
end
context.previousAdjacency = historyPage;
context.previousAdjacencyHistory = repmat( ...
    historyPage, 1, 1, 2);

base = buildFormationBackboneBundleProtectionControl(context);
assert(all(base.formationProtectionScore(3:4) > 0.02));
assert(all(base.cardinalityEvidenceMask(3:4)));
assert(all(base.singleActionSafetyMask(3:4)));
assert(isequal(base.requestedFormationIds, [3, 4]));
assert(isequal(base.selectedFormationIds, 4));
assert(isequal(base.projectionRemovalOrder, 3));
assert(~base.referenceFallbackUsed);
assert(base.evaluatedRouteCount == 7);

[adjacency, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy( ...
        context);
assert(isequal(adjacency, base.selectedAdjacency));
assert(isequal(details.protectionRequestedFormationIds, [3, 4]));
assert(isequal(details.protectionBaseSelectedFormationIds, 4));
assert(isequal(details.protectionSelectedFormationIds, 4));
assert(isequal(details.protectionBaseProjectionRemovalOrder, 3));
assert(isempty(details.protectionReleasedFormationIds));
assert(isequal(details.retentionDebtRequestedFormationIds, [3, 4]));
assert(isequal(details.retentionDebtProjectionRemovalOrder, 3));
assert(isfinite(details.selectionSeconds));
assert(details.oneStepJointProjectionUsed);

[paretoAdjacency, paretoDetails] = ...
    selectFormationBackboneBundleParetoRiskRuntimePolicy(context);
assert(nnz(paretoAdjacency) <= nnz(referenceAdjacency));
assert(paretoDetails.taskRisk <= paretoDetails.baselineTaskRisk);
assert(isequal(paretoAdjacency, referenceAdjacency));
assert(paretoDetails.paretoRiskRejected);
assert(paretoDetails.referenceFallbackUsed);
assert(~paretoDetails.noSparseProposal);
assert(paretoDetails.paretoSparseProposalCount == 2);
assert(paretoDetails.paretoRiskFeasibleSparseCount == 0);
assert(strcmp(paretoDetails.selectionState, ...
    'all-sparse-proposals-risk-rejected'));
assert(strcmp(paretoDetails.actionName, 'reference'));
end

function assertStaggeredRelease()
pathEdges = [1, 2; 2, 3; 3, 4];
context = buildContext(6, pathEdges, 4, ...
    [0.011, 0.30, 0.011, 0.82]);
[referenceAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, []);
[incumbentAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, [2, 4]);
context.previousAdjacency = incumbentAdjacency;
context.previousAdjacencyHistory = cat( ...
    3, referenceAdjacency, incumbentAdjacency);

control = buildFormationBackboneBundleStaggeredRecoveryControl( ...
    context);
assert(isequal(control.incumbentFormationIds, [2, 4]));
assert(control.staggeredReleaseUsed);
assert(isequal(control.releaseSchedule.releaseOrder, [4, 2]));
assert(control.releaseSchedule.candidateCount == 1);
assert(control.candidateSafetyMask(1));
assert(control.candidateEligibilityMask(1));
assert(control.selectedProtectionCoverageFraction >= 0.80);
assert(control.selectedDisagreementImprovementFraction >= 0.0025);
assert(isequal(control.selectedFormationIds, 2));
assert(isequal(control.releasedFormationIds, 4));

paretoControl = buildFormationBackboneBundleParetoRiskControl(context);
assert(strcmp(paretoControl.selectionState, ...
    'all-sparse-proposals-risk-rejected'));
assert(paretoControl.referenceFallbackUsed);
assert(paretoControl.referenceRiskFeasibleSparseCount == 0);
retainedTwoIdx = find(cellfun(@(candidate) ...
    isequal(candidate.formationIds, 2), ...
    paretoControl.candidateBank), 1, 'first');
assert(~isempty(retainedTwoIdx));
retainedTwo = paretoControl.candidateBank{retainedTwoIdx};
assert(any(strcmp(retainedTwo.provenanceSources, ...
    'requested-safe-single')));
assert(any(strcmp(retainedTwo.provenanceSources, ...
    'safe-staggered-release')));
assert(isequal(retainedTwo.releasedFormationIds, 4));
assert(isempty(retainedTwo.newlySuspendedFormationIds));

[~, details] = ...
    selectFormationBackboneBundleStaggeredRecoveryRuntimePolicy( ...
        context);
assert(isequal(details.protectionRequestedFormationIds, [2, 4]));
assert(isequal(details.protectionBaseSelectedFormationIds, [2, 4]));
assert(isequal(details.protectionSelectedFormationIds, 2));
assert(isempty(details.protectionBaseProjectionRemovalOrder));
assert(isequal(details.protectionReleasedFormationIds, 4));
assert(details.reserveSchedule.staggeredReleaseUsed);
end

function assertRejectsRepeatedB3Violation(context)
[suspendedAdjacency, ~] = ...
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
context.previousAdjacency = suspendedAdjacency;
context.previousAdjacencyHistory = repmat( ...
    suspendedAdjacency, 1, 1, 2);
failed = false;
try
    selectTemporalFormationBackboneInputBundleSuspensionPolicy( ...
        context, 2);
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationBundleSuspension:Infeasible');
end
assert(failed);
end

function context = buildContext( ...
        sensorsPerFormation, backboneEdges, labelCount, ...
        existenceByFormation)
formationCount = 4;
if nargin < 1
    sensorsPerFormation = 6;
end
if nargin < 2
    backboneEdges = [1, 2; 2, 3; 3, 4];
end
if nargin < 3
    labelCount = 1;
end
if nargin < 4
    existenceByFormation = 0.9 * ones(1, formationCount);
end
nodeCount = formationCount * sensorsPerFormation;
groupIds = repelem(1:formationCount, sensorsPerFormation);
model = generateMultisensorModel( ...
    nodeCount, [0, 0], 0.9 * ones(1, nodeCount), ...
    3 * ones(1, nodeCount), 'GA', 'LBP');
staticAdjacency = buildStatic(groupIds, backboneEdges);
model.dynamicTopologyScenario = struct( ...
    'config', struct('sensorGroupIds', groupIds), ...
    'staticAdjacency', staticAdjacency);
posteriors = cell(1, nodeCount);
for sensorIdx = 1:nodeCount
    objects = repmat(model.birthParameters(1), 1, labelCount);
    for objectIdx = 1:labelCount
        objects(objectIdx).birthTime = 1;
        objects(objectIdx).birthLocation = objectIdx;
        objects(objectIdx).r = ...
            existenceByFormation(groupIds(sensorIdx));
        objects(objectIdx).numberOfGmComponents = 1;
        objects(objectIdx).w = 1;
        objects(objectIdx).mu = {zeros(model.xDimension, 1)};
        objects(objectIdx).Sigma = {eye(model.xDimension)};
    end
    posteriors{sensorIdx} = objects;
end
context = struct();
context.localPosteriorBySensor = posteriors;
context.model = model;
context.commConfig = struct( ...
    'forceDelivery', true, ...
    'pDropByEdge', zeros(nodeCount), ...
    'outageSchedule', []);
context.triggerConfig = buildMixtureAwareKlaReferenceConfig();
context.currentTime = 10;
context.physicalAdjacency = logical(ones(nodeCount) - eye(nodeCount));
context.baseAdjacency = staticAdjacency;
context.edgeBudget = nodeCount;
context.directedMessageBudget = 2 * nodeCount;
context.positions = zeros(2, nodeCount);
[referenceAdjacency, ~] = ...
    selectFormationBackboneResidualTourPolicy(context);
context.previousAdjacency = referenceAdjacency;
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
context.previousAdjacencyHistoryCount = 2;
context.previousAdjacencyHistoryTimes = [8, 9];
context.previousAdjacencyHistoryConvention = ...
    'receiver-row-sender-column-directed-oldest-to-newest';
context.previousAdjacencyHistorySource = 'synthetic-test-history';
context.edgeScores = [];
context.localInnovationHistory = [];
context.localAssociationConfidenceHistory = [];
context.localNisNormHistory = [];
context.localNisDeviationHistory = [];
context.localUpdateHistoryTimes = zeros(1, 0);
context.nodeCount = nodeCount;
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
