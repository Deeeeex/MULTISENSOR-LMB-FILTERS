function test_formation_b4_v46_tracking_evaluation_registry()
% Outcome definitions are frozen before outcome scoring is authorized.

evaluation = getFormationB4V46TrackingEvaluationRegistry();
source = getFormationB4V46TrackingSourceRegistry();
structural = getFormationCausalMinimalEditV46Protocol();
assert(evaluation.metricDefinitionFrozen);
assert(strcmp(evaluation.structuralProtocolCanonicalSha256, ...
    structural.canonicalSha256));
assert(isequal(evaluation.orderedArmIds, structural.primaryArms));
assert(evaluation.referenceArmOrdinal == 1);
assert(evaluation.candidateArmOrdinal == 2);
assert(strcmp(evaluation.referenceArmId, ...
    'v46-repaired-reference-a70-e05'));
assert(strcmp(evaluation.candidateArmId, ...
    'v46-repaired-sync-all-b4-e20-mc'));
assert(evaluation.armRoleAssignmentFrozen);
assert(~evaluation.armSwapAllowed);

assert(strcmp(evaluation.primaryMetricName, ...
    'Position-only Euclidean OSPA (position E-OSPA)'));
assert(evaluation.primaryMetricIsPositionOnly);
assert(strcmp(evaluation.primaryMetricPhysicalUnit, 'metre'));
assert(isequal(evaluation.primaryMetricStateIndices, [1.0, 2.0]));
assert(~evaluation.velocityCoordinatesUsedByPrimaryMetric);
assert(~evaluation.primaryMetricIsExtendedOspa);
assert(~evaluation.legacyFullStateOspaUsedForGate);
assert(strcmp(evaluation.primaryMetricFunctionSourceSha256, ...
    computeFileSha256(which(evaluation.primaryMetricFunction))));
assert(strcmp(evaluation.assignmentFunctionSourceSha256, ...
    computeFileSha256(which(evaluation.assignmentFunction))));
assert(evaluation.eOspaComponentOrdinal == 1);
assert(strcmp(evaluation.eOspaComponentMeaning, ...
    'total-position-localization-plus-cardinality'));
assert(evaluation.rawPositionEospaTotalBySensorTimeRequired);
assert(evaluation.rawPositionEospaLocalizationBySensorTimeRequired);
assert(evaluation.rawPositionEospaCardinalityBySensorTimeRequired);
assert(evaluation.rawEstimatedCardinalityBySensorTimeRequired);
assert(evaluation.rawTruthCardinalityByTimeRequired);
assert(evaluation.rawConsensusPositionOspaByTimeRequired);
assert(evaluation.rawAttemptedAndDeliveredMasksRequired);
assert(evaluation.rawAttemptedAndDeliveredPayloadLedgersRequired);
assert(evaluation.rawAttemptedAndDeliveredScalarLedgersRequired);
assert(evaluation.rawRepairFlagsByTimeRequired);
assert(evaluation.firstRepairTimeRequired);
assert(~evaluation.nonfiniteRawMetricAllowed);
assert(~evaluation.incompleteCaseAllowed);
assert(~evaluation.imputationAllowed);
assert(~evaluation.unfavorableCaseExclusionAllowed);
assert(~evaluation.thresholdTuningAfterDevelopmentAllowed);
assert(evaluation.communicationReferenceTotalMustBePositive);
assert(~evaluation.communicationZeroZeroRelativeChangeDefined);
assert(evaluation.zeroReferenceCandidatePositiveFailsBlock);
assert(~evaluation.undefinedCaseChangeExclusionAllowed);
assert(evaluation.allDescriptiveMetricsMustBeReported);
assert(numel(evaluation.descriptiveMetricSpecs) == 10);
assert(isequal(evaluation.descriptiveMetricIds, ...
    {evaluation.descriptiveMetricSpecs.id}));
assert(strcmp(evaluation.outcomeEvaluatorFunctionSourceSha256, ...
    computeFileSha256(which(evaluation.outcomeEvaluatorFunction))));

accounting = evaluation.communicationAccounting;
assert(accounting.directedMessages);
assert(accounting.selfMessagesExcluded);
assert(accounting.payloadEventType == 2);
assert(accounting.bytesPerScalar == 8);
assert(strcmp(accounting.primaryCommunicationGateLedger, ...
    'attemptedPayloadBytes'));
assert(accounting.deliveredPayloadLedgerIsDescriptive);
assert(~accounting.serializationPaddingBytesIncluded);
assert(~accounting.linkNetworkTransportOverheadIncluded);
assert(~accounting.meanOfPerTimeSavingsAllowed);
assert(~accounting.meanOfPerEdgeSavingsAllowed);
assert(strcmp(accounting.payloadEstimatorSourceSha256, ...
    computeFileSha256(which(accounting.payloadEstimatorFunction))));

assert(evaluation.developmentPrimaryPairCount == 6);
assert(evaluation.developmentStressPairCount == 2);
assert(isequal(evaluation.developmentPrimaryCaseOrdinals, ...
    [1, 6, 11, 21, 26, 31]));
assert(isequal(evaluation.developmentStressCaseOrdinals, [16, 36]));
assert(evaluation.caseCount == source.caseCount);
assert(isequal({evaluation.cases.caseId}, {source.cases.id}));
assert(~evaluation.filterExecutionAuthorized);
assert(~evaluation.stateEstimateAccessAuthorized);
assert(~evaluation.groundTruthAccessAuthorized);
assert(~evaluation.trackingOutcomeScoringAuthorized);
assert(~evaluation.armSelectionAuthorized);
assert(~evaluation.developmentAdvanceDecisionAuthorized);
assert(~evaluation.confirmationTrackingAuthorized);
assert(~evaluation.validationClaimAllowed);

thresholds = evaluation.developmentThresholds;
assert(thresholds.attemptedMessageSavingExact == 0.375);
assert(thresholds.attemptedByteSavingPerPairMinimum == 0.20);
assert(thresholds.attemptedByteSavingPrimaryMedianMinimum == 0.30);
assert(thresholds.fullHorizonEospaIncreasePerPairMaximum == 0.05);
assert(thresholds.fullHorizonEospaIncreasePrimaryMedianMaximum == 0.02);
assert(thresholds.focusWindowEospaIncreasePerPairMaximum == 0.05);
assert(thresholds.worstSensorEospaIncreasePerPairMaximum == 0.10);
assert(thresholds.meanConsensusOspaIncreasePerPairMaximum == 0.10);
assert(thresholds.allEightRuntimeAuditsMustPass);
assert(~thresholds.primaryGateUsesCrossingStressPairs);
assert(thresholds.crossingStressPairsMustComplete);
assert(thresholds.developmentNoninferiorityOnly);
assert(~thresholds.accuracyImprovementClaimAllowed);
assert(~thresholds.statisticalValidationClaimAllowed);

confirmation = evaluation.confirmationAnalysis;
assert(strcmp(confirmation.independentUnit, 'source-seed'));
assert(confirmation.independentSeedCount == 4);
assert(confirmation.fixedPrimaryStratumCount == 6);
assert(confirmation.primaryPairCount == 24);
assert(confirmation.stressPairCount == 8);
assert(confirmation.withinSeedScenesAreCorrelated);
assert(~confirmation.sensorTimeValuesAreIndependentReplicates);
assert(isequal(confirmation.orderedPrimarySceneOrdinals, ...
    [1, 2, 3, 5, 6, 7]));
assert(isequal(confirmation.orderedStressSceneOrdinals, [4, 8]));
assert(isequal(confirmation.orderedConfirmationSeeds, ...
    source.confirmationSeeds));
assert(isequal(confirmation.primaryCaseOrdinalMatrix, [ ...
    2, 7, 12, 22, 27, 32; ...
    3, 8, 13, 23, 28, 33; ...
    4, 9, 14, 24, 29, 34; ...
    5, 10, 15, 25, 30, 35]));
assert(isequal(confirmation.stressCaseOrdinalMatrix, [ ...
    17, 37; 18, 38; 19, 39; 20, 40]));
assert(confirmation.requiredPassingSeedBlockCount == 4);
assert(confirmation.allConfirmationSeedsMustPass);
assert(~confirmation.poolingMayRescueSeedFailure);
assert(~confirmation.poolingMayRescueStratumFailure);
assert(~confirmation.stressAffectsPrimaryDecision);
assert(~confirmation.stressUndefinedRelativeChangeFailsPrimaryDecision);
assert(~confirmation.significanceTestingAllowed);
assert(~confirmation.pValueReportingAllowed);
assert(~confirmation.bootstrapConfidenceIntervalAllowed);
assert(~confirmation.parametricConfidenceIntervalUsedForGate);
assert(~confirmation.confidenceIntervalReportingAllowed);
assert(~confirmation.parametricConfidenceIntervalReportingAllowed);
assert(~confirmation.descriptiveRangesMayBeCalledConfidenceIntervals);
assert(confirmation. ...
    scaleSummariesComputedIndependentlyWhenOtherScaleUndefined);
assert(~confirmation.populationGeneralizationAllowed);
assert(~confirmation.descriptiveSummaryIsConfidenceInterval);
assert(strcmp(confirmation.perSeedPrimaryRuleCanonicalSha256, ...
    computeCanonicalValueSha256(thresholds)));

for caseIdx = 1:source.caseCount
    value = evaluation.cases(caseIdx);
    contract = source.cases(caseIdx);
    scene = source.scenes(contract.sceneOrdinal);
    config = buildDynamicTopologyScenarioConfig(contract.presetName);
    assert(value.caseOrdinal == contract.ordinal);
    assert(strcmp(value.coreCaseCanonicalSha256, ...
        contract.caseCanonicalSha256));
    assert(value.sceneOrdinal == contract.sceneOrdinal);
    assert(strcmp(value.sceneId, contract.sceneId));
    assert(value.scale == scene.scale);
    assert(strcmp(value.style, scene.style));
    assert(value.seed == contract.seed);
    assert(isequal(value.focusWindow, contract.focusWindow));
    assert(value.simulationLength == contract.simulationLength);
    assert(value.positionOspaParameters.cutoffMetres == ...
        config.ospaPositionCutoff);
    assert(value.positionOspaParameters.order == 2.0);
    assert(isequal(value.positionOspaParameters.positionStateIndices, ...
        [1.0, 2.0]));
end
assert(strcmp(computeCanonicalValueSha256( ...
    rmfield(evaluation, 'canonicalSha256')), ...
    evaluation.canonicalSha256));
fprintf('PASS: V46 tracking evaluation registry tests\n');
end
