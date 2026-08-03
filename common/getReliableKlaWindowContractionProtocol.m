function protocol = getReliableKlaWindowContractionProtocol()
% GETRELIABLEKLAWINDOWCONTRACTIONPROTOCOL Frozen v41 theory boundary.
%
% The certificate is a causal structural constraint, not a standalone
% topology objective or a tracking-risk guarantee.  A later controller may
% compare candidate route sequences only after it separately models the
% posterior/innovation disturbance introduced by local Bayes updates and
% approximate LMB fusion.

payload = struct();
payload.id = 'reliable-kla-window-contraction-v41-v1';
payload.contractVersion = ...
    'reliable-kla-window-contraction-protocol-v1';
payload.certificateContractVersion = ...
    'reliable-kla-window-contraction-certificate-v1';
payload.meanSquareCertificateContractVersion = ...
    'reliable-kla-window-mean-square-contraction-certificate-v1';
payload.runtimeSemanticsContractVersion = ...
    'reliable-kla-linear-mixing-runtime-semantics-v1';
payload.communicationSemanticsContractVersion = ...
    'reliable-kla-communication-runtime-semantics-v1';
payload.matrixOrientation = 'receiver-row-sender-column';
payload.windowHorizonRule = ...
    'at-least-node-count-minus-one-unless-longer-causal-plan-supplied';
payload.staticPageDevelopmentProbeHorizonRule = 'node-count-minus-one';
payload.adaptiveReferenceSquaredContractionTarget = 0.90;
payload.adaptiveMaximumHorizonMultiplier = 4;
payload.adaptiveHorizonCalibrationMode = ...
    'runtime-configured-missing-neighbor-mode';
payload.adaptiveHorizonRule = [ ...
    'first-repeated-current-reference-horizon-at-or-below-target-', ...
    'within-four-node-count-minus-one'];
payload.windowOperationalUnit = ...
    'one-online-fusion-update-per-tracking-step';
payload.decisionActionPageCount = 1;
payload.additionalWithinStepFusionRounds = 0;
payload.failedCalibrationCandidateActionRule = ...
    'reference-only-no-candidate-evaluation';
payload.requiredScheduleRule = ...
    'best-root-greedy-temporal-broadcast-max-q-times-beta';
payload.expectedCoefficientBound = ...
    'expected-dobrushin-at-most-one-minus-q-times-beta';
payload.primaryStructuralCertificate = ...
    'exact-expected-window-centered-l2-quadratic-form';
payload.singleEventDobrushinCertificateRole = ...
    'rigorous-diagnostic-not-runtime-ranking-signal';
payload.requiredDeliveryProbabilityModel = ...
    'independent-receiver-sender-time-deliveries';
payload.realizedLinkUniformsAllowedInPolicyContext = false;
payload.forcedDeliveryAllowed = false;
payload.deterministicOutageScheduleAllowed = false;
payload.communicationRuntimeAttestationRequired = true;
payload.registeredDeliverySampler = [ ...
    'runEventTriggeredDistributedLmbFilter-simulateDelivery-', ...
    'independent-uniform-per-sender-receiver-time'];
payload.meanSquareMomentFactorization = ...
    'independent-receiver-rows-and-independent-time-pages';
payload.allowedMissingNeighborWeightModes = {'renormalize', 'self'};
payload.maximumIncomingCount = 4;
payload.runtimeReliabilitySource = ...
    'current-page-repeat-or-registered-causal-forecast-only';
payload.realizedFutureReliabilityAllowed = false;
payload.realizedDeliveryUniformsAllowed = false;
payload.truthUsed = false;
payload.futureOutcomeUsed = false;
payload.posteriorUsedByStructuralCertificate = false;
payload.exactKlaSetDensityLogRatioClaim = ...
    'common-positive-support-and-no-new-local-update-disturbance';
payload.marginalExistenceLogOddsClaimAllowed = false;
payload.spatialOverlapNormalizerRequiresDisturbanceTerm = true;
payload.localBayesUpdateRequiresDisturbanceTerm = true;
payload.labelSupportLossRequiresDisturbanceTerm = true;
payload.mixtureApproximationRequiresDisturbanceTerm = true;
payload.pruningRequiresDisturbanceTerm = true;
payload.graphCertificateIsStandaloneTrackingSafetyTest = false;
payload.graphCertificateIsStandaloneSelectionObjective = false;
payload.candidateActionMustVaryCrossFormationMixing = true;
payload.laterValueModelRole = ...
    'predict-window-task-value-among-structurally-certified-actions';
payload.advancementThresholdFrozen = false;
payload.m24TrackingAuthorized = false;
payload.x36TrackingAuthorized = false;
payload.validationClaimAllowed = false;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '88354e5ab93b168d7da617f9dbfed5e5e9f16cb0d6cccae0f65b68126832662a';
if ~strcmp(actualSha256, expectedSha256)
    error('ReliableKlaWindowContraction:UnregisteredDrift', [ ...
        'The v41 protocol changed without a protocol-version update: ', ...
        'actual=%s expected=%s.'], actualSha256, expectedSha256);
end
protocol = payload;
protocol.canonicalSha256 = actualSha256;
end
