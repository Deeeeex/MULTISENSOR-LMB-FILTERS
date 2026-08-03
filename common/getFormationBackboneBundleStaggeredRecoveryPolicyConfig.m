function config = getFormationBackboneBundleStaggeredRecoveryPolicyConfig()
% GETFORMATIONBACKBONEBUNDLESTAGGEREDRECOVERYPOLICYCONFIG Shared v38 method.
%
% This file contains method parameters only.  It deliberately contains no
% preset, seed, window, expected action, cache, result, or authorization.

payload = struct();
payload.id = 'formation-backbone-bundle-staggered-recovery-v38-v1';
payload.contractVersion = ...
    'formation-backbone-bundle-staggered-recovery-policy-config-v1';
payload.referenceRouteId = ...
    'fixed-index-plus-registered-formation-backbone-residual-tour-v1';
payload.dominantWeight = 0.70;
payload.residualWeight = 0.05;
payload.maximumFormationCount = 8;
payload.minimumSuspensionAgeSteps = 1;
payload.minimumRetainedProtectionCoverageFraction = 0.80;
payload.minimumIncumbentDisagreementImprovementFraction = 0.0025;
payload.minimumMessageSavingCount = 1;
payload.maximumIncomingCountForOutcomeEnumeration = 4;
payload.maximumExistenceRetentionRisk = 0.01;
payload.minimumFormationMeanCardinalityChange = -0.05;
payload.minimumSupportedLabelRetentionRatio = 0.80;
payload.maximumDecisionThresholdCrossingCount = 0;
payload.referenceSupportThreshold = 0.05;
payload.decisionExistenceThreshold = 0.50;
payload.retentionReceiverTailFraction = 0.25;
payload.retentionReceiverTailWeight = 0.50;
payload.protectionScoreOnFraction = 0.02;
payload.protectionScoreOffFraction = 0.01;
payload.minimumReferenceFormationExpectedCardinality = 1.0;
payload.maximumControlRouteEvaluations = ...
    3 * payload.maximumFormationCount + 1;
payload.rollingConnectivityWindowSteps = 3;
payload.referenceForecastSteps = 2;
payload.sceneSpecificParametersAllowed = false;
payload.truthUsed = false;
payload.futureOutcomeUsed = false;

actualSha256 = computeCanonicalValueSha256(payload);
expectedSha256 = ...
    '6c7d049e5169944b6bee0206d790fce224e87e7bd6090ffcad483eb49b07e247';
if ~strcmp(actualSha256, expectedSha256)
    error('FormationBackboneBundlePolicy:UnregisteredDrift', [ ...
        'The shared v38 method changed without a policy-version ', ...
        'update: actual=%s expected=%s.'], ...
        actualSha256, expectedSha256);
end
config = payload;
config.canonicalSha256 = actualSha256;
end
