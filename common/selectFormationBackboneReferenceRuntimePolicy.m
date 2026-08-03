function [adjacency, details] = ...
    selectFormationBackboneReferenceRuntimePolicy(context)
% SELECTFORMATIONBACKBONEREFERENCERUNTIMEPOLICY Frozen v38 reference arm.

timerId = tic;
policy = getFormationBackboneBundleStaggeredRecoveryPolicyConfig();
[adjacency, details] = selectFormationBackboneResidualTourPolicy( ...
    context, struct( ...
        'dominantWeight', policy.dominantWeight, ...
        'residualWeight', policy.residualWeight));
details.contractVersion = ...
    'formation-backbone-reference-runtime-policy-v1';
details.mode = 'formation-backbone-reference-runtime';
details.actionName = 'reference';
details.policyConfigSha256 = policy.canonicalSha256;
details.selectionSeconds = toc(timerId);
details.posteriorUsed = false;
details.currentPosteriorUsed = false;
details.currentLinkReliabilityUsed = false;
details.truthUsed = false;
details.groundTruthUsed = false;
details.futureMeasurementUsed = false;
details.futureOutcomeUsed = false;
details.recursiveSafetyClaimed = false;
end
