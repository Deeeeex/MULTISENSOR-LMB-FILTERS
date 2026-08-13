function protocol = getCombinedOutputSafetyV139Protocol()
% GETCOMBINEDOUTPUTSAFETYV139PROTOCOL Complementary output-only gates.

v138 = getPostFusionLabelReadoutV138Protocol();
v137 = getPredictiveEvidenceFallbackV137Protocol();
protocol = v138;
protocol.id = 'combined-output-safety-v139-v1';
protocol.contractVersion = ...
    'v139-combined-output-safety-protocol-v1';
protocol.methodName = ...
    'post-fusion label readout plus output-only predictive fallback';
protocol.outcomePolicyName = ...
    'combined-output-safety-v139-screen-v1';
protocol.predictiveEvidence = v137.predictiveEvidence;
protocol.predictiveEvidence.fallbackState = ...
    'current-output-only-exact-reference-relay';
protocol.predictiveEvidence.workingStateMutationAllowed = false;
protocol.screen.minimumSensorGainFraction = -1e-4;
protocol.screen.minimumFormationGainFraction = 0;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v139', 'combined_output_safety_v1');
end
