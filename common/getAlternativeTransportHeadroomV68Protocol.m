function protocol = getAlternativeTransportHeadroomV68Protocol()
% GETALTERNATIVETRANSPORTHEADROOMV68PROTOCOL Frozen relay edge diagnostic.

relay = getSignedCrossFormationOpportunityV67Protocol();
protocol = struct();
protocol.id = 'alternative-transport-headroom-v68-relay-t124-v1';
protocol.contractVersion = ...
    'alternative-transport-headroom-v68-protocol-v1';
protocol.baseProtocolId = relay.id;
protocol.presetName = relay.presets{1};
protocol.seed = relay.allSeeds;
protocol.currentTime = 124;
protocol.cacheRoot = relay.cacheRoot;
protocol.outputRoot = fullfile( ...
    'RUN', 'GA', 'dynamic_topology', 'evidence', ...
    'tracking_aligned_v68', 'alternative_transport_headroom');
protocol.sourceWeight = 0.05;
protocol.positiveSupportThreshold = 0.20;
protocol.decisionExistenceThreshold = 0.50;
protocol.minimumMaterialHeadroomFraction = 0.01;
protocol.trackingOutcomeScoringAuthorized = false;
protocol.routeExecutionAuthorized = false;
protocol.modelTrainingAuthorized = false;
protocol.validationClaimAllowed = false;
protocol.openedDevelopmentEvidenceOnly = true;
protocol.evidenceBoundary = [ ...
    'V68 evaluates unused physical cross-formation senders at the ', ...
    'source-only relay t=124 sign-reversal state. Each diagnostic ', ...
    'candidate replaces one registered residual sender at the same ', ...
    'weight and message count, using the installed projected-Gaussian ', ...
    'label-wise KLA. It is not connectivity projected and therefore ', ...
    'does not authorize route execution, tracking, training, or ', ...
    'validation.'];
end
