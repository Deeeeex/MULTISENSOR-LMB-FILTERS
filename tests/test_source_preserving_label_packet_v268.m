function test_source_preserving_label_packet_v268()
% TEST_SOURCEPRESERVINGLABELPACKET_V268 Two-hop source/age semantics.

model = generateMultisensorModel( ...
    4, zeros(1, 4), 0.9 * ones(1, 4), ...
    3 * ones(1, 4), 'GA', 'LBP');
model.dynamicTopologyScenario.config.sensorGroupIds = [1, 2, 3, 3];
label = [1; 4];
source = makeObject(model, label, 0.80, [1; 2; 0.5; -0.25]);
relay = makeObject(model, [1; 2], 0.60, zeros(4, 1));
target = makeObject(model, label, 0.55, [4; 5; 0; 0]);
targetHigh = makeObject(model, label, 0.75, [5; 6; 0; 0]);
posteriors = {source, relay, target, targetHigh};
relayBefore = posteriors{2};

physical = false(4);
physical(1, 2) = true;
physical(2, 1) = true;
physical(2, 3) = true;
physical(3, 2) = true;
physical(2, 4) = true;
physical(4, 2) = true;
delivered = false(4);
delivered(1, 2) = true;
delivered(2, 3) = true;
delivered(2, 4) = true;
edgeMask = false(4);
edgeMask(2, 3) = true;
config = struct( ...
    'topologyPolicySourcePreservingLabelPacketMaximumAge', 1, ...
    'topologyPolicySourcePreservingLabelPacketMetadataScalars', 2, ...
    'topologyPolicyLabelInputRouteEtaProjectionEnabled', true, ...
    'topologyPolicyLabelInputRouteMinimumMapExistence', 0.50, ...
    'topologyPolicyLabelInputRouteMaximumReferenceLogOddsDrop', 0.25, ...
    'topologyPolicyLabelInputRouteMaximumSubMapReferenceLogOddsDrop', 0, ...
    'topologyPolicyLabelInputRouteMaximumReferenceLogOddsIncrease', 0.25, ...
    'topologyPolicyLabelInputRouteEtaIdentityTolerance', 1e-8);
schedule = struct( ...
    'contractVersion', ...
        'source-preserving-label-packet-v268-schedule-v1', ...
    'labelShortcutActive', true, ...
    'selectedLocalizationLabel', label, ...
    'selectedLocalizationFormationId', 3, ...
    'labelShortcutSourceSensorId', 1, ...
    'labelShortcutReceiverSensorId', 2, ...
    'labelShortcutSourceWeight', 0.05, ...
    'labelShortcutSourceWeightGrid', [0.05, 0.025], ...
    'labelShortcutWeightMode', 'bounded-proportional-share');

[route1, cache, page1] = ...
    prepareSourcePreservingLabelPacketRouteV268( ...
        posteriors, physical, delivered, edgeMask, schedule, ...
        model, 1, struct([]), config);
assert(~route1.active);
assert(page1.firstHopScheduled && page1.firstHopDelivered);
assert(page1.firstHopPacketStored && ~page1.secondHopScheduled);
assert(page1.attemptedPayloadBytes == 240);
assert(page1.deliveredPayloadBytes == 240);
assert(cache.active && cache.sourceTime == 1);
assert(isequal(posteriors{2}, relayBefore));

idleSchedule = struct('labelShortcutActive', false);
[route2, cache, page2] = ...
    prepareSourcePreservingLabelPacketRouteV268( ...
        posteriors, physical, delivered, edgeMask, idleSchedule, ...
        model, 2, cache, config);
assert(route2.active && route2.forceAppendPreservedSource);
assert(route2.sourceId == 1 && route2.transportSenderId == 2);
assert(isequal(route2.receiverIds, 3));
assert(route2.sourceTimestamp == 1 && route2.packetAge == 1);
assert(page2.secondHopScheduled && page2.secondHopDelivered);
assert(page2.secondHopBackboneAligned);
assert(page2.attemptedPayloadBytes == 240);
assert(page2.deliveredPayloadBytes == 240);
assert(abs(route2.sourceObject.r - ...
    model.survivalProbability * source.r) <= 1e-12);
assert(norm(route2.sourceObject.mu{1} - ...
    (model.A * source.mu{1} + model.u)) <= 1e-12);
assert(isempty(cache));

% V269 changes only the beneficiary rule: maximum label-existence margin
% takes S4 even though the current backbone edge points to S3.
[~, marginCache] = prepareSourcePreservingLabelPacketRouteV268( ...
    posteriors, physical, delivered, edgeMask, schedule, ...
    model, 1, struct([]), config);
config.topologyPolicySourcePreservingLabelPacketBeneficiaryMode = ...
    'maximum-existence-margin';
[marginRoute, marginCache, marginPage] = ...
    prepareSourcePreservingLabelPacketRouteV268( ...
        posteriors, physical, delivered, edgeMask, idleSchedule, ...
        model, 2, marginCache, config);
assert(marginRoute.active && isequal(marginRoute.receiverIds, 4));
assert(~marginPage.secondHopBackboneAligned);
assert(abs(marginPage.secondHopReceiverExistence - 0.75) <= 1e-12);
assert(isempty(marginCache));
fprintf('test_source_preserving_label_packet_v268 passed\n');
end

function object = makeObject(model, label, existence, mu)
object = model.object;
object(1).birthTime = label(1);
object(1).birthLocation = label(2);
object(1).r = existence;
object(1).numberOfGmComponents = 1;
object(1).w = 1;
object(1).mu = {mu};
object(1).Sigma = {eye(model.xDimension)};
object(1).associationEntropy = 0;
object(1).detectionAssociationEntropy = 0;
object(1).detectionAssociationMass = 0;
object(1).associationAmbiguity = 0;
object(1).associationConfidence = 1;
object(1).trajectoryLength = 0;
object(1).trajectory = zeros(model.xDimension, 0);
object(1).timestamps = zeros(1, 0);
end
