function test_receiver_domain_transport_projection_v71()
% TEST_RECEIVERDOMAINTRANSPORTPROJECTIONV71 Fixed-message composition.

[context, control, transport, local, groupIds] = syntheticState();
projection = buildReceiverDomainTransportProjectionV71( ...
    context, control, transport, local, groupIds);

assert(strcmp(projection.contractVersion, ...
    'receiver-domain-transport-projection-v71-v1'));
assert(isequal(projection.nominatedFormationIds, [1, 2]));
assert(projection.candidateCount == 3);
assert(projection.feasibleCandidateCount == 3);
assert(~projection.referenceFallbackUsed);
assert(isequal(projection.selectedFormationIds, [1, 2]));
assert(isequal(projection.selectedSlotIndices, [1, 2]));
assert(abs(projection.selectedUtilityMass - 0.35) < 1e-12);
assert(projection.messageCountParityWithReference);
assert(projection.perReceiverMessageCountParityWithReference);
assert(projection.perReceiverWeightMultisetParityWithReference);
assert(all(projection.selectedRollingB3SensorPass));
assert(all(projection.selectedRollingB3FormationPass));
assert(projection.routeConstructed && ~projection.routeExecuted);
assert(~projection.trackingOutcomeRead && ~projection.truthUsed);
assert(projection.selectedAdjacency(1, 4));
assert(~projection.selectedAdjacency(1, 3));
assert(projection.selectedAdjacency(3, 2));
assert(~projection.selectedAdjacency(3, 1));
assert(abs(projection.selectedFusionWeights(1, 4) - 0.05) < 1e-12);
assert(abs(projection.selectedFusionWeights(3, 2) - 0.05) < 1e-12);

local.alternativeTransportActionMask(:) = false;
local.preferredActionByFormation(:) = {'fallback'};
fallback = buildReceiverDomainTransportProjectionV71( ...
    context, control, transport, local, groupIds);
assert(fallback.referenceFallbackUsed);
assert(fallback.candidateCount == 0);
assert(isequal(fallback.selectedAdjacency, control.referenceAdjacency));

fprintf('test_receiver_domain_transport_projection_v71 passed\n');
end

function [context, control, transport, local, groupIds] = syntheticState()
groupIds = [1, 1, 2, 2];
nodeCount = numel(groupIds);
referenceAdjacency = false(nodeCount);
referenceAdjacency(1, [2, 3]) = true;
referenceAdjacency(2, [1, 4]) = true;
referenceAdjacency(3, [4, 1]) = true;
referenceAdjacency(4, [3, 2]) = true;
referenceWeights = zeros(nodeCount);
for receiverIdx = 1:nodeCount
    senders = find(referenceAdjacency(receiverIdx, :));
    localSender = senders(groupIds(senders) == groupIds(receiverIdx));
    crossSender = senders(groupIds(senders) ~= groupIds(receiverIdx));
    referenceWeights(receiverIdx, receiverIdx) = 0.25;
    referenceWeights(receiverIdx, localSender) = 0.70;
    referenceWeights(receiverIdx, crossSender) = 0.05;
end

context = struct();
context.physicalAdjacency = logical(ones(nodeCount) - eye(nodeCount));
context.previousAdjacencyHistory = repmat( ...
    referenceAdjacency, 1, 1, 2);
control = struct( ...
    'referenceAdjacency', referenceAdjacency, ...
    'referenceFusionWeights', referenceWeights);

networkMass = 20;
slots = repmat(emptySlot(), 1, 2);
slots(1) = makeSlot(1, 3, 4, 0.20 / networkMass);
slots(2) = makeSlot(3, 1, 2, 0.15 / networkMass);
transport = struct( ...
    'networkReferenceExistenceMass', networkMass, ...
    'slotRecords', slots);

local = struct();
local.groups = [1, 2];
local.groupIds = groupIds;
local.referenceExistenceMassByFormation = [10, 10];
local.alternativeTransportNetMassByFormation = [0.20, 0.15];
local.localAlternativeTransportNetFractionByFormation = [0.02, 0.015];
local.alternativePositiveSlotCountByFormation = [1, 1];
local.alternativeTransportActionMask = [true, true];
local.preferredActionByFormation = {'transport', 'transport'};
end

function slot = makeSlot(receiver, incumbent, candidate, net)
slot = emptySlot();
slot.receiverIdx = receiver;
slot.incumbentSenderIdx = incumbent;
slot.sourceWeight = 0.05;
slot.bestSafe = true;
slot.bestCandidateSenderIdx = candidate;
slot.bestLinkReliability = 1;
slot.bestTransportGainFraction = net;
slot.bestSupportedHarmFraction = 0;
slot.bestNetHeadroomFraction = net;
slot.bestUpwardCrossingCount = 1;
slot.bestDownwardCrossingCount = 0;
end

function slot = emptySlot()
slot = struct( ...
    'receiverIdx', NaN, ...
    'incumbentSenderIdx', NaN, ...
    'sourceWeight', NaN, ...
    'bestSafe', false, ...
    'bestCandidateSenderIdx', NaN, ...
    'bestLinkReliability', NaN, ...
    'bestTransportGainFraction', NaN, ...
    'bestSupportedHarmFraction', NaN, ...
    'bestNetHeadroomFraction', NaN, ...
    'bestUpwardCrossingCount', NaN, ...
    'bestDownwardCrossingCount', NaN);
end
