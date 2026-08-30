function test_formation_common_label_kla_repair_v190()
% A residual KLA action must preserve both receiver and source evidence.

receiver = makeObject(0, 9, 0.8);
source = makeObject(6, 4, 0.9);
posterior = {receiver, receiver, source};
proposal = struct( ...
    'formationId', 1, ...
    'receiverIds', [1, 2], ...
    'sourceId', 3, ...
    'label', [1; 1], ...
    'sourceObject', source, ...
    'safetyPassed', true, ...
    'rollingConnectivityPassed', true);
model = struct( ...
    'xDimension', 4, ...
    'object', repmat(receiver, 1, 0), ...
    'existenceThreshold', 0.01);

[updated, details] = applyFormationCommonLabelKlaRepairV190( ...
    posterior, proposal, 1, model, struct('sourceWeight', 0.5));
assert(details.appliedActionCount == 1);
assert(details.appliedReceiverCount == 2);
assert(~details.hardReplacementApplied);
assert(~details.truthUsed);
receiverMean = mixtureMean(receiver);
sourceMean = mixtureMean(source);
for receiverIdx = 1:2
    fused = updated{receiverIdx};
    assert(fused.birthTime == 1 && fused.birthLocation == 1);
    assert(fused.numberOfGmComponents >= 1);
    fusedMean = mixtureMean(fused);
    assert(fusedMean(1) > receiverMean(1));
    assert(fusedMean(1) < sourceMean(1));
end
assert(isequal(updated{3}, source));

fprintf('test_formation_common_label_kla_repair_v190 passed\n');
end

function object = makeObject(positionX, variance, existence)
object = struct();
object.numberOfGmComponents = 2;
object.w = [0.6, 0.4];
object.mu = { ...
    [positionX; 0; 0; 0], ...
    [positionX + 1; 0; 0; 0]};
object.Sigma = { ...
    diag([variance, variance, 1, 1]), ...
    diag([variance + 1, variance + 1, 1, 1])};
object.r = existence;
object.birthTime = 1;
object.birthLocation = 1;
object.associationConfidence = 0.8;
object.detectionAssociationMass = 0.8;
end

function value = mixtureMean(object)
value = zeros(numel(object.mu{1}), 1);
for componentIdx = 1:object.numberOfGmComponents
    value = value + object.w(componentIdx) * object.mu{componentIdx};
end
end
