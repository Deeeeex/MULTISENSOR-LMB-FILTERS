function test_complete_label_kla_pair_diagnostics_v217()
% The diagnostic must expose the exact normalizer used by the live operator.

model = struct('xDimension', 4, 'object', makeObject(0, 1, 0.8));
model.object = model.object([]);
receiver = makeObject(0, 4, 0.8);
sameSource = makeObject(0, 4, 0.8);
same = computeCompleteLabelKlaPairDiagnosticsV217( ...
    receiver, sameSource, model, struct('sourceWeight', 0.5));
assert(same.operatorIdentityPassed);
assert(abs(same.spatialLogNormalizer) <= 1e-10);
assert(abs(same.spatialNormalizer - 1) <= 1e-10);
assert(abs(same.fusedExistence - receiver.r) <= 1e-10);
assert(same.positionSupport99Passed);
assert(abs(same.positionIsotropicSquaredDistance) <= 1e-12);
assert(~same.exactArbitraryGmKlaClaimed);

farSource = makeObject(20, 4, 0.8);
conflict = computeCompleteLabelKlaPairDiagnosticsV217( ...
    receiver, farSource, model, struct('sourceWeight', 0.5));
assert(conflict.operatorIdentityPassed);
assert(conflict.spatialLogNormalizer < -1);
assert(conflict.spatialNormalizer < 0.5);
assert(conflict.receiverLogOddsDelta < -1);
assert(conflict.fusedExistence < receiver.r);
assert(~conflict.positionSupport99Passed);
assert(conflict.positionIsotropicSquaredDistance > ...
    conflict.positionChiSquare99Threshold);

fprintf('test_complete_label_kla_pair_diagnostics_v217 passed\n');
end

function object = makeObject(positionX, variance, existence)
object = struct();
object.numberOfGmComponents = 1;
object.w = 1;
object.mu = {[positionX; 0; 0; 0]};
object.Sigma = {diag([variance, variance, 1, 1])};
object.r = existence;
object.birthTime = 1;
object.birthLocation = 1;
object.associationConfidence = 0.8;
object.detectionAssociationMass = 0.8;
end
