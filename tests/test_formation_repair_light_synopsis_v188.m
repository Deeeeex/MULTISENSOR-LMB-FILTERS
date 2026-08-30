function test_formation_repair_light_synopsis_v188()
% TEST_FORMATIONREPAIRLIGHTSYNOPSISV188 Wire layout and feature boundary.

inputs = generateDynamicTopologyScenarioInputs( ...
    'm24-formation-fov', 1601);
model = inputs.model;
sensorCount = model.numberOfSensors;
groupIds = reshape(model.dynamicTopologyScenario. ...
    config.sensorGroupIds, 1, []);
object = simpleObject();
local = cell(1, sensorCount);
fused = cell(1, sensorCount);
for sensorIdx = 1:sensorCount
    local{sensorIdx} = object;
    fused{sensorIdx} = object;
end
physical = logical(inputs.graphData.physicalAdjacency(:, :, 1));
physical = physical | physical';
physical(1:sensorCount+1:end) = false;

cache = buildFormationRepairLightSynopsisCacheV188( ...
    fused, local, model, 1, struct());
protocol = getBudgetRecycledFormationRepairV188Protocol();
assert(cache.bytesPerLabelRecord == 24);
assert(~cache.fullCovarianceIncluded);
assert(~cache.gmComponentsIncluded);
assert(cache.totalAttemptedBytes == sensorCount * ( ...
    protocol.lightSynopsisHeaderBytes + ...
    2 * protocol.lightSynopsisBytesPerLabel));

[values, names, details] = buildFormationRepairValueFeaturesV188( ...
    fused, local, physical, groupIds, model, 1, struct(), ...
    struct('lightSynopsisCache', cache));
assert(size(values, 1) == numel(unique(groupIds)));
assert(size(values, 2) == 17);
assert(numel(names) == 17);
assert(all(isfinite(values(:))));
assert(details.lightSynopsisAttemptedBytes == ...
    cache.totalAttemptedBytes);
assert(~details.truthUsed);
assert(~details.futureInformationUsed);
assert(~details.fullCovarianceUsedByValueFeatures);
assert(~details.gmComponentsUsedByValueFeatures);

fprintf('test_formation_repair_light_synopsis_v188 passed\n');
end

function object = simpleObject()
object = struct();
object.numberOfGmComponents = 1;
object.w = 1;
object.mu = {[0; 0; 0; 0]};
object.Sigma = {diag([25, 25, 4, 4])};
object.r = 0.8;
object.birthTime = 1;
object.birthLocation = 1;
object.associationConfidence = 0.75;
object.detectionAssociationMass = 0.65;
end
