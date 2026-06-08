function test_rigorous_ablation_arms()
% TEST_RIGOROUS_ABLATION_ARMS Verify the causal component-ablation matrix.

arms = buildRigorousAblationArms(struct());
assert(numel(arms) == 11);

expectedNames = { ...
    'Fixed Metropolis', ...
    'Cov only', ...
    'Link only', ...
    'Exist only', ...
    'Cov + Link', ...
    'Cov + Link + Exist, shared weights', ...
    'Cov + Link + Exist, branch-decoupled', ...
    '+ structure-aware', ...
    '+ EMA/floor', ...
    '+ FID-FIA existence only', ...
    'Balanced + FID-FIA existence'};
assert(isequal({arms.name}, expectedNames));

assert(~arms(1).adaptiveFusion.enabled);
assertFactorMask(arms(2).adaptiveFusion, true, false, false);
assertFactorMask(arms(3).adaptiveFusion, false, true, false);
assertFactorMask(arms(4).adaptiveFusion, false, false, true);
assertFactorMask(arms(5).adaptiveFusion, true, true, false);
assertFactorMask(arms(6).adaptiveFusion, true, true, true);

assert(~arms(6).adaptiveFusion.useDecoupledKla);
assert(arms(7).adaptiveFusion.useDecoupledKla);
assert(~arms(7).adaptiveFusion.useStructureAwareKla);
assert(arms(8).adaptiveFusion.useStructureAwareKla);

assertOnlyFieldsDiffer(arms(5).adaptiveFusion, arms(6).adaptiveFusion, ...
    {'useExistenceConfidence'});
assertOnlyFieldsDiffer(arms(6).adaptiveFusion, arms(7).adaptiveFusion, ...
    {'useDecoupledKla', 'spatialDecouplingStrength', 'existenceDecouplingStrength'});
assertOnlyFieldsDiffer(arms(7).adaptiveFusion, arms(8).adaptiveFusion, ...
    {'useStructureAwareKla', 'spatialStructureStrength', ...
    'existenceStructureStrength', 'structureReliabilityPower', ...
    'structureReliabilityMinScore'});

for armIdx = 2:8
    assertNoStabilization(arms(armIdx).adaptiveFusion);
end

stableCfg = arms(9).adaptiveFusion;
assert(stableCfg.emaAlpha == 0.7);
assert(stableCfg.minWeight == 0.05);
assert(stableCfg.spatialEmaAlpha == 0.7);
assert(stableCfg.existenceEmaAlpha == 0.7);
assert(stableCfg.spatialMinWeight == 0.05);
assert(stableCfg.existenceMinWeight == 0.05);
assertOnlyFieldsDiffer(arms(8).adaptiveFusion, stableCfg, ...
    {'emaAlpha', 'minWeight', 'spatialEmaAlpha', 'existenceEmaAlpha', ...
    'spatialMinWeight', 'existenceMinWeight'});

fidCfg = arms(10).adaptiveFusion;
assert(fidCfg.useFidFiaExistence);
assert(fidCfg.existenceEmaAlpha == stableCfg.existenceEmaAlpha);
assert(fidCfg.existenceMinWeight == stableCfg.existenceMinWeight);
assertOnlyFieldsDiffer(stableCfg, fidCfg, ...
    {'useFidFiaExistence', 'fidFiaExistenceStrength', ...
    'fidFiaExistenceMinScore', 'fidFiaExistencePower', ...
    'fidFiaQuadraturePoints', 'fidFiaUseDetectionProbability', ...
    'fidFiaUseExistenceWeight'});

deployedCfg = arms(11).adaptiveFusion;
assert(deployedCfg.useFidFiaExistence);
assertNoStabilization(deployedCfg);
assertOnlyFieldsDiffer(arms(8).adaptiveFusion, deployedCfg, ...
    {'useFidFiaExistence', 'fidFiaExistenceStrength', ...
    'fidFiaExistenceMinScore', 'fidFiaExistencePower', ...
    'fidFiaQuadraturePoints', 'fidFiaUseDetectionProbability', ...
    'fidFiaUseExistenceWeight'});

fprintf('test_rigorous_ablation_arms passed\n');
end

function assertFactorMask(cfg, useCovariance, useLinkQuality, useExistenceConfidence)
assert(cfg.useCovariance == useCovariance);
assert(cfg.useLinkQuality == useLinkQuality);
assert(cfg.useExistenceConfidence == useExistenceConfidence);
end

function assertNoStabilization(cfg)
assert(cfg.emaAlpha == 0.0);
assert(cfg.minWeight == 0.0);
assert(cfg.spatialEmaAlpha == 0.0);
assert(cfg.existenceEmaAlpha == 0.0);
assert(cfg.spatialMinWeight == 0.0);
assert(cfg.existenceMinWeight == 0.0);
end

function assertOnlyFieldsDiffer(left, right, allowedFields)
allFields = unique([fieldnames(left); fieldnames(right)]);
for fieldIdx = 1:numel(allFields)
    fieldName = allFields{fieldIdx};
    leftHasField = isfield(left, fieldName);
    rightHasField = isfield(right, fieldName);
    valuesEqual = leftHasField && rightHasField && ...
        isequaln(left.(fieldName), right.(fieldName));
    if ~valuesEqual
        assert(any(strcmp(fieldName, allowedFields)), ...
            'Unexpected configuration change in field: %s', fieldName);
    end
end
end
