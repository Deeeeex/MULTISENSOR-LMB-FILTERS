function calibration = calibrateCvarResidualGraphSafetyMargin( ...
        model, calibrationBlocks, protocol)
% CALIBRATECVARRESIDUALGRAPHSAFETYMARGIN Whole-seed safety calibration.

if nargin < 3 || isempty(protocol)
    protocol = getCvarResidualGraphPolicyDatasetProtocol();
end
if isempty(calibrationBlocks) || ...
        numel(unique([calibrationBlocks.seed])) ~= 1
    error(['CVaR graph safety calibration requires exactly one ', ...
        'whole calibration seed.']);
end
blockQuantiles = nan(1, numel(calibrationBlocks));
observedCounts = zeros(1, numel(calibrationBlocks));
for blockIdx = 1:numel(calibrationBlocks)
    block = calibrationBlocks(blockIdx);
    scores = scoreCvarResidualGraphPolicyModel( ...
        model, block);
    observed = block.edgeProtectionObservedMask;
    residual = scores.edgeProtectionAdvantage(observed) - ...
        block.edgeProtectionAdvantageTarget(observed);
    if isempty(residual) || any(~isfinite(residual))
        error('CVaR graph safety calibration residual is invalid.');
    end
    blockQuantiles(blockIdx) = empiricalQuantile( ...
        residual, protocol.modelSafetyResidualQuantile);
    observedCounts(blockIdx) = numel(residual);
end
margin = empiricalQuantile(blockQuantiles, ...
    protocol.modelSafetyResidualQuantile);
calibration = struct();
calibration.seed = calibrationBlocks(1).seed;
calibration.quantile = ...
    protocol.modelSafetyResidualQuantile;
calibration.blockQuantiles = blockQuantiles;
calibration.observedCounts = observedCounts;
calibration.margin = max(0, margin);
calibration.maximumBlockQuantile = ...
    max(blockQuantiles);
calibration.truthUsed = true;
calibration.futureOutcomeUsed = false;
calibration.deployableCalibration = true;
end

function value = empiricalQuantile(values, probability)
values = sort(reshape(values, 1, []));
if isempty(values) || probability < 0 || probability > 1
    error('Empirical quantile inputs are invalid.');
end
position = 1 + (numel(values) - 1) * probability;
lowerIdx = floor(position);
upperIdx = ceil(position);
if lowerIdx == upperIdx
    value = values(lowerIdx);
else
    fraction = position - lowerIdx;
    value = (1 - fraction) * values(lowerIdx) + ...
        fraction * values(upperIdx);
end
end
