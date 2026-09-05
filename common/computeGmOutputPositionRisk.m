function [risk, parts] = computeGmOutputPositionRisk(object, outputPosition)
% Posterior expected squared position loss of a specified output, not truth risk.
% The output may be a selected GM component, rather than the mixture mean.
% No filtering, existence change, component reduction, or source selection.
count = object.numberOfGmComponents;
weights = reshape(object.w, 1, []);
outputPosition = reshape(outputPosition, [], 1);
assert(count > 0 && count == numel(weights));
assert(count == numel(object.mu) && count == numel(object.Sigma));
assert(numel(outputPosition) == 2 && all(isfinite(outputPosition)));
assert(all(isfinite(weights)) && all(weights >= 0) && sum(weights) > 0);
weights = weights / sum(weights);
means = zeros(2, count); traces = zeros(1, count);
for c = 1:count
    means(:, c) = object.mu{c}(1:2);
    covariance = object.Sigma{c}(1:2, 1:2);
    assert(all(isfinite(covariance(:))) && all(isfinite(means(:, c))));
    traces(c) = trace(covariance);
end
mixtureMean = means * weights';
within = sum(weights .* traces);
between = sum(weights .* sum(bsxfun(@minus, means, mixtureMean).^2, 1));
outputOffset = sum((mixtureMean - outputPosition).^2);
risk = sum(weights .* (traces + sum(bsxfun(@minus, means, outputPosition).^2, 1)));
residual = abs(risk - within - between - outputOffset);
assert(isfinite(risk) && risk >= 0 && residual <= 1e-10 * max(1, risk));
parts = struct('withinComponent', within, 'betweenComponents', between, ...
    'outputMeanOffset', outputOffset, 'mixtureMean', mixtureMean, ...
    'decompositionResidual', residual);
end
