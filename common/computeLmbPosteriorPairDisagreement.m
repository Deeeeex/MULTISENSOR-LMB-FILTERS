function metrics = computeLmbPosteriorPairDisagreement( ...
    leftObjects, rightObjects, model)
% COMPUTELMBPOSTERIORPAIRDISAGREEMENT Label-wise pair dispersion.
%
% The metric is symmetric and truth-free. It combines absolute Bernoulli
% existence disagreement with covariance-normalized spatial moment
% disagreement for shared labels.

leftSummary = summarizeLmbPosteriorForDisagreement( ...
    leftObjects, model);
rightSummary = summarizeLmbPosteriorForDisagreement( ...
    rightObjects, model);
metrics = computeLmbPosteriorSummaryDisagreement( ...
    leftSummary, rightSummary);
end
