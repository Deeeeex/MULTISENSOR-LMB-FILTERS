function [regularizedCovariance, jitterAdded, choleskyFactor] = ...
    regularizeCovarianceForSolve(covariance)
% REGULARIZECOVARIANCEFORSOLVE Add jitter only when Cholesky requires it.

if ~isnumeric(covariance) || ~isreal(covariance) || ...
        isempty(covariance) || ndims(covariance) ~= 2 || ...
        size(covariance, 1) ~= size(covariance, 2) || ...
        any(~isfinite(covariance(:)))
    error('regularizeCovarianceForSolve:InvalidCovariance', ...
        'Covariance must be a nonempty finite real square matrix.');
end

covariance = double(covariance);
regularizedCovariance = (covariance + covariance') / 2;
jitterAdded = 0;
[choleskyFactor, cholFlag] = chol(regularizedCovariance);
if cholFlag == 0
    return;
end

diagonalScale = max(abs(diag(regularizedCovariance)));
jitter = 1e-12 * max(1, diagonalScale);
identity = eye(size(regularizedCovariance));
for attemptIdx = 1:16
    candidate = regularizedCovariance + jitter * identity;
    if all(isfinite(candidate(:)))
        [candidateFactor, cholFlag] = chol(candidate);
        if cholFlag == 0
            regularizedCovariance = candidate;
            jitterAdded = jitter;
            choleskyFactor = candidateFactor;
            return;
        end
    end
    jitter = 10 * jitter;
end

error('regularizeCovarianceForSolve:CholeskyFailure', ...
    'Covariance remained non-SPD after 16 fixed jitter attempts.');
end
