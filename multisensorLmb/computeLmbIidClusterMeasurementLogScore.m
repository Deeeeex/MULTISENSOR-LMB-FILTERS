function [logScore, details] = ...
    computeLmbIidClusterMeasurementLogScore( ...
        predictedObjects, measurements, model, sensorIdx, currentTime)
% COMPUTELMBIIDCLUSTERMEASUREMENTLOGSCORE Cardinality-aware set score.
%
% The LMB measurement process has a Poisson-binomial target-detection
% cardinality convolved with Poisson clutter. Its first moment is the
% predicted measurement intensity. This routine combines the exact
% cardinality law with the normalized intensity to form the IID-cluster
% projection
%
%   f_IID(Z) = |Z|! rho(|Z|) prod_{z in Z} lambda(z) / Lambda,
%
% where rho is the measurement-cardinality PMF and Lambda is the expected
% measurement count. The resulting logarithmic score is proper for this
% cardinality-and-intensity projection and uses no target truth. It is not
% the exact LMB measurement-set likelihood because label-to-measurement
% association dependence is projected out.

if nargin < 5 || isempty(currentTime)
    currentTime = 1;
end
if nargin < 4 || isempty(sensorIdx)
    sensorIdx = 1;
end
if isempty(measurements)
    measurements = cell(0, 1);
end

[~, intensityDetails] = ...
    computeLmbPoissonMeasurementLogScore( ...
        predictedObjects, measurements, model, ...
        sensorIdx, currentTime);
measurementCount = intensityDetails.measurementCount;
targetDetectionProbabilitySum = sum( ...
    intensityDetails.targetDetectionProbabilities);
consistencyTolerance = 1e-10 * max( ...
    1, intensityDetails.targetExpectedCount);
if abs(targetDetectionProbabilitySum - ...
        intensityDetails.targetExpectedCount) > ...
        consistencyTolerance
    error(['IID-cluster score requires normalized LMB mixture ', ...
        'weights and consistent detection mass.']);
end
targetCardinalityPmf = poissonBinomialPmf( ...
    intensityDetails.targetDetectionProbabilities);
clutterRate = intensityDetails.clutterExpectedCount;
clutterCardinalityPmf = poissonPmfThroughCount( ...
    clutterRate, measurementCount);

maximumTargetDetections = min( ...
    measurementCount, numel(targetCardinalityPmf) - 1);
cardinalityProbability = 0;
for targetCount = 0:maximumTargetDetections
    clutterCount = measurementCount - targetCount;
    cardinalityProbability = cardinalityProbability + ...
        targetCardinalityPmf(targetCount + 1) * ...
        clutterCardinalityPmf(clutterCount + 1);
end

totalExpectedCount = intensityDetails.totalExpectedCount;
if totalExpectedCount <= 0
    if measurementCount == 0
        spatialLogScore = 0;
    else
        spatialLogScore = -Inf;
    end
else
    spatialLogScore = sum(log(max( ...
        intensityDetails.measurementIntensity, realmin))) - ...
        measurementCount * log(totalExpectedCount);
end
cardinalityLogScore = log(max( ...
    cardinalityProbability, realmin));
setSymmetryLogFactor = gammaln(measurementCount + 1);
logScore = cardinalityLogScore + ...
    setSymmetryLogFactor + spatialLogScore;

details = intensityDetails;
details.scoreType = ...
    'lmb-cardinality-matched-iid-cluster-measurement-log-score';
details.truthUsed = false;
details.iidClusterProjection = true;
details.exactMeasurementCardinality = true;
details.exactLmbMeasurementSetLikelihood = false;
details.cardinalityModel = ...
    'poisson-binomial-target-detections-plus-poisson-clutter';
details.spatialModel = ...
    'normalized-predicted-measurement-intensity';
details.targetDetectionCardinalityPmf = ...
    targetCardinalityPmf;
details.cardinalityProbability = ...
    cardinalityProbability;
details.cardinalityLogScore = cardinalityLogScore;
details.spatialLogScore = spatialLogScore;
details.setSymmetryLogFactor = ...
    setSymmetryLogFactor;
end

function pmf = poissonBinomialPmf(probabilities)
probabilities = reshape(probabilities, 1, []);
probabilities = min(max(probabilities, 0), 1);
pmf = 1;
for probability = probabilities
    previous = pmf;
    pmf = [ ...
        previous * (1 - probability), 0] + ...
        [0, previous * probability];
end
pmf = pmf / max(sum(pmf), realmin);
end

function pmf = poissonPmfThroughCount(rate, maximumCount)
rate = max(rate, 0);
pmf = zeros(1, maximumCount + 1);
pmf(1) = exp(-rate);
for count = 1:maximumCount
    pmf(count + 1) = ...
        pmf(count) * rate / count;
end
end
