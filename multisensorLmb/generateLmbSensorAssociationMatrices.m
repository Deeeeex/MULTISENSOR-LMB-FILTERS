function [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, z, model, s, currentTime)
% GENERATELMBSENSORASSOCIATIONMATRICES -- Compute the association matrices required for
%    [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, z, model, s)
%    [associationMatrices, posteriorParameters] = generateLmbSensorAssociationMatrices(objects, z, model, s, currentTime)
%
%   This function computes the association matrices required by the LBP,
%   Gibbs sampler, and Murty's algorithms for a given sensor. It also 
%   determines the measurement-updated components requried for each sensor's measurement update.
%   Supports mobile sensors with position-dependent measurements.
%
%   See also runLmbFilter, generateMultisensorModel, loopyBeliefPropagation, lmbGibbsSampling, lmbMurtysAlgorithm
%
%   Inputs
%       objects - struct. A struct containing the prior LMB's Bernoulli components.
%       z - cell array. A cell array of measurements for the
%           current time-step.
%       model - struct. A struct with the fields declared in generateMultisensorModel.
%       s - integer. The sensor number.
%       currentTime - integer. The current time step (optional, for mobile sensors).
%
%   Output
%       associationMatrices - struct. A struct whose fields are the arrays required 
%           by the various data association algorithms.
%       posteriorParameters - struct. A struct whose fields are an object's
%           posterior spatial distribution parameters.
%
%   This function computes the association matrices required by the LBP,
%   Gibbs sampler, and Murty's algorithms for a given sensor. It also 
%   determines the measurement-updated components requried for each sensor's measurement update.
%
%   See also runLmbFilter, generateMultisensorModel, loopyBeliefPropagation, lmbGibbsSampling, lmbMurtysAlgorithm
%
%   Inputs
%       objects - struct. A struct containing the prior LMB's Bernoulli components.
%       z - cell array. A cell array of measurements for the
%           current time-step.
%       model - struct. A struct with the fields declared in generateMultisensorModel.
%       s - integer. The sensor number.
%
%   Output
%       associationMatrices - struct. A struct whose fields are the arrays required 
%           by the various data association algorithms.
%       posteriorParameters - struct. A struct whose fields are an object's
%           posterior spatial distribution parameters.
%

%% Declare output structs
numberOfObjects = numel(objects);
numberOfMeasurements = numel(z);
% Auxillary matrices
L = zeros(numberOfObjects, numberOfMeasurements);
phi = zeros(numberOfObjects, 1);
eta = zeros(numberOfObjects, 1);
nisMin = inf(1, numberOfMeasurements);
% Updated components for the objects' posterior spatial distributions
posteriorParameters.w = [];
posteriorParameters.mu = {};
posteriorParameters.Sigma = {};
posteriorParameters = repmat(posteriorParameters, 1, numberOfObjects);
%% Populate the LBP arrays, and compute posterior components
for i = 1:numberOfObjects
    % Predeclare the object's posterior components, and include missed detection event
    posteriorParameters(i).w = repmat(log(objects(i).w * (1 - model.detectionProbability(s))), numberOfMeasurements + 1, 1);
    posteriorParameters(i).mu  = repmat(objects(i).mu, numberOfMeasurements + 1, 1);
    posteriorParameters(i).Sigma = repmat(objects(i).Sigma, numberOfMeasurements + 1, 1);
    % Populate auxiliary LBP parameters
    phi(i) = (1 -  model.detectionProbability(s)) * objects(i).r;
    eta(i) = 1 - model.detectionProbability(s) * objects(i).r;
    %% Determine marginal likelihood ratio of the object generating each measurement
    for j = 1:objects(i).numberOfGmComponents
        % Update components for a mixture component
        % Phase 1: Support for mobile sensors
        if model.sensorMotionEnabled && nargin >= 4
            sensorPos = model.sensorTrajectories{s}(1:2, currentTime);
            targetPos = objects(i).mu{j}(1:2);
            muZ = sensorPos + model.C{s} * [targetPos - sensorPos; 0; 0];
        else
            muZ = model.C{s} * objects(i).mu{j};
        end
        
        Z = model.C{s} * objects(i).Sigma{j} * model.C{s}' + model.Q{s};
        logGaussianNormalisingConstant = - (0.5 * model.zDimension) * log(2 * pi) - 0.5 * log(det(Z));
        logLikelihoodRatioTerms = log(objects(i).r) + log(model.detectionProbability(s)) + log(objects(i).w(j)) - log(model.clutterPerUnitVolume(s));
        projectionCov = objects(i).Sigma{j} * model.C{s}';
        K = projectionCov / Z;
        SigmaUpdated = (eye(model.xDimension) - K * model.C{s}) * objects(i).Sigma{j};
        % Determine total marginal likelihood, and determine posterior components
        for k = 1:numberOfMeasurements
            % Determine marginal likelihood ratio
            nu = z{k} - muZ;
            nisValue = nu' * (Z \ nu);
            gaussianLogLikelihood = logGaussianNormalisingConstant - 0.5 * nisValue;
            L(i, k) = L(i, k) + exp(logLikelihoodRatioTerms + gaussianLogLikelihood);
            if nisValue < nisMin(k)
                nisMin(k) = nisValue;
            end
            % Determine updated mean and covariance for each mixture component
            posteriorParameters(i).w(k+1, j) = log(objects(i).w(j)) + gaussianLogLikelihood + log(model.detectionProbability(s)) - log(model.clutterPerUnitVolume(s));
            posteriorParameters(i).mu{k+1, j} = objects(i).mu{j} + K * nu;
            posteriorParameters(i).Sigma{k+1, j} = SigmaUpdated;
        end
    end
    % Normalise weights
    maximumWeights = max(posteriorParameters(i).w, [], 2);
    offsetWeights = posteriorParameters(i).w - maximumWeights;
    posteriorParameters(i).w = exp(offsetWeights) ./ sum(exp(offsetWeights), 2);
end
%% Output association matrices
associationMatrices.r = reshape([objects.r], numberOfObjects, 1);
% LBP association matrices
associationMatrices.Psi = L ./ eta;
associationMatrices.phi = phi;
associationMatrices.eta = eta;
% Gibbs sampler association matrices
associationMatrices.P = L./ (L + eta);
associationMatrices.L = [eta L];
associationMatrices.R = [(phi ./ eta) ones(numberOfObjects, numberOfMeasurements)];
% Murty's algorithm association matrices
associationMatrices.C = -log(L);
% Innovation consistency (NIS-based)
if numberOfMeasurements > 0
    cfg = struct();
    if isfield(model, 'adaptiveFusion') && isstruct(model.adaptiveFusion)
        cfg = model.adaptiveFusion;
    end
    validNis = nisMin(isfinite(nisMin));
    if isempty(validNis)
        associationMatrices.innovationScore = 1;
        associationMatrices.innovationPenalty = 1;
        associationMatrices.nisAgg = NaN;
        associationMatrices.nisNorm = NaN;
        associationMatrices.nisLowerBound = NaN;
        associationMatrices.nisUpperBound = NaN;
        associationMatrices.nisDeviation = NaN;
        return;
    else
        useRobust = false;
        if isfield(cfg, 'robustNIS') && cfg.robustNIS && ...
                isfield(model, 'lmbParallelUpdateMode') && strcmpi(model.lmbParallelUpdateMode, 'GA')
            useRobust = true;
        end
        useQuantile = getConfigField(cfg, 'nisQuantileEnabled', true);
        quantileValue = getConfigField(cfg, 'nisQuantile', 0.7);
        if useRobust && useQuantile
            quantileValue = min(max(quantileValue, 0), 1);
            nisAgg = computeQuantile1d(validNis, quantileValue);
        elseif useRobust
            nisAgg = median(validNis);
        else
            nisAgg = mean(validNis);
        end
    end
    dof = max(1, model.zDimension);
    nisNorm = nisAgg / dof;
    confidenceLevel = getConfigField(cfg, 'nisConsistencyConfidence', 0.5);
    confidenceLevel = min(max(confidenceLevel, 1e-3), 0.999);
    lowerTail = 0.5 * (1 - confidenceLevel);
    upperTail = 1 - lowerTail;
    lowerBound = computeChiSquareQuantile(lowerTail, dof) / dof;
    upperBound = computeChiSquareQuantile(upperTail, dof) / dof;
    [lowerDeviation, upperDeviation] = computeLogIntervalDeviation(nisNorm, lowerBound, upperBound);
    lowerScale = getConfigField(cfg, 'nisPenaltyLowerScale', ...
        0.25 * getConfigField(cfg, 'nisPenaltyScale', 4.0));
    upperScale = getConfigField(cfg, 'nisPenaltyUpperScale', ...
        getConfigField(cfg, 'nisPenaltyScale', 4.0));
    lowerPower = getConfigField(cfg, 'nisPenaltyLowerPower', 2.0);
    upperPower = getConfigField(cfg, 'nisPenaltyUpperPower', 2.0);
    penaltyExponent = lowerScale * lowerDeviation^lowerPower + ...
        upperScale * upperDeviation^upperPower;
    penalty = exp(-penaltyExponent);
    minPenalty = 0.0;
    maxPenalty = getConfigField(cfg, 'nisPenaltyMax', 1.0);
    if useRobust
        minPenalty = getConfigField(cfg, 'nisPenaltyMin', ...
            getConfigField(cfg, 'robustNISMin', 0.2));
    end
    penalty = min(max(penalty, minPenalty), maxPenalty);
    associationMatrices.innovationScore = penalty;
    associationMatrices.innovationPenalty = penalty;
    associationMatrices.nisAgg = nisAgg;
    associationMatrices.nisNorm = nisNorm;
    associationMatrices.nisLowerBound = lowerBound;
    associationMatrices.nisUpperBound = upperBound;
    associationMatrices.nisDeviation = lowerDeviation + upperDeviation;
    associationMatrices.nisLowerDeviation = lowerDeviation;
    associationMatrices.nisUpperDeviation = upperDeviation;
else
    associationMatrices.innovationScore = 1;
    associationMatrices.innovationPenalty = 1;
end
end

function value = getConfigField(cfg, fieldName, defaultValue)
if isfield(cfg, fieldName)
    value = cfg.(fieldName);
else
    value = defaultValue;
end
end

function [lowerDeviation, upperDeviation] = computeLogIntervalDeviation(value, lowerBound, upperBound)
value = max(value, eps);
lowerBound = max(lowerBound, eps);
upperBound = max(upperBound, lowerBound + eps);
lowerDeviation = 0;
upperDeviation = 0;
if value < lowerBound
    lowerDeviation = log(lowerBound / value);
elseif value > upperBound
    upperDeviation = log(value / upperBound);
end
end

function value = computeChiSquareQuantile(p, dof)
p = min(max(p, 1e-12), 1 - 1e-12);
value = 2 * gammaincinv(p, dof / 2);
end

function value = computeQuantile1d(values, q)
sortedValues = sort(values(:));
numValues = numel(sortedValues);
if numValues == 0
    value = 0;
    return;
end
if numValues == 1
    value = sortedValues(1);
    return;
end
index = 1 + (numValues - 1) * q;
lowerIndex = floor(index);
upperIndex = ceil(index);
if lowerIndex == upperIndex
    value = sortedValues(lowerIndex);
else
    fraction = index - lowerIndex;
    value = sortedValues(lowerIndex) + fraction * ...
        (sortedValues(upperIndex) - sortedValues(lowerIndex));
end
end
