function snapshot = snapshotLmbPosterior(objects, model)
% SNAPSHOTLMBPOSTERIOR Capture compact, label-sorted posterior moments.
%
% The snapshot deliberately excludes trajectories and Gaussian-mixture
% histories. Each active Bernoulli is represented only by its label,
% existence probability, projected mean, and projected covariance.

stateDimension = validateModel(model);
snapshot = struct( ...
    'labels', zeros(2, 0), ...
    'r', zeros(1, 0), ...
    'means', zeros(stateDimension, 0), ...
    'covariances', zeros(stateDimension, stateDimension, 0));
if isempty(objects)
    return;
end
if ~isstruct(objects)
    error('snapshotLmbPosterior:InvalidObjects', ...
        'objects must be an LMB struct array.');
end
requiredFields = {'birthTime', 'birthLocation', 'r', ...
    'numberOfGmComponents', 'w', 'mu', 'Sigma'};
for fieldIdx = 1:numel(requiredFields)
    if ~isfield(objects, requiredFields{fieldIdx})
        error('snapshotLmbPosterior:MissingObjectField', ...
            'objects are missing field %s.', requiredFields{fieldIdx});
    end
end

objects = objects([objects.numberOfGmComponents] > 0);
if isempty(objects)
    return;
end
labels = double([objects.birthTime; objects.birthLocation]);
if any(~isfinite(labels(:))) || any(labels(:) ~= floor(labels(:)))
    error('snapshotLmbPosterior:InvalidLabel', ...
        'LMB labels must contain finite integer values.');
end
[sortedLabels, order] = sortrows(labels', [1, 2]);
sortedLabels = sortedLabels';
if size(sortedLabels, 2) > 1 && any(all( ...
        sortedLabels(:, 1:end-1) == sortedLabels(:, 2:end), 1))
    error('snapshotLmbPosterior:DuplicateLabel', ...
        'Each posterior label must be unique.');
end

objectCount = numel(order);
existence = zeros(1, objectCount);
means = zeros(stateDimension, objectCount);
covariances = zeros(stateDimension, stateDimension, objectCount);
for sortedIdx = 1:objectCount
    object = objects(order(sortedIdx));
    if ~isnumeric(object.r) || ~isreal(object.r) || ...
            ~isscalar(object.r) || ~isfinite(object.r)
        error('snapshotLmbPosterior:InvalidExistence', ...
            'Existence probabilities must be finite real scalars.');
    end
    [projectedMean, projectedCovariance] = ...
        projectLmbObjectMoments(object, stateDimension);
    if any(~isfinite(projectedMean(:))) || ...
            any(~isfinite(projectedCovariance(:)))
        error('snapshotLmbPosterior:NonfiniteMoment', ...
            'Projected posterior moments must be finite.');
    end
    existence(sortedIdx) = double(object.r);
    means(:, sortedIdx) = reshape(projectedMean, stateDimension, 1);
    covariances(:, :, sortedIdx) = projectedCovariance;
end

snapshot.labels = sortedLabels;
snapshot.r = existence;
snapshot.means = means;
snapshot.covariances = covariances;
end

function stateDimension = validateModel(model)
if ~isstruct(model) || ~isfield(model, 'xDimension') || ...
        ~isnumeric(model.xDimension) || ~isreal(model.xDimension) || ...
        ~isscalar(model.xDimension) || ~isfinite(model.xDimension) || ...
        model.xDimension < 1 || model.xDimension ~= floor(model.xDimension)
    error('snapshotLmbPosterior:InvalidModel', ...
        'model.xDimension must be a positive integer.');
end
stateDimension = double(model.xDimension);
end
