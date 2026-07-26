function scores = scoreStructuredFormationTreeRanker(model, X)
% SCORESTRUCTUREDFORMATIONTREERANKER Score truth-free directed edge features.

required = {'featureMean', 'featureScale', 'coefficient'};
if ~isstruct(model) || ~all(isfield(model, required))
    error('Invalid structured formation-tree ranker.');
end
if size(X, 2) ~= numel(model.featureMean)
    error('Structured formation-tree feature dimension mismatch.');
end
Z = bsxfun(@rdivide, ...
    bsxfun(@minus, X, model.featureMean), model.featureScale);
scores = Z * model.coefficient;
if isfield(model, 'intercept')
    scores = scores + model.intercept;
end
end
