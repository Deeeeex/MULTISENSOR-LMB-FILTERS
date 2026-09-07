function distance = computeGospaBetweenEstimates(estA, estB, t, model)
% COMPUTEGOSPABETWEENESTIMATES GOSPA on the same states as consensus OSPA.
% Existing Euclidean OSPA consensus evaluates norm(muA-muB) on complete
% extracted state vectors. This wrapper deliberately preserves that base
% space. Using position only would not be the non-normalized counterpart of
% the reported OSPA consensus metric.

XA = stateCellsToMatrix(estA.mu{t}, model.xDimension);
XB = stateCellsToMatrix(estB.mu{t}, model.xDimension);
if isfield(model, 'gospaParameters')
    params = model.gospaParameters;
else
    params = struct('c', model.ospaParameters.eC, ...
        'p', model.ospaParameters.eP, 'alpha', 2);
end
distance = gospa(XA, XB, params.c, params.p, params.alpha);
end

function X = stateCellsToMatrix(states, stateDimension)
if isempty(states)
    X = zeros(stateDimension, 0);
    return;
end
X = cell2mat(cellfun(@(state) reshape(state, [], 1), states, ...
    'UniformOutput', false));
if size(X, 1) ~= stateDimension
    error('GOSPA state dimension mismatch: expected %d, received %d.', ...
        stateDimension, size(X, 1));
end
end

%!test
%! model.xDimension = 4;
%! model.gospaParameters = struct('c', 5, 'p', 2, 'alpha', 2);
%! estA.mu = {{[0; 0; 0; 0]}};
%! estB.mu = {{[0; 0; 3; 0]}};
%! % A position-only implementation would incorrectly return zero here.
%! assert(computeGospaBetweenEstimates(estA, estB, 1, model), 3, 1e-12);

%!test
%! model.xDimension = 4;
%! model.gospaParameters = struct('c', 5, 'p', 2, 'alpha', 2);
%! estA.mu = {cell(1, 0)};
%! estB.mu = {{[0; 0; 0; 0]}};
%! assert(computeGospaBetweenEstimates(estA, estB, 1, model), sqrt(25 / 2), 1e-12);
