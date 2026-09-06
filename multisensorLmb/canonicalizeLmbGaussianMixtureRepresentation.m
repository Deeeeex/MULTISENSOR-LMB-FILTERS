function [objects, diagnostics] = ...
        canonicalizeLmbGaussianMixtureRepresentation(objects, options)
% CANONICALIZELMBGAUSSIANMIXTUREREPRESENTATION Merge exact GM copies.
%
% A Gaussian-mixture density is unchanged when bit-identical components
% are replaced by one component whose weight is their sum.  Canonicalizing
% before pruning and component-count truncation prevents arbitrary duplicate
% copies from consuming the mixture budget.  With zero pruning threshold
% and an infinite component cap, the operation is exactly density preserving.

if nargin < 2 || isempty(options)
    options = struct();
end
weightThreshold = getField(options, 'weightThreshold', 0);
maximumComponentCount = getField(options, ...
    'maximumComponentCount', inf);
groupingMethod = getField(options,'groupingMethod','pairwise');
if ~any(strcmp(groupingMethod,{'pairwise','sorted'}))
    error('CanonicalLmbGm:InvalidGroupingMethod','Unknown exact grouping method.');
end
if ~isscalar(weightThreshold) || ~isfinite(weightThreshold) || ...
        weightThreshold < 0 || ~isscalar(maximumComponentCount) || ...
        isnan(maximumComponentCount) || maximumComponentCount < 1 || ...
        (~isinf(maximumComponentCount) && ...
         maximumComponentCount ~= round(maximumComponentCount))
    error('CanonicalLmbGm:InvalidOptions', ...
        'The GM canonicalization options are malformed.');
end

diagnostics = struct( ...
    'contractVersion', 'canonical-lmb-gm-representation-v1', ...
    'objectCount', numel(objects), ...
    'inputComponentCount', 0, ...
    'identicalCopyCount', 0, ...
    'canonicalComponentCountBeforeReduction', 0, ...
    'prunedComponentCount', 0, ...
    'truncatedComponentCount', 0, ...
    'discardedNormalizedWeightMass', 0, ...
    'outputComponentCount', 0, ...
    'exactDensityPreserved', true);
for objectIdx = 1:numel(objects)
    object = objects(objectIdx);
    componentCount = object.numberOfGmComponents;
    if ~isscalar(componentCount) || ~isfinite(componentCount) || ...
            componentCount < 0 || componentCount ~= round(componentCount) || ...
            numel(object.w) ~= componentCount || ...
            numel(object.mu) ~= componentCount || ...
            numel(object.Sigma) ~= componentCount
        error('CanonicalLmbGm:MalformedObject', ...
            'An LMB object has an inconsistent Gaussian mixture.');
    end
    diagnostics.inputComponentCount = ...
        diagnostics.inputComponentCount + componentCount;
    if componentCount == 0
        continue;
    end

    weights = reshape(object.w, 1, []);
    if any(~isfinite(weights)) || any(weights < 0)
        error('CanonicalLmbGm:InvalidWeights', ...
            'Gaussian-mixture weights must be finite and nonnegative.');
    end
    if sum(weights) <= 0
        error('CanonicalLmbGm:ZeroWeightMixture', ...
            'A nonempty Gaussian mixture has no positive weight.');
    end
    weights = weights / sum(weights);

    representatives = zeros(1, 0);
    canonicalWeights = zeros(1, 0);
    usedSorted = false;
    if strcmp(groupingMethod,'sorted') && componentCount>64
        [representatives,canonicalWeights,usedSorted] = ...
            sortedExactGroups(object,weights);
    end
    if ~usedSorted
      for componentIdx = 1:componentCount
        representativePosition = 0;
        for position = 1:numel(representatives)
            representativeIdx = representatives(position);
            if isequal(object.mu{componentIdx}, ...
                    object.mu{representativeIdx}) && ...
                    isequal(object.Sigma{componentIdx}, ...
                    object.Sigma{representativeIdx})
                representativePosition = position;
                break;
            end
        end
        if representativePosition == 0
            representatives(end + 1) = componentIdx; %#ok<AGROW>
            canonicalWeights(end + 1) = weights(componentIdx); %#ok<AGROW>
        else
            canonicalWeights(representativePosition) = ...
                canonicalWeights(representativePosition) + ...
                weights(componentIdx);
        end
      end
    end
    canonicalCount = numel(representatives);
    diagnostics.identicalCopyCount = ...
        diagnostics.identicalCopyCount + componentCount - canonicalCount;
    diagnostics.canonicalComponentCountBeforeReduction = ...
        diagnostics.canonicalComponentCountBeforeReduction + canonicalCount;

    [~, ordering] = sortrows( ...
        [-reshape(canonicalWeights, [], 1), ...
         reshape(representatives, [], 1)], [1, 2]);
    canonicalWeights = canonicalWeights(ordering);
    representatives = representatives(ordering);

    keep = canonicalWeights > weightThreshold;
    if ~any(keep)
        keep(1) = true;
    end
    diagnostics.prunedComponentCount = ...
        diagnostics.prunedComponentCount + nnz(~keep);
    diagnostics.discardedNormalizedWeightMass = ...
        diagnostics.discardedNormalizedWeightMass + ...
        sum(canonicalWeights(~keep));
    canonicalWeights = canonicalWeights(keep);
    representatives = representatives(keep);
    keepCount = min(numel(representatives), maximumComponentCount);
    diagnostics.truncatedComponentCount = ...
        diagnostics.truncatedComponentCount + ...
        numel(representatives) - keepCount;
    diagnostics.discardedNormalizedWeightMass = ...
        diagnostics.discardedNormalizedWeightMass + ...
        sum(canonicalWeights((keepCount + 1):end));
    canonicalWeights = canonicalWeights(1:keepCount);
    representatives = representatives(1:keepCount);
    canonicalWeights = canonicalWeights / sum(canonicalWeights);

    object.numberOfGmComponents = keepCount;
    object.w = reshape(canonicalWeights, 1, []);
    object.mu = reshape(object.mu(representatives), 1, []);
    object.Sigma = reshape(object.Sigma(representatives), 1, []);
    objects(objectIdx) = object;
    diagnostics.outputComponentCount = ...
        diagnostics.outputComponentCount + keepCount;
end
diagnostics.exactDensityPreserved = ...
    diagnostics.discardedNormalizedWeightMass == 0;
end

function [representatives,canonicalWeights,used] = sortedExactGroups(object,weights)
% Numeric keys, not rounding or approximate hashing. Retain first occurrence
% and original summation order so the legacy reduction sees the same inputs.
representatives=[]; canonicalWeights=[]; used=false;
n=numel(weights); meanSize=size(object.mu{1}); covarianceSize=size(object.Sigma{1});
width=numel(object.mu{1})+numel(object.Sigma{1}); keys=zeros(n,width);
for j=1:n
    m=object.mu{j}; P=object.Sigma{j};
    if ~isa(m,'double') || ~isa(P,'double') || ~isreal(m) || ~isreal(P) || ...
            ~isequal(size(m),meanSize) || ~isequal(size(P),covarianceSize) || ...
            any(~isfinite(m(:))) || any(~isfinite(P(:)))
        return; % Preserve isequal semantics for unusual representations.
    end
    keys(j,:)=[m(:)',P(:)'];
end
ordered=sortrows([keys,(1:n)']);
newGroup=[true;any(ordered(2:end,1:width)~=ordered(1:end-1,1:width),2)];
groupIds=cumsum(newGroup); first=ordered(newGroup,end);
[representatives,groupOrder]=sort(first);
mapping=zeros(numel(first),1); mapping(groupOrder)=1:numel(first);
membership=zeros(n,1); membership(ordered(:,end))=mapping(groupIds);
canonicalWeights=zeros(1,numel(first));
for j=1:n
    k=membership(j); canonicalWeights(k)=canonicalWeights(k)+weights(j);
end
representatives=representatives'; used=true;
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
