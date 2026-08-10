function certificate = certifyPhysicalTreeReferenceStaticV91(inputs)
% CERTIFYPHYSICALTREEREFERENCESTATICV91 Prove functional static behavior.

requiredFields = {'config', 'model', 'graphData'};
if ~isstruct(inputs) || ~isscalar(inputs) || ...
        ~all(isfield(inputs, requiredFields)) || ...
        ~isfield(inputs.graphData, 'physicalAdjacency')
    error('StaticBaselineV91:InvalidCertificateInput', ...
        'The V91 certificate requires one generated topology scene.');
end
nodeCount = inputs.config.numberOfSensors;
timeCount = inputs.config.simulationLength;
physicalPages = logical(inputs.graphData.physicalAdjacency);
if ~isequal(size(physicalPages), [nodeCount, nodeCount, timeCount])
    error('StaticBaselineV91:InvalidCertificateInput', ...
        'The physical graph schedule has invalid dimensions.');
end

context = struct();
context.localPosteriorBySensor = repmat({struct([])}, 1, nodeCount);
context.model = inputs.model;
context.directedMessageBudget = 2 * nodeCount;
routePages = false(nodeCount, nodeCount, timeCount);
weightPages = zeros(nodeCount, nodeCount, timeCount);
posteriorUsed = false(1, timeCount);
linkReliabilityUsed = false(1, timeCount);
for currentTime = 1:timeCount
    context.currentTime = currentTime;
    context.physicalAdjacency = physicalPages(:, :, currentTime);
    [routePages(:, :, currentTime), details] = ...
        selectPhysicalFormationTreeResidualTourPolicy(context, struct( ...
            'dominantWeight', 0.70, 'residualWeight', 0.05));
    weightPages(:, :, currentTime) = details.fusionWeightMatrix;
    posteriorUsed(currentTime) = logical(details.posteriorUsed);
    linkReliabilityUsed(currentTime) = logical( ...
        details.currentLinkReliabilityUsed);
end

routeCounts = pageCounts(routePages);
physicalUnique = uniquePageCount(physicalPages);
routeUnique = uniquePageCount(routePages);
weightUnique = uniqueNumericPageCount(weightPages);
allSelectedEdgesPhysical = true;
for currentTime = 1:timeCount
    allSelectedEdgesPhysical = allSelectedEdgesPhysical && ...
        ~any(any(routePages(:, :, currentTime) & ...
            ~physicalPages(:, :, currentTime)));
end
certificate = struct();
certificate.contractVersion = ...
    'physical-tree-reference-static-v91-certificate-v1';
certificate.presetName = inputs.config.presetName;
certificate.nodeCount = nodeCount;
certificate.timeCount = timeCount;
certificate.physicalGraphUniqueCount = physicalUnique;
certificate.referenceRouteUniqueCount = routeUnique;
certificate.referenceWeightUniqueCount = weightUnique;
certificate.referenceRouteChangeCount = pageChangeCount(routePages);
certificate.messageCountPerRound = routeCounts(1);
certificate.messageCountFixed = all(routeCounts == routeCounts(1));
certificate.expectedMessageCountPassed = ...
    all(routeCounts == 2 * nodeCount);
certificate.allSelectedEdgesPhysical = allSelectedEdgesPhysical;
certificate.posteriorUsed = any(posteriorUsed);
certificate.currentLinkReliabilityUsed = any(linkReliabilityUsed);
certificate.referenceFunctionallyStatic = ...
    physicalUnique == 1 && routeUnique == 1 && weightUnique == 1 && ...
    certificate.messageCountFixed && ...
    certificate.expectedMessageCountPassed && ...
    allSelectedEdgesPhysical && ...
    ~certificate.posteriorUsed && ...
    ~certificate.currentLinkReliabilityUsed;
certificate.frozenRouteAdjacency = routePages(:, :, 1);
certificate.frozenFusionWeightMatrix = weightPages(:, :, 1);
end

function count = uniquePageCount(pages)
flat = reshape(logical(pages), [], size(pages, 3))';
count = size(unique(flat, 'rows'), 1);
end

function count = uniqueNumericPageCount(pages)
flat = reshape(pages, [], size(pages, 3))';
count = size(unique(flat, 'rows'), 1);
end

function count = pageChangeCount(pages)
count = 0;
for timeIdx = 2:size(pages, 3)
    count = count + ~isequal( ...
        pages(:, :, timeIdx), pages(:, :, timeIdx - 1));
end
end

function counts = pageCounts(pages)
counts = zeros(1, size(pages, 3));
for timeIdx = 1:size(pages, 3)
    counts(timeIdx) = nnz(pages(:, :, timeIdx));
end
end
