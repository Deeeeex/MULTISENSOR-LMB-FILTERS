function summaries = ...
    auditFiniteEpisodeWeakMixingSequence(weightHistory, horizons, options)
% AUDITFINITEEPISODEWEAKMIXINGSEQUENCE Chronological product diagnostics.
%
% Products use the receiver-row/sender-column convention
% P(s:h) = W(s+h-1) ... W(s).  Boolean support is propagated on a
% separate Boolean semiring tree, so the structural certificate does not
% depend on floating-point underflow or a saturated Dobrushin value.

if nargin < 3 || isempty(options)
    options = struct();
end
allowedFields = {'verifyAllProductsDirectly'};
if ~isstruct(options) || ~isscalar(options) || ...
        any(~ismember(fieldnames(options), allowedFields))
    error('FiniteEpisodeWeakMixing:InvalidOptions', ...
        'The weak-mixing audit options are malformed.');
end
verifyAll = getField(options, 'verifyAllProductsDirectly', false);
validateInputs(weightHistory, horizons, verifyAll);
horizons = reshape(horizons, 1, []);

numericTree = buildNumericProductTree(weightHistory);
supportTree = buildSupportProductTree(weightHistory > 0);
summaries = repmat(emptySummary(), 1, numel(horizons));
for horizonIdx = 1:numel(horizons)
    summaries(horizonIdx) = auditHorizon( ...
        numericTree, supportTree, weightHistory, ...
        horizons(horizonIdx), verifyAll);
end
end

function summary = auditHorizon(numericTree, supportTree, history, ...
        horizon, verifyAll)
nodeCount = size(history, 1);
timeCount = size(history, 3);
windowCount = timeCount - horizon + 1;
dobrushin = zeros(1, windowCount);
centeredSpectral = zeros(1, windowCount);
centeredRow = zeros(1, windowCount);
columnSumError = zeros(1, windowCount);
uniformDistance = zeros(1, windowCount);
influenceImbalance = zeros(1, windowCount);
scrambling = false(1, windowCount);
positiveColumn = false(1, windowCount);
positiveSupport = false(1, windowCount);
floatingSupportMatched = true(1, windowCount);
uniform = ones(nodeCount) / nodeCount;
firstProductHash = '';
lastProductHash = '';

if verifyAll
    comparisonStarts = 1:windowCount;
else
    comparisonStarts = unique([1, ceil(windowCount / 2), windowCount]);
end
directNumericMatched = true;
directSupportMatched = true;
for windowStart = 1:windowCount
    windowEnd = windowStart + horizon - 1;
    product = queryNumericProductTree( ...
        numericTree, windowStart, windowEnd);
    support = querySupportProductTree( ...
        supportTree, windowStart, windowEnd);
    diagnostics = computeStochasticMixingDiagnostics(product);
    dobrushin(windowStart) = diagnostics.dobrushinCoefficient;
    centeredSpectral(windowStart) = diagnostics.centeredSpectralNorm;
    centeredRow(windowStart) = diagnostics.maximumCenteredRowL2;
    columnSumError(windowStart) = ...
        max(abs(sum(product, 1) - 1));
    uniformDistance(windowStart) = norm(product - uniform, 2);
    influence = mean(product, 1);
    influenceImbalance(windowStart) = ...
        sum(abs(influence - 1 / nodeCount));
    scrambling(windowStart) = isScramblingSupport(support);
    positiveColumn(windowStart) = any(all(support, 1));
    positiveSupport(windowStart) = all(support(:));
    floatingSupportMatched(windowStart) = ...
        isequal(product > 0, support);
    if windowStart == 1
        firstProductHash = computeCanonicalValueSha256(product);
    end
    if windowStart == windowCount
        lastProductHash = computeCanonicalValueSha256(product);
    end
    if ismember(windowStart, comparisonStarts)
        direct = directNumericProduct(history, windowStart, windowEnd);
        directSupport = directSupportProduct( ...
            history > 0, windowStart, windowEnd);
        directNumericMatched = directNumericMatched && ...
            max(abs(product(:) - direct(:))) < 1e-12;
        directSupportMatched = directSupportMatched && ...
            isequal(support, directSupport);
    end
end
if ~directNumericMatched || ~directSupportMatched
    error('FiniteEpisodeWeakMixing:ProductTreeMismatch', ...
        'A product tree differs from chronological direct multiplication.');
end

payload = struct();
payload.contractVersion = ...
    'finite-episode-weak-mixing-horizon-summary-v2';
payload.productAlgorithmContract = ...
    'chronological-newest-left-balanced-segment-tree-v1';
payload.inputPageSequenceCanonicalSha256 = ...
    computeCanonicalValueSha256(history);
payload.horizon = horizon;
payload.windowStarts = 1:windowCount;
payload.windowCount = windowCount;
payload.dobrushinByWindow = dobrushin;
payload.centeredSpectralNormByWindow = centeredSpectral;
payload.maximumCenteredRowL2ByWindow = centeredRow;
payload.maximumColumnSumErrorByWindow = columnSumError;
payload.distanceToUniformAveragingByWindow = uniformDistance;
payload.influenceWeightL1ImbalanceByWindow = influenceImbalance;
payload.supportScramblingByWindow = scrambling;
payload.positiveColumnByWindow = positiveColumn;
payload.fullPositiveSupportByWindow = positiveSupport;
payload.floatingSupportMatchesBooleanByWindow = floatingSupportMatched;
payload.minimumDobrushin = min(dobrushin);
payload.medianDobrushin = median(dobrushin);
payload.worstDobrushin = max(dobrushin);
payload.worstCenteredSpectralNorm = max(centeredSpectral);
payload.worstMaximumCenteredRowL2 = max(centeredRow);
payload.worstMaximumColumnSumError = max(columnSumError);
payload.worstDistanceToUniformAveraging = max(uniformDistance);
payload.worstInfluenceWeightL1Imbalance = max(influenceImbalance);
payload.supportScramblingWindowCount = nnz(scrambling);
payload.positiveColumnWindowCount = nnz(positiveColumn);
payload.fullPositiveSupportWindowCount = nnz(positiveSupport);
payload.saturatedNumericalDobrushinWindowCount = ...
    nnz(scrambling & dobrushin >= 1);
payload.everyProductSupportScrambling = all(scrambling);
payload.everyProductHasPositiveColumn = all(positiveColumn);
payload.everyProductSupportPositive = all(positiveSupport);
payload.everyProductNumericallyDobrushinBelowOne = ...
    all(dobrushin < 1);
payload.everyFloatingSupportMatchesBoolean = ...
    all(floatingSupportMatched);
payload.productTreeDirectComparisonPassed = ...
    directNumericMatched && directSupportMatched;
payload.productTreeDirectComparisonWindowCount = ...
    numel(comparisonStarts);
payload.everyWindowComparedDirectly = ...
    numel(comparisonStarts) == windowCount;
payload.firstSegmentTreeChronologicalProductCanonicalSha256 = ...
    firstProductHash;
payload.lastSegmentTreeChronologicalProductCanonicalSha256 = ...
    lastProductHash;
payload.dobrushinInterpretation = ...
    'floating-diagnostic-may-saturate-near-one-not-a-support-test';
summary = payload;
summary.canonicalSha256 = computeCanonicalValueSha256(payload);
end

function tree = buildNumericProductTree(history)
nodeCount = size(history, 1);
timeCount = size(history, 3);
base = nextPowerOfTwo(timeCount);
pages = zeros(nodeCount, nodeCount, 2 * base);
for leafIdx = 1:base
    if leafIdx <= timeCount
        pages(:, :, base + leafIdx - 1) = history(:, :, leafIdx);
    else
        pages(:, :, base + leafIdx - 1) = eye(nodeCount);
    end
end
for nodeIdx = base-1:-1:1
    pages(:, :, nodeIdx) = pages(:, :, 2 * nodeIdx + 1) * ...
        pages(:, :, 2 * nodeIdx);
end
tree = struct('base', base, 'nodeCount', nodeCount, 'pages', pages);
end

function tree = buildSupportProductTree(historySupport)
nodeCount = size(historySupport, 1);
timeCount = size(historySupport, 3);
base = nextPowerOfTwo(timeCount);
pages = false(nodeCount, nodeCount, 2 * base);
for leafIdx = 1:base
    if leafIdx <= timeCount
        pages(:, :, base + leafIdx - 1) = ...
            historySupport(:, :, leafIdx);
    else
        pages(:, :, base + leafIdx - 1) = logical(eye(nodeCount));
    end
end
for nodeIdx = base-1:-1:1
    pages(:, :, nodeIdx) = booleanMultiply( ...
        pages(:, :, 2 * nodeIdx + 1), ...
        pages(:, :, 2 * nodeIdx));
end
tree = struct('base', base, 'nodeCount', nodeCount, 'pages', pages);
end

function product = queryNumericProductTree(tree, firstTime, lastTime)
leftIdx = tree.base + firstTime - 1;
rightIdx = tree.base + lastTime - 1;
leftProduct = eye(tree.nodeCount);
rightProduct = eye(tree.nodeCount);
while leftIdx <= rightIdx
    if mod(leftIdx, 2) == 1
        leftProduct = tree.pages(:, :, leftIdx) * leftProduct;
        leftIdx = leftIdx + 1;
    end
    if mod(rightIdx, 2) == 0
        rightProduct = rightProduct * tree.pages(:, :, rightIdx);
        rightIdx = rightIdx - 1;
    end
    leftIdx = floor(leftIdx / 2);
    rightIdx = floor(rightIdx / 2);
end
product = rightProduct * leftProduct;
end

function product = querySupportProductTree(tree, firstTime, lastTime)
leftIdx = tree.base + firstTime - 1;
rightIdx = tree.base + lastTime - 1;
leftProduct = logical(eye(tree.nodeCount));
rightProduct = logical(eye(tree.nodeCount));
while leftIdx <= rightIdx
    if mod(leftIdx, 2) == 1
        leftProduct = booleanMultiply( ...
            tree.pages(:, :, leftIdx), leftProduct);
        leftIdx = leftIdx + 1;
    end
    if mod(rightIdx, 2) == 0
        rightProduct = booleanMultiply( ...
            rightProduct, tree.pages(:, :, rightIdx));
        rightIdx = rightIdx - 1;
    end
    leftIdx = floor(leftIdx / 2);
    rightIdx = floor(rightIdx / 2);
end
product = booleanMultiply(rightProduct, leftProduct);
end

function product = directNumericProduct(history, firstTime, lastTime)
product = eye(size(history, 1));
for currentTime = firstTime:lastTime
    product = history(:, :, currentTime) * product;
end
end

function product = directSupportProduct(historySupport, firstTime, lastTime)
product = logical(eye(size(historySupport, 1)));
for currentTime = firstTime:lastTime
    product = booleanMultiply( ...
        historySupport(:, :, currentTime), product);
end
end

function product = booleanMultiply(left, right)
product = (double(left) * double(right)) > 0;
end

function passed = isScramblingSupport(support)
passed = true;
for firstIdx = 1:size(support, 1)-1
    for secondIdx = firstIdx+1:size(support, 1)
        if ~any(support(firstIdx, :) & support(secondIdx, :))
            passed = false;
            return;
        end
    end
end
end

function value = nextPowerOfTwo(timeCount)
value = 1;
while value < timeCount
    value = 2 * value;
end
end

function validateInputs(weightHistory, horizons, verifyAll)
validHistory = isa(weightHistory, 'double') && ...
    isreal(weightHistory) && ~issparse(weightHistory) && ...
    ndims(weightHistory) <= 3 && ...
    size(weightHistory, 1) == size(weightHistory, 2) && ...
    size(weightHistory, 1) >= 2 && size(weightHistory, 3) >= 1 && ...
    all(isfinite(weightHistory(:))) && all(weightHistory(:) >= 0);
if validHistory
    nodeCount = size(weightHistory, 1);
    timeCount = size(weightHistory, 3);
    rowSums = sum(weightHistory, 2);
    diagonals = zeros(nodeCount, timeCount);
    for currentTime = 1:timeCount
        diagonals(:, currentTime) = ...
            diag(weightHistory(:, :, currentTime));
    end
    validHistory = all(abs(rowSums(:) - 1) <= 1e-12) && ...
        all(diagonals(:) > 0);
end
validHorizons = isnumeric(horizons) && isreal(horizons) && ...
    isvector(horizons) && ~isempty(horizons) && ...
    all(isfinite(horizons)) && all(horizons == round(horizons)) && ...
    all(horizons >= 1) && all(horizons <= size(weightHistory, 3)) && ...
    numel(unique(horizons)) == numel(horizons) && ...
    isequal(reshape(horizons, 1, []), ...
        sort(reshape(horizons, 1, [])));
if ~validHistory || ~validHorizons || ...
        ~islogical(verifyAll) || ~isscalar(verifyAll)
    error('FiniteEpisodeWeakMixing:InvalidInput', ...
        ['Finite nonnegative row-stochastic pages with positive ', ...
         'diagonals, ordered valid horizons, and a logical audit ', ...
         'mode are required.']);
end
end

function value = emptySummary()
value = struct();
end

function value = getField(structure, name, defaultValue)
if isfield(structure, name)
    value = structure.(name);
else
    value = defaultValue;
end
end
