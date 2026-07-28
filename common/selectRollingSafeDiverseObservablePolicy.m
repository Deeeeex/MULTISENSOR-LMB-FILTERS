function [adjacency, details] = ...
    selectRollingSafeDiverseObservablePolicy( ...
        context, baseMode, alternativeIndex, options)
% SELECTROLLINGSAFEDIVERSEOBSERVABLEPOLICY Nearby truth-free safe action.
%
% The observable base scorer is solved once. Alternative k forbids the
% k-th selected cross-formation edge, ordered from the smallest to the
% largest base-score contribution, and replays the same exact rolling-B3
% projection. No target truth or future outcome is read.

if nargin < 4 || isempty(options)
    options = struct();
end
baseMode = lower(strrep(char(baseMode), '_', '-'));
if ~ismember(baseMode, {'posterior-analytic', 'link-aware'})
    error(['Diverse observable routing supports posterior-analytic ', ...
        'or link-aware base scores.']);
end
alternativeIndex = round(alternativeIndex);
if ~isscalar(alternativeIndex) || ...
        ~isfinite(alternativeIndex) || alternativeIndex < 1
    error(['Diverse observable alternativeIndex must be a positive ', ...
        'integer.']);
end
baseOptions = struct( ...
    'sourceWeight', getField(options, 'sourceWeight', 0.70), ...
    'payloadToleranceFraction', getField( ...
        options, 'payloadToleranceFraction', inf));
[defaultAdjacency, defaultDetails] = ...
    selectRollingSafeRoutingPolicy( ...
        context, baseMode, baseOptions);

scoreDetails = requireScoreDetails(defaultDetails);
selectedExamples = reshape( ...
    defaultDetails.selectedCrossExampleIndices, 1, []);
selectedScores = reshape( ...
    scoreDetails.residualScores(selectedExamples), 1, []);
[~, order] = sort(selectedScores, 'ascend');
orderedExamples = selectedExamples(order);
if alternativeIndex > numel(orderedExamples)
    adjacency = defaultAdjacency;
    details = defaultDetails;
    details.diverseObservableBaseMode = baseMode;
    details.diverseObservableAlternativeRequested = alternativeIndex;
    details.diverseObservableAlternativeFeasible = false;
    details.diverseObservableBannedExampleIndex = NaN;
    details.diverseObservableBannedReceiver = NaN;
    details.diverseObservableBannedSender = NaN;
    details.diverseObservableDefaultSelectedSources = ...
        defaultDetails.selectedSourcesByReceiver;
    details.diverseObservableDefaultSelectedCrossExampleIndices = ...
        selectedExamples;
    details.diverseObservableChangedAction = false;
    details.diverseObservableScoreReuse = true;
    details.truthUsed = false;
    return;
end

bannedExample = orderedExamples(alternativeIndex);
allowedMask = true(numel(scoreDetails.residualScores), 1);
allowedMask(bannedExample) = false;
replayOptions = baseOptions;
replayOptions.currentAllowedExampleMask = allowedMask;
replayOptions.edgeScoreFcnReturnsDetails = true;
replayOptions.edgeScoreFcn = ...
    @(unusedContext, receiverIndices, senderIndices, ...
        unusedCycleSources) replayScores( ...
            receiverIndices, senderIndices, scoreDetails);
replayOptions.posteriorUsed = ...
    strcmp(baseMode, 'posterior-analytic');
replayOptions.truthUsed = false;
replayOptions.currentLinkReliabilityUsed = true;

alternativeFeasible = true;
try
    [adjacency, details] = ...
        selectRollingSafeRoutingPolicy( ...
            context, 'external-scores', replayOptions);
catch errorInfo
    if ~isProjectionInfeasible(errorInfo)
        rethrow(errorInfo);
    end
    adjacency = defaultAdjacency;
    details = defaultDetails;
    alternativeFeasible = false;
end

details.diverseObservableBaseMode = baseMode;
details.diverseObservableAlternativeRequested = alternativeIndex;
details.diverseObservableAlternativeFeasible = ...
    alternativeFeasible;
details.diverseObservableBannedExampleIndex = bannedExample;
details.diverseObservableBannedReceiver = ...
    scoreDetails.receiverIndices(bannedExample);
details.diverseObservableBannedSender = ...
    scoreDetails.senderIndices(bannedExample);
details.diverseObservableDefaultSelectedSources = ...
    defaultDetails.selectedSourcesByReceiver;
details.diverseObservableDefaultSelectedCrossExampleIndices = ...
    selectedExamples;
details.diverseObservableChangedAction = ...
    ~isequal(adjacency, defaultAdjacency);
details.diverseObservableScoreReuse = true;
details.truthUsed = false;
end

function scoreDetails = requireScoreDetails(details)
if ~isfield(details, 'scoreDetails') || ...
        ~isstruct(details.scoreDetails)
    error('Diverse observable routing could not recover edge scores.');
end
scoreDetails = details.scoreDetails;
required = {'receiverIndices', 'senderIndices', 'residualScores'};
for fieldIdx = 1:numel(required)
    if ~isfield(scoreDetails, required{fieldIdx})
        error('Diverse observable score details lack %s.', ...
            required{fieldIdx});
    end
end
end

function [scores, details] = replayScores( ...
        receiverIndices, senderIndices, cachedDetails)
if ~isequal(reshape(receiverIndices, [], 1), ...
        reshape(cachedDetails.receiverIndices, [], 1)) || ...
        ~isequal(reshape(senderIndices, [], 1), ...
        reshape(cachedDetails.senderIndices, [], 1))
    error('Diverse observable cached scores do not match the action set.');
end
scores = reshape(cachedDetails.residualScores, [], 1);
details = cachedDetails;
details.replayedWithoutScoreRecomputation = true;
details.truthUsed = false;
end

function infeasible = isProjectionInfeasible(errorInfo)
infeasible = strcmp(errorInfo.identifier, ...
    'RollingMatching:Infeasible') || ...
    ~isempty(strfind(lower(errorInfo.message), 'infeasible'));
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
