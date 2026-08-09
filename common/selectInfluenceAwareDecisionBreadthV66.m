function selection = selectInfluenceAwareDecisionBreadthV66( ...
        baseRiskMetrics, influenceMetrics, options)
% SELECTINFLUENCEAWAREDECISIONBREADTHV66 Guard V65 by causal breadth.

if nargin < 3 || isempty(options)
    options = getInfluenceAwareDecisionBreadthV66Protocol();
end
minimumExposure = getField(options, ...
    'minimumNormalizedRobustMarginExposure', 0.05);
if ~isstruct(baseRiskMetrics) || ~isstruct(influenceMetrics) || ...
        ~isfield(baseRiskMetrics, 'groups') || ...
        ~isfield(influenceMetrics, 'groups') || ...
        ~isfield(influenceMetrics, ...
            'formationNormalizedRobustMarginExposure') || ...
        ~isfield(influenceMetrics, ...
            'formationNormalizedDecisionExposure') || ...
        ~isfield(influenceMetrics, 'formationUpwardCrossings') || ...
        ~isfield(influenceMetrics, 'formationAffectedReceiverCount') || ...
        ~isequal(reshape(baseRiskMetrics.groups, 1, []), ...
            reshape(influenceMetrics.groups, 1, [])) || ...
        ~isscalar(minimumExposure) || ~isfinite(minimumExposure) || ...
        minimumExposure < 0
    error('InfluenceAwareSelectorV66:InvalidInput', ...
        'The V65 risk and V66 influence metrics do not align.');
end

baseSelection = selectNetworkAdditiveFormationRiskSetV65( ...
    baseRiskMetrics, options);
groups = reshape(influenceMetrics.groups, 1, []);
selection = struct();
selection.contractVersion = ...
    'influence-aware-decision-breadth-selector-v66-v1';
selection.baseSelection = baseSelection;
selection.minimumNormalizedRobustMarginExposure = minimumExposure;
selection.baseSelectedFormationIds = ...
    baseSelection.selectedFormationIds;
selection.actionEnabled = false;
selection.selectedFormationIds = zeros(1, 0);
selection.selectedFormationMask = false(size(groups));
selection.selectedNormalizedRobustMarginExposure = 0;
selection.selectedNormalizedDecisionExposure = 0;
selection.selectedUpwardCrossings = 0;
selection.selectedAffectedReceiverCount = 0;
selection.influenceBreadthGatePassed = false;
selection.fallbackReason = baseSelection.fallbackReason;
selection.truthUsed = false;
selection.futureMeasurementsUsed = false;
selection.futureOutcomesUsed = false;
if ~baseSelection.actionEnabled
    return;
end

mask = ismember(groups, baseSelection.selectedFormationIds);
robustExposure = sumFinite(influenceMetrics. ...
    formationNormalizedRobustMarginExposure(mask));
decisionExposure = sumFinite(influenceMetrics. ...
    formationNormalizedDecisionExposure(mask));
upwardCrossings = sumFinite( ...
    influenceMetrics.formationUpwardCrossings(mask));
affectedReceivers = sumFinite( ...
    influenceMetrics.formationAffectedReceiverCount(mask));
selection.selectedNormalizedRobustMarginExposure = robustExposure;
selection.selectedNormalizedDecisionExposure = decisionExposure;
selection.selectedUpwardCrossings = upwardCrossings;
selection.selectedAffectedReceiverCount = affectedReceivers;
selection.influenceBreadthGatePassed = ...
    robustExposure >= minimumExposure - 1e-12;
if ~selection.influenceBreadthGatePassed
    selection.fallbackReason = ...
        'insufficient-influence-weighted-decision-breadth';
    return;
end

selection.actionEnabled = true;
selection.selectedFormationIds = baseSelection.selectedFormationIds;
selection.selectedFormationMask = mask;
selection.fallbackReason = '';
end

function value = sumFinite(values)
values = reshape(values, 1, []);
if isempty(values) || any(~isfinite(values))
    error('InfluenceAwareSelectorV66:NonfiniteMetric', ...
        'A selected V66 influence metric is nonfinite.');
end
value = sum(values);
end

function value = getField(data, name, fallback)
if isstruct(data) && isfield(data, name)
    value = data.(name);
else
    value = fallback;
end
end
