function schedule = ...
        normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
            updateModes, sourceWeights, pageCount, idealDeliveryTeacherMode)
% NORMALIZEBUDGETRECYCLEDFORMATIONREPAIRUPDATESCHEDULEV205 Per-mode updates.
%
% A scalar mode/weight retains the legacy V188 behavior.  A mixed teacher
% schedule may use residual label KLA on supported labels and complete-label
% hard restoration when a receiver formation has lost the label entirely.

if ~isscalar(pageCount) || ~isnumeric(pageCount) || ...
        ~isfinite(pageCount) || pageCount < 1 || ...
        pageCount ~= round(pageCount) || ...
        ~validBoolean(idealDeliveryTeacherMode)
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'A positive page count and valid teacher flag are required.');
end

scalarModeInput = ischar(updateModes);
if scalarModeInput
    modes = repmat({normalizeMode(updateModes)}, 1, pageCount);
elseif iscell(updateModes) && numel(updateModes) == pageCount && ...
        all(cellfun(@ischar, updateModes))
    modes = reshape(cellfun(@normalizeMode, updateModes, ...
        'UniformOutput', false), 1, []);
else
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'Update modes must be one character vector or one per page.');
end

scalarWeightInput = isnumeric(sourceWeights) && isscalar(sourceWeights);
if scalarWeightInput
    weights = repmat(sourceWeights, 1, pageCount);
elseif isnumeric(sourceWeights) && numel(sourceWeights) == pageCount
    weights = reshape(sourceWeights, 1, []);
else
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'Source weights must be one scalar or one numeric value per page.');
end
if any(~isfinite(weights)) || any(weights <= 0) || any(weights > 1)
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'Every source weight must lie in (0,1].');
end

hardMask = strcmp(modes, 'hard-replacement');
labelKlaMask = strcmp(modes, 'label-kla');
if any(hardMask & abs(weights - 1) > 1e-12) || ...
        any(labelKlaMask & ...
            (~logical(idealDeliveryTeacherMode) | weights >= 1))
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        ['Hard restoration requires unit source weight; residual label ', ...
         'KLA remains an ideal-delivery teacher action with weight below one.']);
end

schedule = struct();
schedule.contractVersion = ...
    'budget-recycled-formation-repair-update-schedule-v205-v1';
schedule.pageCount = pageCount;
schedule.modes = modes;
schedule.sourceWeights = weights;
schedule.hardReplacementMask = hardMask;
schedule.labelKlaMask = labelKlaMask;
schedule.mixedUpdateModes = numel(unique(modes)) > 1;
schedule.scalarModeInput = scalarModeInput;
schedule.scalarWeightInput = scalarWeightInput;
schedule.idealDeliveryTeacherMode = logical(idealDeliveryTeacherMode);
schedule.truthUsed = false;
schedule.futureInformationUsed = false;
end

function mode = normalizeMode(mode)
if ~ischar(mode) || isempty(mode)
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'Every update mode must be a nonempty character vector.');
end
mode = lower(strrep(mode, '_', '-'));
if ~ismember(mode, {'hard-replacement', 'label-kla'})
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        'Only hard-replacement and label-kla updates are registered.');
end
end

function valid = validBoolean(value)
valid = isscalar(value) && ...
    (islogical(value) || ...
    (isnumeric(value) && isfinite(value) && (value == 0 || value == 1)));
end
