function schedule = ...
        normalizeBudgetRecycledFormationRepairUpdateScheduleV205( ...
            updateModes, sourceWeights, pageCount, idealDeliveryTeacherMode)
% NORMALIZEBUDGETRECYCLEDFORMATIONREPAIRUPDATESCHEDULEV205 Per-mode updates.
%
% A scalar mode/weight retains the legacy V188 behavior.  A mixed teacher
% schedule may use residual label KLA on supported labels and complete-label
% hard restoration when a receiver formation has lost the label entirely.
% The successor V216 action uses the distinct causal-label-kla mode: the
% label is selected after the ordinary page fusion from current observable
% posteriors, and delivery is sampled rather than granted by the teacher.

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
teacherLabelKlaMask = strcmp(modes, 'label-kla');
causalLabelKlaMask = strcmp(modes, 'causal-label-kla');
labelKlaMask = teacherLabelKlaMask | causalLabelKlaMask;
if any(hardMask & abs(weights - 1) > 1e-12) || ...
        any(teacherLabelKlaMask & ...
            (~logical(idealDeliveryTeacherMode) | weights >= 1)) || ...
        any(causalLabelKlaMask & ...
            (logical(idealDeliveryTeacherMode) | weights >= 1))
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        ['Hard restoration requires unit source weight; label-kla is an ', ...
         'ideal-delivery teacher action; causal-label-kla requires sampled ', ...
         'delivery. Both KLA modes require source weight below one.']);
end

schedule = struct();
schedule.contractVersion = ...
    'budget-recycled-formation-repair-update-schedule-v205-v1';
schedule.pageCount = pageCount;
schedule.modes = modes;
schedule.sourceWeights = weights;
schedule.hardReplacementMask = hardMask;
schedule.labelKlaMask = labelKlaMask;
schedule.teacherLabelKlaMask = teacherLabelKlaMask;
schedule.causalLabelKlaMask = causalLabelKlaMask;
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
if ~ismember(mode, { ...
        'hard-replacement', 'label-kla', 'causal-label-kla'})
    error('BudgetRecycledRepairUpdateV205:InvalidSchedule', ...
        ['Only hard-replacement, label-kla, and causal-label-kla ', ...
         'updates are registered.']);
end
end

function valid = validBoolean(value)
valid = isscalar(value) && ...
    (islogical(value) || ...
    (isnumeric(value) && isfinite(value) && (value == 0 || value == 1)));
end
