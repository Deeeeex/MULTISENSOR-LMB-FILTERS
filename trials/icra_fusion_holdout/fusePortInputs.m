function [objects,stats]=fusePortInputs(inputs,weights,model,details,cfg,rule,t)
% Keep one diagnostic structure across all locally marked/unmarked arms.
if strcmp(rule,'mil_support')
    [objects,mil]=fuseValidationInputs(inputs,weights,model,details,cfg,rule,t);
    stats=struct('weightChange',mil.weightChange,'lineageExcluded',mil.lineageExcluded, ...
        'observableAbsences',mil.observableAbsences,'records',zeros(0,26));
else
    if strcmp(rule,'qualified_exist'),rule='er';end
    [objects,stats]=fuseMarkedInputsStable(inputs,weights,model,details,cfg,rule,t);
end
end
