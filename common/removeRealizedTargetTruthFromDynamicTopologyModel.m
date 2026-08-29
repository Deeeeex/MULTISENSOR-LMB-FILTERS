function runtimeModel = ...
    removeRealizedTargetTruthFromDynamicTopologyModel(model)
% REMOVEREALIZEDTARGETTRUTHFROMDYNAMICTOPOLOGYMODEL Sanitize filter input.

runtimeModel = model;
if isfield(runtimeModel, 'explicitTargetTrajectories')
    runtimeModel = rmfield(runtimeModel, 'explicitTargetTrajectories');
end
if isfield(runtimeModel, 'dynamicTopologyScenario') && ...
        isstruct(runtimeModel.dynamicTopologyScenario)
    fields = intersect({'targetTrajectories', 'target'}, ...
        fieldnames(runtimeModel.dynamicTopologyScenario), 'stable');
    if ~isempty(fields)
        runtimeModel.dynamicTopologyScenario = rmfield( ...
            runtimeModel.dynamicTopologyScenario, fields);
    end
end
end
