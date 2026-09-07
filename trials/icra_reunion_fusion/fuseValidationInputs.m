function [objects,stats]=fuseValidationInputs(inputs,weights,model,details,cfg,arm,t)
if ismember(arm,{'lineage_recent','qualified_exist'})
    cfg.missingLabelFusionMode='fov-aware-censored';
    cfg.untouchedPriorExclusionEnabled=true;
    [objects,stats]=fuseQualifiedReunionInputs(inputs,weights,model,details,cfg,arm,t);
elseif strcmp(arm,'confirmed_exist')
    [objects,stats]=fuseConfirmedReunionInputs(inputs,weights,model,details,cfg,arm,t);
elseif strcmp(arm,'mil_support')
    stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0);
    objects=model.object; keys=[];
    for s=1:numel(inputs)
        for k=1:numel(inputs{s})
            keys(:,end+1)=[inputs{s}(k).birthTime;inputs{s}(k).birthLocation]; %#ok<AGROW>
        end
    end
    if ~isempty(keys), keys=unique(keys','rows')'; end
    for k=1:size(keys,2)
        pieces=cell(size(inputs)); present=false(size(inputs));
        for s=1:numel(inputs)
            pieces{s}=inputs{s}([inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k));
            present(s)=~isempty(pieces{s}) && weights(s)>0;
        end
        w=weights(present); w=w/sum(w);
        one=fuseCommonLabelLmbMil(pieces(present),w,model,cfg.mixtureAwareMaxFusedComponents);
        objects=[objects,one]; %#ok<AGROW>
    end
else
    [objects,stats]=fuseRobotReunionInputs(inputs,weights,model,details,cfg,arm,t);
end
% Certificate is metadata only for the non-confirmation arms. Never refresh
% local direct-opportunity timestamps from this propagated observation flag.
for k=1:numel(objects)
    confirmed=false; informed=false;
    for s=1:numel(inputs)
        idx=find([inputs{s}.birthTime]==objects(k).birthTime & ...
            [inputs{s}.birthLocation]==objects(k).birthLocation);
        if ~isempty(idx) && weights(s)>0
            confirmed=confirmed || inputs{s}(idx).positiveConfirmation;
            informed=informed || inputs{s}(idx).hasObservationLineage;
        end
    end
    objects(k).positiveConfirmation=confirmed;
    if ismember(arm,{'mil','mil_support'}), objects(k).hasObservationLineage=informed; end
end
end
