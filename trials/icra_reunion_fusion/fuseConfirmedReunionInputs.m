function [objects,stats]=fuseConfirmedReunionInputs(inputs,weights,model,details,cfg,arm,t)
% Qualification changes participation; recency changes existence weights.
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0);
objects=model.object; allKeys=[];
for s=1:numel(inputs)
    for k=1:numel(inputs{s})
        allKeys(:,end+1)=[inputs{s}(k).birthTime;inputs{s}(k).birthLocation]; %#ok<AGROW>
    end
end
if isempty(allKeys), return; end
keys=unique(allKeys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs)); factors=ones(size(weights));
    confirmed=false; informed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if ~isempty(pieces{s}) && weights(s)>0
            confirmed=confirmed || pieces{s}.positiveConfirmation;
            informed=informed || pieces{s}.hasObservationLineage;
            stamp=pieces{s}.lastDirectOpportunity;
            factors(s)=.25;
            if stamp>0, factors(s)=.25+.75*exp(-(t-stamp)*model.T/5); end
        end
    end
    localCfg=cfg;
    localCfg.missingLabelFusionMode='fov-aware-censored';
    localCfg.untouchedPriorExclusionEnabled=confirmed || strcmp(arm,'qualified_exist');
    if strcmp(arm,'confirmed_lineage'), existence=weights;
    elseif ismember(arm,{'confirmed_exist','qualified_exist'})
        existence=weights.*factors; existence=existence/sum(existence);
    else, error('Unregistered confirmation arm'); end
    stats.weightChange=stats.weightChange+sum(abs(existence-weights));
    [one,diagnostics]=fuseLmbPosteriorsByLabel(pieces,weights,model,existence,details,localCfg);
    stats.lineageExcluded=stats.lineageExcluded+diagnostics.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+diagnostics.observableCensoredSourceCount;
    if ~isempty(one)
        one.positiveConfirmation=confirmed;
        if ~localCfg.untouchedPriorExclusionEnabled, one.hasObservationLineage=informed; end
    end
    objects=[objects,one]; %#ok<AGROW>
end
end
