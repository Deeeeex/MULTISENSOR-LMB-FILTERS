function [objects,stats]=fuseQualifiedReunionInputs(inputs,weights,model,details,cfg,arm,t)
% Controlled qualification/age composition. Spatial and existence weights
% differ only in the qualified_exist arm; this is not exact ordinary KLA.
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
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if ~isempty(pieces{s})
            stamp=pieces{s}.lastDirectOpportunity;
            factors(s)=.25;
            if stamp>0, factors(s)=.25+.75*exp(-(t-stamp)*model.T/5); end
        end
    end
    existence=weights.*factors; existence=existence/sum(existence);
    if strcmp(arm,'lineage_recent'), spatial=existence;
    elseif strcmp(arm,'qualified_exist'), spatial=weights;
    else, error('Unregistered extension arm'); end
    stats.weightChange=stats.weightChange+sum(abs(existence-weights));
    [one,diagnostics]=fuseLmbPosteriorsByLabel(pieces,spatial,model,existence,details,cfg);
    stats.lineageExcluded=stats.lineageExcluded+diagnostics.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+diagnostics.observableCensoredSourceCount;
    objects=[objects,one]; %#ok<AGROW>
end
end
