function [objects,stats]=fuseRobotReunionInputs(inputs,weights,model,details,cfg,arm,t)
% Per-label wrapper around the existing estimator; no full-filter changes.
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0);
objects=model.object;
allKeys=[];
for s=1:numel(inputs)
    for k=1:numel(inputs{s})
        allKeys(:,end+1)=[inputs{s}(k).birthTime;inputs{s}(k).birthLocation]; %#ok<AGROW>
    end
end
if isempty(allKeys), return; end
keys=unique(allKeys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs)); recentWeight=ones(size(weights));
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if strcmp(arm,'recent') && ~isempty(pieces{s})
            stamp=pieces{s}.lastDirectOpportunity;
            recentWeight(s)=.25;
            if stamp>0, recentWeight(s)=.25+.75*exp(-(t-stamp)*model.T/5); end
        end
    end
    selected=weights.*recentWeight; selected=selected/sum(selected);
    stats.weightChange=stats.weightChange+sum(abs(selected-weights));
    [one,diagnostics]=fuseLmbPosteriorsByLabel(pieces,selected,model,selected,details,cfg);
    stats.lineageExcluded=stats.lineageExcluded+diagnostics.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+diagnostics.observableCensoredSourceCount;
    if ~strcmp(arm,'lineage') && ~isempty(one)
        informed=false;
        for s=1:numel(pieces)
            if selected(s)>0 && ~isempty(pieces{s})
                informed=informed || pieces{s}.hasObservationLineage;
            end
        end
        one.hasObservationLineage=informed;
    end
    objects=[objects,one]; %#ok<AGROW>
end
end
