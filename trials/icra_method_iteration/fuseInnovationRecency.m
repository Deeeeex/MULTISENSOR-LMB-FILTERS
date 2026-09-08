function [objects,stats]=fuseInnovationRecency(inputs,weights,model,details,cfg,arm,t)
% Change only existence: recency redistributes the CURRENT local increment.
% Original frozen ER is also supported for instrumentation parity checks.
assert(ismember(arm,{'qualified_exist','innovation_recency','lineage'}));
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0, ...
    'records',zeros(0,26));
objects=model.object;keys=[];
cfg.missingLabelFusionMode='fov-aware-censored';
cfg.untouchedPriorExclusionEnabled=true;
cfg.captureLabelKlaDiagnosticsEnabled=true;
for s=1:numel(inputs)
    if ~isempty(inputs{s})
        keys=[keys,[[inputs{s}.birthTime];[inputs{s}.birthLocation]]]; %#ok<AGROW>
    end
end
if isempty(keys),return;end
keys=unique(keys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs));factors=ones(size(weights));
    increments=zeros(size(weights));stamps=zeros(size(weights));confirmed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if isempty(pieces{s}),continue;end
        assert(numel(pieces{s})==1);
        o=pieces{s};stamps(s)=o.lastDirectOpportunity;factors(s)=.25;
        if stamps(s)>0,factors(s)=.25+.75*exp(-(t-stamps(s))*model.T/5);end
        increments(s)=o.localLogOddsIncrement;
        assert(isfinite(increments(s)) && stamps(s)<=t);
        confirmed=confirmed || (o.positiveConfirmation && weights(s)>0);
    end
    q=weights.*factors;q=q/sum(q);
    if strcmp(arm,'qualified_exist'),requested=q;else,requested=weights;end
    [one,d]=fuseLmbPosteriorsByLabel(pieces,weights,model,requested,details,cfg);
    stats.lineageExcluded=stats.lineageExcluded+d.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+d.observableCensoredSourceCount;
    stats.weightChange=stats.weightChange+sum(abs(q-weights));
    if isempty(one),continue;end
    assert(numel(one)==1 && numel(d.labelKlaRecords)==1);
    rec=d.labelKlaRecords(1);active=rec.existenceParticipating;
    b=weights.*active;b=b/sum(b);q=q.*active;q=q/sum(q);
    logits=zeros(size(weights));logits(active)=safeLogit(rec.inputExistence(active));
    base=sum(b.*logits)+rec.spatialLogNormalizer;
    posteriorShift=sum((q-b).*logits);
    incrementShift=sum((q-b).*increments);
    rNoAge=logistic(base);rEr=logistic(base+posteriorShift);rIr=logistic(base+incrementShift);
    if strcmp(arm,'innovation_recency'),one.r=rIr;
    elseif strcmp(arm,'qualified_exist'),assert(abs(one.r-rEr)<2e-12);
    else,assert(abs(one.r-rNoAge)<2e-12);end
    one.positiveConfirmation=confirmed;
    if isfield(cfg,'captureIterationRecords') && cfg.captureIterationRecords
        assert(numel(weights)==2,'Detailed replay rows are a two-source schema.');
        stats.records(end+1,:)=[t,details.sourceIndices(1),keys(:,k)',one.mu{1}(1:2)', ...
            one.r,rNoAge,rEr,rIr,rec.spatialLogNormalizer,q,b,factors, ...
            rec.inputExistence,increments,stamps,posteriorShift,incrementShift,posteriorShift-incrementShift]; %#ok<AGROW>
    end
    objects=[objects,one]; %#ok<AGROW>
end
end

function value=safeLogit(value)
value=min(max(value,1e-9),1-1e-9);value=log(value)-log1p(-value);
end
function value=logistic(value)
if value>=0,value=1/(1+exp(-value));else,e=exp(value);value=e/(1+e);end
end
