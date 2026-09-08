function [objects,stats]=fuseSelectiveInnovations(inputs,weights,model,details,cfg,arm,t)
% Conservative inherited posterior with selectively admitted current evidence.
assert(ismember(arm,{'selective','selective_no_history','selective_no_mark','selective_signed'}));
assert(numel(unique(details.sourceIndices))==numel(details.sourceIndices));
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0,'records',zeros(0,35));
objects=model.object;keys=[];
cfg.missingLabelFusionMode='fov-aware-censored';cfg.untouchedPriorExclusionEnabled=true;
cfg.captureLabelKlaDiagnosticsEnabled=true;
for s=1:numel(inputs)
    if ~isempty(inputs{s}),keys=[keys,[[inputs{s}.birthTime];[inputs{s}.birthLocation]]];end %#ok<AGROW>
end
if isempty(keys),return;end
keys=unique(keys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs));factors=ones(size(weights));present=false(size(weights));
    increments=zeros(size(weights));gates=zeros(size(weights));stamps=zeros(size(weights));
    original=zeros(2,numel(inputs));confirmed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);present(s)=~isempty(pieces{s});
        if ~present(s),continue;end
        assert(numel(pieces{s})==1);
        o=pieces{s};stamps(s)=o.lastDirectOpportunity;factors(s)=.25;
        if stamps(s)>0,factors(s)=.25+.75*exp(-(t-stamps(s))*model.T/5);end
        increments(s)=o.localLogOddsIncrement;
        if stamps(s)==t,gates(s)=o.positiveInnovationSupport;end
        assert(isfinite(increments(s)) && stamps(s)<=t && gates(s)>=0 && gates(s)<=1);
        if isfield(details,'originalLabels'),original(:,s)=details.originalLabels{s}(:,mask);
        else,original(:,s)=[o.birthTime;o.birthLocation];end
        confirmed=confirmed || (weights(s)>0 && o.positiveConfirmation);
    end
    q=weights.*factors;q=q/sum(q);
    [one,d]=fuseLmbPosteriorsByLabel(pieces,weights,model,q,details,cfg);
    stats.lineageExcluded=stats.lineageExcluded+d.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+d.observableCensoredSourceCount;
    stats.weightChange=stats.weightChange+sum(abs(q-weights));
    if isempty(one),continue;end
    assert(numel(one)==1 && numel(d.labelKlaRecords)==1);
    rec=d.labelKlaRecords(1);active=rec.existenceParticipating & weights>0;
    b=weights.*active;b=b/sum(b);q=q.*active;q=q/sum(q);
    logits=zeros(size(weights));logits(active)=safeLogit(rec.inputExistence(active));
    base=sum(b.*logits)+rec.spatialLogNormalizer;age=sum((q-b).*logits);
    beta=b;
    if ~strcmp(arm,'selective_no_history') && age < -1e-12,beta=q;end
    fresh=max(increments,0);if strcmp(arm,'selective_signed'),fresh=increments;end
    boost=0;
    if sum(active)>=2 && all(present(active)),boost=sum((double(active)-beta).*gates.*fresh);end
    correction=sum((beta-b).*logits)+boost;
    one.r=logistic(base+correction);one.positiveConfirmation=confirmed;
    one.positiveInnovationSupport=0;
    assert(isfinite(one.r) && one.r>=0 && one.r<=1);
    if isfield(cfg,'captureIterationRecords') && cfg.captureIterationRecords
        assert(numel(weights)==2);
        stats.records(end+1,:)=[t,details.sourceIndices(1),keys(:,k)',one.mu{1}(1:2)', ...
            one.r,logistic(base),logistic(base+age),one.r,rec.spatialLogNormalizer,q,b,factors, ...
            rec.inputExistence,increments,stamps,age,correction,age-correction, ...
            gates,beta,boost,original(:)']; %#ok<AGROW>
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
