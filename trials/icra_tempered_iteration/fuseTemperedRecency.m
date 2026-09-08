function [objects,stats]=fuseTemperedRecency(inputs,weights,model,details,cfg,arm,t)
% Temper only the positive log-odds recency shift by current direct support.
arm=erase(arm,'marked_');
assert(any(strcmp(arm,{'lineage','er','tempered_association','tempered_score','tempered_calibrated'})));
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0,'records',zeros(0,26));
objects=model.object;keys=[];
cfg.missingLabelFusionMode='fov-aware-censored';cfg.untouchedPriorExclusionEnabled=true;
cfg.captureLabelKlaDiagnosticsEnabled=true;
assert(numel(unique(details.sourceIndices))==numel(inputs));
for s=1:numel(inputs)
    if ~isempty(inputs{s}),keys=[keys,[[inputs{s}.birthTime];[inputs{s}.birthLocation]]];end %#ok<AGROW>
end
if isempty(keys),return;end
keys=unique(keys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs));factors=ones(size(weights));stamps=zeros(size(weights));
    support=zeros(size(weights));confirmed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if isempty(pieces{s}),continue;end
        assert(numel(pieces{s})==1);o=pieces{s};stamps(s)=o.lastDirectOpportunity;factors(s)=.25;
        if stamps(s)>0,factors(s)=.25+.75*exp(-(t-stamps(s))*model.T/5);end
        if stamps(s)==t,support(s)=o.directEvidenceCeiling;end
        assert(stamps(s)<=t && support(s)>=0 && support(s)<=1);
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
    logits=zeros(size(weights));v=min(max(rec.inputExistence(active),1e-9),1-1e-9);
    logits(active)=log(v)-log1p(-v);
    base=sum(b.*logits)+rec.spatialLogNormalizer;
    r0=logistic(base);rEr=one.r;eligible=active & q>b+1e-12 & rec.inputExistence>=.5;
    cap=max([0,support(eligible)]);
    shift=sum((q-b).*logits);
    if strcmp(arm,'lineage')
        one.r=r0;
    elseif ~strcmp(arm,'er') && shift>0 && cap<1
        one.r=logistic(base+cap*shift);
    end
    one.positiveConfirmation=confirmed;one.directEvidenceCeiling=0;
    assert(isfinite(one.r));
    if ~strcmp(arm,'lineage'),assert(one.r<=rEr+1e-14 && one.r>=min(r0,rEr)-1e-14);end
    if isfield(cfg,'captureIterationRecords') && cfg.captureIterationRecords
        assert(numel(weights)==2);
        rr=min(max(one.r,1e-9),1-1e-9);admitted=log(rr)-log1p(-rr)-base;
        if strcmp(arm,'lineage'),admitted=0;end
        stats.records(end+1,:)=[t,details.sourceIndices(1),keys(:,k)',one.mu{1}(1:2)', ...
            one.r,r0,rEr,one.r,rec.spatialLogNormalizer,q,b,factors, ...
            rec.inputExistence,support,stamps,shift,admitted,shift-admitted]; %#ok<AGROW>
    end
    objects=[objects,one]; %#ok<AGROW>
end
if strcmp(arm,'lineage'),stats.weightChange=0;end
end

function r=logistic(z)
if z>=0,r=1/(1+exp(-z));else,e=exp(z);r=e/(1+e);end
end
