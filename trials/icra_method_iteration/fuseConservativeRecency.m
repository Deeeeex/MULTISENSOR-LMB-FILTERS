function [objects,stats]=fuseConservativeRecency(inputs,weights,model,details,cfg,arm,t)
% ER objective subject to r <= the same-input no-age result.
assert(strcmp(arm,'conservative_recency'));
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0,'records',zeros(0,26));
objects=model.object;keys=[];
cfg.missingLabelFusionMode='fov-aware-censored';cfg.untouchedPriorExclusionEnabled=true;
cfg.captureLabelKlaDiagnosticsEnabled=true;
for s=1:numel(inputs)
    if ~isempty(inputs{s}),keys=[keys,[[inputs{s}.birthTime];[inputs{s}.birthLocation]]];end %#ok<AGROW>
end
if isempty(keys),return;end
keys=unique(keys','rows')';
for k=1:size(keys,2)
    pieces=cell(size(inputs));factors=ones(size(weights));stamps=zeros(size(weights));confirmed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);
        if isempty(pieces{s}),continue;end
        o=pieces{s};stamps(s)=o.lastDirectOpportunity;factors(s)=.25;
        if stamps(s)>0,factors(s)=.25+.75*exp(-(t-stamps(s))*model.T/5);end
        assert(stamps(s)<=t);
        confirmed=confirmed || (weights(s)>0 && o.positiveConfirmation);
    end
    q=weights.*factors;q=q/sum(q);
    [one,d]=fuseLmbPosteriorsByLabel(pieces,weights,model,q,details,cfg);
    stats.lineageExcluded=stats.lineageExcluded+d.lineageExcludedSourceCount;
    stats.observableAbsences=stats.observableAbsences+d.observableCensoredSourceCount;
    stats.weightChange=stats.weightChange+sum(abs(q-weights));
    if isempty(one),continue;end
    assert(numel(one)==1 && numel(d.labelKlaRecords)==1);
    rec=d.labelKlaRecords(1);active=rec.existenceParticipating;
    b=weights.*active;b=b/sum(b);q=q.*active;q=q/sum(q);
    logits=zeros(size(weights));v=min(max(rec.inputExistence(active),1e-9),1-1e-9);
    logits(active)=log(v)-log1p(-v);
    base=sum(b.*logits)+rec.spatialLogNormalizer;
    if base>=0,r0=1/(1+exp(-base));else,e=exp(base);r0=e/(1+e);end
    rEr=one.r;one.r=min(r0,rEr);one.positiveConfirmation=confirmed;
    assert(one.r<=r0 && one.r<=rEr && isfinite(one.r));
    if isfield(cfg,'captureIterationRecords') && cfg.captureIterationRecords
        assert(numel(weights)==2);
        shift=sum((q-b).*logits);
        % Columns 10 and 25 are CR r and its nonpositive correction; the
        % other two scalar slots are zeros (CR uses no increment metadata).
        stats.records(end+1,:)=[t,details.sourceIndices(1),keys(:,k)',one.mu{1}(1:2)', ...
            one.r,r0,rEr,one.r,rec.spatialLogNormalizer,q,b,factors, ...
            rec.inputExistence,0,0,stamps,shift,min(shift,0),max(shift,0)]; %#ok<AGROW>
    end
    objects=[objects,one]; %#ok<AGROW>
end
end
