function [objects,stats]=fuseGaussianEvidence(inputs,weights,model,details,cfg,arm,t)
% Conservative inherited posterior with selectively admitted current evidence.
assert(ismember(arm,{'gaussian_evidence','gaussian_evidence_no_curvature','gaussian_evidence_no_history','gaussian_evidence_no_mark'}));
assert(numel(unique(details.sourceIndices))==numel(details.sourceIndices));
stats=struct('weightChange',0,'lineageExcluded',0,'observableAbsences',0,'records',zeros(0,60));
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
    increments=zeros(size(weights));gates=zeros(size(weights));negativeGates=zeros(size(weights));stamps=zeros(size(weights));
    original=zeros(2,numel(inputs));confirmed=false;
    for s=1:numel(inputs)
        mask=[inputs{s}.birthTime]==keys(1,k) & [inputs{s}.birthLocation]==keys(2,k);
        pieces{s}=inputs{s}(mask);present(s)=~isempty(pieces{s});
        if ~present(s),continue;end
        assert(numel(pieces{s})==1);
        o=pieces{s};stamps(s)=o.lastDirectOpportunity;factors(s)=.25;
        if stamps(s)>0,factors(s)=.25+.75*exp(-(t-stamps(s))*model.T/5);end
        increments(s)=o.localLogOddsIncrement;
        if stamps(s)==t,gates(s)=o.positiveInnovationSupport;negativeGates(s)=o.negativeInnovationSupport;end
        assert(isfinite(increments(s)) && stamps(s)<=t && gates(s)>=0 && gates(s)<=1);
        assert(isfinite(negativeGates(s)) && negativeGates(s)>=0 && negativeGates(s)<=1);
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
    if ~strcmp(arm,'gaussian_evidence_no_history') && age < -1e-12,beta=q;end
    fresh=gates.*max(increments,0)+negativeGates.*min(increments,0);
    boost=0;
    if sum(active)>=2 && all(present(active)),boost=sum((double(active)-beta).*fresh);end
    correction=sum((beta-b).*logits)+boost;
    scalarR=logistic(base+correction);oldMean=one.mu{1};
    alpha=rec.activeSpatialWeights;kept=zeros(size(weights));allowed=true(size(weights));aggregateFallback=false;
    logIntegral=rec.spatialLogNormalizer;
    rawKappa=(double(active)-beta).*(gates.*(increments>=0)+negativeGates.*(increments<0));
    if sum(active)<2 || ~all(present(active)),rawKappa(:)=0;end
    if any(rawKappa>0)
        [mu,P,logIntegral,kept,allowed,aggregateFallback]=applyGaussianEvidence( ...
            pieces,alpha,rawKappa,~strcmp(arm,'gaussian_evidence_no_curvature'));
        if any(kept>0)
            one.mu={mu};one.Sigma={P};
        else
            logIntegral=rec.spatialLogNormalizer;
        end
    end
    one.r=logistic(sum(beta.*logits)+sum(kept.*increments)+logIntegral);
    one.positiveConfirmation=confirmed;one.localSpatialLogRatio=zeros(1,15);
    one.positiveInnovationSupport=0;one.negativeInnovationSupport=0;
    assert(isfinite(one.r) && one.r>=0 && one.r<=1);
    if isfield(cfg,'captureIterationRecords') && cfg.captureIterationRecords
        assert(numel(weights)==2);
        stats.records(end+1,:)=[t,details.sourceIndices(1),keys(:,k)',one.mu{1}(1:2)', ...
            one.r,logistic(base),logistic(base+age),one.r,rec.spatialLogNormalizer,q,b,factors, ...
            rec.inputExistence,increments,stamps,age,correction,age-correction, ...
            gates,beta,boost,original(:)',negativeGates,scalarR,oldMean(1:2)', ...
            one.mu{1}(3:4)',one.Sigma{1}(find(tril(true(4))))',kept,alpha,logIntegral,allowed,aggregateFallback]; %#ok<AGROW>
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
