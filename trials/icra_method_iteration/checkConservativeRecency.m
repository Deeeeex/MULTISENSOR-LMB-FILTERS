function checkConservativeRecency()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'), ...
    fullfile(root,'trials','icra_reunion_fusion'),fullfile(root,'trials','icra_external_fusion'));
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.8;
o.numberOfGmComponents=1;o.w=1;o.mu={zeros(4,1)};o.Sigma={eye(4)};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
p=o;p.r=.2;p.mu={[2;0;0;0]};p.lastDirectOpportunity=1;
model.birthParameters=o;model.object=o([]);cfg=buildMixtureAwareKlaReferenceConfig();
details=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false], ...
    'isSelf',[true,false],'currentTime',20);
for r1=[.05,.2,.8,.95]
    for r2=[.05,.2,.8,.95]
        o.r=r1;p.r=r2;
        [a,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,'lineage',20);
        [e,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,'qualified_exist',20);
        [c,~]=fuseConservativeRecency({o,p},[.5,.5],model,details,cfg,'conservative_recency',20);
        assert(abs(c.r-min(a.r,e.r))<2e-12 && c.r<=a.r+1e-12 && c.r<=e.r);
        assert(isequal(c.mu,a.mu) && isequal(c.Sigma,a.Sigma));
    end
end
[single,~]=fuseConservativeRecency({o,p},[1,0],model,details,cfg,'conservative_recency',20);
assert(abs(single.r-o.r)<1e-12 && isequal(single.mu,o.mu));
p.lastDirectOpportunity=o.lastDirectOpportunity;
[same,~]=fuseConservativeRecency({o,p},[.5,.5],model,details,cfg,'conservative_recency',20);
[a,~]=fuseValidationInputs({o,p},[.5,.5],model,details,cfg,'lineage',20);
assert(abs(same.r-a.r)<1e-12);
% Visible absence remains an eligible censor; it is never a spatial input.
cfg.captureIterationRecords=true;
[missing,stats]=fuseConservativeRecency({o,model.object},[.5,.5],model,details,cfg,'conservative_recency',20);
assert(numel(missing)==1 && stats.observableAbsences==1 && isequal(missing.mu,o.mu));
assert(missing.r<=stats.records(8)+1e-12 && missing.r<=stats.records(9));
fprintf('CR CHECK PASSED: constrained minimum on 16 opposing/agreement inputs, unchanged spatial Gaussian, identity, equal ages, and visible-absence censor.\n');
end
