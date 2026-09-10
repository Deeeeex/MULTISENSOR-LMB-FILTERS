function checkInitialNegative()
model=generateMultisensorModel(2,[3,3],[.9,.9],[1,1],'GA','LBP');model.T=.1;
o=model.birthParameters(1);o.birthTime=1;o.birthLocation=1;o.r=.6;
o.numberOfGmComponents=1;o.w=1;o.mu={[1;0;0;0]};o.Sigma={diag([2,2,4,4])};
o.hasObservationLineage=true;o.lastDirectOpportunity=20;o.positiveConfirmation=false;
o.localLogOddsIncrement=.7;o.positiveInnovationSupport=.8;o.negativeInnovationSupport=.3;
o.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),o.mu{1},o.Sigma{1});
model.birthParameters=o;model.object=o([]);cfg=buildMixtureAwareKlaReferenceConfig();cfg.captureIterationRecords=true;
d=struct('eventType',[0,2],'sourceIndices',[1,2],'isStale',[false,false],'isSelf',[true,false],'currentTime',20);
p=o;p.mu={[-1;1;0;0]};p.r=.4;p.localLogOddsIncrement=-.3;
p.localSpatialLogRatio=gaussianLogRatio(zeros(4,1),diag([8,8,4,4]),p.mu{1},p.Sigma{1});
[original,stats]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[changed,logged,events]=removeInitialNegative(original,stats,20,1);
assert(events(7)<0 && changed.r>original.r && changed.r==logged.records(7) && changed.r==logged.records(10));
shift=log(changed.r)-log1p(-changed.r)-log(original.r)+log1p(-original.r);
assert(abs(shift+events(7))<1e-12);
assert(isequal(changed.mu,original.mu) && isequal(changed.Sigma,original.Sigma));
p.localLogOddsIncrement=.3;
[original,stats]=fuseGaussianEvidence({o,p},[.5,.5],model,d,cfg,'gaussian_evidence',20);
[unchanged,logged,events]=removeInitialNegative(original,stats,20,1);
assert(isequaln(unchanged,original) && isequaln(logged,stats) && events(7)==0);
[objects,logged,events]=removeInitialNegative(original([]),struct('records',zeros(0,60)),20,1);
assert(isempty(objects) && isempty(logged.records) && isempty(events));
fprintf('INITIAL NEGATIVE CHECK PASSED: exact original reconstruction, negative removal, positive-only identity, empty, spatial and metadata identity.\n');
end
