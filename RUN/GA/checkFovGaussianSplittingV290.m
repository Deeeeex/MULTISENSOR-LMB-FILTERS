function checkFovGaussianSplittingV290()
% Bounded formula/branch check, not performance validation.
model=struct('sensorMotionEnabled',true,'sensorFovEnabled',true, ...
    'sensorTrajectories',{{zeros(4,1)}},'sensorFovHeadingRad',0, ...
    'sensorFovRange',inf,'sensorFovHalfAngleDeg',90, ...
    'detectionProbability',0.8,'Q',{{eye(2)}},'sensorQuality',struct('enabled',false));
P=[1,0,.3,0;0,1,0,.2;.3,0,.5,0;0,.2,0,.5];
o=struct('birthTime',1,'birthLocation',1,'r',.7,'numberOfGmComponents',1, ...
    'w',1,'mu',{{[.2;0;0;0]}},'Sigma',{{P}});
options=struct('minimumWeight',.01,'gridRadius',3,'gridPointCount',15);
[q,stats]=splitLmbPredictionAtFovBoundary(o,model,1,1,options);
assert(q.r==o.r && q.birthTime==o.birthTime && q.birthLocation==o.birthLocation);
assert(abs(sum(q.w)-1)<1e-12 && q.numberOfGmComponents>1);
mean=zeros(4,1); integratedPd=0; covariance=zeros(4);
for j=1:q.numberOfGmComponents
    mean=mean+q.w(j)*q.mu{j};
    integratedPd=integratedPd+q.w(j)*evaluateSensorQuality(model,1,q.mu{j},1);
    d=q.mu{j}-o.mu{1}; covariance=covariance+q.w(j)*(q.Sigma{j}+d*d');
    assert(all(eig(q.Sigma{j})>0));
end
assert(norm(mean-o.mu{1})<1e-12);
exactPd=.8*.5*erfc(-.2/sqrt(2)); oldPd=.8;
assert(abs(integratedPd-exactPd)<.5*abs(oldPd-exactPd));
assert(abs(covariance(1,3))>0.1,'Full-state correlation was lost.');
far=o; far.mu={[10;0;0;0]};
unchanged=splitLmbPredictionAtFovBoundary(far,model,1,1,options);
assert(isequal(unchanged,far),'Non-boundary object should be an exact bypass.');
fprintf('V290 formula self-check PASS: mass/mean/labels/r, positive covariance, full-state correlation, exact non-boundary bypass.\n');
fprintf('Half-plane pD: old %.9f, split %.9f, exact %.9f; %d components, depth %d, relative covariance change %.6f.\n', ...
    oldPd,integratedPd,exactPd,q.numberOfGmComponents,stats.maximumDepth,norm(covariance-P,'fro')/norm(P,'fro'));
end
