function checkConditionalSpatialPoolV291()
% Focused analytical check of the changed operator, not tracking validation.
a=struct('birthTime',1,'birthLocation',1,'r',.2, ...
    'numberOfGmComponents',1,'w',1,'mu',{{zeros(4,1)}},'Sigma',{{eye(4)}});
b=a; b.r=.8; b.mu={[10;0;0;0]};
model=struct('object',a([]),'xDimension',4);
c=struct('lmbFusionRule','arithmetic-existence-geometric-spatial', ...
    'mixtureAwareMaxFusedComponents',8);
f=fuseLmbPosteriorsByLabel({a,b},[.25,.75],model,[],[],c);
assert(abs(f.r-.65)<1e-12);
assert(norm(f.mu{1}-[10*.6/.65;0;0;0])<1e-10);
assert(norm(f.Sigma{1}-eye(4),'fro')<1e-10);
mil=c; mil.lmbFusionRule='mil-common-label';
g=fuseLmbPosteriorsByLabel({a,b},[.25,.75],model,[],[],mil);
assert(abs(f.r-g.r)<1e-12 && g.numberOfGmComponents==2);
[f,d]=fuseLmbPosteriorsByLabel({a,a([])},[.25,.75],model,[],[],c);
assert(abs(f.r-.05)<1e-12 && d.missingSourceCount==1 && isequal(f.mu,a.mu));
b.birthLocation=2;
f=fuseLmbPosteriorsByLabel({a,b},[.25,.75],model,[],[],c);
assert(numel(f)==2 && norm([f.r]-[.05,.6])<1e-12);
a.numberOfGmComponents=2; a.w=[.4,.6];
a.mu={zeros(4,1),ones(4,1)}; a.Sigma={eye(4),2*eye(4)};
f=fuseLmbPosteriorsByLabel({a,a([])},[.25,.75],model,[],[],c);
g=fuseLmbPosteriorsByLabel({a,a([])},[.25,.75],model,[],[],mil);
assert(abs(f.r-g.r)<1e-12 && norm(f.w-g.w)<1e-12 && ...
    isequal(f.mu,g.mu) && isequal(f.Sigma,g.Sigma));
threw=false;
try
    fuseLmbPosteriorsByLabel({a,b},[.25,.75],model,[.5,.5],[],c);
catch err
    threw=strcmp(err.identifier,'LmbConditionalPool:IncompatibleOverride');
end
assert(threw);
fprintf('V291 formula self-check PASS: arithmetic existence, conditional geometric weights, zero extension, one-input mixture, override guard.\n');
end
