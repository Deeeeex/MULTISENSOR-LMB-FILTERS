function checkSortedGmGroupingV290()
% One focused equivalence/performance check for the integration bottleneck.
n=300; m=cell(1,n); P=cell(1,n);
for j=1:n
    identity=mod(j,91);
    m{j}=[identity;sin(identity);0;0];
    P{j}=diag([1+identity/100,2,3,4]);
end
% Include zero/tiny weights, repeated kernels, and equal-weight tie handling.
w=mod(1:n,7)+1; w(1:3)=0; w(4)=1e-9; w=w/sum(w);
o=struct('numberOfGmComponents',n,'w',w,'mu',{m},'Sigma',{P},'r',.7);
options=struct('weightThreshold',.001,'maximumComponentCount',20);
started=tic;[old,oldD]=canonicalizeLmbGaussianMixtureRepresentation(o,options);oldTime=toc(started);
options.groupingMethod='sorted';
started=tic;[new,newD]=canonicalizeLmbGaussianMixtureRepresentation(o,options);newTime=toc(started);
assert(isequal(old,new) && isequal(oldD,newD));
options.maximumComponentCount=inf; options.weightThreshold=0;
fast=canonicalizeLmbGaussianMixtureRepresentation(o,options);
options.groupingMethod='pairwise'; slow=canonicalizeLmbGaussianMixtureRepresentation(o,options);
assert(isequal(fast,slow));
fprintf('V290 sorted grouping self-check PASS: exact output/diagnostic equality with duplicate, zero/tiny and tied weights, with and without reduction.\n');
fprintf('300-component fixture: pairwise %.4f s, sorted %.4f s, ratio %.2f. This is not an end-to-end runtime claim.\n', ...
    oldTime,newTime,oldTime/newTime);
end
