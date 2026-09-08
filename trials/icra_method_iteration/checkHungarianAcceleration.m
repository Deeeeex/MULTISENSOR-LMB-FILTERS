function checkHungarianAcceleration()
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));
addpath(fullfile(root,'common'));original=@Hungarian;
assert(strcmp(functions(original).file,fullfile(root,'common','Hungarian.m')));
runtime=fullfile(out,'runtime');addpath(runtime);clean=onCleanup(@()rmpath(runtime)); %#ok<NASGU>
fast=@Hungarian;assert(strcmp(functions(fast).file,fullfile(runtime,'Hungarian.m')));
matrices={[],zeros(1),zeros(3,5),ones(5,3),[4,1,3;2,0,5;3,2,2], ...
    [1,inf,2;inf,3,2],[inf,inf;2,1],[1,1,inf;1,1,inf]};
rng(2109,'twister');
for n=1:12
    matrices{end+1}=randi(7,n,15-n); %#ok<AGROW>
    matrices{end+1}=rand(n,15-n); %#ok<AGROW>
end
for n=[7,17,31]
    m=n+3;a=1e8*ones(n+m);a(1:n,1:m)=50*rand(n,m);
    for i=1:n,a(i,m+i)=50;end
    for j=1:m,a(n+j,j)=50;end
    a(n+1:end,m+1:end)=0;matrices{end+1}=a; %#ok<AGROW>
end
for k=1:numel(matrices)
    [a,ca]=original(matrices{k});[b,cb]=fast(matrices{k});
    assert(isequal(a,b) && isequal(ca,cb),'Runtime:AssignmentMismatch','Original tie break changed on case %d.',k);
    if k==5 || k==numel(matrices),fprintf('Assignment parity %d/%d passed.\n',k,numel(matrices));end
end
[matching,cost]=fast([4,1,3;2,0,5;3,2,2]);
assert(cost==5 && isequal(matching,[0,1,0;1,0,0;0,0,1]));
fprintf('HUNGARIAN ACCELERATION PASS: %d exact matching/cost comparisons, rectangular and tied matrices, Inf masks, and augmented private dummy slots.\n',numel(matrices));
end
