function [objects, stats] = splitLmbPredictionAtFovBoundary(objects, model, sensor, t, options)
% Refine a predicted GM near one sensor's FoV before the ordinary update.
% LeGrand & Ferrari, JAIF 17(2), 2022, Algorithm 1 and (20)--(32).
% This is approximate density refinement, not new evidence or exact covariance
% preservation. Labels and existence are unchanged; all children are retained.
minimumWeight = getOption(options,'minimumWeight',0.01);
gridRadius = getOption(options,'gridRadius',3);
gridCount = getOption(options,'gridPointCount',15);
assert(minimumWeight>0 && minimumWeight<1 && gridRadius>0 && ...
    gridCount>=3 && gridCount==round(gridCount) && mod(gridCount,2)==1);
stats = struct('inputComponents',0,'outputComponents',0,'splitOperations',0, ...
    'splitLabels',0,'maximumComponentsPerLabel',0,'maximumDepth',0, ...
    'terminalBoundaryMass',0,'elapsedSeconds',0);
started=tic;
g = geometry(model,sensor,t);
axisPoints=linspace(-gridRadius,gridRadius,gridCount);
[u,v]=ndgrid(axisPoints,axisPoints); [ix,iy]=ndgrid(1:gridCount,1:gridCount);
keep=u.^2+v.^2<=gridRadius^2+1e-12;
grid=[u(keep)';v(keep)']; indices=[ix(keep)';iy(keep)'];
% Published Table I, R=4, lambda=0.001; not moment-renormalized here.
splitWeights=[0.10766586425362,0.39233413574638,0.39233413574638,0.10766586425362];
splitMeans=[-1.42237156603631,-0.47412385534547,0.47412385534547,1.42237156603631];
splitSigma=0.58160633157686;
for i=1:numel(objects)
    original=objects(i); weights=original.w(:)'; means=original.mu;
    covariances=original.Sigma; depths=zeros(size(weights)); next=1;
    outputWeights=[]; outputMeans={}; outputCovariances={}; didSplit=false;
    stats.inputComponents=stats.inputComponents+numel(weights);
    while next<=numel(weights)
        w=weights(next); m=means{next}; P=covariances{next}; depth=depths(next);
        P=(P+P')/2;
        [V,D]=eig(P(1:2,1:2)); [e,order]=sort(diag(D),'descend'); V=V(:,order);
        assert(all(e>0),'Predicted position covariance must be positive definite.');
        points=bsxfun(@plus,m(1:2),V*diag(sqrt(e))*grid);
        inside=inFov(points,g); boundary=any(inside) && ~all(inside);
        if ~boundary || w<minimumWeight
            outputWeights(end+1)=w; outputMeans{end+1}=m;
            outputCovariances{end+1}=covariances{next};
            if boundary, stats.terminalBoundaryMass=stats.terminalBoundaryMass+w; end
        else
            % Most consistent grid planes; largest position eigenvalue breaks ties.
            scores=zeros(1,2);
            for d=1:2
                for plane=1:gridCount
                    values=inside(indices(d,:)==plane);
                    scores(d)=scores(d)+(all(values) || ~any(values));
                end
            end
            [~,positionDirection]=max(scores);
            [fullV,fullD]=eig(P); [fullE,order]=sort(diag(fullD),'descend');
            fullV=fullV(:,order); assert(all(fullE>0));
            alignment=abs(V(:,positionDirection)'*fullV(1:2,:));
            [~,direction]=max(alignment); vector=fullV(:,direction);
            [~,pivot]=max(abs(vector)); if vector(pivot)<0, vector=-vector; end
            childP=P-(1-splitSigma^2)*fullE(direction)*(vector*vector');
            childP=(childP+childP')/2;
            for child=1:4
                weights(end+1)=w*splitWeights(child);
                means{end+1}=m+sqrt(fullE(direction))*splitMeans(child)*vector;
                covariances{end+1}=childP; depths(end+1)=depth+1;
            end
            stats.splitOperations=stats.splitOperations+1; didSplit=true;
            stats.maximumDepth=max(stats.maximumDepth,depth+1);
        end
        next=next+1;
    end
    if didSplit
        objects(i).numberOfGmComponents=numel(outputWeights);
        objects(i).w=outputWeights;
        objects(i).mu=outputMeans; objects(i).Sigma=outputCovariances;
        stats.splitLabels=stats.splitLabels+1;
    end
    stats.outputComponents=stats.outputComponents+objects(i).numberOfGmComponents;
    stats.maximumComponentsPerLabel=max(stats.maximumComponentsPerLabel, ...
        objects(i).numberOfGmComponents);
end
stats.elapsedSeconds=toc(started);
end

function g=geometry(model,s,t)
assert(model.sensorMotionEnabled && model.sensorFovEnabled, ...
    'FoV splitting requires an enabled geometric FoV.');
trajectory=model.sensorTrajectories{s};
g.position=trajectory(1:2,min(t,size(trajectory,2)));
heading=model.sensorFovHeadingRad(s,t);
assert(isfinite(heading),'FoV splitting requires an explicit current heading.');
g.heading=[cos(heading);sin(heading)];
g.range=sensorValue(model.sensorFovRange,s);
g.halfAngle=sensorValue(model.sensorFovHalfAngleDeg,s);
end

function yes=inFov(x,g)
d=bsxfun(@minus,x,g.position); range=sqrt(sum(d.^2,1));
angle=acosd(min(max((g.heading'*d)./max(range,realmin),-1),1));
angle(range<=1e-9)=0;
yes=range<=g.range & angle<=g.halfAngle;
end

function v=sensorValue(a,s)
if isscalar(a),v=a;else,v=a(s);end
end

function v=getOption(o,name,fallback)
if isfield(o,name),v=o.(name);else,v=fallback;end
end
