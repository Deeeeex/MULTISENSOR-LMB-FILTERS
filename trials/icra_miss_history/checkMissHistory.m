function checkMissHistory()
p=.9;h=0;previous=1;cumulative=0;alpha=p/(1-p);
for count=0:19
    [g,next,pe,rho]=missHistorySupport(0,p,h,'history');
    expected=(p/(1-p))/(p/(1-p)+1+count);
    assert(abs(pe-expected)<1e-14 && abs(next-count-1)<1e-14);
    assert(rho<=previous && abs(g-(p/(2-p))*rho)<1e-14);
    if count==0,assert(rho==1 && g==negativeInnovationSupport(0,p));end
    cumulative=cumulative+rho*log1p(-p);
    assert(abs(cumulative-(betaln(alpha,count+2)-betaln(alpha,1)))<1e-12);
    h=next;previous=rho;
end
[g,h]=missHistorySupport(1,p,h,'history');assert(g==0 && h==0);
[~,~,~,rho]=missHistorySupport(0,p,h,'history');assert(rho==1);
[~,h]=missHistorySupport(.5,p,2,'history');assert(h==1.5);
[g,h,pe,rho]=missHistorySupport(0,0,10,'history');assert(g==0 && h==0 && pe==0 && rho==1);
for mode={'original','half'}
    [g,~,~,rho]=missHistorySupport([0,.4,1],[p,p,p],[0,7,100],mode{1});
    scale=1;if strcmp(mode{1},'half'),scale=.5;end
    assert(all(rho==scale));
    assert(isequal(g,negativeInnovationSupport([0,.4,1],[p,p,p])*scale));
end
[g,h,pe,rho]=missHistorySupport([],[],[],'history');assert(isempty(g)&&isempty(h)&&isempty(pe)&&isempty(rho));
fprintf('MISS HISTORY CHECK PASSED\n');
end
