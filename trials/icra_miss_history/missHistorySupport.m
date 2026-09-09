function [support,after,effective,discount]=missHistorySupport(mass,pd,before,mode)
% Extra negative evidence tempered by source-private consecutive misses.
assert(isequal(size(mass),size(pd),size(before)));
assert(all(isfinite(mass)) && all(mass>=0 & mass<=1));
assert(all(isfinite(pd)) && all(pd>=0 & pd<1));
assert(all(isfinite(before)) && all(before>=0));
assert(ismember(mode,{'original','history','half'}));
opportunity=pd>0;after=zeros(size(mass));effective=zeros(size(mass));
discount=ones(size(mass));
after(opportunity)=(1-mass(opportunity)).*(before(opportunity)+1);
effective(opportunity)=pd(opportunity);
if strcmp(mode,'history')
    changed=opportunity & before>0;
    alpha=pd(changed)./(1-pd(changed));
    effective(changed)=alpha./(alpha+1+before(changed));
    discount(changed)=log1p(-effective(changed))./log1p(-pd(changed));
elseif strcmp(mode,'half')
    discount(opportunity)=.5;
end
support=(1-mass).*pd./(2-pd).*discount;
assert(all(discount>=0 & discount<=1) && all(support>=0 & support<=1));
end

