function [support,mass]=positiveInnovationSupport(W,ratios,opportunity,noMark)
% Bounded positive mark discrimination; missed/no-opportunity branches are zero.
ratios=reshape(ratios,1,[]);
assert(all(isfinite(ratios)) && all(ratios>0));
marks=max(0,tanh(.5*log(ratios)));
if noMark,marks=ones(size(ratios));end
support=directEvidenceCeilings(W,marks,opportunity);
mass=directEvidenceCeilings(W,ones(size(ratios)),opportunity);
assert(all(support>=0 & support<=mass+1e-14 & mass<=1));
end
