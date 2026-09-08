function support=negativeInnovationSupport(associationMass,pd)
% Missed branch times bounded contrast of its actual nominal likelihood.
assert(isequal(size(associationMass),size(pd)));
assert(all(isfinite(associationMass)) && all(associationMass>=0 & associationMass<=1));
assert(all(isfinite(pd)) && all(pd>=0 & pd<=1));
support=(1-associationMass).*pd./(2-pd);
assert(all(support>=0 & support<=1));
end
