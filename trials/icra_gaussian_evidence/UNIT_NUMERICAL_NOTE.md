# Single-source fixture precision, before any tracking

The first unit check passed the centralized Bernoulli/Gaussian fixtures, then
failed an exact mu equality against the raw input in the single-source case.
The measured probability difference was 2.6367796834847468e-15, mean difference
1.7763568394002505e-15, and covariance difference zero. The new and original
asymmetric implementations were exactly identical in probability, mean and
covariance. This is existing spatial normalization arithmetic, not a new
tracking difference.

The fixture now checks numerical identity to the input within 1e-12 and exact
identity to the original implementation. No production function was changed
for this repair. The initial fixture is preserved under initial_unit/ and
both failed logs are retained. No trajectory, source freeze or outcome was
produced before this correction.
