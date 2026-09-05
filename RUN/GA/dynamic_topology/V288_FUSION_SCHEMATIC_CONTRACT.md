# V288 fusion-rule schematic

Core conclusion: disagreeing single-Gaussian sources can make KLA suppress
existence while LMB-MIL preserves two spatial modes; neither operation alone
establishes which location is correct.

This is an analytic one-label illustration, not sampled tracking data. Both
inputs have existence 0.8, equal fusion weights, standard deviation 1 and
means `-d/2` and `d/2`. KLA has `eta=exp(-d^2/8)` and spatial density `N(0,1)`;
its existence is `0.8 eta / (0.2 + 0.8 eta)`. MIL has existence 0.8 and the
equal mixture of the inputs. Panel a uses d=6; panel b varies d from 0 to 6.

Archetype: quantitative analytic comparison, two complementary panels.
Panel a shows the spatial distributions; panel b shows the existence effect.
There is deliberately no target-truth marker, accuracy ranking, training set,
seed, uncertainty interval or inference test. The plotted curves are exact
for these specified inputs, without GM-power or truncation approximations.

Python/matplotlib only, as selected by the user. Final size 178 x 77 mm;
editable SVG text and PDF TrueType fonts, 600-dpi PNG preview, two numerical
CSV files and a JSON parameter manifest. Use labeled/line-style-separated
blue KLA, amber MIL and gray input curves. No external imagery or data.

Keep this explanatory figure in the experiment/baseline notes. It is not a
new empirical-result figure, nor a replacement for the approved Lark board.
The underlying KLA equation is already in the working manuscript; the MIL
rule is Gao et al., author version arXiv:1911.01083v1, Proposition 3.
