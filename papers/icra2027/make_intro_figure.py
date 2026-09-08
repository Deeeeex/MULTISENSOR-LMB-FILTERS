"""Draw the shared-history/current-evidence intuition as an editable schematic."""
import json
from pathlib import Path

import matplotlib as mpl
mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Circle
from make_gaussian_figures import export, INK, TEAL, BLUE

HERE = Path(__file__).resolve().parent
mpl.rcParams.update({'mathtext.fontset': 'stix', 'svg.hashsalt': 'icra-gce-intro'})
GRAY, LIGHT = '#8998a1', '#e8edf0'


def main():
    fig = plt.figure(figsize=(89/25.4, 73/25.4), dpi=300)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set(xlim=(0, 89), ylim=(0, 73))
    ax.axis('off')

    def text(x, y, value, size=7.7, color=INK, **kwargs):
        return ax.text(x, y, value, ha=kwargs.pop('ha', 'center'), va='center',
                       fontsize=size, color=color, **kwargs)

    def arrow(a, b, color=GRAY, dash=False, curve=0):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=6,
                                    lw=.75, color=color, shrinkA=0, shrinkB=0,
                                    linestyle=(0, (2.6, 2)) if dash else '-',
                                    connectionstyle=f'arc3,rad={curve}'))

    def vehicle(x, y):
        for dx in [-4.5, 4.5]:
            for dy in [-2.0, 2.0]:
                ax.add_patch(FancyBboxPatch((x+dx-.75, y+dy-1.1), 1.5, 2.2,
                                            boxstyle='round,pad=0,rounding_size=.3',
                                            facecolor=INK, edgecolor='none'))
        ax.add_patch(FancyBboxPatch((x-4.5, y-3.7), 9, 7.4,
                                    boxstyle='round,pad=0,rounding_size=1.5',
                                    facecolor='#f0f4f5', edgecolor=INK, linewidth=.8))
        ax.add_patch(FancyBboxPatch((x-2.9, y-.2), 5.8, 2.5,
                                    boxstyle='round,pad=0,rounding_size=.5',
                                    facecolor='#c0d1db', edgecolor='none'))
        ax.add_patch(Circle((x, y-2), .7, facecolor=BLUE, edgecolor='none'))

    # Shared history is inherited by both local trackers; sensing is current.
    text(44.5, 69.5, 'Shared past', size=8.2, color=GRAY, weight='bold')
    arrow((34, 66.9), (20, 61), dash=True, curve=.14)
    arrow((55, 66.9), (69, 61), dash=True, curve=-.14)
    vehicle(20, 56.5)
    vehicle(69, 56.5)
    text(20, 49.7, 'Robot A', size=7.4)
    text(69, 49.7, 'Robot B', size=7.4)
    ax.plot([42.5, 44.5, 46.5, 44.5, 42.5], [57, 59, 57, 55, 57], color=BLUE, lw=1.1)
    arrow((25, 56.5), (41, 57), color=BLUE)
    arrow((64, 56.5), (48, 57), color=BLUE)
    text(44.5, 51.7, 'Current sensing', size=7.3, color=BLUE)
    text(20, 43.4, r'$f_1^+\propto f^-\ell_1$', size=9)
    text(69, 43.4, r'$f_2^+\propto f^-\ell_2$', size=9)
    arrow((25, 40.5), (39, 37.4), color=GRAY)
    arrow((64, 40.5), (50, 37.4), color=GRAY)

    # The same inherited prior survives; only current likelihood exponents change.
    ax.add_patch(FancyBboxPatch((2, 29.0), 85, 8.2,
                                boxstyle='round,pad=0,rounding_size=1.2',
                                facecolor='#f2f5f6', edgecolor='none'))
    text(5, 33.2, 'Pool', ha='left', weight='bold')
    text(50, 33.1, r'$f^-\,\ell_1^{1/2}\,\ell_2^{1/2}$', size=11)
    arrow((44.5, 28.5), (44.5, 24.2), color=TEAL)
    text(47.5, 26.5, 'admit current ratios', size=7.1, color=TEAL, ha='left')
    ax.add_patch(FancyBboxPatch((2, 14.8), 85, 9.0,
                                boxstyle='round,pad=0,rounding_size=1.2',
                                facecolor='#eaf5f1', edgecolor='none'))
    text(5, 19.4, 'GCE', ha='left', color=TEAL, weight='bold')
    text(49, 20.0, r'$f^-\,\ell_1^{\omega_1}\,\ell_2^{\omega_2}$', size=11, color=TEAL)
    text(77, 17.0, r'$\frac{1}{2}\leq\omega_j\leq1$', size=7.7, color=TEAL)
    arrow((34, 14.4), (23, 9.8), color=TEAL)
    arrow((55, 14.4), (66, 9.8), color=TEAL)
    text(21, 7.1, 'Spatial density', size=7.6)
    text(68, 7.1, 'Existence', size=7.6)
    arrow((38, 7.0), (55, 7.0), color=TEAL)
    text(46.5, 3.4, 'shared normalizer', size=7.0, color=TEAL)
    source = dict(kind='qualitative_shared_prior_schematic', empirical_data=False, check_text_collisions=True,
                  archetype='schematic-led composite', width_mm=89, height_mm=73,
                  assumption='Two sources with common prior, equal base weights, exact local likelihoods.',
                  pool='f_minus * likelihood_1**0.5 * likelihood_2**0.5',
                  corrected='f_minus * likelihood_1**omega_1 * likelihood_2**omega_2',
                  exponents='omega_j = 0.5 + admitted_kappa_j in [0.5, 1]',
                  message='Admitted current ratios modify spatial density and existence through one normalizer.',
                  full_restoration='Exact shared-prior Bayes requires both admitted gates equal to one.')
    export(fig, 'intro', source)


if __name__ == '__main__':
    main()
