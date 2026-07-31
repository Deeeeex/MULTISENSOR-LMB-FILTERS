# X36 scale-risk v5 development

- Generated: `2026-07-31 17:33:57`
- Contract: `x36-scale-risk-gated-global-payload-v5-development-v1`
- Candidate run commit: `8ab71298843476112a020484a4809c22c864b902`
- Frozen baseline run commit: `55055e579ffaffa2cfe16a92146d430ac10b83ae`
- Scenario: `x36-clean-scale`
- Seeds: `[41 43 47 53 59]`
- Times: `[75 76 77 78 79 80 81 82 83]`
- Selection rule: `among-candidates-passing-hard-safety-five-percent-mean-gain-four-of-five-positive-seeds-two-percent-byte-saving-and-no-aggregate-worst-or-consensus-regression-select-the-largest-minimum-seed-gain-then-largest-mean-gain-then-largest-byte-saving-then-smallest-regret-token`
- Development gate passed: `0`
- Selected regret token: `none`

## Aggregate candidate comparison

| Token | Mean E-OSPA gain | Positive seeds | Minimum seed gain | Worst-node gain | Consensus gain | Byte saving | Hard safety | Tracking | Tail | Communication | Gate |
|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|:--:|:--:|:--:|
| 050 | 8.693% | 4/5 | -0.112% | 6.211% | -0.194% | 8.802% | 1 | 1 | 0 | 1 | 0 |
| 075 | 8.644% | 5/5 | 1.048% | 6.369% | -0.277% | 8.947% | 1 | 1 | 0 | 1 | 0 |

## Paired seed results

| Token | Seed | Baseline E-OSPA | Candidate E-OSPA | Gain | Worst baseline | Worst candidate | Consensus baseline | Consensus candidate | Byte delta | Safe |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 050 | 41 | 35.6940 | 34.5237 | 3.279% | 58.1282 | 54.9703 | 31.3851 | 33.2452 | -9.067% | 1 |
| 050 | 43 | 37.2676 | 34.4131 | 7.659% | 64.3206 | 61.7065 | 31.5488 | 31.3388 | -9.051% | 1 |
| 050 | 47 | 37.7154 | 28.9142 | 23.336% | 58.3589 | 48.2131 | 31.8801 | 29.2701 | -9.331% | 1 |
| 050 | 53 | 33.4706 | 30.6991 | 8.280% | 51.8831 | 48.9588 | 28.8628 | 29.8119 | -8.408% | 1 |
| 050 | 59 | 34.8402 | 34.8792 | -0.112% | 59.8278 | 60.5016 | 34.7222 | 35.0404 | -8.163% | 1 |
| 075 | 41 | 35.6940 | 35.0682 | 1.753% | 58.1282 | 55.4855 | 31.3851 | 33.1169 | -9.061% | 1 |
| 075 | 43 | 37.2676 | 34.4131 | 7.659% | 64.3206 | 61.7065 | 31.5488 | 31.3388 | -9.051% | 1 |
| 075 | 47 | 37.7154 | 28.9142 | 23.336% | 58.3589 | 48.2131 | 31.8801 | 29.2701 | -9.331% | 1 |
| 075 | 53 | 33.4706 | 30.6448 | 8.442% | 51.8831 | 47.9800 | 28.8628 | 30.1465 | -8.561% | 1 |
| 075 | 59 | 34.8402 | 34.4749 | 1.048% | 59.8278 | 60.5016 | 34.7222 | 34.9656 | -8.736% | 1 |

## Gate activity

| Token | Seed | Min objective advantage | Mean objective advantage | Floor | Fallbacks / step | Max fallbacks | Activation | Rejected / step | Global payload pass |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 050 | 41 | -0.3750 | 0.7176 | -0.4082 | 0.333 | 1 | 0.333 | 0.000 | 1.000 |
| 050 | 43 | -0.4000 | 0.6829 | -0.4082 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 050 | 47 | -0.3625 | 0.7854 | -0.4082 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 050 | 53 | -0.3625 | 0.6338 | -0.4082 | 0.222 | 1 | 0.222 | 0.000 | 1.000 |
| 050 | 59 | -0.4000 | 0.5567 | -0.4082 | 0.667 | 2 | 0.556 | 0.000 | 1.000 |
| 075 | 41 | -0.5125 | 0.6773 | -0.6124 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 075 | 43 | -0.4000 | 0.6829 | -0.6124 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 075 | 47 | -0.3625 | 0.7854 | -0.6124 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 075 | 53 | -0.5375 | 0.6229 | -0.6124 | 0.111 | 1 | 0.111 | 0.000 | 1.000 |
| 075 | 59 | -0.4750 | 0.5375 | -0.6124 | 0.444 | 1 | 0.444 | 0.000 | 1.000 |

## Interpretation boundary

These five X36 pairs were already opened by the failed v4 zero-shot test. They may select a v5 regret token, but cannot support an X36 confirmation claim. The v4 reference rows are reused from hash-checked immutable artifacts; only the new candidate arms were rerun from the same continuation caches and RNG offset. The gate uses current posterior and link features only, with no truth or future outcome. A passing token must be frozen before testing distinct X36 and larger-than-X36 pairs.

Protocol boundary: The five completed X36 zero-shot pairs are opened development evidence for v5. The regret token may be selected only on these pairs. Any X36 confirmation or larger-scale claim requires a frozen token and distinct preregistered scenario-seed pairs.
