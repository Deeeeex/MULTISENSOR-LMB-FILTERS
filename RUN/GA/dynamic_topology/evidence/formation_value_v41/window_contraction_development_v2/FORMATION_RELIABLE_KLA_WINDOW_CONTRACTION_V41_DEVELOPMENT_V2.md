# V41 reliable KLA window-contraction development probe

- Contract: `formation-reliable-kla-window-contraction-development-result-v2`
- Protocol SHA-256: `88354e5ab93b168d7da617f9dbfed5e5e9f16cb0d6cccae0f65b68126832662a`
- Generation commit: `4c23802c4e6a68650904db1f4ef6ba2327b3a6c7` (dirty: `0`)
- MAT artifact: `RUN/GA/dynamic_topology/evidence/formation_value_v41/window_contraction_development_v2/FORMATION_RELIABLE_KLA_WINDOW_CONTRACTION_V41_DEVELOPMENT_V2.mat`
- Result SHA-256: `df6875d49e6524e97fd734fd11ecc23e5362275961202c2457e63293b3b3a2f9`

This is a geometry-only development diagnostic. It materializes the complete planned sensor geometry and link-probability schedule before slicing the focus-start page, so it does not satisfy the formal runtime observable-input boundary. It calls no target, measurement or ground-truth generator and scores no tracking outcome.

## Scale calibration

| Preset | Seed | N | N-1 | Event q beta | Event E[delta] upper | rho at N-1 | first rho<1 | Calibration pass | Target H | Evaluation H | rho at evaluation H |
|:--|--:|--:|--:|--:|--:|--:|--:|:--:|--:|--:|--:|
| m24-formation-fov | 41 | 24 | 23 | 5.056e-06 | 0.9999949435 | 1.1964529343 | 28 | 1 | 31 | 31 | 0.8853691526 |
| m24-formation-fov | 43 | 24 | 23 | 5.05e-06 | 0.9999949497 | 1.1964027254 | 28 | 1 | 31 | 31 | 0.8853129426 |
| m24-formation-fov | 47 | 24 | 23 | 5.06e-06 | 0.9999949403 | 1.1959904658 | 28 | 1 | 31 | 31 | 0.8848966574 |
| m24-formation-fov | 53 | 24 | 23 | 5.056e-06 | 0.9999949445 | 1.1963386204 | 28 | 1 | 31 | 31 | 0.8852484418 |
| x36-formation-fov | 41 | 36 | 35 | 9.212e-09 | 0.9999999908 | 2.0105453427 | 89 | 1 | 95 | 95 | 0.8890800193 |
| x36-formation-fov | 43 | 36 | 35 | 9.231e-09 | 0.9999999908 | 2.0103008525 | 89 | 1 | 95 | 95 | 0.8886718989 |
| x36-formation-fov | 47 | 36 | 35 | 9.269e-09 | 0.9999999907 | 2.0092553493 | 89 | 1 | 95 | 95 | 0.8868237251 |
| x36-formation-fov | 53 | 36 | 35 | 9.232e-09 | 0.9999999908 | 2.0102576118 | 89 | 1 | 95 | 95 | 0.8885069538 |

## Adaptive-window route comparison

| Preset | Seed | N | H | Action | Missing mode | Operational? | Messages | Window messages | Delta window messages | Cross mass | MS rho | rho/ref | delta rho | Strict rho<1 |
|:--|--:|--:|--:|:--|:--|:--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| m24-formation-fov | 41 | 24 | 31 | reference | renormalize | yes | 48 | 1488 | +0 | 0.4 | 0.8853691526 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 41 | 24 | 31 | reference | self | no | 48 | 1488 | +0 | 0.4 | 0.9310619274 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f1 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8983245172 | 1.014633 | +0.012955 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f1 | self | no | 46 | 1486 | -2 | 0.3 | 0.9438051245 | 1.013687 | +0.012743 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f2 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9006583245 | 1.017269 | +0.015289 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f2 | self | no | 46 | 1486 | -2 | 0.3 | 0.9466390232 | 1.016730 | +0.015577 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f3 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9010545624 | 1.017716 | +0.015685 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f3 | self | no | 46 | 1486 | -2 | 0.3 | 0.9470228896 | 1.017143 | +0.015961 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f4 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8979594277 | 1.014220 | +0.012590 | 1 |
| m24-formation-fov | 41 | 24 | 31 | suspend-input-bundle-f4 | self | no | 46 | 1486 | -2 | 0.3 | 0.9434576325 | 1.013314 | +0.012396 | 1 |
| m24-formation-fov | 43 | 24 | 31 | reference | renormalize | yes | 48 | 1488 | +0 | 0.4 | 0.8853129426 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 43 | 24 | 31 | reference | self | no | 48 | 1488 | +0 | 0.4 | 0.9310050988 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f1 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8983271133 | 1.014700 | +0.013014 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f1 | self | no | 46 | 1486 | -2 | 0.3 | 0.9438051983 | 1.013749 | +0.012800 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f2 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9005199115 | 1.017177 | +0.015207 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f2 | self | no | 46 | 1486 | -2 | 0.3 | 0.9465016513 | 1.016645 | +0.015497 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f3 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9010741483 | 1.017803 | +0.015761 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f3 | self | no | 46 | 1486 | -2 | 0.3 | 0.9470405008 | 1.017224 | +0.016035 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f4 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8978535855 | 1.014165 | +0.012541 | 1 |
| m24-formation-fov | 43 | 24 | 31 | suspend-input-bundle-f4 | self | no | 46 | 1486 | -2 | 0.3 | 0.9433529282 | 1.013263 | +0.012348 | 1 |
| m24-formation-fov | 47 | 24 | 31 | reference | renormalize | yes | 48 | 1488 | +0 | 0.4 | 0.8848966574 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 47 | 24 | 31 | reference | self | no | 48 | 1488 | +0 | 0.4 | 0.9305957425 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f1 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8978522893 | 1.014641 | +0.012956 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f1 | self | no | 46 | 1486 | -2 | 0.3 | 0.9433396462 | 1.013694 | +0.012744 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f2 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9001910499 | 1.017284 | +0.015294 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f2 | self | no | 46 | 1486 | -2 | 0.3 | 0.9461782314 | 1.016745 | +0.015582 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f3 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9005601892 | 1.017701 | +0.015664 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f3 | self | no | 46 | 1486 | -2 | 0.3 | 0.9465356154 | 1.017129 | +0.015940 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f4 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8975165975 | 1.014261 | +0.012620 | 1 |
| m24-formation-fov | 47 | 24 | 31 | suspend-input-bundle-f4 | self | no | 46 | 1486 | -2 | 0.3 | 0.9430201403 | 1.013351 | +0.012424 | 1 |
| m24-formation-fov | 53 | 24 | 31 | reference | renormalize | yes | 48 | 1488 | +0 | 0.4 | 0.8852484418 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 53 | 24 | 31 | reference | self | no | 48 | 1488 | +0 | 0.4 | 0.9309420909 | 1.000000 | +0.000000 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f1 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8982671875 | 1.014706 | +0.013019 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f1 | self | no | 46 | 1486 | -2 | 0.3 | 0.9437465753 | 1.013754 | +0.012804 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f2 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9004473176 | 1.017169 | +0.015199 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f2 | self | no | 46 | 1486 | -2 | 0.3 | 0.9464307162 | 1.016638 | +0.015489 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f3 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.9010188036 | 1.017815 | +0.015770 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f3 | self | no | 46 | 1486 | -2 | 0.3 | 0.9469864904 | 1.017235 | +0.016044 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f4 | renormalize | yes | 46 | 1486 | -2 | 0.3 | 0.8977801896 | 1.014156 | +0.012532 | 1 |
| m24-formation-fov | 53 | 24 | 31 | suspend-input-bundle-f4 | self | no | 46 | 1486 | -2 | 0.3 | 0.9432813842 | 1.013255 | +0.012339 | 1 |
| x36-formation-fov | 41 | 36 | 95 | reference | renormalize | yes | 72 | 6840 | +0 | 0.6 | 0.8890800193 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 41 | 36 | 95 | reference | self | no | 72 | 6840 | +0 | 0.6 | 0.9506116871 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f1 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8964912861 | 1.008336 | +0.007411 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f1 | self | no | 70 | 6838 | -2 | 0.5 | 0.9580438862 | 1.007818 | +0.007432 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f2 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8892013042 | 1.000136 | +0.000121 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f2 | self | no | 70 | 6838 | -2 | 0.5 | 0.9507516194 | 1.000147 | +0.000140 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f3 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8900117663 | 1.001048 | +0.000932 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f3 | self | no | 70 | 6838 | -2 | 0.5 | 0.9516609054 | 1.001104 | +0.001049 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f4 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8957879081 | 1.007545 | +0.006708 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f4 | self | no | 70 | 6838 | -2 | 0.5 | 0.9577377624 | 1.007496 | +0.007126 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f5 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8985061040 | 1.010602 | +0.009426 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f5 | self | no | 70 | 6838 | -2 | 0.5 | 0.9599910841 | 1.009867 | +0.009379 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f6 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8846165630 | 0.994980 | -0.004463 | 1 |
| x36-formation-fov | 41 | 36 | 95 | suspend-input-bundle-f6 | self | no | 70 | 6838 | -2 | 0.5 | 0.9461205914 | 0.995276 | -0.004491 | 1 |
| x36-formation-fov | 43 | 36 | 95 | reference | renormalize | yes | 72 | 6840 | +0 | 0.6 | 0.8886718989 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 43 | 36 | 95 | reference | self | no | 72 | 6840 | +0 | 0.6 | 0.9502105116 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f1 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8961113276 | 1.008371 | +0.007439 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f1 | self | no | 70 | 6838 | -2 | 0.5 | 0.9576709434 | 1.007851 | +0.007460 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f2 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8887929301 | 1.000136 | +0.000121 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f2 | self | no | 70 | 6838 | -2 | 0.5 | 0.9503501771 | 1.000147 | +0.000140 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f3 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8896060003 | 1.001051 | +0.000934 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f3 | self | no | 70 | 6838 | -2 | 0.5 | 0.9512622155 | 1.001107 | +0.001052 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f4 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8953744341 | 1.007542 | +0.006703 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f4 | self | no | 70 | 6838 | -2 | 0.5 | 0.9573309362 | 1.007494 | +0.007120 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f5 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8980931753 | 1.010602 | +0.009421 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f5 | self | no | 70 | 6838 | -2 | 0.5 | 0.9595854268 | 1.009866 | +0.009375 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f6 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8841950090 | 0.994962 | -0.004477 | 1 |
| x36-formation-fov | 43 | 36 | 95 | suspend-input-bundle-f6 | self | no | 70 | 6838 | -2 | 0.5 | 0.9457056801 | 0.995259 | -0.004505 | 1 |
| x36-formation-fov | 47 | 36 | 95 | reference | renormalize | yes | 72 | 6840 | +0 | 0.6 | 0.8868237251 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 47 | 36 | 95 | reference | self | no | 72 | 6840 | +0 | 0.6 | 0.9483945400 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f1 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8942544358 | 1.008379 | +0.007431 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f1 | self | no | 70 | 6838 | -2 | 0.5 | 0.9558469265 | 1.007858 | +0.007452 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f2 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8869448749 | 1.000137 | +0.000121 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f2 | self | no | 70 | 6838 | -2 | 0.5 | 0.9485343320 | 1.000147 | +0.000140 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f3 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8877525879 | 1.001047 | +0.000929 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f3 | self | no | 70 | 6838 | -2 | 0.5 | 0.9494407684 | 1.001103 | +0.001046 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f4 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8935115693 | 1.007541 | +0.006688 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f4 | self | no | 70 | 6838 | -2 | 0.5 | 0.9555010247 | 1.007493 | +0.007106 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f5 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8962543019 | 1.010634 | +0.009431 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f5 | self | no | 70 | 6838 | -2 | 0.5 | 0.9577796382 | 1.009896 | +0.009385 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f6 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8823587090 | 0.994965 | -0.004465 | 1 |
| x36-formation-fov | 47 | 36 | 95 | suspend-input-bundle-f6 | self | no | 70 | 6838 | -2 | 0.5 | 0.9439009726 | 0.995262 | -0.004494 | 1 |
| x36-formation-fov | 53 | 36 | 95 | reference | renormalize | yes | 72 | 6840 | +0 | 0.6 | 0.8885069538 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 53 | 36 | 95 | reference | self | no | 72 | 6840 | +0 | 0.6 | 0.9500521412 | 1.000000 | +0.000000 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f1 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8959532131 | 1.008381 | +0.007446 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f1 | self | no | 70 | 6838 | -2 | 0.5 | 0.9575191710 | 1.007860 | +0.007467 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f2 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8886277290 | 1.000136 | +0.000121 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f2 | self | no | 70 | 6838 | -2 | 0.5 | 0.9501915311 | 1.000147 | +0.000139 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f3 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8894423214 | 1.001053 | +0.000935 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f3 | self | no | 70 | 6838 | -2 | 0.5 | 0.9511051942 | 1.001108 | +0.001053 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f4 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8952118039 | 1.007546 | +0.006705 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f4 | self | no | 70 | 6838 | -2 | 0.5 | 0.9571747821 | 1.007497 | +0.007123 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f5 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8979218045 | 1.010596 | +0.009415 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f5 | self | no | 70 | 6838 | -2 | 0.5 | 0.9594206712 | 1.009861 | +0.009369 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f6 | renormalize | yes | 70 | 6838 | -2 | 0.5 | 0.8840257281 | 0.994956 | -0.004481 | 1 |
| x36-formation-fov | 53 | 36 | 95 | suspend-input-bundle-f6 | self | no | 70 | 6838 | -2 | 0.5 | 0.9455427563 | 0.995254 | -0.004509 | 1 |

## Claim boundary

- `formalRuntimeObservableBoundaryPassed=false`; this artifact is not a policy permit.
- H counts successive online fusion updates (one per tracking step), not extra within-step rounds. The current reliability page is repeated as a causal development approximation.
- The bound concerns propagation of existing exact KLA set-density log-ratio disagreement under common support.
- Marginal existence log odds, local Bayes updates, label loss, mixture approximation and pruning require disturbance control.
- M24/X36 tracking and validation remain unauthorized.
