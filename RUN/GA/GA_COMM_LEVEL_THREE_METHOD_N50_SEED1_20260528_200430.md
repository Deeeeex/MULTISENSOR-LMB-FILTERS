# Communication-Level Three-Method Comparison

- Trials: 50
- baseSeed: 1 (fixed=1)
- trialSeeds: [2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51]
- CSV: `C:\Users\Deeee\Desktop\Projects\MULTISENSOR-LMB-FILTERS\RUN\GA\GA_COMM_LEVEL_THREE_METHOD_N50_SEED1_latest.csv`

## Network Metrics

### OSPA consensus error

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 1.627862 | 0.040867 | [1.616303, 1.639421] | 50 |
| 0 | none | Balanced mode | 1.423437 | 0.035910 | [1.413280, 1.433594] | 50 |
| 0 | none | Cardinality-critical mode | 1.431515 | 0.034624 | [1.421722, 1.441308] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 1.701150 | 0.042565 | [1.689111, 1.713189] | 50 |
| 1 | bandwidth cap | Balanced mode | 1.481795 | 0.039483 | [1.470627, 1.492963] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 1.472859 | 0.035932 | [1.462696, 1.483022] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 2.383288 | 0.176309 | [2.333420, 2.433156] | 50 |
| 2 | tiered link loss | Balanced mode | 1.760324 | 0.075878 | [1.738862, 1.781785] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 1.669298 | 0.046269 | [1.656211, 1.682385] | 50 |
| 3 | node outage | Fixed Metropolis | 2.655978 | 0.235892 | [2.589258, 2.722699] | 50 |
| 3 | node outage | Balanced mode | 1.807677 | 0.079978 | [1.785056, 1.830298] | 50 |
| 3 | node outage | Cardinality-critical mode | 1.688969 | 0.063485 | [1.671013, 1.706926] | 50 |

### Matched localization disagreement

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 1.378083 | 0.032106 | [1.369002, 1.387164] | 50 |
| 0 | none | Balanced mode | 1.201467 | 0.029773 | [1.193046, 1.209889] | 50 |
| 0 | none | Cardinality-critical mode | 1.302605 | 0.182787 | [1.250905, 1.354305] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 1.447328 | 0.053166 | [1.432291, 1.462366] | 50 |
| 1 | bandwidth cap | Balanced mode | 1.241721 | 0.034320 | [1.232014, 1.251428] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 1.344238 | 0.183096 | [1.292450, 1.396025] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 2.263492 | 0.340230 | [2.167260, 2.359723] | 50 |
| 2 | tiered link loss | Balanced mode | 1.535770 | 0.148402 | [1.493796, 1.577745] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 1.570572 | 0.192670 | [1.516077, 1.625068] | 50 |
| 3 | node outage | Fixed Metropolis | 2.700860 | 0.534480 | [2.549686, 2.852034] | 50 |
| 3 | node outage | Balanced mode | 1.570408 | 0.155862 | [1.526323, 1.614492] | 50 |
| 3 | node outage | Cardinality-critical mode | 1.574408 | 0.206357 | [1.516041, 1.632774] | 50 |

### Cardinality dispersion

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 0.086675 | 0.020237 | [0.080951, 0.092399] | 50 |
| 0 | none | Balanced mode | 0.068850 | 0.015740 | [0.064398, 0.073302] | 50 |
| 0 | none | Cardinality-critical mode | 0.048450 | 0.013369 | [0.044669, 0.052231] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 0.126075 | 0.020432 | [0.120296, 0.131854] | 50 |
| 1 | bandwidth cap | Balanced mode | 0.085025 | 0.021137 | [0.079047, 0.091003] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 0.049350 | 0.011645 | [0.046056, 0.052644] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 0.650100 | 0.223202 | [0.586969, 0.713231] | 50 |
| 2 | tiered link loss | Balanced mode | 0.179975 | 0.042789 | [0.167872, 0.192078] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 0.065625 | 0.016924 | [0.060838, 0.070412] | 50 |
| 3 | node outage | Fixed Metropolis | 0.879550 | 0.297870 | [0.795300, 0.963800] | 50 |
| 3 | node outage | Balanced mode | 0.200925 | 0.047909 | [0.187374, 0.214476] | 50 |
| 3 | node outage | Cardinality-critical mode | 0.065925 | 0.021157 | [0.059941, 0.071909] | 50 |

## Local Metrics

### Local E-OSPA

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 1.897744 | 0.038258 | [1.886923, 1.908565] | 50 |
| 0 | none | Balanced mode | 1.827794 | 0.038581 | [1.816881, 1.838706] | 50 |
| 0 | none | Cardinality-critical mode | 1.758807 | 0.037831 | [1.748106, 1.769507] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 1.986237 | 0.039562 | [1.975047, 1.997427] | 50 |
| 1 | bandwidth cap | Balanced mode | 1.892100 | 0.039919 | [1.880809, 1.903391] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 1.805010 | 0.038562 | [1.794104, 1.815917] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 2.781144 | 0.128910 | [2.744683, 2.817605] | 50 |
| 2 | tiered link loss | Balanced mode | 2.292981 | 0.081787 | [2.269849, 2.316114] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 2.003315 | 0.044321 | [1.990779, 2.015851] | 50 |
| 3 | node outage | Fixed Metropolis | 2.999069 | 0.142441 | [2.958781, 3.039358] | 50 |
| 3 | node outage | Balanced mode | 2.350608 | 0.071561 | [2.330367, 2.370848] | 50 |
| 3 | node outage | Cardinality-critical mode | 2.031221 | 0.047603 | [2.017756, 2.044685] | 50 |

### Local RMSE

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 1.441797 | 0.034311 | [1.432092, 1.451501] | 50 |
| 0 | none | Balanced mode | 1.375123 | 0.034522 | [1.365359, 1.384888] | 50 |
| 0 | none | Cardinality-critical mode | 1.454574 | 0.130110 | [1.417773, 1.491374] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 1.476539 | 0.036326 | [1.466264, 1.486814] | 50 |
| 1 | bandwidth cap | Balanced mode | 1.417160 | 0.035254 | [1.407189, 1.427132] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 1.498565 | 0.130704 | [1.461596, 1.535533] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 1.630027 | 0.049051 | [1.616154, 1.643901] | 50 |
| 2 | tiered link loss | Balanced mode | 1.601289 | 0.048619 | [1.587537, 1.615040] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 1.732578 | 0.155817 | [1.688507, 1.776650] | 50 |
| 3 | node outage | Fixed Metropolis | 1.707800 | 0.126214 | [1.672101, 1.743499] | 50 |
| 3 | node outage | Balanced mode | 1.628663 | 0.048768 | [1.614869, 1.642456] | 50 |
| 3 | node outage | Cardinality-critical mode | 1.744279 | 0.167414 | [1.696927, 1.791631] | 50 |

### Local cardinality error

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 0.272725 | 0.025362 | [0.265552, 0.279898] | 50 |
| 0 | none | Balanced mode | 0.260600 | 0.023734 | [0.253887, 0.267313] | 50 |
| 0 | none | Cardinality-critical mode | 0.189300 | 0.018460 | [0.184079, 0.194521] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 0.375475 | 0.037157 | [0.364965, 0.385985] | 50 |
| 1 | bandwidth cap | Balanced mode | 0.302525 | 0.030587 | [0.293874, 0.311176] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 0.196150 | 0.019701 | [0.190578, 0.201722] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 1.364300 | 0.260181 | [1.290710, 1.437890] | 50 |
| 2 | tiered link loss | Balanced mode | 0.550925 | 0.084961 | [0.526895, 0.574955] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 0.215425 | 0.029127 | [0.207187, 0.223663] | 50 |
| 3 | node outage | Fixed Metropolis | 1.737000 | 0.379427 | [1.629682, 1.844318] | 50 |
| 3 | node outage | Balanced mode | 0.583075 | 0.087192 | [0.558413, 0.607737] | 50 |
| 3 | node outage | Cardinality-critical mode | 0.215875 | 0.028476 | [0.207821, 0.223929] | 50 |

## Runtime

### Filter runtime

| Level | Constraint | Method | Mean | Std | 95% CI | N |
|--:|:--|:--|--:|--:|:--|--:|
| 0 | none | Fixed Metropolis | 5.364530 | 0.908992 | [5.107428, 5.621632] | 50 |
| 0 | none | Balanced mode | 5.828246 | 0.992577 | [5.547502, 6.108989] | 50 |
| 0 | none | Cardinality-critical mode | 14.872406 | 2.421637 | [14.187464, 15.557348] | 50 |
| 1 | bandwidth cap | Fixed Metropolis | 4.921979 | 0.533860 | [4.770981, 5.072977] | 50 |
| 1 | bandwidth cap | Balanced mode | 5.430245 | 0.561910 | [5.271313, 5.589177] | 50 |
| 1 | bandwidth cap | Cardinality-critical mode | 13.749198 | 1.268108 | [13.390523, 14.107873] | 50 |
| 2 | tiered link loss | Fixed Metropolis | 5.399734 | 0.577303 | [5.236448, 5.563020] | 50 |
| 2 | tiered link loss | Balanced mode | 5.893988 | 0.623895 | [5.717524, 6.070452] | 50 |
| 2 | tiered link loss | Cardinality-critical mode | 15.433452 | 1.679361 | [14.958457, 15.908447] | 50 |
| 3 | node outage | Fixed Metropolis | 5.053264 | 0.763740 | [4.837245, 5.269282] | 50 |
| 3 | node outage | Balanced mode | 5.353273 | 0.863029 | [5.109171, 5.597375] | 50 |
| 3 | node outage | Cardinality-critical mode | 14.127465 | 2.156943 | [13.517390, 14.737541] | 50 |

