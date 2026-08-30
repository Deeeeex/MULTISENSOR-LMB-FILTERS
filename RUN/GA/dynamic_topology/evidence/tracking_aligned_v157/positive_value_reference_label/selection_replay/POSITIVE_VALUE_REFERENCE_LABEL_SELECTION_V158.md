# V158 positive-value label selection replay

- Preset / seed: `x36-formation-fov / 211`
- Registered receiver-time cells: `36`
- Selected complete-label edits: `144`
- Deletes / restores: `0 / 144`
- Cells selecting 0..4 edits: `[0 0 0 0 36]`
- Zero-edit cells: `0`
- Replayed actual / frozen worst-case payload: `187560 / 198144 B` (`94.66%` utilization)
- Sum of accepted current-step E-OSPA reductions: `174.088608`

- Marginal reduction by selection rank: `[114.538 37.5461 21.4971 0.507675]`
- Gain share by selection rank: `[65.793 21.567 12.348 0.29162]%`
- Marginal thresholds: `[1 0.1 0.01 0.001]`
- Edits above thresholds: `[29 35 86 136]`
- Gain retained above thresholds: `[97.951 98.743 99.882 99.997]%`

| t | Formation | Sensor | Candidates | Selected | Delete | Restore | E-OSPA before | E-OSPA after | Bytes | Edits |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| 76 | 2 | 7 | 23 | 4 | 0 | 4 | 82.0162 | 76.3927 | 5504 | (1,2):restore:5.608, (19,13):restore:0.008, (13,9):restore:0.006, (19,15):restore:0.002 |
| 76 | 2 | 8 | 24 | 4 | 0 | 4 | 87.3241 | 69.7964 | 5504 | (13,12):restore:5.466, (31,24):restore:5.849, (1,2):restore:6.203, (7,6):restore:0.009 |
| 76 | 2 | 9 | 23 | 4 | 0 | 4 | 81.8873 | 69.7926 | 4328 | (1,2):restore:5.715, (13,12):restore:5.298, (31,24):restore:1.043, (31,21):restore:0.039 |
| 76 | 2 | 10 | 23 | 4 | 0 | 4 | 87.3937 | 76.0209 | 4328 | (31,24):restore:5.468, (1,2):restore:5.736, (13,11):restore:0.150, (7,7):restore:0.018 |
| 76 | 2 | 11 | 23 | 4 | 0 | 4 | 81.8385 | 75.9360 | 4328 | (1,2):restore:5.745, (31,24):restore:0.006, (13,11):restore:0.135, (7,7):restore:0.016 |
| 76 | 2 | 12 | 23 | 4 | 0 | 4 | 81.7651 | 69.7584 | 4328 | (13,12):restore:5.137, (31,24):restore:0.651, (1,2):restore:6.202, (7,7):restore:0.016 |
| 77 | 1 | 1 | 24 | 4 | 0 | 4 | 81.4344 | 69.0488 | 5504 | (19,13):restore:5.958, (25,19):restore:6.375, (31,24):restore:0.043, (31,22):restore:0.009 |
| 77 | 1 | 2 | 24 | 4 | 0 | 4 | 81.3717 | 61.8716 | 5504 | (19,13):restore:5.950, (25,18):restore:6.425, (25,19):restore:7.054, (19,15):restore:0.071 |
| 77 | 1 | 3 | 23 | 4 | 0 | 4 | 81.3406 | 75.3135 | 4664 | (25,19):restore:5.957, (31,23):restore:0.062, (19,14):restore:0.005, (31,24):restore:0.003 |
| 77 | 1 | 4 | 24 | 4 | 0 | 4 | 81.3414 | 75.3766 | 4664 | (25,19):restore:5.913, (31,23):restore:0.029, (19,16):restore:0.018, (19,15):restore:0.004 |
| 77 | 1 | 5 | 24 | 4 | 0 | 4 | 81.3421 | 75.3684 | 5504 | (25,19):restore:5.915, (31,23):restore:0.028, (19,16):restore:0.021, (19,15):restore:0.009 |
| 77 | 1 | 6 | 24 | 4 | 0 | 4 | 81.3392 | 68.8722 | 4328 | (19,13):restore:5.976, (25,19):restore:6.433, (31,23):restore:0.041, (19,16):restore:0.017 |
| 78 | 1 | 1 | 24 | 4 | 0 | 4 | 69.0704 | 62.1068 | 5504 | (25,18):restore:6.869, (31,24):restore:0.047, (25,19):restore:0.035, (19,13):restore:0.013 |
| 78 | 1 | 2 | 24 | 4 | 0 | 4 | 69.0457 | 68.9173 | 5504 | (25,19):restore:0.050, (31,24):restore:0.042, (19,13):restore:0.029, (13,12):restore:0.007 |
| 78 | 1 | 3 | 24 | 4 | 0 | 4 | 75.4786 | 75.2765 | 5504 | (31,23):restore:0.068, (19,15):restore:0.074, (25,19):restore:0.055, (19,14):restore:0.006 |
| 78 | 1 | 4 | 24 | 4 | 0 | 4 | 75.5245 | 68.8761 | 4328 | (19,13):restore:6.375, (25,19):restore:0.104, (31,23):restore:0.069, (19,15):restore:0.100 |
| 78 | 1 | 5 | 24 | 4 | 0 | 4 | 75.5138 | 68.8712 | 5504 | (25,18):restore:6.449, (31,23):restore:0.069, (19,15):restore:0.079, (25,19):restore:0.045 |
| 78 | 1 | 6 | 24 | 4 | 0 | 4 | 69.0948 | 68.8101 | 5504 | (25,19):restore:0.135, (31,23):restore:0.049, (19,15):restore:0.068, (31,24):restore:0.032 |
| 78 | 6 | 31 | 23 | 4 | 0 | 4 | 86.9681 | 81.4289 | 3992 | (25,19):restore:5.515, (1,4):restore:0.013, (1,2):restore:0.006, (31,22):restore:0.006 |
| 78 | 6 | 32 | 24 | 4 | 0 | 4 | 87.0359 | 86.9826 | 5504 | (1,4):restore:0.018, (1,2):restore:0.018, (31,22):restore:0.014, (1,1):restore:0.003 |
| 78 | 6 | 33 | 23 | 4 | 0 | 4 | 81.4864 | 81.4801 | 5504 | (1,3):restore:0.003, (31,22):restore:0.002, (1,4):restore:0.001, (13,10):restore:0.001 |
| 78 | 6 | 34 | 23 | 4 | 0 | 4 | 87.0744 | 87.0660 | 5504 | (1,2):restore:0.003, (31,22):restore:0.003, (1,4):restore:0.002, (13,10):restore:0.000 |
| 78 | 6 | 35 | 23 | 4 | 0 | 4 | 87.0275 | 87.0203 | 5504 | (31,22):restore:0.003, (1,2):restore:0.002, (1,4):restore:0.002, (13,10):restore:0.000 |
| 78 | 6 | 36 | 23 | 4 | 0 | 4 | 87.0730 | 87.0655 | 5168 | (31,22):restore:0.003, (1,2):restore:0.002, (1,4):restore:0.002, (13,10):restore:0.000 |
| 79 | 1 | 1 | 24 | 4 | 0 | 4 | 61.9643 | 61.7304 | 5504 | (19,15):restore:0.202, (13,11):restore:0.016, (1,4):restore:0.009, (19,13):restore:0.007 |
| 79 | 1 | 2 | 24 | 4 | 0 | 4 | 62.1376 | 62.0014 | 5504 | (13,11):restore:0.026, (19,15):restore:0.024, (13,9):restore:0.063, (19,13):restore:0.022 |
| 79 | 1 | 3 | 24 | 4 | 0 | 4 | 69.3082 | 69.1486 | 5504 | (25,18):restore:0.048, (19,15):restore:0.032, (13,9):restore:0.061, (13,11):restore:0.019 |
| 79 | 1 | 4 | 24 | 4 | 0 | 4 | 69.1577 | 62.2178 | 5504 | (25,18):restore:6.856, (13,11):restore:0.055, (19,13):restore:0.019, (25,20):restore:0.011 |
| 79 | 1 | 5 | 24 | 4 | 0 | 4 | 62.1448 | 62.0735 | 5504 | (13,11):restore:0.052, (19,13):restore:0.010, (25,20):restore:0.006, (1,4):restore:0.004 |
| 79 | 1 | 6 | 24 | 4 | 0 | 4 | 69.3126 | 62.1142 | 5504 | (19,13):restore:7.100, (25,18):restore:0.048, (13,11):restore:0.042, (13,9):restore:0.008 |
| 79 | 6 | 31 | 23 | 4 | 0 | 4 | 81.4725 | 75.5173 | 5504 | (25,19):restore:5.929, (31,24):restore:0.020, (13,12):restore:0.003, (1,3):restore:0.003 |
| 79 | 6 | 32 | 24 | 4 | 0 | 4 | 75.6226 | 75.5891 | 5504 | (25,19):restore:0.017, (31,24):restore:0.008, (13,10):restore:0.005, (13,12):restore:0.004 |
| 79 | 6 | 33 | 23 | 4 | 0 | 4 | 75.6044 | 75.5992 | 5504 | (1,3):restore:0.001, (13,12):restore:0.001, (7,8):restore:0.001, (13,10):restore:0.001 |
| 79 | 6 | 34 | 23 | 4 | 0 | 4 | 81.5429 | 81.5371 | 5504 | (13,12):restore:0.002, (1,3):restore:0.002, (7,8):restore:0.001, (13,10):restore:0.001 |
| 79 | 6 | 35 | 23 | 4 | 0 | 4 | 81.4871 | 81.4816 | 5504 | (13,12):restore:0.002, (1,3):restore:0.001, (7,8):restore:0.001, (25,20):restore:0.001 |
| 79 | 6 | 36 | 23 | 4 | 0 | 4 | 81.4849 | 81.4786 | 5504 | (13,12):restore:0.002, (7,8):restore:0.002, (1,3):restore:0.001, (25,20):restore:0.001 |

## Evidence boundary

V158 deterministically replays the privileged V157 selector on captured pre-rollback candidate and static-reference posteriors. It identifies edit type and capacity demand only. It does not show that a runtime node can source the selected reference label or predict its value without truth.
