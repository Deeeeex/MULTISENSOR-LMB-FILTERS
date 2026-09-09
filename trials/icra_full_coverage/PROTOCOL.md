# Complete the public V2V4Real sequence coverage

Registered on 2026-09-09 before tracking the remaining sequences with the
current GCE. The user's request is to run every available sequence in the
background. Base checkout: `cb67ee81` on `codex/icra`.

## Data and counting

The CVPR 2023 dataset paper reports 67 selected scenarios and about 20,000
LiDAR frames. The twelve public UCLA ZIP directories available here contain
32 train, 9 test and 3 validation sequence entries. Train 0000 and test 0000
are the same scene and 147-frame trajectory. Count that scene once, using
its existing development result. Thus this release has 43 distinct scene
directories and 9,699 paired timestamps. Validation also contains 18
unpaired frames from the second sensor; these are outside a paired replay.

The published 67-scenario count has no verified mapping to these 43
directories. Do not invent extra sequences or split long trajectories to
increase the sample count.

Current primary comparisons cover 25 train sequences, all 9 test-backed
development sequences, and all 3 validation sequences: 37 distinct scenes,
8,342 paired timestamps. Complete the six remaining distinct train
sequences: 0005, 0010, 0015, 0020, 0025, 0030, totaling 1,357 timestamps.
These six appeared in earlier fusion experiments. Their role is complete
release coverage under the frozen current method.

## Fixed experiment

Use all frames, the existing released detections, geometry, measurement and
truth domains, full-nine score calibration and positive prior. Keep the
primary CV model, pD=0.9, GCE parameters and radio seeds unchanged. Run both
reliable and intermittent conditions for No-age KLA, Recency, Scalar,
Guarded Scalar, Joint without curvature, Fixed Ratio 0.25, and GCE. This
adds 84 sequence-method-condition outputs. Reuse existing audited results
for the other 37 scenes.

Before the six-sequence stage, reproduce GCE and Scalar on development
0000 under both link conditions. Compare all saved recursive fields except
runtime against the frozen original outputs. The new MATLAB entry point
differs only in its name and the added path to the existing review helpers.
The filters, fusion functions and replay body remain unchanged.

## Completion and reporting

Two single-thread MATLAB workers run complete sequences. Retain native
exit codes, completion lines, individual logs and every saved trajectory.
Only continue to the main stage after the four preflight outputs pass the
existing independent probability, extraction and OSPA audit. Audit the 84
main outputs in the same way, then summarize all 43 scenes.

Register input, source and reused evidence SHA-256 values before launch.
Report sequence-macro and frame-weighted OSPA, the three existing cohorts
plus the six added scenes, and recording-grouped summaries. Frames and
communication repetitions are not independent samples. Keep all outcomes.
Write a completion report automatically. Update manuscript statistics only
after this entire chain has passed; a started process is not a result.

Dataset source: https://openaccess.thecvf.com/content/CVPR2023/papers/Xu_V2V4Real_A_Real-World_Large-Scale_Dataset_for_Vehicle-to-Vehicle_Cooperative_Perception_CVPR_2023_paper.pdf

Public archive source: https://ucla.app.box.com/v/UCLA-MobilityLab-V2V4REAL/folder/279924274808
