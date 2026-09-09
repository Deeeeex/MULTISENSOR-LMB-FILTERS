# Frozen V2X-Real vehicle-pair transfer

Select every scene in the official 64-beam validation archive that contains
both mobile agents 1 and 2, using archive-directory metadata only. This gives
five scenes and 619 paired frames. Exclude the one scene with mobile agent 2
and infrastructure only; it cannot instantiate the two-vehicle experiment.
Use all paired source frames in each selected scene. No tracking outcome is
available when this cohort is chosen. The scenes were collected in 2023,
separately from the 2022 V2V4Real release. Group descriptive uncertainty by
collection date, because the archive does not establish independence among
same-day recording chunks.

Keep the V2V4Real PointPillars checkpoint, preprocessing range, detection
threshold, postprocessing, old nine-sequence score calibration and likelihood
prior unchanged. Use the official V2X-Real input conventions: float32 XYZI
`.bin` data with the intensity channel zeroed, and six-value poses in
`[x,y,z,roll,yaw,pitch]` degrees. Agent 1 is the frame origin. Platform poses
are used for spatial conversion; no annotation enters detection or tracking.
Use the official `vehicle` meta-class (LongVehicle, Car, PoliceCar), with
world-frame object centers projected into the current ego frame, for scoring.
The binary format, zero intensity and class mapping are recorded transfer
adapter differences, rather than fitted parameters.

Detection inference may run during development because it is shared by all
fusion arms and cannot change their definitions. Freeze the final fusion
candidate and comparison arms before any V2X tracking outcome. Every arm
runs its own complete recursion with the same frames, input marks, radio
draws, pD=0.9 and primary current-ego CV model. Preserve the older GCE and
all comparison outputs. Apply the same set-scoring domain, robot exclusion,
extraction and OSPA/GOSPA implementation used for V2V4Real.

Verify archive byte ranges, CRC32, raw SHA-256, pose conventions, finite
point inputs, shared object centers, unique IDs and immutable detection
files. Verify native completion and independently reconstruct fusion
probabilities, Gaussian states, packet accounting, extraction and scores.
Report all five scenes and the full aggregate, including any reversals.

Official sources:

- https://github.com/ucla-mobility/V2X-Real
- https://mobility-lab.seas.ucla.edu/v2x-real/
- Public validation archive: https://ucla.box.com/s/est8t7lxirg85ohkgoueietxd0xqpf36
- Author code inspected at commit 2b8acd5c15b22fdb47326e30b8805e172d9b1259.

The author repository and raw data stay in the external cache. Redistributable
trial artifacts contain references, hashes and our adapter, rather than a
copy of the downloaded author implementation or point clouds.
