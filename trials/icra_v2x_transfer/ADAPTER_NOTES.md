# Input inspection before fusion evaluation

The first full paired-label check found that annotations sharing an ID have
different world centers in the two source YAML files. Among 4,924 shared
vehicle IDs across frames, the largest separation is 2.797785661924853 m.
The exact discrepancies are retained in `SHARED_ANNOTATION_DIAGNOSTIC.json`.
The two poses themselves match the official transform function exactly.

The official `BasePostprocessor.generate_gt_bbx` merges object IDs by keeping
the first occurrence (`object_id_list.index`), with the ego source first.
Our conversion follows this rule: agent 1's annotation is retained when the
ID also occurs for agent 2; the second source supplies otherwise missing IDs.
We do not average centers or use a tracker's output to select annotations.
The earlier equality assertion was an input-format hypothesis, which failed;
the full input check now records these differences and verifies the actual
published union rule. No V2X tracking result existed during this correction.

The final 2D truth uses world centers and full roll/yaw/pitch projection,
with the same final bounding-box x/y crop and downstream set-scoring domain
as the detector/replay. The optional known-motion input is prepared from
poses separately; primary comparisons use the existing current-ego CV.

Source: pinned official `opencood/data_utils/post_processor/base_postprocessor.py`,
commit 2b8acd5c15b22fdb47326e30b8805e172d9b1259, lines 65-101.
