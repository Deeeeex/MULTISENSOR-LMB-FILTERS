# Process crash and unchanged-code replay

The initial five-arm development process terminated in MATLAB R2024a's
native interpreter on 2026-09-08, during sequence 0001 intermittent. The log
contains a segmentation violation in matrix destruction / interpreter code;
the process PID 83538 was verified absent afterwards. It completed and saved
both 0000 conditions and 0001 reliable. No 0001 intermittent or later result
is counted complete. The shell pipe's zero status was not used as evidence
of successful MATLAB completion.

Preserve those three complete files and the crash log. Re-run every sequence
0000--0008 in a separate fresh MATLAB process using exactly the existing
frozen 1242-source snapshot, calibration, five arms and both conditions.
Do not change the algorithm or numerical parameters. Verify each child exit
status and completion line. Compare every overlapping saved result field,
excluding the elapsed-runtime measurement only, against the original files.
Any recurrent failure or parity difference is a diagnostic gate; do not
silently drop a sequence or change the method to get a passing output.

The cause of the native crash is not established. Process isolation is an
execution retry, not a tracking-method change or a performance claim.
