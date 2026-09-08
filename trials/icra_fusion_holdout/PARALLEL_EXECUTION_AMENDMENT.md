# Execution scheduling amendment; scientific registration unchanged

After the first six complete sequences passed independent audits, dense
sequence 0008 required roughly 80 s for an individual density arm and much
longer for the MIL arm. The remaining work is therefore scheduled across
three independent single-computational-thread MATLAB processes, one distinct
sequence per process, using the unchanged frozen common runner.

The current audit watcher and the single owned MATLAB process for index 6
were intentionally terminated with SIGTERM. The original launcher recorded
return -15, no completion line and three completed arm files for 0008. This
is an intentional execution interruption, not a completed sequence and not
a native-runtime failure. Preserve that runtime record, its log and all
three complete files before replaying the entire sequence in a fresh process.
All first-six sequence results remain byte-for-byte unchanged. Compare the
three saved complete arm outputs against the replay, excluding runtime only.

The method, primary contrasts, all 25 sequence IDs, all 16 arms, model,
calibration, packet encoding, radio seeds and source manifest are unchanged.
The original METHOD_FREEZE.json and run_holdout.py remain immutable. Record
the new execution driver's hash separately. Each worker writes to a unique
sequence path, and records real child exit, completion line and all 32 files.
The active completion ledger may be rebuilt from complete entries after
preserving the interrupted attempt. The independent auditor uses the same
unchanged code and verifies cached results before continuing. Parallel wall
times must not be presented as a controlled per-method runtime benchmark.
