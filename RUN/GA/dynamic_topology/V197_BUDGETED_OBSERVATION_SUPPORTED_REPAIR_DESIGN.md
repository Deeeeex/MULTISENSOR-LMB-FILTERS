# V197 budgeted observation-supported repair

## Motivation

V194 correctly identifies useful first-page full-posterior repairs, but its
independent per-formation projection releases too many payloads recursively.
V196 further shows that individually useful X36 releases do not compose
additively and that a later `F3` repair degrades E-OSPA, RMSE, consensus and
communication simultaneously.

## Policy

V197 keeps the fixed physical carrier, nominal KLA weights, V99 omission
proposal and ordinary payload accounting unchanged.  It adds a repair token:

1. build the V194 current observation-supported set-entry projection;
2. if no release occurred on the preceding two completed pages, select at
   most one flagged formation;
3. choose the formation with the largest maximum receiver set-entry risk;
4. restore that formation's ordinary full posterior and record the release;
5. refill the token after two release-free pages.

The previous release history is produced by the filter from past requested and
executed omission sets, exposed through the observable-only context, and
contains no truth or outcome value.  A continuation begins with an available
token because no V197 decision predates policy activation.

## Frozen development expectation

At the opened anchors, the top-one rule should select M24 `F4` and X36 `F2`.
The H=3 candidate must reproduce the corresponding single-release teacher
sequences before its tracking metrics are interpreted.  These anchors remain
development evidence; broader scenes and seeds are deferred until the method
passes the paired M24/X36 mechanism check.

