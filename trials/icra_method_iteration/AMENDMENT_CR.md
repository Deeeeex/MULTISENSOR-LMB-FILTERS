# Development amendment: conservative existence recency

2026-09-08, after the IR first-three-sequence audit and before any CR output.
The complete IR cohort remains scheduled and must be retained regardless of
the outcome below. Original ER's instrumented outputs are bitwise identical
on 1,620 audited node-frames. IR does not repair the observed problem there:
reliable mean OSPA is 5.37375 versus ER 5.37798 and no-age 4.81284; intermittent
IR is 6.22333 versus ER 5.73703 and no-age 5.62589. This is development evidence.

Same-input counterfactuals show that the inherited-prior hypothesis is not
the whole explanation. On reliable sequences 0000 and 0001, removing age on
the ER recursive inputs lowers OSPA and false cost, whereas changing age
from full log odds to current increments yields the same extracted output
as ER. Positive existence amplification remains. The new test therefore
addresses that amplification directly, without changing positive-evidence
likelihoods or tuning an age decay constant.

Define r_0 as the ordinary, observation-qualified no-age fusion result and
r_ER as the original recency result, for the *same* eligible inputs and
spatial pool. Conservative recency (CR) is

    r_CR = min(r_0, r_ER),       p_CR = p_0 = p_ER.

This is the solution of the original ER fixed-input variational objective
with the additional constraint r <= r_0. Thus current spatial data and the
ordinary existence pool can support a positive existence claim, while time
since sensing alone can only lower it. The extra constraint is a deliberate
design choice, not a consequence of the Bernoulli Bayes model. It may delay
discovery or lose weak real tracks; measure those costs. There is no theorem
that recursive OSPA or false cost must improve. This rule introduces no new
parameter, metadata, packet field, detector threshold or motion change.

Keep IR source/results immutable. Run CR on the same nine development
sequences, both radio conditions, and compare every arm/sequence. The
existing complete no-age and ER results remain the reference. If CR is
competitive on development data, freeze its source and evaluate it on the
reserved seven-sequence transfer cohort before choosing a paper-facing
method. Check the same synthetic cases and no-new-target control; report
any loss of the earlier discovery gain. Do not try a sign threshold, cap
scale or another decay setting based on the reserved cohort.
