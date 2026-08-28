# V144 pre-outcome freeze: reference-anchored burst role schedule

Frozen at `2026-08-29 00:19 CST`, while the V143 M24 process was still
running and before any V143 tracking outcome or byte result was available.

## Decision rule

The successor schedule is selected only from the saved nominal fusion
matrices, not from tracking truth.  Among periodic schedules that start with
one all-reference page and then use only working pages until the next
reference page, choose the largest working-page fraction satisfying both:

- maximum reference age no greater than three pages;
- terminal maximum row total-variation distance from the static mixing
  product no greater than `0.025` on both M24 and X36.

The unique tested choice is `R-W-W-W`, repeated from the intervention anchor.
It assigns 75% of protected-page local wire opportunities to W while every
cross-formation opportunity remains R.  Once no formation remains protected,
all opportunities are R and W rejoins the multiplexed R state.

## Structural preflight

| Scale | Static Dobrushin | V143 R-W Dobrushin / TV | V144 R-W-W-W Dobrushin / TV |
|:--|--:|--:|--:|
| M24 | 0.367079 | 0.373537 / 0.009270 | 0.382647 / 0.024098 |
| X36 | 0.515065 | 0.517756 / 0.009410 | 0.521470 / 0.024492 |

The next longer periodic burst, `R-W-W-W-W`, fails the frozen TV bound on
both scales (`0.029425` and `0.031843`).  Starting with W is also rejected:
the `W-W-R` ordering has terminal TV `0.063099` and `0.064963`.  The initial
reference page is therefore a substantive anchor, not cosmetic phase naming.

## Outcome gate

V144 is authorized only if V143 shows that one working page per two-page
cycle leaves insufficient tracking headroom or if its role evidence directly
shows temporal under-propagation.  It inherits the one-payload, zero-auxiliary,
no-byte-increase, output-safety, rejoin, and cross-scale tracking gates.  A
below-gate result remains repository-only.  No threshold or schedule is to be
changed after the V143 result opens.

