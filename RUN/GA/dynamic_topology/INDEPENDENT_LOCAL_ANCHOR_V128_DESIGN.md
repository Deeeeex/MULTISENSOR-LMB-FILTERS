# V128 independent local anchor

V127 shows that the receiver's current local posterior is not independent of
the working path: it is predicted from the previous fused state and therefore
inherits accumulated protection bias.

V128 forks a second state at the common t=72 local posterior.  The working path
continues V105 communication and fusion.  The anchor path performs only each
node's own prediction and measurement update, with an isolated random stream,
and never consumes a neighbour posterior.  At the opened V126 rollback cells,
the working posterior is replaced by that node's current independent anchor.
This adds local filtering time and one posterior state per node but no
inter-node message or payload byte.

The strict V126 estimation, local-safety, consensus, communication and B3 gate
is unchanged.  Runtime includes anchor maintenance; auxiliary memory is noted
but not yet quantified.  The opened rollback mask means a passing result would
establish only a viable low-communication state source.  A causal online trigger
must still be designed before any deployment or generalization claim.
