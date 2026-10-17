# Controller comparison — figure-eight under matched roll-axis ReducedRate fault

Common scenario family: `scenarios/diff-flatness-figure-eight-*`. Common disturbance:
`EffectorFault::ReducedRate { factor = 0.7 }` on the roll-torque effector.
All rows use the same deterministic seed so the synthetic-sensor stream is aligned.
Metrics computed over the post-liftoff window (`t >= 0.05 s`) of the
8 s scenario. Saturation fraction is the fraction of post-liftoff per-axis samples
whose corresponding direct-torque effector hit the ±0.35 N·m rail.
This is one documented operating point, not a best-vs-best controller ranking.

| Rate loop | max \|ω\| (rad/s) | RMS \|ω\| (rad/s) | Peak τx (N·m) | Peak τy (N·m) | Peak τz (N·m) | Saturation fraction |
|---|---|---|---|---|---|---|
| PID baseline | 0.0642 | 0.0283 | 0.3500 | 0.3500 | 0.3500 | 0.0023 |
| PID + L1 | 0.0307 | 0.0047 | 0.2601 | 0.3267 | 0.3500 | 0.0004 |
| LQR | 0.1206 | 0.0540 | 0.2097 | 0.3500 | 0.3307 | 0.0021 |
| INDI | 0.0831 | 0.0340 | 0.2456 | 0.3500 | 0.3500 | 0.0292 |
