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
| PID baseline | 0.5921 | 0.4016 | 0.3500 | 0.3500 | 0.1553 | 0.4107 |
| PID + L1 | 0.0508 | 0.0434 | 0.1768 | 0.2183 | 0.1125 | 0.0000 |
| LQR | 0.7014 | 0.4337 | 0.3500 | 0.3500 | 0.3500 | 0.5781 |
| INDI | 0.7504 | 0.4424 | 0.3500 | 0.3500 | 0.3500 | 0.6992 |
