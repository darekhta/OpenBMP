# Controller comparison — figure-eight under matched roll-axis ReducedRate fault

Common scenario family: `scenarios/diff-flatness-figure-eight-*`. Common disturbance:
`EffectorFault::ReducedRate { factor = 0.7 }` on the roll-torque effector.
Metrics computed over the post-liftoff window (`t > 0.05 s`) of the
8 s scenario. Saturation fraction is the fraction of per-axis samples at which
any direct-torque effector hit the ±0.35 N·m rail.

| Rate loop | max \|ω\| (rad/s) | RMS \|ω\| (rad/s) | Peak τx (N·m) | Peak τy (N·m) | Peak τz (N·m) | Saturation fraction |
|---|---|---|---|---|---|---|
| PID baseline | 0.5921 | 0.4017 | 0.3500 | 0.3500 | 0.1553 | 0.4107 |
| PID + L1 | 0.0508 | 0.0434 | 0.1768 | 0.2219 | 0.1146 | 0.0000 |
| LQR | 0.7018 | 0.4339 | 0.3500 | 0.3500 | 0.3500 | 0.5781 |
| INDI | 0.7512 | 0.4427 | 0.3500 | 0.3500 | 0.3500 | 0.6987 |
