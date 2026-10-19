# Pitch-Rate Inner Loop

```text
Kq(s) = -21.5(s+0.136)(s+0.160) / [s(s+0.0015)(s+0.31)(s+10.7)(s+14.5)]
e = q_command - q_measured
elevator = elevator_trim + Kq * e
```

Use radians and seconds. Elevator trim is +0.766074 deg; nominal actuator and sensor dynamics are unity.

| Nominal metric at 200 kt, 300 ft | Result |
| --- | --- |
| Closed-loop bandwidth / open-loop crossover | 4.237 / 2.017 rad/s |
| Settling, final-value / command 2% bands | 1.607 / 1.602 s |
| Steady-state error / overshoot | 0.116% / 8.897% |
| Phase / gain margin | 60.301 deg / 10.806 dB |
| Open-loop / closed-loop magnitude at 20 rad/s | -41.390 / -41.409 dB |
| Absolute elevator, 0--10 s | 0.4194 to 0.7661 deg |

All nine nominal checks pass in `requirements.csv`, including at least 40 dB attenuation throughout the checked 20--100000 rad/s band. The previous controller missed the noise target; the updated low-frequency compensation and extra roll-off pole correct it.

## Reports and Run

- [Chinese report](inner_loop_report.pdf) and [LaTeX](inner_loop_report.tex).
- [English report](inner_loop_report_english.pdf) and [LaTeX](inner_loop_report_english.tex).

From the assignment root, with MATLAB and Control System Toolbox:

```matlab
run('inner_loop/run_inner.m')
open_sisotool(3)
```

SISOtool uses `Pq = minreal(-Gq/s)` and `Fq = -s*Kq`, including the single external integrator. Keep the adjacent `Gqde`, `A_B_Matrix` and `trim` folders.

## Files

| Files | Content |
| --- | --- |
| `controller.mat/json/txt` | Final rounded controller and feedback models |
| `zpk.txt`, `zpk.csv` | Numerical ZPK forms |
| `requirements.csv`, `comparison.csv`, `results.json`, `validation.mat` | Acceptance checks and validation results |
| `iteration_1/2/3.pdf/png`, `comparison.pdf/png` | Root locus, Bode, step and sensitivity plots |
| `speed_check.csv`, `actuator.csv`, `delay.csv`, `uncertainty.csv`, `frequency_trade.csv` | Off-nominal checks |
| `*_nonlinear.csv`, `nonlinear_summary.csv`, `digital.csv`, `noise.csv` | Simulation histories |
| `search_noise.py`, `refine_noise.py`, `noise_*.json/csv` | Noise-constrained search, refinement and rounding comparisons |
| `search_inner.m`, `refine_inner.m`, `search.mat`, `refinement.mat` | Preliminary searches |

`run_inner.m` calls `design_inner.m`, `validate_inner.m`, `export_inner.m` and `plot_inner.m`. `inner_metrics.m` defines the metrics. Optional Python search scripts use NumPy and SciPy; the final MATLAB reproduction does not require them. `refine_noise.py` uses the recorded seeds in `noise_flexible.json`. JSON represents nonfinite values as null; CSV/MAT retain Infinity.

## Validation Range

- Separate retrimmed 180, 200 and 210 kt cases meet the main time-domain and noise targets.
- Assumed 25/50 ms actuators pass the main nominal targets. At 100 ms, overshoot is 22.31% and settling is 3.757 s; both exceed their limits.
- All 27 parameter cases retain stable minimal q tracking; 21 meet all time-domain thresholds and 18 meet the noise criterion. These separate screens do not give a combined envelope guarantee.
- The unreduced system retains a neutral pitch-angle mode. Persistent commands and sensor bias can accumulate drift.
- Position and assumed 60 deg/s rate limits remain inactive; saturation recovery is untested. Actual structural/stall limits and actuator dynamics are unspecified.
- The 100 Hz ZOH/Tustin check verifies stable minimal tracking and the unreduced neutral mode. Noise RMS applies to an assumed 10 s record.

Run `py -3 build_report.py` to regenerate the detailed reports and cumulative sources in `../AI_draft`. Compile each LaTeX source twice with XeLaTeX. Chinese sources use Windows ctex fonts.
