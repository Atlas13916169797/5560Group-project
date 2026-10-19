# AERO5560 Group 8 Project

| Folder | Description |
| --- | --- |
| `A_B_Matrix` | Contains the A and B matrices for the assigned aircraft. |
| `AI_draft` | Contains AI-prepared LaTeX sources and compiled PDF reports documenting the work completed to date. |
| `Assignment_requirement` | Contains the assignment requirements and report template downloaded from Canvas. |
| `Gqde` | Contains the elevator-to-pitch-rate transfer function, $G_{q\delta_e}(s)$, required by the first part of Longitudinal Task 1. |
| `Group8_AircraftData` | Contains the aircraft data provided on Canvas. |
| `trim` | Contains the checks and results verifying the trim of the supplied initial flight condition. |
| `inner_loop` | Contains three pitch-rate compensator designs, the selected controller, root-locus/Bode/step plots, engineering checks, and detailed Chinese/English reports. |

The selected pitch-rate loop has a nominal closed-loop bandwidth of 4.237 rad/s and an open-loop crossover of 2.017 rad/s at approximately 200 knots and 300 ft. Both open-loop and closed-loop noise magnitudes remain below -40 dB over the checked noise band. See [the inner-loop guide](inner_loop/README.md) for results and reproduction instructions.

## Updating the GitHub Repository

See [GIT_GUIDE.md](GIT_GUIDE.md) for the local Git workflow. After saving and reviewing your changes, run the following command from this folder to commit and push them:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\push.ps1 -Message "Update project files"
```
