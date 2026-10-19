# Cumulative Progress Reports

- `report.tex` / `report.pdf`: Chinese cumulative report.
- `report_english_version.tex` / `report_english_version.pdf`: English backup.

These cover aircraft data, trim verification, full and subsystem A/B matrices, the elevator-to-pitch-rate transfer function, labelled feedback architecture, and the selected inner-loop design with three compensator options and engineering limitations.

Full per-iteration plots and engineering checks are in the [Chinese inner-loop report](../inner_loop/inner_loop_report.pdf) and [English inner-loop report](../inner_loop/inner_loop_report_english.pdf).

Compile each source twice with XeLaTeX, starting in this folder. Keep `trim.png` alongside the sources and the neighbouring `../inner_loop` folder containing the comparison figure. The Chinese report uses Windows ctex fonts. Printed coefficients are rounded; use MAT files for numerical work.

`loop.m` remains the generic feedback-construction function for a supplied signed controller. The selected controller and reproduction entry point are in `../inner_loop/controller.mat` and `../inner_loop/run_inner.m`.
