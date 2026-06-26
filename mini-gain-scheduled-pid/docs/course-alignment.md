# Course Alignment -- mini-gain-scheduled-pid

## Nine-School Curriculum Mapping

| School | Course | Mapping |
|--------|--------|---------|
| **MIT** | 6.302 Feedback Systems | Gain scheduling as nonlinear control; frozen-time analysis |
| **Stanford** | ENGR205 Process Control | Scheduled PID for varying operating conditions; IMC design |
| **Berkeley** | ME233 Advanced Control | LPV gain scheduling; Lyapunov stability |
| **CMU** | 24-677 Adv Ctrl Systems | Stability-preserving interpolation (Stillwell 2000) |
| **Georgia Tech** | ECE 6550 Nonlinear Control | Slow-variation theorem; describing function |
| **Purdue** | ME 575 Industrial Control | PID tuning rules; anti-windup; bumpless transfer |
| **RWTH Aachen** | Industrial Control Systems | PLC gain scheduling; TIA Portal PIDE |
| **Tsinghua** | Process Control Engineering | Temperature/pH industrial cases |
| **ISA/IEC** | ISA-101 / IEC 61131-3 | Standard PID forms; scheduled control patterns |

## Key References

1. Rugh & Shamma, "Research on gain scheduling", Automatica, 36, 2000.
2. Apkarian & Gahinet, "Self-scheduled H-infinity control", IEEE TAC, 40(5), 1995.
3. Astrom & Wittenmark, Adaptive Control, 2nd Ed., Addison-Wesley, 1995.
4. Leith & Leithead, "Survey of gain-scheduling analysis & design", IJC, 73, 2000.
5. Shamma & Athans, "Analysis of gain scheduled control...", IEEE TAC, 35(8), 1990.
6. Stillwell, "Stability Preserving Interpolation...", Ph.D. Thesis, CMU, 2000.
7. Fritsch & Carlson, "Monotone Piecewise Cubic Interpolation", SINUM, 1980.
8. Akima, "A New Method of Interpolation...", JACM, 17(4), 1970.

## Course-to-Module Mapping Matrix

| Course Topic | This Module |
|-------------|-------------|
| Gain scheduling principle | gs_design_frozen_parameter |
| LPV system representation | gain_schedule_core.h (SCHED_VAR_STATE_VECTOR) |
| Frozen-parameter design | gs_design_zn_pid, etc. (6 rules) |
| Interpolation methods | gs_interp_linear/cubic/spline/akima/rbf |
| Stability analysis | gs_stability_frozen_time/slow_variation |
| Anti-windup for scheduled PID | gs_pid_update (saturation + back-calculation) |
| Bumpless transfer | gs_pid_tracking_mode |
| Online adaptation | gs_adaptive_rls_update, gs_adaptive_gradient_update |
| Fuzzy scheduling | gs_adaptive_fuzzy_infer |
