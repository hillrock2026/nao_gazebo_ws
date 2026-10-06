# OPT-0C Confirmation Report

- Document: `docs/gait_optimization/OPT0C_CONFIRMATION_REPORT.md`
- Stage: **Phase OPT-0C — Best-vs-Baseline Confirmation (no optimization)**
- Date: 2026-10-01 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Author: Implementation Agent
- Design: **paired / Common Random Numbers** — baseline and best share the same seed.
- Scope: confirmation experiment only. No optimizer, fitness, bound, generator, or runner change.

```text
OPT0C_RESULT = PASS
LEVEL_3      = ACHIEVED
```

---

## 1. Environment

```text
os            Ubuntu 24.04 (kernel 7.0.0-31-generic)
ros_distro    jazzy (/opt/ros/jazzy)
gz sim        8.15.0, physics gz-physics-dartsim-plugin
path          run_p0a_gait_candidate.run_candidate  (frozen runner, reused unmodified)
seeds         101, 202, 303   (baseline & best paired by seed)
timeout       200.0 s/case     domain_id 77     gz_seed == seed     allow_overwrite = True
episodes      6 (2 θ × 3 seeds); all status=ok, fall=false, no timeout
```

## 2. Frozen Boundary Check

Recomputed at pre-flight **and** stored in `rows.json.provenance.frozen_sha256` — both PASS:

```text
aab60f6fbb18b657736ebf9d7633e65b3d0df021c564cddb537bb806875bd5eb  src/nao_gz_sim/scripts/phase4b_producer.py
00aef7129d65ad78b1bbf219bcec9e9354547c6c0bd8d1ba4fc5948936b820af  tools/run_p0a_gait_candidate.py
4cee76de5db06f7276b9c9803738caa857f0da2d16fbe526d2d86eeb1b6b10fc  src/nao_gz_sim/scripts/nao_gait_phase.py
```

Not modified in this stage (unchanged, and not written by the experiment):
`optimization/fitness.py`, `optimization/parameter_space.py`, `tools/run_gait_optimization.py`,
`config/gait_opt_baseline.yaml`, `optimization_results/{best_theta.yaml,best_fitness.json,baseline.json,generation_log.json,episodes/}`.
Anchor: `docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json` = `116dde70…` (unchanged).

## 3. Baseline θ

Source: audited `ParameterSpace.anchor_normalized()` + `config/gait_opt_baseline.yaml` (not hand-typed).

```text
step_length_m        = 0.0200000000
step_frequency_hz    = 0.5000000000
swing_height_m       = 0.0080000000
double_support_ratio = 0.3000000000
sway_amplitude_rad   = 0.0600000000
```

## 4. Best θ

Source: `optimization_results/best_theta.yaml` → `best.overrides` (read, not hand-typed).
`best_theta.yaml` SHA256 `15371d91739aa168faab14509f4e2e922ca986f1c315ad8da769173027e52278`.

```text
step_length_m        = 0.0218476675
step_frequency_hz    = 0.4000054632
swing_height_m       = 0.0065597182
double_support_ratio = 0.1203659061
sway_amplitude_rad   = 0.0502913025
```

## 5. Six Episode Results

| group | seed | fitness | speed (m/s) | forward (m) | tilt (deg) | yaw (deg) | fall | status |
| ----- | ---: | ------: | ----------: | ----------: | ---------: | --------: | ---- | ------ |
| baseline | 101 | 0.1178 | +0.0004 | +0.0061 | 8.71 | +31.1 | false | ok |
| best | 101 | 0.1998 | +0.0031 | +0.0450 | 6.54 | +25.1 | false | ok |
| baseline | 202 | 0.1422 | −0.0005 | −0.0073 | 7.96 | −19.2 | false | ok |
| best | 202 | 0.2153 | +0.0038 | +0.0548 | 6.68 | +28.3 | false | ok |
| baseline | 303 | 0.1257 | +0.0005 | +0.0077 | 8.83 | −26.5 | false | ok |
| best | 303 | 0.2415 | +0.0034 | +0.0495 | 6.68 | +16.1 | false | ok |

Raw artifacts: `optimization_results/confirmation/episodes/conf_{group}_s{seed}/`; tabular data
`optimization_results/confirmation/rows.json`.

## 6. Aggregate Statistics

Population std (`pstdev`), n = 3 per group.

| metric | baseline mean ± std | best mean ± std | delta (mean) |
| ------ | ------------------- | --------------- | ------------ |
| fitness | 0.12860 ± 0.01014 | 0.21884 ± 0.01721 | **+0.09024** |
| speed_mps | +0.000151 ± 0.000464 | +0.003435 ± 0.000276 | **+0.003284** |
| forward_m | +0.002188 ± 0.006726 | +0.049744 ± 0.004009 | **+0.047556** |
| tilt_deg | 8.4968 ± 0.3833 | 6.6310 ± 0.0673 | −1.8658 |
| yaw_deg | −4.8618 ± 25.594 | +23.1697 ± 5.172 | +28.0315 |
| path_over_net | 147.03 ± 38.59 | 20.05 ± 2.14 | −126.98 |
| max_single_step_m | 0.006607 ± 0.000240 | 0.005450 ± 0.000197 | −0.001156 |

## 7. Paired CRN Analysis

| seed | Δfitness | Δspeed (m/s) | Δforward (m) |
| ---- | -------: | -----------: | -----------: |
| 101 | +0.08190 | +0.00268 | +0.03885 |
| 202 | +0.07309 | +0.00428 | +0.06206 |
| 303 | +0.11572 | +0.00288 | +0.04176 |

All three paired deltas are positive for fitness, speed, and forward displacement.

## 8. Decision

| condition | result |
| --------- | ------ |
| cond1 — best 3/3 ok & no fall | **true** |
| cond2 — Δfitness > 0 for 3/3 pairs | **true** |
| cond3 — mean(best speed) > mean(baseline speed) | **true** |
| cond4 — mean(best forward) > mean(baseline forward) | **true** |

```text
OPT0C_RESULT = PASS
LEVEL_3      = ACHIEVED
```

## 9. Interpretation

- **Relative improvement is confirmed.** Under 3 CRN conditions the OPT-0 best θ improves
  fitness by +0.074 … +0.116 over the frozen anchor, with forward displacement rising from a
  near-zero baseline (some seeds reverse) to a consistent ≈ 0.045–0.055 m, and mode stability
  improving (tilt 8.5°→6.6°, `path_over_net` 147→20, i.e. far less in-place wobbling).
- **Absolute walking quality remains low.** Best mean forward speed is ≈ 0.0034 m/s over the
  analyzed window — a slow, shuffling gait, not a high-performance walk. "Credible" here means
  only: *a stable relative improvement over the current frozen anchor in a small multi-condition
  engineering check.*
- **Residual caveats.** (a) Best gaits still turn while walking (yaw ≈ +16…+28°), so forward
  progress coexists with heading drift. (b) The single-seed non-determinism reported in OPT-0
  persists — the anchor θ scored 0.1769 at gz_seed 101 during OPT-0 but 0.1178 at the same seed
  here, so absolute values are run-varying; the CRN pairing is what makes the comparison valid.
  (c) n = 3 is an MVP engineering check, **not** a statistical significance claim.
- No claim is made about statistical significance, and none about a high-performance gait.

## 10. Next Step

```text
NEXT = OPT-1
```

OPT-0 closes with `LEVEL_3 = FIRST CREDIBLE OPTIMIZED GAIT` (strictly: a stable relative
improvement over the frozen anchor). No further optimization is started in this stage.

## Provenance

```text
git_commit           5526a26e35e2001f295bee3cb4ec89760165089d
anchor_sha256        116dde704ba2d4bbf3f5afaaefc0c5a77a478541d341a86536e38ac0426bc13b
best_theta_sha256    15371d91739aa168faab14509f4e2e922ca986f1c315ad8da769173027e52278
frozen_sha256        see §2 (also in confirmation/rows.json → provenance.frozen_sha256)
rows                 optimization_results/confirmation/rows.json
summary              optimization_results/confirmation/summary.json
episodes             optimization_results/confirmation/episodes/conf_{baseline,best}_s{101,202,303}/
driver               tools/run_opt0c_confirmation.py  (reuses frozen run_candidate; no optimizer)
```

End of report.
