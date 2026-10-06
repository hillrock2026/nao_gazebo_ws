# OPT-0 Implementation Report — Minimum Viable Automatic Gait Optimization

- Document: `docs/gait_optimization/OPT0_IMPLEMENTATION_REPORT.md`
- Stage: **Phase OPT-0 (implemented + first optimization run)**
- Date: 2026-10-01 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Author: Implementation Agent
- Priority order used: **runnable > complete > elegant**
- Status at end: **`OPT0_IMPLEMENTATION_READY`** (awaiting next-round audit)

---

## 0. Summary

The closed loop **works and improves over baseline**:

```text
baseline fitness = 0.1769   ->   best fitness = 0.3275   (+0.1506, ≈ +85%)
```

`CMA-ES (pycma) ask → θ → frozen run_p0a_gait_candidate → Gazebo → FitnessEvaluator → tell`
ran for 3 generations × 4 candidates (+1 baseline) = 13 real Gazebo episodes, all completed
cleanly (`ok=True`, no timeouts, `residual_processes=0`).

No frozen file was modified. The gait generator was used strictly as a black box.

---

## 1. 修改文件 (Files added / modified)

**Added:**

| File | Purpose |
|---|---|
| `config/gait_opt_baseline.yaml` | baseline: anchor path+SHA256, `seed`/`gz_seed`, frozen background vector, 5-axis bounds, fitness weights |
| `optimization/__init__.py` | package marker |
| `optimization/parameter_space.py` | normalized `[0,1]^5` ↔ override mapping (`ParameterSpace`) |
| `optimization/fitness.py` | `FitnessEvaluator` / `FitnessResult` (invalid −2 / fall −1 / normal ∈(0,1)) |
| `tools/run_gait_optimization.py` | pycma CMA-ES loop calling the frozen runner |
| `docs/gait_optimization/evidence/run1_saturated.log`, `run1_generation_log.json` | evidence of run-1 (saturated fitness) |
| `docs/gait_optimization/OPT0_IMPLEMENTATION_REPORT.md` | this report |
| `optimization_results/` | run outputs (30 MB: 13 raw episode dirs + `best_theta.yaml`, `best_fitness.json`, `generation_log.json`, `baseline.json`) |

**Restored (Task 1 — was missing):**

| Path | Note |
|---|---|
| `docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json` | copied verbatim from `backup/`; SHA256 `116dde70…` (matches the backup `SHA256SUMS`) |

**Modified: NONE.** Verified after the run:

```text
aab60f6f…  src/nao_gz_sim/scripts/phase4b_producer.py     (FROZEN — unchanged)
00aef712…  tools/run_p0a_gait_candidate.py                (FROZEN — unchanged)
4cee76de…  src/nao_gz_sim/scripts/nao_gait_phase.py       (FROZEN — unchanged)
```

Note: `backup/` also holds the rest of the `task4_phase4e_pathb_b3_evidence` tree; only the one
file the frozen runner hard-depends on (`ANCHOR`) was restored.

---

## 2. 运行命令 (Run commands)

```bash
cd /home/liu/workspace/Nao_Program/nao_gazebo_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Task 1 — restore the missing anchor the frozen runner requires
cp /home/liu/workspace/Nao_Program/backup/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json \
   docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json

# single-run validation (1 Gazebo episode)
python3 tools/run_gait_optimization.py --baseline-only --out-root optimization_results

# Task 5 — first optimization (13 Gazebo episodes)
python3 -u tools/run_gait_optimization.py --pop 4 --gens 3 --out-root optimization_results
```

Dependency added: `pycma` 4.5.0 (`python3 -m pip install cma`; pulls numpy, already present).
The evaluator/param-space modules are otherwise stdlib-only.

---

## 3. 第一次优化结果 (First optimization result)

Two runs were performed. The first exposed a **fitness bug**; the second (with the fix) is the
reported result.

### Run 1 — saturated fitness (evidence: `evidence/run1_saturated.log`)

- result: baseline `0.0000`, best `0.0000` — **the optimizer could not improve**, because the
  bounded score used `clip(raw, 0, 1)` and every non-fall candidate's `raw` was negative → all
  collapsed to `0.0`; only falls (`-1.0`) were distinct.
- 12 candidates: 5 `ok` (all `0.0`), 7 falls.
- **Fix**: replaced the saturating clip with a bounded *non-saturating* map
  `fitness = 0.5·(tanh(raw/scale)+1) ∈ (0,1)`, `scale=0.3`, and lightened the penalty weights.

### Run 2 — after the fix (the deliverable)

Per-generation best (single seed = 101):

| generation | best fitness | mean fitness |
|---|---|---|
| 0 | 0.1959 | −0.4248 |
| 1 | **0.3275** | +0.2197 |
| 2 | 0.2409 | +0.2071 |

The optimizer moved out of the fall-prone region after generation 0 (gen 1 and 2 produced no
falls; the mean rose from −0.42 to ≈ +0.21).

---

## 4. Baseline fitness

```
baseline (anchor θ) fitness = 0.1769   status=ok
measured: speed 0.0011 m/s, forward 0.0162 m, tilt 8.86°, yaw 12.1°, fall=false
```

(The anchor hardly moves — it is a near-stationary, wobbling-in-place gait with a high
`path_over_net`, so its velocity term is ~0 and its energy term is maximal.)

---

## 5. Best fitness

```
best fitness = 0.3275    status=ok    candidate=opt0_g01_c00 (generation 1)
theta_named:
    step_length        = 0.02185 m
    frequency          = 0.40000 Hz
    swing_height       = 0.00656 m
    double_support     = 0.12037
    sway_amplitude     = 0.05029 rad
sim_result_path: optimization_results/episodes/opt0_g01_c00/result_p0a.json
measured: speed 0.0048 m/s, forward 0.0696 m, tilt 6.84°, yaw 14.4°, fall=false
git_commit: 5526a26e35e2001f295bee3cb4ec89760165089d
```

Outputs: `optimization_results/{best_theta.yaml, best_fitness.json, generation_log.json, baseline.json}`.

---

## 6. 发现的问题 (Discovered problems)

| # | Problem | Severity | Status / note |
|---|---|---|---|
| P-1 | **Anchor file missing** → the frozen runner could not run at all. | block | **fixed** (restored from `backup/`, SHA256 recorded) |
| P-2 | **Fitness saturation**: `clip(raw,0,1)` collapsed every non-fall gait to `0.0` → no gradient. | block | **fixed** (tanh bounded map) |
| P-3 | **Non-reproducible physics**: identical θ **and** identical `gz_seed=101` gave different metrics across runs (run1 g0c2 speed 0.0106/fwd 0.1534 vs run2 speed 0.0042/fwd 0.0609). `--seed 101` is confirmed present in the `gz sim` argv, so this is real dartsim/bridge/real-time non-determinism. | high | **open** → single-run fitness is noisy; recommend N-seed averaging next round |
| P-4 | **Best θ sits on search bounds** (frequency→0.40 lower bound, swing/sway near lower, step≈0.022). The fitness currently rewards slow, low-energy, low-instability gaits; velocity weight may be too low and/or stability penalties too high. | med | **open** → tune weights / widen the productive region |
| P-5 | **Harsh landscape**: 7/12 candidates fell in run1, 2/12 in run2 (one catastrophic: tilt 180°, yaw 3826°). The anchor region is near the stability edge. | med | **open** |
| P-6 | `step_length` upper bound is the producer safety cap 0.08, not the brief's 0.12 (raising it needs a frozen edit). | low | recorded (DQ-1) |
| P-7 | `energy` / `foot_slip` fitness terms are **proxies** (`path_over_net`, `max_single_step`), not true energy/slip. | low | recorded |
| P-8 | Each Gazebo episode is ~55 s wall; 13 episodes ≈ 12 min. | low | throughput note |

All 13 run-2 episodes finished with `ok=True`, `timed_out=False`, `residual_processes=0`.

---

## 7. 复现 (Reproduction)

```text
config:            config/gait_opt_baseline.yaml  (anchor SHA256 116dde70…, seed 101, gz_seed 101)
optimizer:         pycma CMAEvolutionStrategy, popsize 4, sigma0 0.2, bounds [0,1]^5, x0=anchor
fitness:           optimization/fitness.py (invalid −2, fall −1, normal = 0.5(tanh(raw/0.3)+1))
result:            optimization_results/best_fitness.json
raw evidence:      optimization_results/episodes/opt0_g01_c00/
run-1 evidence:    docs/gait_optimization/evidence/run1_saturated.{log}
```

Caveat: because of P-3, the reported `best` is a **single-seed** result and must not be read as
a reproducible improvement until multi-seed confirmation (next round).

---

## 8. Status

```text
OPT0_STATUS = OPT0_IMPLEMENTATION_READY
FROZEN_FILES_CHANGED = 0
NEXT = independent audit (TRAE + GLM-5.3)
```

End of report.
