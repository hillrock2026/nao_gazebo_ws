# STAGE0_ANCHOR_CLOSURE_REPORT

Ring 0 — anchor re-baseline of `fwdf08-stage0-anchor`.

| Field | Value |
|---|---|
| Contract | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` (REV1.1-DRAFT, sha256 `09345946…4c2c`) |
| Canonical runner | `tools/run_phase_a_episode.py` (unmodified, sha256 `a024140f…d6fb`) |
| Driver | `tools/run_stage0_dataset.py` |
| Ring | 0 (anchor re-baseline) — **Rings 1–3 NOT executed** |
| Seeds | `0`, `1`, `2` (RFC §5: ≥3 seeds) |
| Output root | `dataset/stage0/` |
| Run date | 2026-10-06 |
| Repository HEAD | `6d00f90b7d80060f5b42b7d64499699ec3100911` |
| Episodes | 3 executed, 3 retained, 0 dropped |

> **⚠ SUPERSEDED (2026-10-07).** This report covers the **first, pre-registration Ring 0 run**. Those
> episodes (`ring0_anchor_s{0,1,2}`) were executed **before** the pre-registered θ list existed
> (`theta_list_hash = null`) and were therefore **moved** — not deleted — to
> `dataset/_superseded_predeclaration/`. The current, pre-registered anchor evidence is
> `dataset/stage0/ring0/r0_anchor_s{0,1,2}` (θ hash `28e795b2…4998`, integrity PASS ×3,
> `walking_pass` true ×3, forward mean 0.132053 m, std 0.005933 m). For the authoritative corpus,
> file/hash lists and validation see **`docs/STAGE0_CLOSURE_COMPLETION_REPORT.md`**.

**Purpose.** Establish the Stage 0 baseline for the frozen anchor θ under explicit physics-seed wiring
and full provenance — the pre-condition before any other ring may run (RFC §7.3).

---

## 1. Anchor identity verification

| Check | Result |
|---|---|
| Declared θ key count | **27** (RFC §3.3) |
| Resolved θ key count (`run_config.json`) | **27** (no extra/missing keys) |
| `theta_hash` — all 3 episodes | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` |
| Matches `stage0_anchor_theta_hash` (RFC §3.2) | **YES (3/3)** |
| `config_hash` — all 3 episodes | `1c086db391659b9675002a9fef7faf03829c0ffdfa978d7237f1889fe38be553` (identical) |
| `sway_sign` / `gait_phase_mode` | `-1` / `legacy` (RFC §3.3, held) |

The θ hash is identical across all three seeds, i.e. the anchor θ was resolved byte-for-byte the same
each time. **No hash mismatch occurred; no hash and no parameter was changed.**

## 2. Per-seed results

Metrics are read from the frozen analyzer (`tools/analyze_measured_gait.py`, orchestrated read-only);
`gait_outcome` is read from the episode's own `episode_integrity.json`.

| Seed | Episode dir | `theta_hash` | Integrity | Termination | forward (m) | speed (m/s) | path/net | max step (m) | fall | `walking_pass` |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | `dataset/stage0/ring0/ring0_anchor_s0` | `28e795b2…4998` | PASS | normal_completion | 0.133878 | 0.009244 | 5.216 | 0.001736 | false | **true** |
| 1 | `dataset/stage0/ring0/ring0_anchor_s1` | `28e795b2…4998` | PASS | normal_completion | 0.124370 | 0.008587 | 5.417 | 0.001840 | false | **true** |
| 2 | `dataset/stage0/ring0/ring0_anchor_s2` | `28e795b2…4998` | PASS | normal_completion | 0.133935 | 0.009243 | 9.449 | 0.004893 | false | **UNKNOWN** |

- `measurable = true` for **3/3** episodes (frozen analyzer).
- `fall = false` for **3/3** episodes (measured evidence).
- `walking_pass` available for **2/3**; **1/3 UNKNOWN** — see §4.

## 3. Cross-seed dispersion (RFC §5.3 — coverage, not bit reproducibility)

| Metric | n | mean | std | min | max |
|---|---|---|---|---|---|
| `forward_displacement_m` | 3 | 0.130728 | 0.004496 | 0.124370 | 0.133935 |
| `actual_speed_mean_mps` | 3 | 0.009025 | 0.000310 | 0.008587 | 0.009244 |
| `path_over_net` | 3 | 6.6938 | 1.9498 | 5.2156 | 9.4489 |
| `max_single_step_m` | 3 | 0.002823 | 0.001464 | 0.001736 | 0.004893 |

These are **cross-seed dispersions**, reported as such (RFC §5.3). No determinism, reproducibility, or
"no regression" claim is made. The forward displacement is consistent with the historical fwdf08
result (mean 0.1235 m, std 0.0047 m) — same order, slightly higher mean.

## 4. Observation — `walking_pass` unavailable for seed 2 (reported, not worked around)

`ring0_anchor_s2/episode_integrity.json` reports:

```json
"gait_outcome": {"available": false, "fall_detected": null,
                 "note": "gait outcome is reported, not an integrity verdict"}
```

Root cause (verified): the seed-2 trace directory is **missing `trace/phase4b_summary.json`**
(`body_pose_trace.jsonl`, `clock_trace.jsonl`, `command_trace.jsonl`, `contact_trace.jsonl`,
`imu_trace.jsonl`, `state_trace.jsonl` are all present). The frozen integrity tool
(`tools/nao_episode_integrity.py:330-332`) returns `available: false` when that file is absent, rather
than inventing a verdict — correct, non-fabricating behaviour.

This does **not** affect: θ contract, seed verification, `episode_integrity` (PASS), sample dataset, or
the analyzer metrics (which read `body_pose_trace.jsonl`, present). It affects only the secondary
`gait_outcome` channel.

Impact on this report: `behavior_success` is `true / true / unknown`, **not** `true / true / true`.
This is stated deliberately; it is **not** a pass.

Cause is inside the frozen pipeline (a completion race between the collector's `duration_complete`
commit and the trace node's `finalize()`), which Stage 0 MUST NOT modify (RFC §2.1). Recorded for the
audit — see `docs/STAGE0_IMPLEMENTATION_CLOSURE.md` §7.

## 5. Seed verification evidence (RFC §5.2)

Each episode's `gz_sim_cmd.actual.txt` — read from the live `/proc` argv, not reconstructed — proves the
physics seed actually reached `gz sim`:

```
seed 0: gz sim -v 4 -r -s --seed 0 --physics-engine gz-physics-dartsim-plugin …/nao_world.sdf
seed 1: gz sim -v 4 -r -s --seed 1 --physics-engine gz-physics-dartsim-plugin …/nao_world.sdf
seed 2: gz sim -v 4 -r -s --seed 2 --physics-engine gz-physics-dartsim-plugin …/nao_world.sdf
```

`seed_verified = true` for all three (RFC §9.2.4 satisfied).

## 6. Provenance verification

| Field | Value (all 3 episodes) |
|---|---|
| `git_commit` | `6d00f90b7d80060f5b42b7d64499699ec3100911` |
| `source_sha256` entries | 7 (6 frozen sources from episode provenance + this driver) |
| `experiment_seed` / `gazebo_seed` | `0/0`, `1/1`, `2/2` |
| `seed_verified` | `true`, `true`, `true` |
| `config_hash` | identical across the 3 episodes |

**Note (RFC §3.5).** The historical fwdf08 runs wired no physics seed. These Stage 0 anchor runs are a
**re-baseline** of the same θ under explicit seed wiring; they are not a byte-identical reproduction of
the historical runs.

## 7. Artifacts

| Artifact | Path |
|---|---|
| Episodes | `dataset/stage0/ring0/ring0_anchor_s{0,1,2}/` |
| Per-episode records | `dataset/stage0/records/ring0_anchor_s{0,1,2}.record.json` |
| Analyzer outputs | `dataset/stage0/records/ring0_anchor_s{0,1,2}.metrics.json` |
| Runner logs | `dataset/stage0/ring0/ring0_anchor_s{0,1,2}.runner.log` |
| Effective config | `dataset/stage0/ring0/stage0_sampling_config.json` |
| Corpus index | `dataset/stage0/corpus_index.json` |

## 8. Status

**Ring 0 COMPLETE.** All three seeds produced coherent episodes with the contract θ hash, verified
physics seeds, and full provenance. No fall. `walking_pass` available for 2/3 (seed 2 UNKNOWN, §4).

This report is **evidence for independent audit**. It is not an acceptance, and it does not authorize
Rings 1–3.
