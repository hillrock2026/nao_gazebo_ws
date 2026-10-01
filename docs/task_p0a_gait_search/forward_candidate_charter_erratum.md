# Forward-Gait Candidate Charter — Append-only Erratum

Applies to the frozen `docs/task_p0a_gait_search/forward_candidate_charter.md`
(unchanged). This erratum is **append-only provenance**: it corrects *why* the
thresholds were chosen and their range, and it does **not** modify the frozen
charter or overturn the fwdf08 pass under that frozen gate.

Source: `docs/P0-a 单一 Forward-Gait Candidate — GLM-5.3 独立评审报告.txt` §六,
§十二 (P2-2/P3-1/P3-5), and an independent recompute of
`baseline_existing_52_measured.json` over the 51 non-fall measurable episodes.

## E1. `p95 = 0.054` quantile method

The charter's `min_forward_displacement_m = 0.08` justification cited
`|forward|` p95 = 0.054 m. That value is the **lower-rank / nearest-rank**
quantile (no interpolation): the 95th-percentile order statistic.
With linear interpolation the same data gives `p95 = 0.0593 m`. The charter did
not state the method; it used lower-rank. Both are below the 0.08 threshold, so
the choice is immaterial to the conclusion.

## E2. Non-fall `|forward|` outlier

Among the 51 non-fall measurable historical episodes there is **one**
`|forward_displacement| = 0.2313 m` (root `task4_phase2d_active_failure_boundary_
evidence/boundary`, `probe_frequency_high`, seed 1001). Therefore:

```text
min_forward = 0.08 m is above p95 but is NOT an upper bound of the historical
distribution; one historical non-fall run exceeds it.
```

The outlier was not disclosed in the original charter. It does not change the
negative conclusion for A0/A1/A2, and it is not a reproduced candidate (single
run, `path_over_net = 8.8`, `yaw = 16.8°`).

## E3. `path_over_net <= 8` is a new strict candidate gate

The historical non-fall `path_over_net` **median is 33.4** (in-place oscillation).
`path_over_net <= 8` is therefore a **newly set strict candidate gate**, not a
historical-distribution upper bound, and must be read as such.

## E4. Pre-registration timing

The charter file was written **before** all three-seed fwdf08 evidence (mtimes
confirm this) but **after** the single-seed screening runs. It is therefore
**not independently provable** that the thresholds were uninfluenced by the
screening results. Mitigation (independent recompute): the fwdf08 three-replicate
conclusion is unchanged for any `min_forward` in roughly `[0.05, 0.11]`, so the
result is not threshold-sensitive. The frozen gate is applied as written.

## E5. Seed semantics

The original charter/report used the word "seed" for `101/202/303`. These were
**replicate labels**, not a controlled physics RNG seed (gz sim was not started
with `--seed`). See the P1 `P1_charter.md` and `tools/test_p0a_seed_semantics.py`
for the corrected `replicate_id` vs `physics_seed` semantics. This erratum does
**not** rewrite the historical fwdf08 evidence or its recorded `seed` field.

## E6. `truth_check.json` monotonic fraction

The P0-a report's `monotonic_non_decreasing_fraction` (0.807/0.814/0.861) was
produced by a script not previously stored in the repository. It is now
reproduced exactly by `tools/p0a_truth_check.py` with the definition
`count(p[k+1] >= p[k] - 1e-4) / (n_window_samples - 1)` over the blend-excluded
steady window (verified bit-for-bit on the fwdf08 traces). The reviewing
report's different value (0.610) came from a different definition; it is not an
error in the stored `truth_check.json`.

## Boundary

This erratum does not change, and does not permit changing, any frozen threshold,
does not rewrite historical evidence, and does not affect the standing
statements: P0-a unresolved, no three speed tiers, no P0-b, no Gate 2A, no
freeze.
