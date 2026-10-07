# STAGE0_IMPLEMENTATION_CLOSURE

Stage 0 data-production closed loop — implementation closure record.

| Field | Value |
|---|---|
| Contract | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` (REV1.1-DRAFT, sha256 `09345946…4c2c`) |
| Role | Implementation Agent |
| Date | 2026-10-06 |
| Repository HEAD | `6d00f90b7d80060f5b42b7d64499699ec3100911` (branch `main`) |
| Code written | 2 new tools (orchestration only) |
| Frozen code modified | **NONE** (verified, §3) |
| Sampling scope executed | **Ring 0 only** (Rings 1–3 NOT executed) |
| Status | implementation complete; **evidence for independent audit — NOT an acceptance** |

> **⚠ SUPERSEDED for corpus scope (2026-10-07).** This document describes the **Ring 0-only** closure.
> The full pre-registered **Ring 0–3 corpus (32 θ × 3 seeds = 96 episodes)** and the BF-1/BF-2/BF-3
> repairs are recorded in **`docs/STAGE0_CLOSURE_COMPLETION_REPORT.md`**, which is authoritative for
> corpus scope, file/hash lists and validation. Two things below are now out of date and are retained
> only as history: (a) the file hashes in §1 were taken before the driver/index-builder were revised for
> BF-2/BF-3; (b) the episodes it cites as `ring0_anchor_s{0,1,2}` predate the pre-registered θ list and
> were moved to `dataset/_superseded_predeclaration/` (the current anchor episodes are
> `dataset/stage0/ring0/r0_anchor_s{0,1,2}`).

---

## 1. Implemented files

| File | SHA-256 | Purpose |
|---|---|---|
| `tools/run_stage0_dataset.py` | `bf21e3b11203721a5ac4887268ae751a9503f452ed0a9a1b5c9ab83acce5eb41` | thin sampling driver (RFC §8); revised for BF-2 (Ring 0–3) |
| `tools/build_stage0_index.py` | `2f288c4e2bafa2e847f09247930ae03472aebdc46b096a109ba7065699b4dfd3` | corpus index builder (RFC §6.8); revised for BF-3 (atomic + deterministic) |
| `docs/STAGE0_CONTRACT_MANIFEST.md` | see the manifest (self-pinned) | R-1 closure: pins the governing contract |
| `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md` | see the manifest | Ring 0 evidence report (superseded — see banner) |
| `dataset/stage0/corpus_index.json` | `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544` | generated corpus index (96 episodes) |

Contract file edited for closure only: `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md`
(sha256 `09345946…4c2c`) — R-2 Ring 3 wording only (§5).

**Driver design.** `run_stage0_dataset.py` only orchestrates: it validates θ, schedules seeds, invokes
the canonical runner as a subprocess, orchestrates the frozen analyzer read-only, and writes provenance.
It does not generate gait, collect, or analyse. It does **not** derive any metric — it reads metrics
from the frozen analyzer's output and from the episode's own `episode_integrity.json` (RFC §8.2).

**Analyzer orchestration.** The driver invokes `tools/analyze_measured_gait.py` per episode as-is
(`--run <episode_dir> --out <metrics.json>`). The analyzer consumes a Phase A episode directory
unmodified (verified: 3/3 episodes produced `measurable = true`). No analyzer code was changed.

## 2. RFC compliance checklist

| # | RFC requirement | Status | Evidence |
|---|---|---|---|
| A1 | Anchor explicit, 27 keys, no `EXPERIMENT_DEFAULTS`, no `B3-ANCHOR` inheritance | MET | `--check-contract` → 27 keys, 0 errors; `resolved_key_count=27` |
| A2 | θ contract: required keys, unknown-key rejection, producer-range validation | MET | `validate_theta()`; `--check-contract` errors `[]` |
| A3 | One canonical runner; dataset contract; no schema upgrade | MET | only `run_phase_a_episode.py` invoked; schema v1.0.1 unchanged |
| A4 | Provenance: two hashes, `config_hash`, `git_commit`, `source_hash`, seed triple | MET | §6 |
| A5 | No frozen component modified | MET | §3 |
| A6 | Static pre-registration; Ring 3 non-adaptive | MET | Ring 0 only executed; §5 |
| A7 | Preservation (27-param, `sway_sign=-1`, `gait_phase_mode=legacy`, no optimization/learning) | MET | §6, §8 |
| §4.3 | Unknown-key rejection is fail-closed | MET | any key outside the 27 → `ContractError` before launch |
| §4.5 | `max_delta_per_cycle` not a sampling dimension | MET | listed in `NON_EXPOSED`; rejected if present in θ |
| §6.4 | Base-pose tolerance frozen at 0.02 s | MET | `BASE_POSE_TOLERANCE_SEC = 0.02`, recorded per episode |
| §8.2 | Driver MUST NOT derive metrics | MET | metrics read from frozen analyzer / integrity artifacts |
| §8.4 | Output root outside frozen evidence | MET | `check_out_root()`; root is `dataset/stage0/` |

## 3. Unchanged frozen files (integrity proof)

`git status --porcelain --untracked-files=no` → **empty** (no tracked file modified). Content hashes of
the frozen components match the values recorded in the surviving Phase A closure episode provenance
(`docs/task_p0a_gait_search/phase_a_closure_evidence/closure_01/provenance.json`), i.e. they are
byte-identical to the components that produced the reviewed Phase A episodes:

| Frozen file | SHA-256 | Matches Phase A provenance |
|---|---|---|
| `src/nao_gz_sim/scripts/phase4b_producer.py` | `aab60f6fbb18b657736ebf9d7633e65b3d0df021c564cddb537bb806875bd5eb` | YES |
| `src/nao_gz_sim/scripts/phase4d_collector.py` | `af3da50cd807922a54bd9ae7cba831d67e46988cba10145759c08d7435a44f68` | YES |
| `src/nao_gz_sim/scripts/nao_trajectory_schema.py` | `bd4b2bcdb7314ad96420649e4e24d8d3afc97d6b622e3741eadef7de1fa45f3d` | YES |
| `src/nao_gz_sim/launch/nao_sim.launch.py` | `2ed565145441df601455e14a9ab4209333f95264a0e6bb65944373201f2ad787` | YES |
| `tools/run_phase_a_episode.py` (canonical runner) | `a024140f294a293b0fb8503e6f53960028d8919c516f1cb640891d9fc297d6fb` | YES |
| `tools/run_p0a_gait_candidate.py` (not invoked) | `00aef7129d65ad78b1bbf219bcec9e9354547c6c0bd8d1ba4fc5948936b820af` | (non-canonical) |
| `tools/analyze_measured_gait.py` | `1bde955fddcf54605e26cf773a44be808efc061519762c4469c9fa63ab577f6b` | (analyzer, unmodified) |

New untracked files are exactly: `dataset/`, `docs/STAGE0_CONTRACT_MANIFEST.md`,
`tools/build_stage0_index.py`, `tools/run_stage0_dataset.py` (plus this report).

## 4. R-1 closure — contract identity

- **Method chosen: Plan B** (controlled manifest). The contract documents are **git-ignored**
  (`.gitignore:27` `**/*[Rr][Ff][Cc]*.md`; `.gitignore:24` `**/*[Aa][Uu][Dd][Ii][Tt]*.md`). Plan A
  (tracking the RFC) would require modifying `.gitignore`, which is outside the permitted change set
  (task §2).
- Created `docs/STAGE0_CONTRACT_MANIFEST.md` recording: RFC path, **RFC SHA-256**
  (`09345946…4c2c`), revision (`REV1.1-DRAFT`), acceptance status (**NOT ACCEPTED — awaiting
  independent audit**), the two anchor hashes, and the supporting-document hashes. This file is **not**
  ignored (verified) and is the clone-visible pointer to the governing contract.
- **Residual action (owner):** R-1 becomes durable only once the manifest is **committed**. No commit
  was made (committing is a shared-state action left to the owner). Until then R-1 is *recorded*, not
  *durable*.

## 5. R-2 closure — Ring 3 wording

`docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` §7 and §7.1 amended (and the revision marked
`REV1.1-DRAFT`; a changelog row added). Ring 3 is now explicitly:

- declared in the **same pre-registered θ list** as Rings 0–2 and **fixed before any episode of any
  ring executes**;
- **not** derived from, selected by, extended by, narrowed by, or reordered by Ring 1 / Ring 2 results;
- **not** expanded, adjusted, retargeted, or re-seeded because of any observed failure episode;
- free of adaptive-search language; the section now explicitly *prohibits* terms such as
  "push until" / "stop at the first boundary" / "continue in the direction that…" / "expand based on…".

The residual dependency that motivated R-2 ("chosen … on the axes that Rings 1–2 motivated") was
removed. The only remaining occurrences of the prohibited phrases are inside the changelog (describing
the fix) and inside the explicit prohibition clause.

## 6. Anchor execution result, hash and provenance

Full evidence in `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md`. Summary:

| Seed | Integrity | forward (m) | speed (m/s) | `theta_hash` match | seed verified | `walking_pass` |
|---|---|---|---|---|---|---|
| 0 | PASS | 0.133878 | 0.009244 | true | true | true |
| 1 | PASS | 0.124370 | 0.008587 | true | true | true |
| 2 | PASS | 0.133935 | 0.009243 | true | true | **UNKNOWN** |

- **Hash verification:** all 3 resolved θ hash to the contract value
  `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998`; 27 keys each; identical
  `config_hash`. No mismatch; the hash and the parameters were never altered.
- **Provenance verification:** `git_commit = 6d00f90…`, 7 `source_sha256` entries per episode,
  `seed_verified = true` for all (physics seed proven from the live `gz sim` argv in
  `gz_sim_cmd.actual.txt`), identical `config_hash` across the 3 episodes.
- **Cross-seed dispersion:** forward 0.130728 ± 0.004496 m; speed 0.009025 ± 0.000310 m/s. Reported as
  dispersion only (RFC §5.3) — no determinism or "no regression" claim.
- **Rings 1–3: NOT executed** (Ring 0 first, RFC §7.3).

## 7. Known limitations

| # | Limitation | Impact | Handling |
|---|---|---|---|
| L1 | Seed 2's `trace/phase4b_summary.json` is **absent**, so the frozen integrity tool reports `gait_outcome.available = false` for that episode. | `behavior_success` is `true/true/unknown` (not 3/3). | Reported as-is, not worked around. Cause is a completion race inside the **frozen** pipeline (collector commits at `duration_complete`; the trace node may be reaped before `finalize()`). RFC §2.1 forbids modifying it. Flagged for the audit. |
| L2 | `R-1` durability requires a commit (not performed). | Manifest is clone-visible only after commit. | Owner action (§4). |
| L3 | The driver was executed end-to-end **only for Ring 0**. Ring 1–3 code paths exist but are unexercised. | Untested paths: `--config` input beyond the anchor, non-anchor θ. | `--ring != 0` is refused by design in this closure. |
| L4 | `experiment_seed` and `gazebo_seed` are equal by construction (`run_phase_a_episode.py:508` passes `gz_seed = --seed`). | The two RFC concepts are recorded separately but always coincide on the canonical runner. | Recorded as separate fields with the construction noted; not a violation. |
| L5 | The `--check-contract` prediction of the resolved θ replicates the canonical runner's `resolve_config` rather than calling it, because the runner exposes no resolution-only entry point. | A future change to `FROZEN_GAIT` would change the true hash while the anchor constant stays fixed. | Mitigated: the driver also hashes the **actual** `run_config.json` post-run and fails on mismatch; and `--check-contract` is a pre-flight only. |
| L6 | `config_hash` is defined by this implementation (canonical θ + the frozen `LAUNCH_BASE` template); the RFC left the exact material to the implementation. | Cross-implementation reproducibility of `config_hash` is not guaranteed. | Definition documented; identical across the 3 episodes. |

## 8. Dataset rules honoured

| Rule | Status |
|---|---|
| No ranking | `corpus_index.json` orders by `episode_id` only; `policy.ranking = NONE` |
| No best-gait selection | `policy.selection = NONE`; no objective/fitness anywhere |
| Failures preserved | 3 executed / 3 indexed / 0 dropped; `episodes_without_record = 0` |
| Execution vs behavior separated | `status.execution_success` and `status.behavior_success` are independent fields |
| No optimization | no search, no score, no CMA-ES/objective |

## 9. Validation checklist (task §11)

**Contract**
- [x] RFC tracked → *not git-tracked* (ignored); pinned by `docs/STAGE0_CONTRACT_MANIFEST.md` instead → R-1 Plan B
- [x] R-1 closed (Plan B + documented residual commit action)
- [x] R-2 closed
- [x] canonical runner only (`tools/run_phase_a_episode.py`)
- [x] no `run_p0a_gait_candidate.py` execution (only its frozen `LAUNCH_BASE` is imported read-only)

**Theta**
- [x] 27 keys (`resolved_key_count = 27`, 3/3)
- [x] theta hash matches (`28e795b2…4998`, 3/3)
- [x] no new parameters

**Provenance**
- [x] `git_commit` (`6d00f90…`)
- [x] `source_hash` (7 entries/episode)
- [x] `experiment_seed` / `gazebo_seed` (recorded separately; `seed_verified = true`)

**Dataset**
- [x] failures preserved (0 dropped)
- [x] no ranking
- [x] no fitness
- [x] no optimization

## 10. Evidence for audit

| Evidence | Path |
|---|---|
| Governing contract + hash | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md`, `docs/STAGE0_CONTRACT_MANIFEST.md` |
| Driver / index source | `tools/run_stage0_dataset.py`, `tools/build_stage0_index.py` |
| Anchor evidence | `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md` |
| Episodes (3) | `dataset/stage0/ring0/ring0_anchor_s{0,1,2}/` |
| Per-episode records | `dataset/stage0/records/ring0_anchor_s{0,1,2}.record.json` |
| Analyzer outputs | `dataset/stage0/records/ring0_anchor_s{0,1,2}.metrics.json` |
| Corpus index | `dataset/stage0/corpus_index.json` |
| Reproduce (offline) | `python3 tools/run_stage0_dataset.py --check-contract` |

**Status: implementation complete. This document is evidence, not acceptance.** The Stage 0 loop is
`θ → gait generator → Gazebo → episode → measured behavior` with third-party-recoverable provenance;
it does not claim the robot walks faster, and it does not authorize Rings 1–3.
