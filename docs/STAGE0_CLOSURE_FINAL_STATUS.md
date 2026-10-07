# STAGE0_CLOSURE_FINAL_STATUS

Stage 0 Closure Final Status — BF-1 git durability resolution and final freeze declaration.

| Field | Value |
|---|---|
| Role | Implementation Agent (closure executor) |
| Report date | 2026-10-07 |
| Closure commit | `045a21fdcf7d8d7261a0ad2fa0ab5e8acf041d73` (branch `main`) |
| Parent (pre-closure HEAD) | `6d00f90b7d80060f5b42b7d64499699ec3100911` |
| Governing contract | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` (REV1.1) — **unmodified** |
| Stage 0 dataset | **unmodified** (no new sampling, no re-ordering, no filtering) |
| Canonical runner | `tools/run_phase_a_episode.py` — **unmodified** (last touched at initial commit `45c16eb`) |

This document records the closure decision only. It does not restate, amend, or re-open any part of
the Stage 0 contract. Authoritative implementation evidence remains
`docs/STAGE0_CLOSURE_COMPLETION_REPORT.md`; the contract pin remains
`docs/STAGE0_CONTRACT_MANIFEST.md` (both committed at the closure commit).

---

## 1. Repository

**commit hash:**

```
045a21fdcf7d8d7261a0ad2fa0ab5e8acf041d73
```

Commit message:

```
stage0: freeze parameterized gait baseline dataset contract
```

**tracked artifact list** (1263 files added at the closure commit):

| Group | Path | Files | Notes |
|---|---|---|---|
| Ignore rules | `.gitignore` | 1 (modified) | BF-1 negation for the governing RFC + corpus commit scope |
| Governing RFC | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | 1 | REV1.1, sha256-pinned |
| Contract pin | `docs/STAGE0_CONTRACT_MANIFEST.md` | 1 | RFC path + SHA-256 + revision + status |
| Closure docs | `docs/STAGE0_IMPLEMENTATION_CLOSURE.md`, `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md`, `docs/STAGE0_CLOSURE_COMPLETION_REPORT.md` | 3 | superseded-for-scope / authoritative evidence chain |
| Stage 0 driver | `tools/run_stage0_dataset.py` | 1 | orchestration only; invokes frozen runner unmodified |
| Index builder | `tools/build_stage0_index.py` | 1 | deterministic, atomic index rebuild |
| θ list | `dataset/stage0/stage0_theta_list.json` | 1 | 32 θ, pre-registered |
| Corpus index | `dataset/stage0/corpus_index.json`, `dataset/stage0/corpus_index.meta.json` | 2 | self-verifying index + volatile sidecar |
| Per-episode records | `dataset/stage0/records/` | 192 | 96 episodes × (`*.record.json` + `*.metrics.json`) |
| Per-episode metadata | `dataset/stage0/ring{0,1,2,3}/**` | 1060 | manifest / provenance / run_config / episode_integrity / collector_summary / launch evidence / sampling configs (34 / 463 / 331 / 232) |
| **Σ** | | **1263** | metadata + metrics only |

Explicitly excluded by pre-declared scope (owner decision, 2026-10-06; regenerable from the
committed θ list + driver):

- `dataset/stage0/**/trace/` (raw streams)
- `dataset/stage0/**/dataset/samples.jsonl` (canonical sample dumps)
- `dataset/_superseded_predeclaration/` (moved-aside pre-declaration episodes)

**working tree status:**

```
git status --short  →  (empty)
```

Clean: no modifications, no staged changes, no untracked non-ignored files. The only remaining
untracked content is deliberately git-ignored project convention material (`OPT*` audit/review
documents, `docs/STAGE0_REPOSITORY_BASELINE_AUDIT.md`, superseded RFC REV0,
`dataset/_superseded_predeclaration/`), none of which is required by BF-1.

**Clone-recovery proof** (fresh local clone `git clone --no-hardlinks` at the closure commit):

| Check | Result |
|---|---|
| Clone HEAD | `045a21fdcf7d8d7261a0ad2fa0ab5e8acf041d73` |
| `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` present in clone | ✅ |
| `docs/STAGE0_CONTRACT_MANIFEST.md` present in clone | ✅ |
| `dataset/stage0/stage0_theta_list.json` present in clone, 32 θ | ✅ |
| `dataset/stage0/corpus_index.json` present in clone | ✅ |
| `tools/run_stage0_dataset.py`, `tools/build_stage0_index.py` present in clone | ✅ |
| Tracked files under `dataset/stage0` in clone | 1255 (identical to source repo) |
| RFC sha256 in clone | `09345946…` — identical to pinned value |
| `corpus_index.json` sha256 in clone | `260fd9e7…` — identical |
| `theta_list_hash` recomputed in clone (RFC method) | `844646f5…` — identical |

A third party that clones the repository now recovers the full Stage 0 contract identity, the
pre-registered θ list, the corpus index, per-episode metrics/provenance, and the regeneration path
for raw data. BF-1 is resolved.

---

## 2. Artifact integrity

All hashes re-verified in the working tree at closure time (and re-verified inside the fresh clone,
§ 1).

| Artifact | Hash method | Value | Status |
|---|---|---|---|
| **RFC hash** — `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | SHA-256 over exact file bytes | `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c` | ✅ matches `STAGE0_CONTRACT_MANIFEST.md` pin and REV1.1 declaration; file unmodified |
| **theta list hash** — `dataset/stage0/stage0_theta_list.json` | `theta_list_hash` (RFC method): SHA-256 over `json.dumps(theta_list, indent=2, sort_keys=True, allow_nan=False)`, UTF-8, no trailing newline, over the `theta_list` entries array | `844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf` | ✅ matches the in-file declaration and the manifest; 32 θ entries; tamper check re-run (fail-closed) |
| theta list file bytes | SHA-256 over file bytes | `dd52135eed21f3b20c330a2a61e90e89a1f9c13690bddd58330762dc45b9c04a` | ✅ recorded for unambiguous file-level pinning |
| **dataset index hash** — `dataset/stage0/corpus_index.json` | SHA-256 over file bytes (deterministic content; no volatile fields) | `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544` | ✅ stable across rebuilds (BF-3) |
| `corpus_index.json` `content_hash` (self-verification) | SHA-256 over own content excluding `content_hash` | `eaf8795961cad9933ed7dd5aaebe928d220a0d44289624904feec3135d1c6c6d` | ✅ recomputed and matched; sidecar `corpus_index.meta.json` agrees |
| `stage0_anchor_theta_hash` | RFC θ-hash method | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` | ✅ (per completion report §6; anchor unchanged) |

Corpus structural integrity (re-counted at closure): 96 episodes (ring0 = 3, ring1 = 42,
ring2 = 30, ring3 = 21 = 32 θ × 3 seeds), 96 `*.record.json` + 96 `*.metrics.json` in
`dataset/stage0/records/`. No episode was added, removed, re-ordered, or relabelled by this closure.

---

## 3. Final decision

```
Stage0 Closure:

PASS
```

Scope of this PASS (explicit):

| Criterion | Evidence |
|---|---|
| BF-1 — contract & corpus recoverable after `git clone` | closure commit `045a21f`; clone-recovery proof § 1 |
| BF-2 — corpus complete per RFC Ring 0–3, pre-registered, failures preserved | 96/96 episodes, `theta_list_hash` verified (§ 2); evidence in `STAGE0_CLOSURE_COMPLETION_REPORT.md` § 2, § 6 |
| BF-3 — index hash chain deterministic and self-verifying | `260fd9e7…` stable, `content_hash` recomputed OK (§ 2) |
| No frozen component modified | `tools/run_phase_a_episode.py` untouched; commit diff contains no pre-existing tracked source changes |
| No contract modification, no new sampling, no runner change | this closure only performed `git add` + `git commit` of existing artifacts |

Standing caveats (carried forward from the completion report, unchanged — they do not block
closure, and none was altered by this action):

1. The GLM-5.3 Stage 0 independent adversarial audit is still **absent** from the repository;
   this closure PASS is an implementation-side durability freeze, not an independent acceptance.
2. 9/96 episodes have `behavior_success = unknown` (missing `phase4b_summary.json` in the frozen
   trace node); episode validity is unaffected (L1).
3. Raw `trace/` and `samples.jsonl` are excluded from the commit by owner decision and are
   regenerable from the committed θ list + driver (L7).
4. Stage 0 is a coverage/baseline corpus by design: no best-θ selection, no fitness, no ranking.

**Stage 0 is frozen. The dataset contract of record is durable. Stage 1 may proceed to RFC
authorization (see `docs/STAGE1_DYNAMICS_DATASET_RFC.md`) — not to implementation, which requires
its own authorization.**
