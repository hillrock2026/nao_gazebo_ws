# STAGE0_CLOSURE_COMPLETION_REPORT

Stage 0 Closure Completion — BF-1 / BF-2 / BF-3.

| Field | Value |
|---|---|
| Contract | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` (REV1.1, sha256 `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c`) |
| Role | Implementation Agent |
| Report date | 2026-10-07 |
| Corpus execution window | 2026-10-06 12:45:17 → 14:18:25 (+0800) |
| Repository HEAD | `6d00f90b7d80060f5b42b7d64499699ec3100911` (branch `main`) |
| Frozen code modified | **NONE** |
| Status | **completion evidence — awaiting independent (GLM-5.3) audit. NOT an acceptance.** |

> **Audit availability note.** The task brief states the GLM-5.3 Independent Adversarial Audit has
> passed. **No such report exists in this repository** (re-searched: only the Phase A GLM audit and the
> `OPT*` audits are present). This report therefore does **not** claim audit PASS; it records the
> implementation and its evidence, and lists explicit audit entry points (§8).

---

## 1. BF-1 — R-1 contract durability

**Problem.** The Stage 0 contract chain was untracked, so `git clone` could not recover it. (Precision:
`git ls-files` was not empty — 674 files were tracked; the *Stage 0* files were untracked. The
substantive finding stands.)

**Fix — Option A (precise exception).** One negation appended at the **end** of `.gitignore`:

```
!docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md
```

**Recommendation and rationale (A over B).**
- **A (chosen):** a single, scoped negation. Git rule order is last-match-wins, so placing it last
  re-includes exactly one file. Recovery becomes **deterministic** — a plain `git add` works, with no
  dependence on the owner remembering `-f`.
- **B (`git add -f`):** leaves `.gitignore` untouched but makes durability depend on an out-of-band
  step that is easy to omit and invisible in the tree. It also risks the file being silently
  re-ignored for future edits.
- Neither option deletes the broad rule; every other review/audit/RFC/acceptance pattern is intact
  (verified: `STAGE0_REPOSITORY_BASELINE_AUDIT.md` and `STAGE0_PARAMETERIZED_GAIT_RFC.md` remain
  ignored; `OPT2A_INDEPENDENT_AUDIT_REPORT.md` remains ignored).

**Commit scope (owner decision, 2026-10-06 → metadata + metrics).** Two further scoped rules exclude
bulky, regenerable data:

```
dataset/stage0/**/trace/
dataset/stage0/**/dataset/samples.jsonl
dataset/_superseded_predeclaration/
```

**Verification performed** (see §5 for outputs): `git status`, `git diff`, `git check-ignore`,
`git ls-files --others --exclude-standard`, `git add --dry-run`.

**Residual: the commit itself is an owner action** and was **not** performed (per the brief, and per
the project's standing rule that committing is the owner's). R-1 is therefore *prepared*, not yet
*durable*. §4 lists exactly what waits to be committed.

---

## 2. BF-2 — Stage 0 corpus completion

**Problem.** Only Ring 0 (anchor + 3 seeds) existed; the RFC corpus is Ring 0–3; no pre-registered
θ list and no `theta_list_hash`.

**Fix.**

| Step | Action | Artifact |
|---|---|---|
| 1 | Pre-registered θ list with `theta_id`, `ring_id`, 27-key `theta`, `sampling_reason`, `declared_before_execution=true` | `dataset/stage0/stage0_theta_list.json` |
| 2 | `theta_list_hash` per RFC method — SHA-256 over `json.dumps(theta_list, indent=2, sort_keys=True, allow_nan=False)`, UTF-8, no trailing newline | `844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf` |
| 3 | Driver generalized: accepts Ring 0–3, loads and **verifies** the full θ list before any launch, no adaptive logic | `tools/run_stage0_dataset.py` |
| 4 | Full corpus executed; failures preserved | `dataset/stage0/ring{0,1,2,3}/` |

**θ list composition** (declared and hashed **before** the first episode):

| Ring | Meaning | θ count | Episodes (×3 seeds) |
|---|---|---|---|
| 0 | anchor re-baseline | 1 | 3 |
| 1 | local perturbation, one axis at a time, both directions | 14 | 42 |
| 2 | wider single-axis spans + pre-declared 2-axis combinations | 10 | 30 |
| 3 | pre-declared failure-boundary coverage grid | 7 | 21 |
| **Σ** | | **32** | **96** |

**Non-adaptive guarantees (RFC §7 / R-2).** The driver iterates the fixed list only. There is no
result-dependent expansion, no early stopping, no failure-driven selection, no reordering, no
opt/pareto/best-candidate logic. Tamper detection is fail-closed: mutating the list after declaration
raises `theta_list_hash mismatch … STOP` (verified, §5).

**Failures preserved.** 96/96 executed and retained. 26 episodes have `walking_pass=false` and 17 have
measured `fall=true`; **none were dropped**. `corpus_index.json` reports
`policy.dropping = "NEVER — failure episodes are preserved"`.

**Provenance hygiene.** The 3 episodes from the earlier Ring 0 run (`ring0_anchor_s{0,1,2}`) predate
the θ list (`theta_list_hash=null`). Per owner decision they were **moved, not deleted**, to
`dataset/_superseded_predeclaration/` so the corpus root contains exactly the pre-registered 96.

---

## 3. BF-3 — Index hash chain

**Problem.** `corpus_index.json` hash drifted on every rebuild (`b8a15cd7…` → `7212ccbf…` →
`a296f439…` → `12deaaf1…`), because the builder embedded a volatile `generated_at`; it also wrote
directly to the target file (no atomicity).

**Fix (Option A — separate the volatile metadata).**

1. **Atomic write:** serialized to a temp file in the same directory, `flush` + `fsync`, then
   `os.replace`, followed by a directory `fsync`. A reader never sees a partial index.
2. **Deterministic content:** `corpus_index.json` now contains **no volatile field**. It carries
   `content_hash` = SHA-256 over its own deterministic content (excluding `content_hash`), making it
   self-verifying. `generated_at` moved to a sidecar `corpus_index.meta.json`, which records
   `generated_at`, `index_sha256` and `content_hash`.

**Result.** Two consecutive rebuilds produce byte-identical output:

```
run1 sha256 = 260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544
run2 sha256 = 260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544
```

`content_hash = eaf8795961cad9933ed7dd5aaebe928d220a0d44289624904feec3135d1c6c6d`

---

## 4. Files awaiting owner commit

`git status --porcelain --untracked-files=no` → 1 tracked modification. `git diff --stat`:

```
 .gitignore | 21 +++++++++++++++++++++
 1 file changed, 21 insertions(+)
```

New/untracked files to be added (**1261 files, 8.1 MB** — metadata only; no `trace/`, no `samples.jsonl`):

| Group | Path | Notes |
|---|---|---|
| Ignore rules | `.gitignore` (modified) | BF-1 negation + corpus commit scope |
| Contract | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | now trackable (BF-1) |
| Contract pin | `docs/STAGE0_CONTRACT_MANIFEST.md` | RFC path + SHA-256 + revision + status |
| Closure docs | `docs/STAGE0_IMPLEMENTATION_CLOSURE.md`, `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md`, `docs/STAGE0_CLOSURE_COMPLETION_REPORT.md` | |
| Tools | `tools/run_stage0_dataset.py`, `tools/build_stage0_index.py` | orchestration only |
| θ list | `dataset/stage0/stage0_theta_list.json` | 32 θ |
| Index | `dataset/stage0/corpus_index.json`, `corpus_index.meta.json` | |
| Per-episode metadata | `dataset/stage0/records/*`, `dataset/stage0/ring{0,1,2,3}/**` (excluding `trace/`, `dataset/samples.jsonl`) | 96 episodes |

Excluded by design (regenerable from the committed θ list + driver): `**/trace/`, `**/dataset/samples.jsonl`,
and `dataset/_superseded_predeclaration/`.

Suggested staging (explicit paths, not `git add -A`) and a commit message are provided in §9.

---

## 5. Hash list

| Artifact | SHA-256 |
|---|---|
| `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c` |
| `.gitignore` | `3a16c551d37bdfdc65f08f2c542fd5af7afc1cd1b244c37266cd70b00d25fb36` |
| `tools/run_stage0_dataset.py` | `bf21e3b11203721a5ac4887268ae751a9503f452ed0a9a1b5c9ab83acce5eb41` |
| `tools/build_stage0_index.py` | `2f288c4e2bafa2e847f09247930ae03472aebdc46b096a109ba7065699b4dfd3` |
| `dataset/stage0/stage0_theta_list.json` | `dd52135eed21f3b20c330a2a61e90e89a1f9c13690bddd58330762dc45b9c04a` |
| `dataset/stage0/corpus_index.json` | `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544` |
| `dataset/stage0/corpus_index.meta.json` | `b2e7fc549e1109e96bcb3f57607a36141e1d4edc1733e1414e3f1777a9a5b12a` |
| `docs/STAGE0_CONTRACT_MANIFEST.md` | see that file (updated post-completion) |
| `stage0_anchor_theta_hash` | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` |
| `historical_fwdf08_artifact_hash` (reference) | `70855de120a1d99faa53be713886b90a5a336ff6e651b5830b97815871fa2182` |
| `theta_list_hash` | `844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf` |

---

## 6. Validation results

**Contract**

| Check | Result |
|---|---|
| `stage0_anchor_theta_hash` | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` — matched by all 3 ring0 episodes |
| `len(theta) == 27` | ✅ all 96 resolved configs have exactly 27 keys; key sets exact (`theta_contract.ok = true`, 96/96) |
| θ list integrity | 32 θ, `theta_list_hash` verified; tamper → fail-closed STOP |
| Anchor present in list & resolves to contract hash | ✅ |

**Dataset**

| Check | Result |
|---|---|
| `ring0/ ring1/ ring2/ ring3/` present | ✅ `rings_present = ['ring0','ring1','ring2','ring3']` |
| Episodes per ring | ring0=3, ring1=42, ring2=30, ring3=21 (**96**) |
| Required files per episode (`run_config.json`, `provenance.json`, `dataset/samples.jsonl`, `episode_integrity.json`) | ✅ **no missing files** (96/96) |
| Execution success (`episode_integrity = PASS`) | **96/96** |

**Sampling ordering (declared-before-execution proof)**

| Check | Result |
|---|---|
| `theta_list declared_at` | `2026-10-06T12:43:46+0800` |
| earliest episode `wall_start` | `2026-10-06T12:45:17+0800` |
| latest episode `wall_start` | `2026-10-06T14:18:25+0800` |
| `declared_at < every episode start` | ✅ **true for all 96** |
| Every record carries the same `theta_list_hash` | ✅ 96/96 |

**Provenance**

| Check | Result |
|---|---|
| `git_commit` | `6d00f90b7d80060f5b42b7d64499699ec3100911` (96/96) |
| `source_sha256` | ≥7 entries per episode (6 frozen sources + driver) |
| `theta_hash` | present, **32 unique** (one per θ) |
| `seed` / `gz_seed` | sets `{0,1,2}`; `seed_verified = true` for 96/96 |
| `ring_id` | present, 96/96 |

**Index stability**

| Check | Result |
|---|---|
| Two consecutive `build_stage0_index.py` runs | identical: `260fd9e7…0544` (both) |
| Volatile field inside `corpus_index.json` | none (`generated_at` only in the sidecar) |

**Corpus outcome (reported, not ranked)**

| Metric | Value |
|---|---|
| `execution_success` | 96/96 |
| `behavior_success` true / false / unknown | **61 / 26 / 9** |
| analyzer `measurable` | 96/96 |
| measured falls | 17 |
| forward displacement (all measurable) | mean 0.10016 m, min −0.38118 m, max 0.32591 m |
| ring0 anchor (r0_anchor_s0/1/2) | integrity PASS ×3, `walking_pass` true ×3, forward 0.127666 / 0.140441 / 0.128053 m (mean 0.132053, std 0.005933), speed mean 0.009111 m/s |

Per-ring forward dispersion widens monotonically (ring0 σ≈0.006 → ring3 σ≈0.165), consistent with
Ring 3 deliberately reaching the failure boundary. Reported as dispersion only — **no ranking, no
best-θ selection, no fitness, no optimization**.

---

## 7. Answers to Q1–Q3

### Q1 — After `git clone`, can a third party recover the Stage 0 contract identity?

**YES — once the owner commits (§4).** The governing RFC is now trackable without `-f`
(`git add --dry-run docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` → adds it), and
`docs/STAGE0_CONTRACT_MANIFEST.md` pins the RFC path, SHA-256, revision and acceptance status, plus the
θ-list and anchor hashes. Caveats: (a) it depends on the pending commit — until then the recovery is
only *prepared*; (b) the baseline **audit** document remains git-ignored by the project's own
convention (not required by BF-1); (c) raw `trace/` and `samples.jsonl` are excluded by owner decision,
so a third party reconstructs the *process* and *metrics* from the commit, and regenerates raw data by
re-running the committed θ list.

### Q2 — Is the Stage 0 corpus complete against RFC Ring 0–R3?

**YES, structurally and procedurally.** All four rings are present (3/42/30/21 = 96 episodes = 32 θ ×
3 seeds); the θ list is pre-registered, hashed and tamper-evident; every episode starts after the
declaration; there is no adaptive logic; and failures are preserved (26 walking failures, 17 measured
falls, 0 dropped). Caveats stated plainly: (a) 9 episodes have `behavior_success = unknown` because the
frozen pipeline omitted `trace/phase4b_summary.json` for them (§8, L1) — this lowers *behaviour label
coverage*, not episode validity; (b) this is a **coverage** corpus, not an optimization corpus — by
design it contains no "best gait".

### Q3 — Are the data conditions met for Stage 1 (3D-LIPM + residual network)?

**The data-production plumbing is ready; sufficiency for Stage 1 modelling is NOT established here and
must not be claimed.** What is in place: a reconstructible `θ → simulation → measurement` chain for 96
episodes, per-sample trajectory state (24 joint positions + velocities + IMU + contacts + command, with
base pose attachable by the documented causal join), measured behaviour for all 96, and full
provenance. What is **not** established: (a) behaviour label coverage is 61 true / 26 false / 9 unknown,
so supervised pairs are not uniformly available; (b) there are **no CoM / ZMP / foot-contact-force
observables** in the schema, which a dynamics-identification stage may require; (c) physics is not
bit-reproducible (cross-seed dispersion is real); (d) the Stage 0 contract is **not yet independently
audited**. Conclusion: Stage 0 has produced an auditable dataset; whether it satisfies Stage 1's
prerequisites is for the Stage 1 RFC (and the pending audit) to decide — **Stage 1 is not started.**

---

## 8. Known limitations / explicit audit entry points

| # | Finding | Notes |
|---|---|---|
| L1 | 9/96 episodes have `gait_outcome.available = false` (missing `trace/phase4b_summary.json` in the frozen trace node). | Frozen-pipeline completion race; integrity is still PASS and analyzer metrics are present. Not worked around (RFC §2.1). |
| L2 | 26 behaviour failures and 17 measured falls are in the corpus. | Intentional (Ring 3 boundary grid) and preserved, not filtered. |
| L3 | The GLM-5.3 Stage 0 adversarial audit is **absent** from the repository. | This report claims no audit PASS. |
| L4 | R-1 durability depends on the pending owner commit. | §4. |
| L5 | `experiment_seed == gazebo_seed` by construction on the canonical runner. | Recorded as separate fields; not a violation. |
| L6 | `config_hash` material was implementation-defined. | Documented in the driver; identical across episodes of the same θ. |
| L7 | Raw `trace/` and `samples.jsonl` are excluded from the commit. | Owner decision; regenerable from the committed θ list. |

---

## 9. Owner commit commands (copy-paste)

```bash
cd /home/liu/workspace/Nao_Program/nao_gazebo_ws

git add .gitignore \
        docs/STAGE0_CONTRACT_MANIFEST.md \
        docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md \
        docs/STAGE0_IMPLEMENTATION_CLOSURE.md \
        docs/STAGE0_ANCHOR_CLOSURE_REPORT.md \
        docs/STAGE0_CLOSURE_COMPLETION_REPORT.md \
        tools/run_stage0_dataset.py \
        tools/build_stage0_index.py \
        dataset/stage0

git status --short
git commit -F- <<'MSG'
stage0: complete parameterized gait dataset closure (Ring0-3)

- Add thin Stage 0 driver (tools/run_stage0_dataset.py) and corpus index
  builder (tools/build_stage0_index.py); orchestration only. The canonical
  runner tools/run_phase_a_episode.py is invoked unmodified; no frozen
  component changed.
- Pre-register the full Ring0-3 theta list (32 theta x 3 seeds = 96 episodes)
  with theta_list_hash 844646f5...; fixed and hashed before any episode ran.
- BF-1: negate the broad RFC ignore rule for the Stage 0 governing RFC only,
  so a fresh clone can recover the contract; every other review/audit rule
  remains intact.
- BF-3: corpus_index.json is now deterministic (volatile generated_at moved
  to a sidecar) and written atomically; its hash is stable across rebuilds.
- Commit scope: metadata + metrics + provenance. Raw trace/ and
  dataset/samples.jsonl are excluded as bulky and regenerable from the
  committed theta list.
MSG

git show --stat --oneline HEAD | head -5
```

---

**Status: Stage 0 Closure Completion delivered as evidence. Awaiting GLM-5.3 Independent Adversarial
Audit. Stage 1 is NOT started and MUST NOT start until Stage 0 receives an independent PASS.**
