# STAGE0_CONTRACT_MANIFEST

**Purpose.** Make the Stage 0 contract identity discoverable to a third party **after `git clone`** —
i.e. answer "which RFC revision governed the Stage 0 dataset production?" (R-1 / BF-1).

This manifest is a **controlled file** (matched by no `.gitignore` rule) and is intended to be
committed. It pins the contract documents by content hash, including the RFC, which — although now
trackable — is listed here so the chain is explicit.

Repository: `nao_gazebo_ws`
Recorded: 2026-10-07
Recorded at HEAD: `6d00f90b7d80060f5b42b7d64499699ec3100911` (branch `main`)

---

## 1. Governing contract

| Field | Value |
|---|---|
| **Governing RFC** | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` |
| **Revision** | `REV1.1` (REV1 + R-2 wording closure) |
| **RFC SHA-256** | `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c` |
| **Acceptance status** | **NOT ACCEPTED — awaiting independent audit.** No acceptance token has been emitted. |
| **Supersedes** | `docs/STAGE0_PARAMETERIZED_GAIT_RFC.md` (REV0), SHA-256 `8d83405abb45ddace69bb8fe4e592b790f97e0c5fae2de1a9b5b4fad4ae72f5c` |

The SHA-256 above is over the **exact bytes** of the RFC file. Any edit invalidates this pin and
requires a new manifest revision.

## 2. Supporting documents

| Role | Path | SHA-256 | Status |
|---|---|---|---|
| Stage 0 baseline audit | `docs/STAGE0_REPOSITORY_BASELINE_AUDIT.md` | `3a7ca059f3536d017e7b763e306ba5db1e7da79a02364c7c4ab7c0b17911ceea` | delivered (READY WITH CONDITIONS) |
| RFC REV0 | `docs/STAGE0_PARAMETERIZED_GAIT_RFC.md` | `8d83405abb45ddace69bb8fe4e592b790f97e0c5fae2de1a9b5b4fad4ae72f5c` | superseded |
| RFC REV1.1 (governing) | `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c` | **governing** |
| Implementation closure (Ring 0) | `docs/STAGE0_IMPLEMENTATION_CLOSURE.md` | `d4fc6262c5a29dc09deef3d84f782569c4c1cf3f8bcb8664fe8fd6cb196874cb` | superseded for corpus scope |
| Anchor closure report (Ring 0) | `docs/STAGE0_ANCHOR_CLOSURE_REPORT.md` | `8ede064cb2f36b144e5840bac3ba037f59a137eb37d50e167cbc798b8a72bac6` | superseded (episodes relocated) |
| **Closure completion report (Authoritative)** | `docs/STAGE0_CLOSURE_COMPLETION_REPORT.md` | `17779383101dbed73e185404aa991cd67c256cf6025fc6ced7a31d33abab9914` | **authoritative for corpus scope** |
| Independent audit | — **ABSENT** | — | **not present in the repository** |
| Adversarial audit (GLM-5.3, Stage 0) | — **ABSENT** | — | **not present in the repository** |

## 3. Git-tracking status (BF-1)

| File | Tracking |
|---|---|
| `docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` | **trackable** — re-included by the precise negation `!docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` appended at the **end** of `.gitignore` (last-match-wins). `git add` works without `-f`. |
| `docs/STAGE0_CONTRACT_MANIFEST.md` | trackable (not matched by any ignore rule) |
| `docs/STAGE0_CLOSURE_COMPLETION_REPORT.md` | trackable |
| `docs/STAGE0_REPOSITORY_BASELINE_AUDIT.md` | **ignored** — `.gitignore` `**/*[Aa][Uu][Dd][Ii][Tt]*.md` (project convention; not required by BF-1) |
| `docs/STAGE0_PARAMETERIZED_GAIT_RFC.md` (REV0) | ignored — `**/*[Rr][Ff][Cc]*.md` (superseded) |
| other `OPT*` audits / reviews | ignored (unchanged) |

**Corpus commit scope** (owner decision 2026-10-06 — metadata + metrics only): `.gitignore` also
excludes `dataset/stage0/**/trace/`, `dataset/stage0/**/dataset/samples.jsonl`, and
`dataset/_superseded_predeclaration/`. Raw data is regenerable from the committed θ list + driver.

**Remaining action (owner).** Durability is effective only once the files are **committed**. No commit
was performed by the implementation (committing is a shared-state action left to the owner). See
`docs/STAGE0_CLOSURE_COMPLETION_REPORT.md` §9 for the staging command and commit message.

## 4. Governing anchors

| Item | Value |
|---|---|
| Canonical Stage 0 anchor | `fwdf08-stage0-anchor` (27 keys) |
| `stage0_anchor_theta_hash` | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` |
| `historical_fwdf08_artifact_hash` (26 keys, reference only) | `70855de120a1d99faa53be713886b90a5a336ff6e651b5830b97815871fa2182` |
| `theta_list_hash` (pre-registered Ring 0–3 list, 32 θ) | `844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf` |
| Corpus index (self-verifying) | `dataset/stage0/corpus_index.json` sha256 `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544` |
| θ-hash method | SHA-256 over `json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)`, UTF-8, no trailing newline |
| Canonical runner | `tools/run_phase_a_episode.py` (unmodified) |

## 5. Revision history

| Rev | Date | Change |
|---|---|---|
| 1.0 | 2026-10-06 | Initial manifest (R-1 closure): pins the REV1.1 RFC and supporting documents by SHA-256. |
| 1.1 | 2026-10-07 | BF-1 addendum: RFC now trackable via a precise `.gitignore` negation; corpus commit scope recorded; Closure Completion report added as authoritative. |
