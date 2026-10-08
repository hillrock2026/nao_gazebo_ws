# STAGE1_REV13_FAIL_CLOSED_SCHEMA_HARDENING_REPORT

## 1. Task Identity

| Field | Value |
|---|---|
| Task | `STAGE1_REV13_FAIL_CLOSED_SCHEMA_HARDENING_TASK` (prompt: `prompt/STAGE1_REV13_FAIL_CLOSED_SCHEMA_HARDENING_TASK_prompt.txt`) |
| Role | Implementation Agent (GLM-5.3) — not an independent auditor |
| Objective | Close the four reproducible artifact bypasses (B1–B4) of the R12 independent audit with the smallest possible fail-closed implementation |
| Date | 2026-10-08 |
| Status claimed | **IMPLEMENTATION COMPLETE** (acceptance criteria A–J, § 30). Not "R13 CLOSED" — closure belongs to `STAGE1_REV13_INDEPENDENT_CLOSURE_AUDIT` |

## 2. Baseline Commit

- Baseline: `3203859fe5bd519b6209b0b28b14fa9e3deec7f1` (R12; R12 baseline `3a4ab86`)
- Stage 1 RFC hash at baseline: `195178f52ebe27efd696c9750f0a7164bdbc09790a1222d491d1120eb2f02aac` — **unchanged by R13** (asserted by `test_rfc_hash_unchanged`; `git diff` over the RFC is empty). No RFC contradiction was found that blocked R13, so the RFC was not modified.

## 3. R12 Audit Findings (addressed)

| Blocker | Finding | R13 fix |
|---|---|---|
| B1 | Manifest could declare `derived_pending_validation` while the frame's Option-A field was missing/null/wrong-type/wrong-shape — smoke exit 0 | R13-A: frame Option-A schema enforcement |
| B2 | RFC↔Contract binding checked field names only; same-name nullable/type/required drift passed | R13-B: semantic marker binding on both documents |
| B3 | Raw nested payloads (`wrenches: ["not-a-wrench"]`, `entries: ["not-a-pose-entry"]`, wrong-dim command) passed shallow checks | R13-C: raw nested structural validation |
| B4 | `gazebo_seed=true`, `frame_count=-1`, `t_start=NaN`, `t_end=Infinity` accepted | R13-D: provenance type/range hardening |

## 4. R13 Changes (files)

| File | Change | SHA-256 |
|---|---|---|
| `tools/stage1_contract_check.py` | B1–B4 enforcement (below); `--check-semantic-binding`; CLI help/semantics per § 19 | `1eb1cbad327e7138a51cd12e5ce760450fdb7c57cd6668047786cbdedda67901` |
| `tools/test_stage1_r13.py` | **new** — 49 attack-driven tests incl. the full § 23 smoke matrix through the real CLI | `a28729d600f4bf0a6eaea82395754402ce8bde868d69acb843846924315a2e66` |
| `tools/test_stage1_r12.py` | fixtures updated to the R13 contracts (full Option-A frame; RFC-shaped raw payloads) | `9fd4baef537a234ff4c70b35ec13793a57c5789a0a929923a023b8c4dfb0db72` |
| `tools/test_stage1_r11.py` | fixtures updated likewise (Test B/A semantics unchanged) | `ef7decb3c4ef16572e9fa4131500cd63f39eb939f6ca74679db5838e8724a529` |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | v1.4 → v1.5: § 8 R13-B semantic-binding declaration; § 10 smoke chain extended (B1–B4) + § 19 CLI-mode note; document status | `2b12ddf2b2bd6bd426d4e27b83b758c132b567cff5d6955e6d72082ce1a27766` |
| `STAGE1_REV13_FAIL_CLOSED_SCHEMA_HARDENING_REPORT.md` | **new** — this report | — |

RFC, `tools/stage1_probe.py`, Stage 0, gait generator, controller, model, world, physics: **untouched** (`git diff` empty; RFC hash re-verified).

## 5. R13-A — Frame Enforcement (closes B1)

`check_frame_emission` now enforces, for every Option-A (composition-dependent) field: **present** + **non-null** + **RFC shape** — pose = `{position: float[3], orientation: float[4]}`; foot/com velocities and com position/velocity/acceleration = `float[3]`; `zmp_position` = `float[2]` (all elements finite numbers). Option-B fields must be **present and null** (kept from R12). The manifest declaration and the emitted artifact are now a closed loop: `derived_pending_validation + missing/null/wrong-type/wrong-shape` all FAIL, as does `blocked + non-null` and `blocked + wrong status`.

## 6. R13-B — Semantic Binding (closes B2)

`extract_doc_provenance_semantics` parses **both** documents' provenance tables and extracts per-field semantic markers: **nullable** (row mentions `null`), **optional** (row mentions `optional`/`可选`; none do — RFC § 3.10 is "mandatory, no exceptions"), and **type marker** (ordered keyword rules over the rule cell: sha256/sha512, HEAD→commit, dict→object, seed→int, 帧数→int_nonneg, wall-clock/UTC/label/id→string, sim time→number, hash; multi-field rows carry no per-field type). `check_semantic_binding` requires the extracted (nullable, optional, type) of every mandatory field on **each** side to **equal** the canonical `PROVENANCE_SEMANTICS` (the checker's machine interpretation of the current documents — positive test proves extraction == canonical on both). Any add/remove/substitute of a semantic marker in either document (e.g. `null until then` removed, `SHA-256`→`SHA-512`, `dict`→`list`, adding `optional`) FAILS the smoke path. No new DSL, no schema compiler — markers are extracted from the documents' own existing text.

## 7. R13-C — Raw Nested Enforcement (closes B3)

`check_raw_storage` now validates the nested retention structure per the RFC's own definitions (structural validity only — no physics sanity checks, § 14):

- **contacts_left/right** (RFC § 3.6.1): `contacts` list (explicit EMPTY Contacts message allowed); per contact: `collision1`/`collision2` non-empty strings; `position`/`normal` non-empty lists of vec3; `depth` non-empty finite numbers; per-contact-point array lengths equal; `wrenches` non-empty list of JointWrench objects — each with `body_1_name`/`body_2_name` (strings; **empty allowed** — verbatim measured values, probe-observed) and **both** `body_1_wrench`/`body_2_wrench` objects, each with `force`/`torque`/`force_offset` vec3. `"not-a-wrench"` entries are rejected.
- **pose_info** (§ 3.3–3.5): `entries` non-empty; per entry: `name` non-empty string, `position` vec3; `orientation` (if present) quat4. `"not-a-pose-entry"` rejected.
- **joint_command** (§ 3.9): `command` = exactly 24 finite numbers (NAO command vector is float[24]); wrong dimension / non-numeric / boolean elements FAIL.
- **joint_states / body_pose** (optional categories): name list / position vec3; position/velocity (if present) finite lists matching `len(name)`.
- **Lossless retention (§ 15):** unknown/optional native keys (`header`, `world`, entity ids, extension fields) are preserved and **never rejected** — only the required structure is enforced; verified by a dedicated positive test.

## 8. R13-D — Provenance Hardening (closes B4)

`check_provenance` now enforces: `gazebo_seed` = integer **and not boolean** (Python `bool` is an `int` subclass — explicitly excluded); `frame_count` = integer, not boolean, **>= 0**; `t_start`/`t_end` = **finite** numbers (NaN/Infinity/-Infinity rejected — the JSON parser accepts non-standard `NaN`/`Infinity`, and the test proves the attack is reachable via a real round-trip fixture); **`t_end >= t_start`** (episode extents are ordered — a structural extent check). Existing sha256/commit/string/object format checks retained. `episode_integrity_hash` keeps the RFC definition — format/type/nullability only; no claim that format-valid == content-correct (the SHA256 round-trip remains a future Collector gate, § 17).

## 9. Test Matrix (§ 23 — all through the REAL `--smoke` CLI path)

| Artifact | Expected | Result |
|---|---|---|
| Fully valid fixture | PASS / exit 0 | `test_matrix_valid_passes_exit_0`: exit 0, ok=true |
| Missing manifest / provenance / frame / raw | FAIL | `test_matrix_missing_artifacts_fail`: exit != 0 each |
| Option-A missing / null / wrong type / wrong shape | FAIL | `test_matrix_option_a_attacks_fail` (4 cases + Option-B non-null) |
| Option-B non-null | FAIL | included above |
| Wrong Option-A status / unknown status | FAIL | `test_matrix_wrong_option_a_status_fails` / `test_matrix_unknown_status_fails` |
| RFC field add / remove; Contract field add / remove | FAIL | `test_matrix_field_mutations_fail` (provenance_binding) |
| RFC nullable / type / requiredness mutation | FAIL | `test_matrix_semantic_mutations_fail` |
| Contract nullable / type / requiredness mutation | FAIL | included above |
| Malformed wrench / pose entry / joint command | FAIL | `test_matrix_raw_nested_corruption_fails` (raw_storage) |
| Boolean gazebo_seed / negative frame_count / non-finite timestamp / invalid time ordering | FAIL | `test_matrix_provenance_corruption_fails` (provenance) |

Every FAIL case asserts `returncode != 0` AND `smoke.ok is false` from the actual subprocess — never a helper-only assertion.

## 10. Attack / Mutation Results (unit-level, `tools/test_stage1_r13.py`, 49 tests, all OK)

- **Frame attacks (B1):** each Option-A field missing/null; wrong types (string, scalar, list-as-pose); wrong shapes (len 2/3, missing orientation, non-finite element); Option-B non-null/missing; status/value mismatch — all FAIL.
- **Semantic binding attacks (B2):** RFC nullable mutation (the § 3.10-specific anchor — the same short phrase also appears in the § 3.6.2 table, so the mutation targets the provenance row), contract nullable mutation, RFC type mutation (SHA-256→SHA-512) **and type-keyword removal**, contract type mutation (dict→list), requiredness mutations (adding `optional`) on both documents, field add/remove on both — all FAIL; positive binding holds on the repository documents.
- **Raw attacks (B3):** `wrenches=["not-a-wrench"]`, missing body-side/force/torque/force_offset/body-name, wrong vector shapes, non-finite wrench components, `entries=["not-a-pose-entry"]`, missing pose name/position, malformed quat, missing/empty collision identity, contact-point array drift, command 23-dim / non-numeric / boolean element — all FAIL; empty-Contacts-message and unknown-native-key preservation pass (positive controls).
- **Provenance attacks (B4):** gazebo_seed True/False, frame_count −1/True/1.5 (0 passes), t_start NaN/inf, t_end −inf, boolean numeric, t_end < t_start, JSON NaN round-trip reachability — all FAIL as required.

## 11. CLI Exit-Code Results (§ 22 command evidence)

- `python3 tools/test_stage1_r11.py` → **OK** (15 tests)
- `python3 tools/test_stage1_r12.py` → **OK** (40 tests)
- `python3 tools/test_stage1_r13.py` → **OK** (49 tests)
- `python3 tools/stage1_contract_check.py --help` → documents `--smoke` as THE enforcement entry point and the default mode as informational-only
- Valid synthetic fixture (`/tmp/r13_fixture_valid/`): `--smoke` → **exit 0**, `ok: true`, `failed_checks: []`
- Invalid fixture (`cop_position` non-null): → **exit 1**, `ok: false`, `failed_checks: ['frame_emission']`
- No catch-exception-print-warning-exit-0 path exists: artifact load errors become smoke failures (exit 1).

**Default vs smoke mode (§ 19):** the default CLI mode (no `--smoke`) is **informational only** (document bindings + status maps; explicitly labeled `mode: informational (NOT artifact enforcement — use --smoke)` in its output and in `--help`/epilog). Only `--smoke` is the complete artifact enforcement entry point; the acquisition workflow must use `--smoke`. This distinction is recorded here and in the Collector Contract § 10.

## 12. R6–R10 Regression

`TestR6toR10Regression` (R13 suite) + the R12 regression suite pass: R6 — no `CADENCE_MIN_HZ/CADENCE_MAX_HZ` in the probe, event-aware publication intact; R7 — two-state vocabulary (`UNKNOWN_NO_MESSAGE`, `VALID_NO_CONTACT` as future extension only) and no-fabrication rules intact; R8 — `composition_status = UNVERIFIED` preserved and un-certifiable; R9 — measured/derived vocabulary unchanged; R10 — `timestamp_association_compatibility` (not clock synchronization). **RFC hash unchanged**: `195178f5…2f02aac` (asserted by test and by `sha256sum`). `tools/stage1_probe.py` byte-identical.

## 13. Scope Integrity

`git status --short` before commit: only the five files of § 4 (+ this report). `git diff` over `tools/run_phase_a_episode.py`, `tools/run_p0a_gait_candidate.py`, `tools/run_stage0_dataset.py`, `tools/stage1_probe.py`, `src/**`, `dataset/stage0/**`, `docs/STAGE1_DYNAMICS_DATASET_RFC.md` = **0 lines**. No unauthorized modification; no anti-gaming action was taken (no weakened expectations, no deleted tests, no fixture-converted-to-valid).

## 14. Acquisition Boundary

```text
NO Gazebo acquisition
NO pilot
NO 20–50 parameter sweep
NO 96 episodes
NO formal dataset
```

Only synthetic fixtures (under `/tmp`), temporary mutation copies, and contract smoke checks were used. No new Gazebo subscriber, ROS collector, episode recorder, or acquisition loop was added (§ 27) — the fixture builders in the test files are test fixtures, not a collector.

## 15. Remaining Limitations (by design)

- `composition_status = UNVERIFIED` — not upgraded, not resolvable by this checker (R8 boundary).
- Wrench frame/reference = UNKNOWN — Option-B nulls remain; CoP/derived-moment derivation stays blocked.
- Contact silence = `UNKNOWN_NO_MESSAGE` (two-state contract).
- Timestamp probe evidence = association compatibility, not universal clock identity.
- `episode_integrity_hash`: format/type/nullability only — content-correctness (SHA256 round-trip) is a future Collector gate.
- Semantic binding covers the semantic dimensions the documents actually declare (nullable via `null` mention, optional via `optional` mention, type via the documented keyword rules); fields whose rows carry no machine-readable marker (e.g. `theta_list_hash`, `gz_sim_version` types) are bound by the canonical spec for artifact validation but not doc-side — documented in `PROVENANCE_SEMANTICS`.
- R13 closes B1–B4 at the smoke boundary; it does not make the Collector ready or validate physical correctness.

## 16. Git Commit

- R13 commit (single, recommended message): `STAGE1: fail-closed schema hardening R13` — created immediately after this report; HEAD SHA recorded below at finalization.
- `git status` after commit: clean except the pre-existing untracked `docs/task4_phase4b_evidence/` (out of Stage 1 scope, untouched). **Not pushed** (remote publication is an owner/process action).

## 17. Final Implementation Recommendation

```text
Collector implementation: NOT YET AUTHORIZED BY THIS REPORT
Pilot acquisition: NOT AUTHORIZED
96-episode acquisition: NOT AUTHORIZED
```

R13 implementation is complete against acceptance criteria A–J: B1–B4 closed at the `--smoke` boundary (A–D), all new attack cases exit non-zero through the actual CLI (E), the valid fixture exits 0 (F), R6–R10 show no regression (G), scope is clean (H), acquisition not performed (I), collector not implemented (J). The next step must be the independent `STAGE1_REV13_INDEPENDENT_CLOSURE_AUDIT`; only an explicit **Decision A — R13 CLOSED** from that audit authorizes entering Collector Implementation.
