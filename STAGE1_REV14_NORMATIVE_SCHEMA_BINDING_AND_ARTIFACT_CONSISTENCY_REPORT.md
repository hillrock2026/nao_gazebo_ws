# STAGE1_REV14_NORMATIVE_SCHEMA_BINDING_AND_ARTIFACT_CONSISTENCY_REPORT

## 1. R13 blocker mapping

| R13 audit bypass | Root cause | R14 closure |
|---|---|---|
| B1 — Option-A schema mutation (com_position float[3]→[4], orientation [4]→[5], ZMP shape) passed `--smoke` | shapes hardcoded in the checker, no binding to the RFC normative schema | shapes load from the RFC Appendix D normative block at run time; an RFC schema mutation makes the old artifact FAIL (M1–M3) |
| B2 — `cop_derivation_hash: nullable → non-null` prose still parsed as nullable ("null" substring); "may be omitted" not recognized | fragile natural-language keyword parsing | prose semantic parsing REMOVED; explicit `required`/`nullable`/`type` flags in the normative block; RFC block == Contract block canonical-equality binding (M4–M7) |
| B3 — deleting RFC-required raw retention fields (`body_1_id`, `body_2_id`, Contact headers, raw pose orientation) passed | raw nested schema not fully bound to RFC normative requirements | raw requiredness per the block: Contact `header`, `body_1_id`/`body_2_id`, JointWrench members, pose `orientation`, contact-point arrays, 24-vector command (M8–M11); unknown native keys still preserved |
| B4 — `frame_count = 0` with a non-empty frame artifact passed | no artifact/count consistency | frame artifact is an ARRAY of frame records; `frame_count` == actual record count; empty episode `0/[]` valid; never forced `> 0` (M12–M14) |

## 2. Chosen normative schema architecture (task § 5, Option A)

A **normative machine-readable JSON block embedded in the RFC (new Appendix D, REV1.5)** is the single machine source of truth. The Collector Contract embeds an **identical copy** (new § 8.4, inserted programmatically from the RFC block to guarantee identity), and the checker **loads the schema from the RFC at run time** — the checker owns no independent schema (its public constants are derived at import from the block). No new DSL, no schema compiler: plain JSON with explicit `required` / `nullable` / `type` / shape / status / raw-structure / count-rule declarations.

```text
RFC Appendix D (normative JSON block)   ←— identical copy —→   Contract § 8.4 block
                │                                              │
                └────────── canonical-equality binding ───────┘
                                  │ (checker loads from the RFC at run time)
                                  ↓
                          Smoke Checker (--smoke)
                                  ↓
                            Actual Artifact
```

Prose tables (RFC § 3.10, Contract § 8 checklist) are checked for **field-name consistency** against their document's block only; **semantics are never parsed from prose** (B2). Appendix D states: in case of discrepancy, the block governs.

## 3. RFC changes (authorized per task § 14; full diff enumerated)

`docs/STAGE1_DYNAMICS_DATASET_RFC.md` — REV1.4 → **REV1.5-NORMATIVE-SCHEMA** (+214/−22 vs `88aa3fa`):

1. Header table: Revision / Status / Author / Date rows (REV1.5).
2. § 0: REV1.4 paragraph marked superseded (text preserved); new REV1.5 paragraph (B1–B4, Appendix D, no research-objective / R6–R10 / acquisition-requirement / scientific-meaning change).
3. § 3.10 note: machine-readable semantics live in Appendix D; prose names must equal the block names; checker never parses prose keywords.
4. § 0.0 Level-3 note: "REV1.5 is pending".
5. **New Appendix D**: preamble (single machine source of truth; block governs on discrepancy; modification = RFC revision; drift = validation failure) + the normative JSON block + binding-semantics notes (optional keys, raw vector shapes as type interpretation, frame-artifact count rule with empty episodes allowed).
6. Revision history: REV1.5 entry.
7. Document status line.

**New RFC SHA-256: `f8662d1673f1a30e9cfb2d8156214d7124eb45c08301087d85c4fae1ca894efe`** (was `195178f5…`; pinned by `test_rfc_hash_pinned_rev15`). No research-objective, R6–R10, acquisition-requirement, or raw/derived scientific-meaning change — the block is a machine transcription of the existing prose contract.

## 4. RFC ↔ Contract binding

`check_semantic_binding` (rewritten): loads both blocks and requires **canonical JSON equality**; drift is reported per section (`drifting_sections`). The Contract's § 8.4 block was inserted as a byte-identical copy of the RFC block. Verified: `--check-semantic-binding` → `ok: true`. Any block mutation in either document (field add/remove, nullable, required/optional, type, shape, raw requiredness, count rule) FAILS the smoke path (M4–M7 + the contract-mutation tests).

## 5. Contract ↔ checker binding

The checker has **no hardcoded normative schema**: `load_normative_schema(path)` parses the fenced `stage1-normative` JSON block; all public constants (`MANDATORY_PROVENANCE_FIELDS`, `COMPOSITION_DEPENDENT_FIELDS`, `WRENCH_FRAME_DEPENDENT_FIELDS`, `OPTION_A_FIELD_SHAPES`, `COMMAND_VECTOR_DIM`, raw structures, status vocabulary) are **derived at import from the RFC block**, and every check accepts a `schema=` parameter — `run_smoke` passes the schema loaded from the **given `--rfc`** (a mutated RFC changes the enforced contract; an unloadable schema fails every check closed, never a silent fallback to the default). `test_checker_constants_derived_from_schema` asserts constants == block. The R13 prose-keyword machinery (`_TYPE_MARKER_RULES`, `extract_doc_provenance_semantics`, `PROVENANCE_SEMANTICS`) was **deleted**.

## 6. B1 closure — Option-A schema enforcement

`check_frame_emission` validates each Option-A field per the **block spec**: pose = `{position[N], orientation[M]}` with N/M from the block; vector = length from the block. Evidence (CLI, unchanged checker): mutated RFC `com_position length 3→4` → old artifact fails with `frame[0]: com_position (Option-A): must have 4 elements (got 3)` AND block-drift `provenance_binding` failure → **exit 1** (M1); orientation 4→5 (M2); zmp 2→4 (M3). Unit test `test_option_a_shapes_come_from_schema` proves the same [3]-value passes under the repository schema and fails under the mutated schema.

## 7. B2 closure — semantic binding

Explicit flags only: `{"name": "cop_derivation_hash", "required": true, "nullable": true, "type": "sha256"}`. M4 mutates `nullable: true → false` in the RFC block → the artifact's `cop_derivation_hash: null` violates the mutated schema (`provenance` FAIL) and the blocks drift (`provenance_binding` FAIL) → exit 1. `test_prose_semantics_never_parsed` proves that changing the prose to "must be non-null" leaves the machine semantics untouched (the flag governs). M5 (`required → false`, ring), M6 (`optional → required`: `world` moved into `contact_required_keys` — artifacts lacking `world` fail `raw_storage`), M7 (type `object → string`, theta) all FAIL; the same mutations on the **Contract's** block copy also FAIL (block inequality). A block with a missing semantic key is structurally invalid → fail-closed load.

## 8. B3 closure — raw nested requiredness

The block declares per raw structure: `contacts` — required record keys (`stamp_s`, `contacts`), per-contact required keys (**`header`**, `collision1`, `collision2`, `position`, `normal`, `depth`, `wrenches`), wrench required keys (**`body_1_name`, `body_1_id`, `body_2_name`, `body_2_id`, `body_1_wrench`, `body_2_wrench`**), wrench member required keys (`force`, `torque`, `force_offset`); `pose_info` — entry required keys (`name`, `position`, **`orientation`**); `joint_command` — `command_length: 24`; optional-known keys (`world`, `n_contacts`, entry `id`, joint_states `position`/`velocity`, body_pose `orientation`) may be present; **unknown native/extension keys are always preserved** (lossless retention — positive test). M8–M11 (delete `body_1_id` / `body_2_id` / contact `header` / pose `orientation` from the artifact) all FAIL via the real CLI. Only RFC-declared required fields are enforced — no Gazebo-native field is blanket-required (§ 15 error 3 avoided).

## 9. B4 closure — artifact/count consistency

The smoke frame artifact is a **JSON array of frame records** (`frame_artifact: {type: array_of_frames, count_must_equal: frame_count, empty_episode_allowed: true}` in the block). `check_frame_artifact` validates every record and requires `frame_count == len(frames)` when the provenance artifact is available. `frame_count` remains `int_nonneg` (never `> 0` — § 15 error 2 avoided). Evidence: M12 (`frame_count=0`, 1 frame) → exit 1; M13 (`frame_count=2`, 1 frame) → exit 1; M14 (`frame_count=0`, `frames=[]`) → **exit 0** (legal empty episode); `frame_count=2` with 2 frames → exit 0; a non-array frame artifact → exit 1.

## 10. Mutation matrix (all via the REAL `--smoke` CLI, `tools/test_stage1_r14.py`, 32 tests OK)

| ID | Mutation | Expected | Result |
|---|---|---|---|
| M1 | RFC block `com_position` length 3→4 | FAIL | exit 1 (`frame_emission` + `provenance_binding`) |
| M2 | RFC block pose orientation 4→5 | FAIL | exit 1 |
| M3 | RFC block zmp length 2→4 | FAIL | exit 1 |
| M4 | RFC block cop nullable→false | FAIL | exit 1 (`provenance` + `provenance_binding`) |
| M5 | RFC block ring required→optional | FAIL | exit 1 (`provenance_binding`) |
| M6 | RFC block world optional→required | FAIL | exit 1 (`raw_storage` + `provenance_binding`) |
| M7 | RFC block theta type object→string | FAIL | exit 1 (`provenance` + `provenance_binding`) |
| — | Contract block nullable/required/type mutations | FAIL | exit 1 (`provenance_binding`) |
| M8 | artifact: delete `body_1_id` | FAIL | exit 1 (`raw_storage`) |
| M9 | artifact: delete `body_2_id` | FAIL | exit 1 (`raw_storage`) |
| M10 | artifact: delete contact `header` | FAIL | exit 1 (`raw_storage`) |
| M11 | artifact: delete pose `orientation` | FAIL | exit 1 (`raw_storage`) |
| M12 | `frame_count=0` + 1 frame | FAIL | exit 1 (`frame_emission`) |
| M13 | `frame_count=2` + 1 frame | FAIL | exit 1 (`frame_emission`) |
| M14 | `frame_count=0` + `frames=[]` | PASS | exit 0 |
| + | unknown extra native fields present | PASS | exit 0 (lossless retention) |
| + | normative block removed from the RFC | FAIL | exit 1 (`normative_schema` + all schema-driven checks fail closed) |

## 11. Test independence (§ 11)

- R14 fixtures are **hand-written literals** (field names and values written out; the provenance literal is asserted equal to the independently parsed prose tables in the r13 suite). No fixture is generated from checker constants.
- The key independence evidence is **`mutated RFC → UNCHANGED checker → smoke → FAIL`**: the checker binary is untouched; only the `--rfc` input changes, and the old artifact is rejected (M1–M7, plus the CLI demo: `MUTATED-RFC EXIT=1 ... com_position (Option-A): must have 4 elements (got 3)`).
- The r13 suite's semantic-mutation helpers were retargeted from prose to the normative block (the R14 architecture removed prose semantic parsing by design); the attacks themselves (nullable/type/requiredness/field mutations) were kept, not deleted.

## 12. R11/R12/R13 regression

```text
python3 tools/test_stage1_r11.py → OK (15 tests)
python3 tools/test_stage1_r12.py → OK (40 tests)
python3 tools/test_stage1_r13.py → OK (49 tests)
python3 tools/test_stage1_r14.py → OK (32 tests)
```

Fixture updates were required by the R14 contracts (raw: contact `header`, body ids, pose `orientation`; frame artifact arrays with matching `frame_count`; the r13 RFC-hash pin updated to the REV1.5 hash). No test expectation was weakened; no attack test was deleted (§ 31 anti-gaming rules respected).

## 13. R6–R10 regression

`TestR6toR10Regression` (r14) + the r13/r12 regression suites pass: R6 — no cadence band in the probe; R7 — two-state vocabulary + no-fabrication rules; R8 — `composition_status: UNVERIFIED` (from the block; un-certifiable otherwise); R9 — measured/derived/`derived_pending_validation` vocabulary unchanged; R10 — `timestamp_association_compatibility` (not clock synchronization). `tools/stage1_probe.py` is byte-identical to `88aa3fa` (asserted by `test_frozen_files_untouched` via `git diff 88aa3fa`).

## 14. Scope integrity

`git diff 88aa3fa` — frozen set (runner, `stage1_probe.py`, gait generator/controller, `src/**`, `dataset/stage0/**`): **0 lines**. Changed files: `docs/STAGE1_DYNAMICS_DATASET_RFC.md` (R14-authorized, § 3 above), `docs/STAGE1_COLLECTOR_CONTRACT.md` (§ 8.4 + v1.6), `tools/stage1_contract_check.py` (schema-driven rewrite), `tools/test_stage1_r11.py` / `r12` / `r13` (fixture/mutation updates), `tools/test_stage1_r14.py` (new), this report. No Collector implementation, no acquisition, no new V-gate, no framework rewrite (the checker's public API and check names are preserved).

## 15. Git diff (summary vs `88aa3fa`)

```text
docs/STAGE1_COLLECTOR_CONTRACT.md        | 120 ++++++ (§ 8.4 block + v1.6 + doc status)
docs/STAGE1_DYNAMICS_DATASET_RFC.md      | 214 +++++-- (Appendix D + REV1.5)
tools/stage1_contract_check.py           | schema-driven rewrite (R14)
tools/test_stage1_r11.py                 | fixture updates (raw keys, R14 contracts)
tools/test_stage1_r12.py                 | fixture updates (frame arrays, raw keys)
tools/test_stage1_r13.py                 | block-targeted mutations + REV1.5 hash pin
tools/test_stage1_r14.py                 | NEW (32 attack tests, M1-M14)
STAGE1_REV14_..._REPORT.md               | NEW (this report)
```

## 16. Hashes

```text
f8662d1673f1a30e9cfb2d8156214d7124eb45c08301087d85c4fae1ca894efe  docs/STAGE1_DYNAMICS_DATASET_RFC.md   (REV1.5; was 195178f5…)
0384aad2d1f4db92c7fc02729c71fa9daf4d3542a26df8e72d2682ecfdab0295  docs/STAGE1_COLLECTOR_CONTRACT.md    (v1.6)
e7103fe5c4c8cc435f91f60570465af1a7af47d3cc2bc95484159b3a88207e1a  tools/stage1_contract_check.py
2aea2d2535eb64ad9e0a40c103e21368b6222b954a5df1eb2f0ce6f3954585eb  tools/test_stage1_r14.py
e6c0c5ccdce5deae5f7feabb127c587ed070fcdc638af5e28b2d1ca03be5583c  tools/test_stage1_r13.py
a7a7d7f2262893f0844777a86e2caa29c09f1797821f1f0b84e8c11811ace94e  tools/test_stage1_r12.py
238c9108ff2276bd1372d41958a63d20116791e75b321d152241c77df227e7d0  tools/test_stage1_r11.py
```

## 17. Current HEAD

- Implementation commit: `3032780c120404849046b53cf691a12123675cd1` ("STAGE1: normative schema binding and artifact consistency R14"; this § 17 hash reference was finalized in the immediately following report-finalization commit). Working tree after the commits: clean except the pre-existing untracked `docs/task4_phase4b_evidence/` (out of Stage 1 scope, untouched). **Not pushed** — remote publication remains an owner/process action.

## 18. Explicit authorization state

```text
Collector implementation: NOT YET AUTHORIZED BY THIS REPORT
Pilot acquisition: NOT AUTHORIZED
96-episode acquisition: NOT AUTHORIZED
```

Status: **R14 implementation complete — WAITING FOR INDEPENDENT CLOSURE AUDIT** (the next-round independent adversarial audit; only an explicit **Decision A — R14 CLOSED** authorizes Collector Implementation, and even then pilot/96-episode acquisition remain separately unauthorized).
