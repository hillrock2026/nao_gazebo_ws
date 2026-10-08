# STAGE1_REV11_MINIMAL_HARDENING_REPORT

| Field | Value |
|---|---|
| Role | Implementation Agent (GLM-5.3) — R11 minimal hardening executor |
| Task | `STAGE1_REV11_MINIMAL_HARDENING_TASK` — address the two blocking issues of the GPT-5.6 independent adversarial audit of REV1.3 (**Decision B — READY AFTER MINOR FIXES**) |
| Baseline | `STAGE1_REV13` (RFC REV1.3-MINIMAL-HARDENING; probe run-8 evidence V-1/V-2 double PASS) |
| Revision produced | RFC **REV1.4-HARDENING** (R11-A + R11-B); collector contract **v1.3**. R11 continues the correction numbering (R1–R5 → REV1.2, R6–R10 → REV1.3, R11 → REV1.4) |
| Date | 2026-10-08 |
| Acquisition status | **NOT AUTHORIZED.** No pilot collection, no 96-episode acquisition, no formal Stage 1 dataset; no new V-gate; R6–R10 decisions not reopened; REV1.4 pending final independent audit acceptance |

---

## 1. Audit finding addressed

### Blocker A → R11-A: pending-validation propagation

**Audit finding:** `composition_status = UNVERIFIED` is correctly retained, but derived fields that (transitively) depend on composed poses / CoM do not consistently and machine-actionably inherit pending-validation semantics in the contract/schema.

**Resolution (minimal, no field added/removed, no new status enums):**

- **Binding dependency rule** (RFC REV1.4 § 3.0 "Validation-status propagation"; collector contract § 4.7, verbatim-consistent): *any derived field whose computation depends — directly or indirectly — on an input whose validation status is `UNVERIFIED` (composition) or pending/blocked (wrench frame/reference) MUST carry `derived_pending_validation` (or remain `null` + blocked where derivation is already blocked) and MUST NOT be represented as validated derived data.* Status propagates along every dependency chain and is lifted only when the root input's status changes.
- **Null-vs-pending implementation codified as adopted by the current schema** (not changed): **Option A** — value retained + `derived_pending_validation` — for composition-dependent fields; **Option B** — `null` + blocked — for wrench-frame-dependent fields (`left/right_contact_force/moment`, `cop_position`). Downstream consumers cannot mistake either for validated physical ground truth.
- **Propagation chains declared** (per the repository's actual schema, not invented fields):
  1. raw model pose + raw relative link pose → `left/right_foot_pose` (pending) → `left/right_foot_velocity` (pending);
  2. raw model/link poses → `com_position` (pending) → `com_velocity` (pending) → `com_acceleration` (pending); `zmp_position` (CoM chain, pending);
  3. pose + wrench → `cop_position` / world-frame moment: **blocked + `null`** while the wrench frame/reference is undetermined.
- **Non-propagation rule:** raw measured data (raw `Pose_V` entries, model pose, raw `Contacts`, joint states, IMU, command) are **not** invalidated by the pending composition status — stored verbatim as `measured`.
- **Ordinary derived fields unaffected** (no unverified input): `joint_acceleration`, `base_velocity`, `base_angular_velocity` (base pose = `/body_pose` cross-checked model entry), `gait_phase`, `desired_velocity`.

The goal was **not** UNVERIFIED → VERIFIED; it was UNVERIFIED → *machine-actionable, cannot be silently consumed as validated*. `composition_status` remains `UNVERIFIED` until a genuinely independent world-pose reference exists. The collector's own recomputation of `T_world_link = T_world_model × T_model_link` verifies implementation consistency only and **cannot** upgrade the status (explicitly stated in contract § 4.7 / § 10 item 4).

### Blocker B → R11-B: provenance completeness

**Audit finding:** the collector contract's checklist was a paraphrased subset of the RFC provenance requirements — risk of "RFC says A+B+C+D…, collector implements only A+B+C" contract drift.

**Resolution:**

- RFC § 3.10 is declared the **single source of truth** for collector provenance completeness (binding note added to § 3.10 itself, R11-B).
- Collector contract § 8 rewritten as a **complete 18-field checklist explicitly derived from / must-equal RFC § 3.10** (no simplified copy): `theta`, `theta_id`, `theta_hash`, `theta_list_hash`, `git_commit`, `config_hash`, `cop_derivation_hash` (nullable until first derivation), `gazebo_seed`, `robot_model_version`, `world_model_version`, `gz_sim_version`, `stage1_rfc_hash`, `acquisition_wallclock_utc`, `ring`, `frame_count`, `t_start`, `t_end`, `episode_integrity_hash`.
- Smoke validation (contract § 10 item 6) upgraded: **every** RFC § 3.10 mandatory field must be present — a missing field is a validation FAILURE (previously "§8 字段全量存在" pointed at the paraphrased subset).
- **Machine binding:** `tools/stage1_contract_check.py` holds `MANDATORY_PROVENANCE_FIELDS` and *parses the RFC § 3.10 table* to verify the list equals it (`--check-rfc-binding`; drift in either direction is a failure) — the binding itself is tested (Test D).

## 2. Files changed

| File | Change | SHA-256 |
|---|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | REV1.3 → **REV1.4-HARDENING**: § 3.0 validation-status propagation rule (R11-A); pending annotations on `com_velocity`, `com_acceleration`, `left/right_foot_velocity`, `zmp_position`; Option-B note on `cop_position`; § 3.10 R11-B source-of-truth note; § 0.0 Level-3 note; revision history + status | (see § 9) |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | v1.2 → **v1.3**: new § 4.7 (propagation rule + chains + Option A/B + non-propagation + no-upgrade rule); § 8 rewritten as complete RFC-§ 3.10-derived checklist; § 10 items 3/4/6 updated + new item 8; document status | (see § 9) |
| `tools/stage1_contract_check.py` | **new** — read-only machine enforcement: `MANDATORY_PROVENANCE_FIELDS` (bound to RFC § 3.10 by parsing), `check_rfc_binding`, `check_provenance`, `check_pending_propagation`, `check_frame_emission`, `check_raw_storage`, field-status maps | (see § 9) |
| `tools/test_stage1_r11.py` | **new** — Tests A–D (unittest; 15 test cases) | (see § 9) |
| `STAGE1_REV11_MINIMAL_HARDENING_REPORT.md` | **new** — this report | (see § 9) |

No other file changed. `tools/stage1_probe.py` was **not** touched (R11 changes no probe-observable semantics — § 13 of the task); no probe re-run was needed. `STAGE1_REV13_CORRECTION_REPORT.md` is a committed historical record and was **not** edited; this report extends (supersedes-by-extension) it.

## 3. Schema / contract changes (fields, statuses, dependency rule)

- **Dependency rule:** as quoted in § 1 (R11-A). One rule, no exceptions, no new enums — vocabulary is exactly `measured` / `derived` / `derived_pending_validation` (existing REV1.3 terms).
- **Fields now annotated `derived_pending_validation` (inheritance made explicit):** `left_foot_pose`, `right_foot_pose` (already, R8/R9), `left_foot_velocity`, `right_foot_velocity`, `com_position` (already), `com_velocity`, `com_acceleration`, `zmp_position` — all depend directly or transitively on the UNVERIFIED composition.
- **Fields Option B (`null` + blocked while wrench frame/reference is UNKNOWN):** `left_contact_force`, `right_contact_force`, `left_contact_moment`, `right_contact_moment`, `cop_position` (unchanged semantics, now explicitly labeled as the Option-B arm of R11-A). No claim of "CoP validated / ZMP validated / world-frame moment validated" is made anywhere; the wrench contract was not redesigned.
- **Unaffected fields:** all `measured` fields (non-propagation) and ordinary derived fields (`joint_acceleration`, `base_velocity`, `base_angular_velocity`, `gait_phase`, `desired_velocity`).
- **Machine maps** (`tools/stage1_contract_check.py`): `MEASURED_FIELDS`, `ORDINARY_DERIVED_FIELDS`, `COMPOSITION_DEPENDENT_FIELDS`, `WRENCH_FRAME_DEPENDENT_FIELDS` + the two root statuses (`composition_status = UNVERIFIED`, `wrench_frame_status = UNKNOWN`).

## 4. Provenance binding chain

```text
RFC § 3.10 (single source of truth, 18 mandatory fields)
   → collector contract § 8 (complete checklist; "derived from / must equal")
   → collector contract § 10 item 6 (smoke: every field must be present)
   → tools/stage1_contract_check.py (MANDATORY_PROVENANCE_FIELDS,
      bound by PARSING the RFC § 3.10 table — drift = failure)
   → tools/test_stage1_r11.py Tests C/D (missing field FAILS / complete PASSES)
```

Episode-integrity hashing was **not** redesigned (§ 11 of the task): `episode_integrity_hash` keeps the RFC's existing definition (sha256 over the episode's canonical sample file); R11 only requires provenance *field completeness*, which the existing definition already covers.

## 5. Tests

`python3 tools/test_stage1_r11.py` — 15 test cases, **all OK** (unittest, 0.002 s):

| Test | Name | Result |
|---|---|---|
| A | `test_correct_manifest_passes` | ok |
| A | `test_every_composition_dependent_field_rejects_plain_derived` (each of 8 fields × {derived, validated, measured}) | ok |
| A | `test_missing_composition_dependent_field_fails` (each of 8 fields) | ok |
| A | `test_wrench_fields_reject_validated_or_derived_status` (5 fields × {derived, validated}) | ok |
| A | `test_wrench_fields_reject_non_null_emission` (+ null-positive case) | ok |
| A | `test_unverified_composition_never_certified` (a "VERIFIED" status is not certifiable by this checker) | ok |
| B | `test_raw_records_storage_passes` | ok |
| B | `test_measured_fields_pass_under_unverified_composition` | ok |
| B | `test_raw_storage_independent_of_composition_status` | ok |
| C | `test_each_missing_field_fails` (each of the 18 mandatory fields removed → FAIL) | ok |
| C | `test_null_non_nullable_field_fails` | ok |
| C | `test_empty_record_fails` | ok |
| D | `test_complete_record_passes` | ok |
| D | `test_nullable_cop_derivation_hash_allowed_null` | ok |
| D | `test_rfc_binding_no_drift` (checker list == parsed RFC § 3.10) | ok |

Also run: `python3 tools/stage1_contract_check.py --check-rfc-binding` → `ok: true` (exit 0).

## 6. Negative tests (explicit)

- **Unverified composition cannot appear as validated:** for every one of the 8 composition-dependent fields, a manifest marking it `derived` / `validated` / `measured` is REJECTED while `composition_status = UNVERIFIED` (Test A); a non-emitted field is also rejected. For every wrench-frame-dependent field, `derived`/`validated` status or a **non-null emitted value** (e.g. a numeric `cop_position`) is REJECTED while the wrench frame/reference is UNKNOWN (Option B).
- **Missing provenance fails validation:** removing any single one of the 18 RFC § 3.10 mandatory fields makes `check_provenance` FAIL with that field named in `missing`; a null value in any non-nullable field FAILS (Test C).
- Conversely, the complete record PASSES (Test D) — the checks are fail-closed, not unconditional rejections.

## 7. Scope integrity

- **Stage 0 unchanged** (`dataset/stage0/**`, Stage 0 tools/docs: git diff empty; `corpus_index.json` = `260fd9e7…` re-verified).
- **Controller unchanged, gait generator unchanged** (`src/nao_gz_sim/**`, frozen runners: diff empty).
- **NAO model / Gazebo world / physics unchanged** (`nao_simple.sdf` = `44ed8285…`, `nao_world.sdf` = `383341d1…` re-verified).
- **No acquisition performed** — no pilot episode, no 96-episode run, no formal Stage 1 dataset, no collector writer executed. Only unit tests / contract validation ran (no simulation, no probe execution).
- **Only R11-scope files modified** (see § 2): two contract docs + two new read-only tooling files + this report. No new V-gate; R6/R7/R8-probe/R10/V1/V2 methodology untouched.

## 8. Remaining known limitations (unchanged by design)

- `composition_status = UNVERIFIED` — the Gazebo pose composition is not physically validated (no independent world-pose reference); R11 makes this machine-actionable, it does not resolve it.
- Wrench `frame/reference = UNKNOWN` — CoP / derived-moment / world-frame-force derivation remains blocked (Option B nulls); no wrench contract redesign.
- Contact silence = `UNKNOWN_NO_MESSAGE` (two-state contract, REV1.3/R7) — missing-evidence semantics, never zero wrench / flight / no-contact fabrication.
- Timestamp probe result = **association compatibility**, not universal clock identity (REV1.3/R10).
- Process gates: REV1.4 pending final independent audit acceptance; remote publication of the Stage 0 closure commit and the contract remains an owner action; Level 3 acquisition not authorized.

## 9. Git state

- HEAD commit: `git log -1 --format=%H` at the time of this report — the R11 hardening commit (created immediately after this file; see the session log). Not pushed; remote publication is an owner/process action.
- Working tree after the commit: clean except the pre-existing untracked `docs/task4_phase4b_evidence/` (out of Stage 1 scope, untouched).
- Changed files (this task): `docs/STAGE1_DYNAMICS_DATASET_RFC.md`, `docs/STAGE1_COLLECTOR_CONTRACT.md`, `tools/stage1_contract_check.py` (new), `tools/test_stage1_r11.py` (new), `STAGE1_REV11_MINIMAL_HARDENING_REPORT.md` (new).
- Artifact SHA-256 (at commit time; this report's own hash is not self-embeddable):

```text
195178f52ebe27efd696c9750f0a7164bdbc09790a1222d491d1120eb2f02aac  docs/STAGE1_DYNAMICS_DATASET_RFC.md
92a20daab623ae35603ebcd0eb9ac44121a7885511f2447fd2b47304986a4af3  docs/STAGE1_COLLECTOR_CONTRACT.md
3a90e3647831fb630e073b689a37d8a4d75e3580092506d937ecdc412855d28f  tools/stage1_contract_check.py
0702964540b9c8d07cc224bcd6515aec19d5f8a851fdbdc29f6cf6bbfe7b6229  tools/test_stage1_r11.py
```

No remote publication claimed.
