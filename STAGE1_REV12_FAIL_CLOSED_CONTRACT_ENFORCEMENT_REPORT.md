# STAGE1_REV12_FAIL_CLOSED_CONTRACT_ENFORCEMENT_REPORT

| Field | Value |
|---|---|
| Role | Implementation Agent (GLM-5.3) — R12 executor |
| Task | `STAGE1_REV12_FAIL_CLOSED_CONTRACT_ENFORCEMENT_TASK` — elevate the R11 checker from helper-level validation to a **truly executable, fail-closed, regression-testable contract enforcement layer** |
| Baseline commit | `3a4ab86be03f7a13f01fbe75644b01efe3edb2cb` (R11 report finalization; RFC REV1.4-HARDENING, contract v1.3) |
| Audit input | R11 Independent Closure Audit: **Decision C — R11 BLOCKED** (5 findings: helper not wired into an actual fail-closed smoke path; RFC provenance binding does not cover Contract § 8; Option-B unknown-status bypass; raw-storage helper too lenient; tests circular) |
| Date | 2026-10-08 |
| Status claimed | **R12 implementation complete** — NOT "R12 CLOSED" (closure is the independent audit's decision) |
| Acquisition | **NO** (see § 15) |

---

## 1. Baseline commit

`3a4ab86` — HEAD before this task. RFC `docs/STAGE1_DYNAMICS_DATASET_RFC.md` is **byte-identical** across this task (sha256 `195178f5…2f02aac`, unchanged from R11) — R12 changed no RFC rule. `tools/stage1_probe.py` is **byte-identical** (no probe change; no probe run). All R12 changes live in the checker, the tests, and two minimal Collector Contract clarifications (§ 8 machine-binding sentence, § 10 smoke entrypoint; contract v1.3 → v1.4).

## 2. R12 objective

Turn the R11 contract into a **minimal, executable, fail-closed, auditable enforcement layer**: actual artifacts → one smoke path → PASS/FAIL with non-zero exit; close the Option-B unknown-status bypass; machine-bind RFC § 3.10 ↔ Collector Contract § 8; make raw storage a structural contract; prove it all with adversarial, mutation, anti-circularity, and CLI exit-code tests. Minimal hardening only — no redesign, no new V-gate, no schema change, no reopening of R6–R10.

## 3. R12-A — Actual Artifact Enforcement

`tools/stage1_contract_check.py --smoke` is a real CLI over **actual artifacts** (default paths point at the repository's actual RFC and Collector Contract):

```bash
python3 tools/stage1_contract_check.py --smoke \
    --manifest <status_manifest.json> \
    --provenance <episode_provenance.json> \
    --frame <sample_frame.json> \
    --raw <raw_records.json>
```

The single fail-closed chain (any failure ⇒ `ok=false` ⇒ **exit != 0**):

```text
provenance_binding  (RFC § 3.10 == Contract § 8 == checker list; R12-C)
    ↓
raw_category_binding  (required raw streams bound to Contract § 6 tokens)
    ↓
provenance completeness + contract § 8 formats
    ↓
status manifest  (root declarations + R11-A/R12-B allowlists)
    ↓
frame emission  (wrench Option-B: present and null)
    ↓
raw storage structure  (R12-D)
    ↓
overall PASS/FAIL
```

Missing artifact files, a missing `--manifest` argument, or an unloadable JSON is a **FAILURE**, never "nothing to check". `--help` documents the smoke interface.

## 4. R12-B — Fail-Closed Pending Validation

R11 rules kept verbatim; enforcement made allowlist-only:

- Composition-dependent fields (8): the **only** accepted status is `derived_pending_validation` — `derived`, `validated`, `measured`, any unknown/unrecognized string, or a missing status FAIL.
- Wrench-frame-dependent fields (5): the **only** accepted statuses are the explicit blocked statuses (`null_blocked`, `blocked`) — the R11-audit bypass (`cop_position = null` + `status = UNRECOGNIZED_STATUS`) **now FAILS**. Emitted frame values must be **present and null** (a missing Option-B field is a failure, not "not emitted").
- The status-manifest artifact **must** declare `composition_status: UNVERIFIED` and `wrench_frame_status: UNKNOWN` (missing roots FAIL; any other value FAILS — the checker certifies no other state, so `UNVERIFIED` can never be upgraded by tooling).
- Unknown `field_status` keys FAIL (the manifest is a status map over the schema fields).
- `composition_status` remains `UNVERIFIED` — R12 makes the fact non-bypassable, it does not resolve it.

## 5. R12-C — RFC ↔ Collector Contract Binding

- `parse_contract_provenance_fields()` machine-parses the Collector Contract § 8 checklist table (numbered rows, backticked field cell).
- `check_provenance_binding()` requires **RFC § 3.10 == Contract § 8 == checker list** — the two documents are proven not to be two independent declarations; drift in either document fails the smoke path.
- `check_raw_category_binding()` binds the required raw categories (`contacts_left`, `contacts_right`, `pose_info`, `joint_command`) to Collector Contract § 6 text tokens (`/nao/left_foot_contact_raw`, `/nao/right_foot_contact_raw`, `/world/nao_world/pose/info`, `controller_command`) — no raw stream is invented, and a contract that drops a stream makes the checker's requirement detectably unbound (FAIL).
- Contract § 8 was given one clarifying sentence (dual-parse binding; drift = FAIL); the RFC was **not** modified.

## 6. R12-D — Tests (`tools/test_stage1_r12.py`, 40 cases, all OK)

- **Anti-circularity (audit finding 5):** the provenance fixture is a hand-written **literal** (18 fields written out, pinned hashes written literally) and is asserted equal to the **independently parsed** RFC § 3.10 / Contract § 8 tables — the tests can detect checker↔contract disagreement instead of only proving the checker accepts its own constants.
- **Negative matrix (task § 11):** every listed case (see § 8 below).
- **Mutation tests (task § 7.2):** RFC + 1 field / − 1 field → binding FAIL; Contract § 8 + 1 field / − 1 field → FAIL; Contract § 6 raw-token removal → raw-category binding FAIL.
- **CLI exit codes:** valid fixture → 0; invalid manifest / invalid frame / missing provenance field / missing artifact file / missing `--manifest` / invalid raw / mutated RFC document → != 0.
- **R6–R10 regression invariants:** probe contains no cadence band and keeps event semantics + `timestamp_association_compatibility`; RFC/contract keep the two-state vocabulary (`UNKNOWN_NO_MESSAGE`, `VALID_NO_CONTACT` as future extension only, no-fabrication rules); checker keeps `COMPOSITION_STATUS_UNVERIFIED = 'UNVERIFIED'` and refuses to certify `VERIFIED`; measured/derived vocabulary unchanged.
- `tools/test_stage1_r11.py` (15 cases) still **OK** — its fixtures were minimally strengthened to satisfy the R12 raw/frame contracts (raw records: all required categories + `collision2`; frame: all 5 Option-B fields present as null); the R11 test assertions' semantics are unchanged.

## 7. CLI smoke evidence (task § 17)

- `python3 tools/test_stage1_r11.py` → **OK** (15 tests).
- `python3 tools/test_stage1_r12.py` → **OK** (40 tests).
- `python3 tools/stage1_contract_check.py --help` → smoke interface documented.
- Valid synthetic fixture (`/tmp/r12_fixture_valid/`): `--smoke` → **exit 0**, `ok: true`, `failed_checks: []`.
- Invalid synthetic fixture (`/tmp/r12_fixture_invalid/`, `cop_position` non-null): → **exit 1**, `ok: false`, `failed_checks: ['frame_emission']` with error `cop_position: emitted a non-null value … must be null + blocked (Option B, R11-A/R12-B)`. No case of "console says FAIL but exit code = 0" — the exit code is the binding verdict.
- Binding-only mode: `--check-rfc-binding --check-contract-binding` → both `ok: true` (RFC == Contract § 8 == checker, 18 fields).

## 8. Negative-test evidence (selected)

| Case | Result |
|---|---|
| any of 8 composition fields = `derived` / `validated` / `measured` / unknown string / missing | FAIL |
| any of 5 wrench fields = unknown/unrecognized status (incl. the audit's `UNRECOGNIZED_STATUS` bypass) | FAIL |
| wrench fields accept only `null_blocked` / `blocked` | PASS (positive control) |
| manifest missing `composition_status` / `wrench_frame_status` roots, or declaring `VERIFIED` / `DETERMINED` | FAIL |
| manifest unknown `field_status` key | FAIL |
| frame missing an Option-B field (present-and-null required) | FAIL |
| frame `cop_position` non-null | FAIL (CLI exit 1) |
| provenance: any one of 18 fields missing (each tested) | FAIL |
| provenance: bad sha256 format / bad git commit / non-int seed / non-dict theta / non-null bad cop hash | FAIL |
| raw: `[{"unrelated": 1}]` | FAIL |
| raw: any required stream category missing (each tested) | FAIL |
| raw: unknown stream category; structurally invalid record (empty lists / empty collision names) | FAIL |

## 9. Mutation-test evidence

| Mutation | Result |
|---|---|
| RFC § 3.10 + 1 table row (`r12_mutant_field`) | `check_rfc_binding` FAIL, full smoke FAIL (`provenance_binding`) |
| RFC § 3.10 − 1 row (`ring`) | `check_rfc_binding` FAIL |
| Contract § 8 + 1 checklist row | `check_contract_binding` FAIL, full binding FAIL |
| Contract § 8 − 1 row (`ring`) | `check_contract_binding` FAIL |
| Contract § 6 pose-topic token removed | `check_raw_category_binding` FAIL |

## 10. R6–R10 regression

All invariant tests pass (`TestR6toR10Regression`): R6 — no `CADENCE_MIN_HZ/CADENCE_MAX_HZ` in the probe, event-aware publication present; R7 — two-state vocabulary and no-fabrication rules intact in RFC/contract; R8 — `composition_status = UNVERIFIED` preserved and un-certifiable otherwise; R9 — measured/derived/`derived_pending_validation` vocabulary unchanged; R10 — `timestamp_association_compatibility` (not clock synchronization). Additionally: `stage1_probe.py` byte-identical (never modified in R12); the R6–R10 designs were not touched.

## 11. Scope integrity

- `git diff` over the frozen set is **empty**: `tools/run_phase_a_episode.py`, `tools/run_p0a_gait_candidate.py`, `tools/run_stage0_dataset.py`, `tools/stage1_probe.py`, `src/**`, `dataset/stage0/**` (Stage 0, controller, gait generator, NAO model, world, physics, probe).
- Stage 0 hashes re-verified: `corpus_index.json` = `260fd9e7…`, `nao_simple.sdf` = `44ed8285…` (pins intact).
- RFC sha256 **unchanged** from R11 (`195178f5…2f02aac`) — no RFC rule modified.
- No acquisition (see § 15); fixtures are synthetic and stored under `/tmp` (never `dataset/stage1/`).

## 12. Files changed (vs baseline `3a4ab86`)

| File | Change |
|---|---|
| `tools/stage1_contract_check.py` | rewritten as the fail-closed enforcement checker (R12-A smoke CLI, R12-B allowlists + manifest artifact validation, R12-C contract parsing + dual binding + raw-category binding, R12-D raw structural contract, provenance formats) |
| `tools/test_stage1_r12.py` | **new** — 40 adversarial/mutation/anti-circularity/CLI/regression tests |
| `tools/test_stage1_r11.py` | fixtures minimally strengthened for the R12 raw/frame contracts (semantics unchanged) |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | v1.3 → v1.4: § 8 dual machine-binding sentence; § 10 R12-A smoke entrypoint + fail-closed exit semantics; document status row |
| `STAGE1_REV12_FAIL_CLOSED_CONTRACT_ENFORCEMENT_REPORT.md` | **new** — this report |

## 13. Hashes (at report time)

```text
195178f52ebe27efd696c9750f0a7164bdbc09790a1222d491d1120eb2f02aac  docs/STAGE1_DYNAMICS_DATASET_RFC.md   (unchanged from R11)
ee917103f539aeeb670bca605b56c31aca2fc2c8a5a1d1e7f63aacd4e971f3bc  tools/stage1_contract_check.py
d8e3d36d7e1870b209c1f49d95ba477f03553a41bbca61d9a9b48eee38846586  tools/test_stage1_r12.py
4e794713842c3c7dae0814523b1174f298682398a5ce465c556988f32119ab73  tools/test_stage1_r11.py
b22fe24a8dd5420959ef95ddf71c7f773ed78b17235a96c10c4f1b32f70b01ba  docs/STAGE1_COLLECTOR_CONTRACT.md
```

## 14. Git state

- R12 commit: created immediately after this report (single commit containing the five files above); HEAD at report finalization: `git log -1 --format=%H`. Not pushed — remote publication remains an owner/process action.
- Working tree after commit: clean except the pre-existing untracked `docs/task4_phase4b_evidence/` (out of Stage 1 scope, untouched).

## 15. Explicit statements

```text
Acquisition performed: NO
Pilot performed: NO
96-episode acquisition performed: NO
```

Only synthetic fixtures, unit tests, and contract smoke checks ran — no Gazebo acquisition, no real Stage 1 dataset, no probe execution. Status: **R12 implementation complete** (not CLOSED — closure belongs to the next Independent Adversarial Audit). Remaining by design: `composition_status = UNVERIFIED`, wrench frame/reference = UNKNOWN (Option-B nulls), silence = `UNKNOWN_NO_MESSAGE`, timestamp evidence = association compatibility (not clock identity).
