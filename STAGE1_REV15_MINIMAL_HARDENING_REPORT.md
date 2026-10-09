# STAGE1_REV15_MINIMAL_HARDENING_REPORT

R15: normative-schema integrity pin, strict JSON parsing, and
frame-count runtime enforcement. Scope = the two Blockers of the GPT-5.6
Independent Adversarial Audit of R14 (B5, B6), nothing else.

**Status: R15 IMPLEMENTATION COMPLETE — NOT CLOSED. R15 requires a fresh
GPT-5.6 independent adversarial audit (Decision A) before any Collector
implementation. Pilot/96-episode acquisition needs separate
authorization. Nothing in R15 authorizes acquisition.**

Implementation commit: `b097ba6` (this report records that hash; the
report itself is the follow-up commit). All commits are LOCAL — not
pushed; remote publication is the owner's action.

---

## 1. Baseline (Phase 0, verified before any change)

- HEAD `d9289edadc8cee4a4f5c8999ad1025aefe8267f1` (branch `main`) =
  the R14 report-hash commit; R14 implementation = `3032780`; workspace
  clean except the pre-existing untracked `docs/task4_phase4b_evidence/`
  (present before R15, untouched by it).
- RFC: `docs/STAGE1_DYNAMICS_DATASET_RFC.md` (REV1.5-NORMATIVE-SCHEMA),
  one ```json normative block at lines 916–1025, file sha256
  `f8662d1673f1a30e9cfb2d8156214d7124eb45c08301087d85c4fae1ca894efe`.
- Contract: `docs/STAGE1_COLLECTOR_CONTRACT.md` (v1.6), one ```json block
  at lines 324–433, byte-identical to the RFC block (block span sha256
  `eb620b233b3a4499cceaf23a1c1054a7c2693415e018874501b773d713b25241`),
  file sha256 `0384aad2d1f4db92c7fc02729c71fa9daf4d3542a26df8e72d2682ec`
  `fdab0295`.
- Schema extraction at R14: `re.finditer(r'```json\n(.*?)```', ...)` in
  `_find_normative_block` — returns the **FIRST** fenced json block that
  plain-`json.loads`-parses to a dict whose `schema_version` startswith
  `stage1-normative`; duplicate blocks are silently ignored.
- `frame_count` chain at R14: `main(--smoke)` → `run_smoke` →
  `check_frame_artifact`, where the check
  `len(frames) != frame_count` executed ONLY IF
  `schema['frame_artifact']['count_must_equal'] == 'frame_count'` — a
  rule declared inside the two mutable documents.
- JSON parsing at R14: plain `json.loads`/`json.load` everywhere
  (schema documents and artifacts) — duplicate keys last-wins,
  `NaN`/`Infinity` accepted.

## 2. Original reproduction of both Blockers (R14 baseline, real CLI)

All reproductions run the REAL production smoke CLI
(`tools/stage1_contract_check.py --smoke`) over temporary mutated copies
of the actual documents + hand-written valid artifacts
(`frame_count=2`, ONE frame record in the frame artifact — a mismatch
that must FAIL):

```
python3 tools/stage1_contract_check.py --smoke \
  --rfc  <mutated-rfc-copy.md> --contract <mutated-contract-copy.md> \
  --manifest manifest.json --provenance provenance.json \
  --frame frame.json --raw raw.json
```

| # | Attack (mutation applied to temp copies) | R14 exit | R14 smoke | Verdict |
|---|---|---:|---|---|
| 0 | Authorized documents, count 2 vs 1 frame (control) | 1 | FAIL | count check active |
| 1 | **B5**: `"count_must_equal": "frame_count"` → `"frame_records"` in BOTH docs (synced, copies still equal) | **0** | **PASS** | **bypass reproduced** |
| 2 | **B5b**: count rule LINE deleted from BOTH docs | **0** | **PASS** | **bypass reproduced** |
| 3 | **B6a**: an exact duplicate of the normative fenced block appended to the RFC (+ valid artifacts) | **0** | **PASS** | **duplicate accepted** |
| 4 | **B6b**: duplicate `"frame_artifact"` JSON key (last-wins value drops the count rule) inserted into BOTH docs | **0** | **PASS** | **bypass reproduced** |
| 5 | **B6b2**: duplicate top-level `"schema_version"` key in BOTH docs | **0** | **PASS** | **duplicate keys accepted** |

Post-R15, every attack above exits 1 with smoke ok=false (§ 8 gives the
full per-test evidence).

## 3. Root causes

- **B5**: the ONLY schema-integrity invariant was
  RFC Appendix D block == Contract § 8.4 block (canonical JSON
  equality of two mutable copies). A synchronized mutation of both
  copies preserves that equality, and the `frame_count` consistency
  check was conditional on a schema-declared rule — so removing or
  weakening the declaration in both documents silently removed the
  check. No authorized-version anchor existed anywhere.
- **B6**: (a) `_find_normative_block` picked the first matching fenced
  block — a second normative block (duplicate, or a tampered copy
  inserted earlier) was silently ignored; (b) plain `json.loads`
  resolves duplicate object keys last-wins and accepts NaN/Infinity, so
  an ambiguous block could weaken the norm (e.g. a second
  `frame_artifact` key) while remaining "canonically equal" across the
  two documents.

## 4. Minimal fix design and rationale

Three changes, all inside the checker + tests. NO RFC byte, NO Contract
byte, NO frozen file changed (verified by hashes in § 11).

- **R15-A (pin)**: the checker pins the authorized schema by version
  string + SHA-256 of the block's exact fenced byte span. Every
  document path the checker reads a schema from (RFC and Contract)
  must match the pin BEFORE the schema is used. Rationale: the only
  artifact an attacker can mutate without touching code is a document;
  the pin anchors the norm in code (version-controlled, reviewed),
  which a document-pair edit cannot reach. This is deliberately NOT a
  second hardcoded schema (task § 4.3): the pin is 2 constants; all
  semantics still load from the (now pinned) block.
- **R15-B (uniqueness + strict JSON)**: deterministic CommonMark-style
  fence scanning; exactly ONE normative ```json block per document;
  strict JSON (duplicate keys / non-standard constants rejected) for
  the schema blocks AND for artifact loading. Rationale: closes the
  "silent first-match" and "last-wins" ambiguity classes at the exact
  point they enter the checker.
- **R15-C (runtime invariant)**: `frame_count == len(frames)` is
  enforced unconditionally in `check_frame_artifact`; the check no
  longer reads `frame_artifact.count_must_equal` to decide whether to
  run. Rationale (task § 6.1): the constraint must not depend on the
  two documents' declarations — with R15-A the declaration cannot be
  tampered, and with R15-C it cannot even be *authorized away* without
  a code change; `_validate_schema_shape` cross-checks that the
  authorized schema still declares the same rule (code invariant and
  pinned declaration must agree).

## 5. Integrity pin: representation, computation, update policy

- **Representation**: TWO constants in
  `tools/stage1_contract_check.py:103-105` —
  `_AUTHORIZED_SCHEMA_VERSION = 'stage1-normative-schema/1.0'` and
  `_AUTHORIZED_SCHEMA_SHA256 = 'eb620b23…b25241'`. ONE hash convention
  (raw block bytes) is used for both documents — no mixed
  canonical/raw hashing, no hashing of arbitrary document fragments.
- **Extraction boundary (deterministic)**: line-based CommonMark-style
  fence pairing (`_scan_fenced_regions`, tools file line 137): a fence
  line is a line whose stripped form starts with ``` ; while a fence is
  open, only a bare ``` line closes it (an info-carrying line such as
  ```json is content and cannot close); an unclosed fence at EOF is a
  structural error. The block text = the lines strictly between the
  ```json opening line and its closing ``` line, read with
  universal-newline normalization and joined with '\n'. For the
  repository documents this span is 5126 bytes and hashes to
  `eb620b233b3a4499cceaf23a1c1054a7c2693415e018874501b773d713b25241`
  in BOTH documents (they are byte-identical copies, R14 property).
  Recompute: `sha256('\n'.join(lines_between_fences).encode('utf-8'))`.
- **Why raw bytes, not canonical JSON**: canonical-JSON hashing would
  be blind to key order/whitespace-only edits and would have to be
  computed AFTER duplicate-key resolution — it cannot substitute for
  duplicate-key detection (task § 4.2). Raw bytes pin the exact
  authorized text; strict parsing then guarantees the pinned text has
  an unambiguous JSON meaning. Strict parsing and the pin are
  complementary, not alternatives.
- **Consistency**: the pin corresponds to RFC **REV1.5** / Contract
  **v1.6** / `schema_version` `stage1-normative-schema/1.0` (the R14
  authorized state; the R15 tests pin the RFC and Contract file hashes
  and the block pin against both documents).
- **Update policy (governed version upgrade)**: to change the schema,
  an authorized change must (1) edit the block in the RFC Appendix D
  AND the Contract § 8.4 byte-identically, (2) bump `schema_version`,
  (3) recompute the block span SHA-256, (4) update
  `_AUTHORIZED_SCHEMA_*` and the affected test expectations (incl.
  file-hash pins) in the same reviewed change. Unknown versions or pin
  mismatches fail closed; there is no auto-accept path and no
  RFC/Contract-synced bypass (proven by the § 8 attack matrix).

## 6. Schema-block uniqueness implementation

`_find_normative_block` (tools file line 177):

1. `_scan_fenced_regions` pairs every ``` fence deterministically and
   reports an unterminated fence as a structural error (missing end
   marker; also catches a duplicated closing marker — the stray fence
   stays open to EOF).
2. A region is **normative-relevant** iff it is a ```json fence whose
   content text contains `stage1-normative`. Ordinary example JSON
   blocks never carry the prefix and are never mistaken for the schema
   (task § 5.1). The prefix inside a NON-json fence (bare/text/math) is
   an explicit marker-order/fence-type error (close-before-open
   attack).
3. Exactly ONE normative-relevant region must exist: zero → "not
   found"; ≥2 → "N duplicate normative schema blocks… no silent
   first/last-match selection". A duplicated opening marker inside the
   block becomes block content → strict parse failure. No block is
   ever silently chosen.
4. The single candidate is strict-parsed (§ 7) and must be a JSON
   object declaring a `stage1-normative…` schema_version.

Verified against the actual documents: the RFC has 10 fenced regions
(math/text/json), the Contract 7 (bare/json), exactly one ```json
normative block each; no structural errors.

## 7. Duplicate-JSON-key rejection policy

- `_strict_json_loads` (line 129) = `json.loads` with
  `object_pairs_hook=_reject_duplicate_keys` (raises on any duplicate
  object key at ANY nesting depth — top level, `frame_fields`,
  `status_semantics`, nested structures) and
  `parse_constant=_reject_nonstandard_constant` (NaN/Infinity/-Infinity
  rejected — non-standard constants are parse failures, never values).
  No reliance on parser default last-wins behavior anywhere.
- Applied at: `load_normative_schema` (both documents' blocks — a
  duplicate key must fail BEFORE the schema is accepted) and
  `load_json_artifact` (line 1291 — manifest/provenance/frame/raw;
  closes the artifact-side ambiguity, e.g. a frame with two
  `cop_position` keys smuggling a non-null value past the Option-B
  check).
- Array-level duplicates: `_validate_schema_shape` (line 236) rejects
  duplicate field names inside every `frame_fields.*.fields` group
  (already checked for `provenance_fields` since R14) — a duplicate
  field *spec* is ambiguous, not a silently-deduplicated list.
- Test fixtures for duplicate keys are built by RAW-TEXT surgery on the
  documents / on `json.dumps` output — never via Python dicts (a dict
  cannot represent duplicate keys; task § 5.3 note).

## 8. frame_count runtime enforcement — actual call path

`main(--smoke)` (line 1503) → `run_smoke` (line 1371) →
`check_frame_artifact` (line 1311). Inside it, AFTER per-frame emission
checks and BEFORE returning, the invariant runs UNCONDITIONALLY
(R15-C; the R14 `count_rule == 'frame_count'` gate is deleted):

```
fc = provenance.get('frame_count')           # must be int-not-bool
if not _is_int_not_bool(fc):  -> FAIL        # bool/float/missing/null
elif fc < 0:                  -> FAIL        # negative
elif len(frames) != fc:       -> FAIL        # the invariant
frames not a JSON array      -> FAIL        # null/object/missing file
```

`len(frames)` is the count of records actually loaded from the frame
artifact file; `fc` comes from the provenance artifact. A schema
mutation can no longer disable it (R15-A pins the schema; R15-C makes
the check schema-independent); `_validate_schema_shape` requires the
authorized schema to keep declaring `count_must_equal: "frame_count"`
so a future authorized rule change must update code + docs + pin
together.

## 9. New attack fixtures — real CLI commands, exit codes, results

All 51 cases in `tools/test_stage1_r15.py` drive the REAL
`--smoke` CLI via subprocess (command shape as § 2; each test writes
its own mutated temp copies and artifacts). Summarized verdicts
(0 = PASS, 1 = FAIL — all as designed):

R15-A integrity pin (10): authorized+valid → 0; authorized empty
episode → 0; RFC-only mutation → 1 (pin); Contract-only mutation → 1
(pin, message locates `contract.md`); **synced identical unauthorized
schema → 1 (pin — THE B5 closure)**; synced rule removal → 1; synced
semantic type mutation → 1; unknown version (synced) → 1 ("is not the
authorized"); version-vs-pin error classes distinguished → 1/1 with
distinct messages; prose-only edit → 0 (unrelated doc edits stay
non-blocking).

R15-B uniqueness + strict JSON (14): unique block normal rules → 0;
RFC duplicate block → 1; Contract duplicate block → 1; RFC missing
block → 1; Contract missing block → 1; duplicate opening marker → 1;
duplicate closing marker → 1 (unterminated fence); close-before-open →
1 (prefix inside a bare-fenced region); top-level duplicate JSON key →
1; nested duplicate key → 1; **duplicate `frame_artifact` key weakening
the count rule in BOTH docs → 1 (THE B6b closure)**; NaN in block → 1;
unknown native extra fields in artifacts → 0 (lossless retention kept);
frame artifact duplicate key (raw text) → 1; raw-record duplicate
`stream` key → 1.

R15-C frame_count invariant (11): matrix (0,0)→0, (1,1)→0, (2,2)→0,
(0,1)→1, (2,1)→1, (1,0)→1; negative → 1; bool → 1; float → 1;
frames=null (raw) → 1; frames=object → 1; frames file missing → 1;
invalid frame with equal count → 1 (count equality alone never passes a
bad frame); synced rule deletion → 1; synced rule optionalized → 1;
synced frame_artifact metadata change (copies equal) → 1; authorized
zero-frame episode → 0.

Unit level (7): strict JSON dup keys at depth / constants / normal
accept; fence scanning unterminated + info-line-cannot-close; shape
validation rejects duplicate frame-field names and requires the count
rule declaration.

R6–R10 + frozen regression (9): see § 10.

## 10. Regression results (all actual runs, 2026-10-09)

| Suite | Tests | Result |
|---|---:|---|
| `tools/test_stage1_r11.py` | 15 | OK |
| `tools/test_stage1_r12.py` | 40 | OK |
| `tools/test_stage1_r13.py` | 49 | OK |
| `tools/test_stage1_r14.py` | 32 | OK |
| `tools/test_stage1_r15.py` | 51 | OK |

R14 suite note (task § 7 discipline): exactly TWO R14 unit tests
changed — `test_option_a_shapes_come_from_schema` and
`test_m6_world_required_enforced_by_mutated_schema`. At R14 they loaded
a schema from a MUTATED document copy; R15-A intentionally makes
mutated documents unloadable, so the mutation is now applied to an
in-memory copy of the authorized schema. The evidenced property
(shapes/requiredness come from the schema object, not checker
hardcoding) is unchanged and the assertions are equivalent in
strength — this is a test-method update forced by the new authorized
-version policy, NOT a weakening to make a regression pass. All other
30 R14 tests pass byte-identical, including the full M1–M14 CLI
mutation matrix (mutated-RFC now fails earlier — fail-closed at the
pin — which is the intended stronger behavior).

R6–R10 (re-checked, unchanged): R6 no cadence band / event-aware
publication tokens in probe; R7 two-state vocabulary
(`VALID_CONTACT`/`UNKNOWN_NO_MESSAGE`, no fabricated flight labels);
R8 `composition_status == 'UNVERIFIED'` from the pinned schema; R9
measured/derived group separation unchanged (measured=10 fields with
status `measured`, ordinary_derived status `derived`, foot/CoM fields
in the pending-validation groups); R10
`timestamp_association_compatibility` token + "universal clock
identity" caveat present.

Frozen state: all 13 Phase B0 freeze-record anchors re-verified intact
(§ 11); `tools/run_phase_a_episode.py`, `tools/stage1_probe.py`,
`nao_simple.sdf`, `nao_world.sdf` byte-identical to the R13 baseline
commit `88aa3fa` (git diff empty — asserted by tests). Stage 0, gait
generator, controller, robot model, world, physics, probe: untouched.

## 11. Hashes (actual, recomputed 2026-10-09)

| Object | SHA-256 |
|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` (unchanged by R15) | `f8662d1673f1a30e9cfb2d8156214d7124eb45c08301087d85c4fae1ca894efe` |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` (unchanged by R15) | `0384aad2d1f4db92c7fc02729c71fa9daf4d3542a26df8e72d2682ecfdab0295` |
| normative block fenced span (RFC AND Contract, byte-identical) | `eb620b233b3a4499cceaf23a1c1054a7c2693415e018874501b773d713b25241` (= the pin) |
| `tools/stage1_contract_check.py` (R15-modified) | `0b29b26f21a4c06f91f8135047b35be101116a086c902bfc44d5f28f2e212ced` |
| `tools/stage1_probe.py` (frozen) | `9db101d94a6f66e18288daeda5cf9f0cc54fff116f52b9b90600b1852d7edf4b` |
| `tools/run_phase_a_episode.py` (frozen) | `a024140f294a293b0fb8503e6f53960028d8919c516f1cb640891d9fc297d6fb` |
| `src/nao/nao_description/urdf/nao_simple.sdf` (frozen) | `44ed828509db9e869bfa701b491233862dcd1448183bb2ec0a1c3b497aa46400` |
| `src/nao_gz_sim/worlds/nao_world.sdf` (frozen) | `383341d11d831d0376cae074b1205e5ad7784841a6670bcf0db33813065709cc` |

All 13 `Phase_B0_Freeze_Record.md` § 3 anchors verified intact
(13/13 match, 0 mismatched, 0 missing).

## 12. Commits and workspace state

- Implementation commit: `b097ba6` —
  `tools/stage1_contract_check.py` (modified), `tools/test_stage1_r14.py`
  (modified, two unit-test method updates, § 10),
  `tools/test_stage1_r15.py` (new, 51 cases).
- Report commit: this file (records `b097ba6`).
- Diff summary vs the R14 baseline `d9289ed`:
  `tools/stage1_contract_check.py` 339 lines changed (+282/−57: docstring,
  pin constants + strict-JSON/fence-scan/load-authorization sections,
  shape-validation additions, unconditional count invariant, strict
  artifact loading, CLI text); `tools/test_stage1_r14.py` (+26/−17, two
  unit-test method updates, § 10); `tools/test_stage1_r15.py` (+851,
  new, 51 cases).
- Workspace after the commits: clean apart from the pre-existing
  untracked `docs/task4_phase4b_evidence/` (existed before R15; not
  part of it). **Commits are LOCAL — NOT pushed.** Remote publication
  remains the owner's explicit action.

## 13. Open issues and known limitations

1. The pin lives in checker code: an attacker able to modify the
   checker itself could move the pin. Mitigation: the checker is
   version-controlled and audit-reviewed; documents (the
   collector-writable surface) cannot reach it. This is the intended
   trust boundary, not a defect — but the audit should confirm it.
2. Fence scanning uses simplified CommonMark rules (any ```-prefixed
   line is a fence line; only a bare ``` closes; ~~~ fences are not
   tracked). Deterministic and verified against both real documents;
   pathological markdown (e.g. a ```json line inside a ~~~ fence) would
   be treated as a normative-relevant region — fail-closed, not a
   bypass.
3. Normative relevance is prefix-string-based: a NON-normative example
   json block that happens to contain the literal `stage1-normative`
   string would be counted as a second normative block → hard failure
   (fail-closed; the string must not appear in examples).
4. Duplicate keys in NON-normative example JSON elsewhere in the
   documents are not policed (out of scope; only the normative block
   and artifacts are strict).
5. A version bump changes the block bytes, so a version-mismatch case
   also reports the pin mismatch alongside it (both facts are true;
   the version message identifies the class — tested in § 9).
6. `frame_count` consistency is checked over the artifacts supplied to
   `--smoke`; frames never delivered cannot be counted (inherent to
   file-level enforcement; the Collector contract still governs
   emission).
7. Two R14 unit tests re-evidenced via in-memory schema mutation
   (§ 10) — disclosed for the auditor's judgment.
8. Composition remains UNVERIFIED, wrench frame UNKNOWN (CoP blocked),
   silence = UNKNOWN_NO_MESSAGE, timestamp = association compatibility
   — all unchanged by design (R6–R10 semantics preserved).

## 14. Explicit authorization state

- R15 implementation tests all pass (187 across r11–r15). **This is NOT
  acceptance.** R15 is CLOSED only by a GPT-5.6 independent adversarial
  audit Decision A.
- Not authorized and not performed: Collector implementation; Stage 1
  acquisition (any episode); 20–50-episode pilot; 96-episode corpus;
  Gazebo runs; changes to Stage 0 / gait generator / controller /
  robot model / world / physics / probe; framework rewrite; fitness
  redesign; 3D-LIPM / Residual / PINN / MPC / CEM / sim-to-real work;
  marking pose composition verified; interpreting contact silence as
  zero force; calling timestamp association clock synchronization.
- Next required step: `STAGE1_REV15_INDEPENDENT_ADVERSARIAL_AUDIT`
  (GPT-5.6) over this report, commit `b097ba6`, and the reproduction
  commands of § 2/§ 9.
