# STAGE1_REV12_CORRECTION_REPORT

| Field | Value |
|---|---|
| Role | Implementation Agent (GLM-5.3) — correction executor |
| Task | `STAGE1_REV1.2_CORRECTION_TASK` — apply the minimal corrections identified by the GPT-5.6 independent audit of the REV1.1 preparation artifacts |
| Objective | Move Stage 1 from *Preparation BLOCKED* to *acquisition contract corrected and ready for re-audit* |
| Date | 2026-10-08 |
| Correction commits | `9f64d01` (RFC + probe corrections, incl. previously uncommitted REV1.1 preparation set); final evidence commit — see § 6.2 |
| Acquisition status | **NOT AUTHORIZED — no acquisition authorization is claimed by this report.** No Stage 1 episode was started; REV1.2 is pending re-audit |

---

## 1. Changed files

| File | Change | SHA-256 (after correction) |
|---|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | REV1.1-FINAL → **REV1.2-CORRECTED**: semantic corrections R1–R5 (§ 2 below); population, seeds, rings, schema shape, formulas, and all § 0.1 frozen pins unchanged | `f006c0f223708ae719e131ca1d9d046e9de209d012cb9e9f2c1c4f4dcbc2781b` |
| `tools/stage1_probe.py` | V-1 criterion 4 rewritten (envelope criterion removed; nearest-neighbor statistics); V-2 criterion 2 renamed `world_pose_semantics` → `pose_frame_semantics` with corrected semantics; R5 wording; `--report-suffix`; `config_sha256`; schema `stage1-probe/1.1` → `1.2`; bridge demoted to optional (R1) | `d9e9ec555a424f6e6978de51a8151233bc60ad7511270211a823b4dd77e3cbab` |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | v1.0 → v1.1: governing RFC → REV1.2-CORRECTED; evidence base → run-6 REV12 reports (+ run-4 end-hold archive); `UNKNOWN_NO_MESSAGE` naming; timestamp/frame-semantics criteria updated to REV1.2; § 6.1 states **native gz raw capture is mandatory** and RFC source path == collector source path; § 11 revision statuses; § 12 REV12 verdicts | `c765dd4a3ae083e9c94a9f2e131b16b34588a68c7764db9849f18d89efc6715f` |
| `dataset/stage1/probe/V1_report_REV12.json` | REV1.2 probe re-run evidence (run 6, in-gait window) | `55ab7e855e2cd7d2ad7a709d303b1e622f4a0e6fc9f7c5333a1810fcda316ff9` |
| `dataset/stage1/probe/V2_report_REV12.json` | REV1.2 probe re-run evidence (run 6, in-gait window) | `0a2107969e1aaa6f5e5d4e81e84aad90b71ecc55039484c83604394ee264a27e` |
| `dataset/stage1/probe/V1_report_REV12.run4_endhold.json` / `V2_report_REV12.run4_endhold.json` | archived run 4 (REV1.2 criteria, but the collection window fell into the post-gait end-hold phase: both feet contact=false, zero contact messages) — not used for verdicts; documents that the gait end-hold phase is message-silent on both contact topics | `7958b0367054d5929040fb356ec9fe7dba24a4c839cf2c9ffa57969c1c86ddb5` / `441809bf5eab86a1f842757bc2c355e8a5005aefb0c1cb40a85a99783352b5e7` |

No other file was created or modified. `docs/task4_phase4b_evidence/` (pre-existing untracked, not Stage 1 scope) was left untouched.

## 2. Why each correction was required

| # | Audit finding (why required) | Correction applied |
|---|---|---|
| **R1 — raw contact source** | The RFC treated the ROS bridge contact stream as the canonical source. The bridge is field-lossy (measured: per-contact `header` stamp, `Contact.world`, `Wrench.force_offset` have no ROS counterpart — all three are mandatory raw-retention fields). Canonical evidence must be the lossless native stream; the bridge can only be a derived cross-validation copy. | RFC § 3.6.1: source corrected to the **Gazebo native `Contacts` stream** (gz-transport subscribe of `/nao/left_foot_contact_raw`, `/nao/right_foot_contact_raw`); native payload retention mandatory; bridge demoted to **optional derived stream** with documented field loss; retention table re-ordered to the audit's field list (header timestamp, collision identity, body identity, both body-side wrench incl. force/torque/force_offset, position, normal, depth); prohibition added: derived per-foot force/moment may never replace raw evidence. Appendix A6 updated; probe criterion 1 no longer blocks on a dead bridge (optional derived stream). |
| **R2 — V-1 timestamp validation** | The REV1.1 envelope-overlap criterion (`fraction_within_joint_envelope` ≥ 0.95) interpreted **subscriber capture-window overlap** as timestamp-domain validation. Measured cause of the run-3 failure (0.861/0.868): rclpy/DDS subscription discovery skew (0.86–2.3 sim s) — a probe instrumentation artifact, not stamp-domain disagreement. | RFC App. B V-1.4 rewritten: **timestamp-domain validation on source stamps only** — same simulation clock domain established by **nearest-neighbor statistics** (median / p95 / max of \|contact source stamp − nearest `/joint_states` source stamp\`); monotonicity + measured dt; **callback arrival time is not valid measurement time**; the envelope criterion is explicitly deleted. Probe criterion 4 rewritten accordingly (p95 ≤ contact association window 0.05 s as the pre-registered domain decision rule; run 6 measured median 0.003 s / p95 0.005 s → `confirmed`). |
| **R3 — contact validity semantics** | REV1.1's third state `UNKNOWN` did not make the failure mode explicit, and the zero-wrench-from-silence prohibition needed sharpening. | RFC § 3.6.2–§ 3.6.4: fail-closed rule kept; states are now `VALID_CONTACT` / `UNKNOWN_NO_MESSAGE` / `VALID_NO_CONTACT`; **no contact message → `UNKNOWN_NO_MESSAGE`; no zero wrench from silence**; `VALID_NO_CONTACT` requires an **explicit empty contact message** **or** an **independent validated no-contact source** (exactly two pre-registered forms). Probe/contract naming updated. |
| **R4 — V-2 pose semantics** | RFC REV1.1 App. B V-2.2 assumed each `Pose_V` entry is the link's **world pose**. Probe evidence (runs 1/3/6) falsifies this: link entries are **parent/model-relative** poses (`base_link_identity_fraction = 1.0` while the `nao` model entry moves; raw `l_ankle` z < 0). World poses must be recovered by composition. | RFC: new § 3.3 "pose frame semantics" note — `T_world_link = T_world_model · T_model_link`, both factors from the same `Pose_V` message; entries are **never** world poses; § 3.4 CoM inputs and § 3.5 foot poses reclassified (foot pose class measured → **derived** via composition); Appendix A7 / App. B V-2.2 corrected. Probe criterion 2 renamed `pose_frame_semantics` and reports the semantics as **model-relative, never world pose**; `model_relative_for_links` now *confirms* the corrected contract (world-frame entries would contradict it). Standing-geometry/ground checks demoted to informational (walk-scenario swing feet are expected outside the standing envelope). |
| **R5 — validation wording** | The probe and contract claimed the composition formula was "validated end-to-end" — but the only comparison ever performed is the **identity link** (`base_link`, which *is* the model frame, so the rotation term is never exercised). That claim overstated the evidence. | All "end-to-end validated" claims removed and replaced by: the base_link comparison is an **identity-link consistency check only**; the **composition method requires verification using selected non-identity links** (e.g. `l_ankle`/`r_ankle`) before composed poses are treated as validated world poses (RFC § 3.3 note, App. B V-2.2; probe composition evidence + determined facts; contract § 4.3). |

## 3. No frozen component modified

- `git diff` over the frozen set is **empty**: `tools/run_phase_a_episode.py`, `tools/run_p0a_gait_candidate.py`, `tools/run_stage0_dataset.py`, `src/**` (controller / gait generator / launch / converters), `dataset/stage0/**`.
- Pinned hashes re-verified identical to RFC § 0.1 / Stage 0 pins:
  - `nao_simple.sdf` = `44ed828509db9e869bfa701b491233862dcd1448183bb2ec0a1c3b497aa46400`
  - `nao_world.sdf` = `383341d11d831d0376cae074b1205e5ad7784841a6670bcf0db33813065709cc`
  - `dataset/stage0/corpus_index.json` = `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544`
  - `dataset/stage0/stage0_theta_list.json` = `dd52135eed21f3b20c330a2a61e90e89a1f9c13690bddd58330762dc45b9c04a`
- No Stage 1 acquisition episode was started (no 96-episode run, no pilot episode, no collector writer). No dataset schema change, no new gate framework (V-3/V-4/V-5 not introduced), no gait-behavior change, no optimization.

## 4. Probe result summary (REV1.2 re-run — run 6)

Command (mirrors run-3 configuration; walk + FROZEN_GAIT parameters identical to the frozen runner; in-gait window):

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe6 --report-suffix REV12
```

Pinning: `git_commit = 9f64d012d9eaee07c350cdf075dc3e8d1a0bb82e`; `rfc.sha256 = f006c0f2…` (REV1.2); `config_sha256 = 3ccf1480097f119354d3e815479c3fb1ae31f2049f5727a3ce3ef2f138fc07f0`; wallclock 2026-10-08T00:59–01:01 UTC. `PROBE_EXIT = 1` (any FAIL — fail-closed exit codes unchanged).

**V-1 (`V1_report_REV12.json`) — verdict FAIL**, blocking = `['population_cadence', 'publication_mode']`:

| Criterion | Status | Evidence |
|---|---|---|
| population_cadence | **contradicted (blocking)** | effective rate 181.85 Hz (L) / 157.72 Hz (R) — outside the REV1.1 [50, 150] band. The effective rate = burst rate × duty is gait-phase dependent (run 3, later window: 136.05/128.90 in-band). Criterion semantics unchanged by REV1.2 (not in correction scope R1–R5) → revision item (contract § 11 R2). |
| body_identity | confirmed | fraction_expected_pair = 1.0 both feet; verbatim pair `nao::l_ankle::l_ankle_collision \| ground_plane::link::collision`; 0 cross-contamination |
| publication_mode | **contradicted (blocking — unchanged, genuine)** | event-only publication; 0 empty messages in the window; no-contact unobservable with no independent validated signal (RFC App. B V-1 failure condition) |
| timestamp_semantics | **confirmed (R2 correction effective)** | nearest-neighbor source-stamp statistics: median 0.003 s / p95 0.005 s (L), median 0.003 s / p95 0.005 s (R); max 1.259 s is a window-boundary artifact (joint stream ends before the contact stream at snapshot cutoff; p95 decision unaffected); 0 backwards / 0 duplicates; ROS-bridge exact-stamp preservation 1.0 |
| frame_reference | unknown (non-blocking) | N3L plausibility only; wrench frame/reference determination still requires common-frame/common-origin comparison |

**V-2 (`V2_report_REV12.json`) — verdict PASS**, blocking = `[]`:

| Criterion | Status | Evidence |
|---|---|---|
| unique_entities | confirmed | all 25 links, bare names, `fraction_messages_with_all_25_links = 1.0` |
| pose_frame_semantics | **confirmed (R4 correction effective)** | `link_entry_frame_semantics = model_relative_for_links` (n = 1336; `base_link_identity_fraction = 1.0`, `nao_entry_identity_fraction = 0.0`); reported as model-relative, never world pose; `/body_pose` exact-stamp match fraction 1.0, max position diff 0.0 m (composition anchor validated); identity-link composition consistency 0.0 m (R5: consistency check only) |
| source_stamps_cadence | confirmed | pose/info 62.95 Hz measured (dt median 0.016 s), joint 100.0 Hz exact, 0 backwards / 0 duplicates |
| alignment_hit_rate | confirmed | 0.02 s causal last-not-after hit rate = 1.0 (1862/1862 joint frames) |

Net effect of the corrections against the run-3 baseline: `timestamp_semantics` contradicted → **confirmed** (R2); `world_pose_semantics` contradicted → `pose_frame_semantics` **confirmed** and **V-2 FAIL → PASS** (R4); `publication_mode` remains the genuine blocker; `population_cadence` regressed to contradicted purely through window-phase dependence (documented; criterion untouched by design).

Run 6 scenario: robot walked the full gait (142 foot alternations, net displacement 0.531 m, body z ≥ 0.330 throughout — no fall; phase4b summary at `/tmp/stage1_probe_run6_evidence/phase4b_summary.json`). Additional reproducible finding: the gait end-hold phase (stop ramp + hold, sim ≈62–66 s) has **both feet contact = false and complete message silence** on both contact topics (run 3 and run 4 traces agree) — further fail-closed evidence for the event-only publication mode; archived in `*.run4_endhold.json`.

## 5. Remaining blockers (Level 3 not authorized)

1. **V-1 `publication_mode` (blocking).** Gz contact publication is event-only; no empty messages; no independent validated no-contact source → `VALID_NO_CONTACT` is not emittable. Per RFC App. B V-1: acquisition blocked until the RFC is revised — either validate an independent no-contact signal (candidate: `contact_converter` boolean, requires a dedicated probe of its independence) or declare the two-state subset (`VALID_CONTACT` / `UNKNOWN_NO_MESSAGE`) as the contract. **No force-free fallback.**
2. **V-1 `population_cadence` (blocking, run 6).** The fixed [50, 150] effective-rate band is gait-phase dependent (run 3 in-band at 136/129 Hz, run 6 out-of-band at 182/158 Hz, run 2 standing 1000 Hz). The gated-semantics rewrite (contact-active rate band + silence-during-swing) is revision item R2 of the collector contract and was **outside the audit's correction scope (R1–R5)** — deliberately not changed here.
3. **Composition verification on non-identity links (R5).** The composition method (declared recovery path for link world poses) has only identity-link consistency evidence; verification using selected non-identity links is required before composed foot/CoM poses are treated as validated. Until then they remain derived-pending-verification fields.
4. **Wrench frame/reference (V-1 `frame_reference` = unknown, non-blocking).** CoP / derived-moment derivation stays blocked pending a common-frame/common-origin comparison.
5. **Process gates (owner actions).** REV1.2 re-audit (this revision does not re-freeze itself); probe-report pinning finalized by commit (done locally, see § 6.2); remote publication of the Stage 0 closure commit and this contract (GPT-5.6 correction 8 — owner action); V-1 pass required before the first production episode (RFC § 0.0 Level 3).

## 6. Deliverables

### 6.1 git diff summary

- Commit `9f64d01` — "stage1: apply REV1.2 audit corrections to RFC and probe": 10 files, +10354 / −77. Includes the previously uncommitted REV1.1 preparation set (RFC REV1.1, probe, run-3 evidence, collector contract v1.0, completion report — the REV1.1 state was never committed by the owner; its SHA-256 values are pinned in `STAGE1_PREPARATION_COMPLETION_REPORT.md`) plus the REV1.2 deltas to RFC and probe. Note: since the REV1.1 state was never committed, the REV1.1→REV1.2 delta is documented by this report's hash table and the correction descriptions, not by a standalone git diff.
- Final evidence commit (this session's second commit) — collector contract v1.1 (docs/STAGE1_COLLECTOR_CONTRACT.md, +73/−55) + REV12 probe reports (run 6) + run-4 end-hold archives + this report. Hash: see § 6.2.

### 6.2 Commit hashes and report hashes

- Correction commit: `9f64d012d9eaee07c350cdf075dc3e8d1a0bb82e` (pinned inside both REV12 reports as `git_commit`)
- Final evidence commit: created after this report — run `git log -1 --format=%H` for the hash; both commits are on `main`, not pushed (push is an owner action).
- Report pinning (hashes in § 1): RFC REV1.2 `f006c0f2…`, probe `d9e9ec55…`, collector contract v1.1 `c765dd4a…`, `V1_report_REV12.json` `55ab7e85…`, `V2_report_REV12.json` `0a210796…`.

### 6.3 Probe results

See § 4. Reports: `dataset/stage1/probe/V1_report_REV12.json` (FAIL, fail-closed) and `dataset/stage1/probe/V2_report_REV12.json` (PASS); both schema `stage1-probe/1.2`, each containing the invocation command (`argv`), timestamp (`wallclock_utc`), git commit, RFC hash, and `config_sha256`.

---

## Status statement

**Stage 1 status: acquisition contract corrected (REV1.2) and ready for re-audit.** The five audit corrections (R1–R5) are implemented in the RFC, the probe, and the collector contract; the REV1.2 probe re-run confirms the corrected criteria (timestamp domain via nearest-neighbor statistics; model-relative pose frame semantics) and preserves the fail-closed blocking failures that remain. **No acquisition authorization is claimed or implied**: Level 3 remains gated on the remaining blockers of § 5 and the owner/re-audit process. No frozen component was modified; Stage 0 is untouched; no episode was collected.
