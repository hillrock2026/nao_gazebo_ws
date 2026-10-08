# STAGE1_REV13_CORRECTION_REPORT

| Field | Value |
|---|---|
| Role | Implementation Agent (GLM-5.3) — minimal-hardening executor |
| Task | `STAGE1_REV13_MINIMAL_HARDENING_TASK` — apply the corrections (R6–R10) identified by the GPT-5.6 independent re-audit of REV1.2 |
| Objective | Move REV1.2 *READY AFTER MINOR FIXES* → **REV1.3 acquisition-contract-ready** (pending final independent audit) |
| Date | 2026-10-08 |
| Correction commits | `fe606f2` (R6–R10 in RFC + probe), `559c243` (common-window evaluation fix), final evidence commit — § 5 |
| Acquisition status | **NOT AUTHORIZED — no acquisition authorization is claimed.** No pilot collection, no 96-episode acquisition, no Stage 0 / controller / gait / optimization change, no new V-gate, no dataset schema change |

---

## 1. Changed files

| File | Change | SHA-256 (after correction) |
|---|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | REV1.2-CORRECTED → **REV1.3-MINIMAL-HARDENING** (R6–R10, § 2 below); status ACQUISITION-CONTRACT-READY, pending final independent audit | `d6a5e02286bb306b78925ed7d977ec41bf7d73e90990ec2e5cffc5ba1578084b` |
| `tools/stage1_probe.py` | R6 event-aware publication criterion (`contact_publication_events`), R7 two-state publication_mode, R8 `composition_evidence` + `composition_status`, R10 criterion rename `timestamp_association_compatibility` (+ common-window evaluation); schema `stage1-probe/1.2` → `1.3` | `9db101d94a6f66e18288daeda5cf9f0cc54fff116f52b9b90600b1852d7edf4b` |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | v1.1 → v1.2: two-state validity table (§ 3.1/3.2/7 — [BLOCKED] resolved), event-aware cadence (§ 3.3), composition `UNVERIFIED` / `derived_pending_validation` (§ 4.3), association compatibility + common-window trimming (§ 3.7/5.3), REV13 evidence + verdicts (§ 1/§ 12), revision statuses (§ 11) | `f5bccc7b91138f6003672baa1c92f91c7e840f421fe869e305519166088f7f04` |
| `dataset/stage1/probe/V1_report_REV13.json` | REV1.3 probe evidence (run 8, in-gait) — **V-1 PASS** | `72491f85c29f3234fa48e46bb19b88849878b2b696b69462a1eb26b8c92080c2` |
| `dataset/stage1/probe/V2_report_REV13.json` | REV1.3 probe evidence (run 8, in-gait) — **V-2 PASS** | `492bfbf33c108671996a1bc1225b46c19e017c534680a796db07b9dbdae83640` |
| `dataset/stage1/probe/V1_report_REV13.run7_discovery_skew.json` / `V2_...json` | archived run 7 (first REV1.3 run: DDS discovery skew made the nearest-neighbor p95 measure the joint-subscription start boundary → spurious contradiction; motivated the common-window evaluation, now documented and archived) | `1a6044c5247382300acc9ff2478ec4e9b16c0c817604c52fb25467d39c0425fa` / `53c3638a4f481adbf3fcc19fc4afa827cc1fec4d84969673bbabffacc2345089` |

No other file was created or modified. `docs/task4_phase4b_evidence/` (pre-existing untracked, not Stage 1 scope) was left untouched.

## 2. Why each correction was required

| # | Re-audit finding (why required) | Correction applied |
|---|---|---|
| **R6 — contact cadence criterion** | The absolute effective-contact-frequency band [50, 150] Hz measured duty × burst-rate over a window and therefore varied with the gait phase of the sampling window (run 3: 136.1/128.9 Hz in-band; run 6: 181.9/157.7 Hz out-of-band, same configuration) — the criterion was measuring window phase, not publication behavior. | Replaced by **event-aware contact publication validation** in RFC App. B V-1.1, the probe (criterion `contact_publication_events`), and the contract § 3.3: metrics `active_contact_event_rate`, `median_event_dt`, `p95_event_dt`, `max_event_dt` (within contact bursts), burst duration; only intervals containing contact messages are evaluated; silence intervals are counted and are **not** cadence failures; silence semantics remain `UNKNOWN_NO_MESSAGE`. Run 8: bursts 28/24, median event dt 0.001 s, active rate 898/937 Hz, 27/23 silence intervals — confirmed. |
| **R7 — two-state contact validity** | The three-state contract kept `VALID_NO_CONTACT` as an acquirable requirement even though no independent no-contact source exists (event-only publication, empty messages never sent) — the publication_mode blocker was a direct consequence. | RFC § 3.6.2–3.6.4, contract § 3.1/3.2/7, probe criterion 3: the **Stage 1 acquisition contract operates exactly `VALID_CONTACT` / `UNKNOWN_NO_MESSAGE`**; rules: no message never equals a zero wrench; no fabricated flight labels; no fabricated no-contact state. `VALID_NO_CONTACT` is kept **only as a future extension** (explicit empty contact message or independent validated no-contact source), not required for Stage 1. Event-only publication is contract-consistent and no longer a blocking condition (RFC App. B V-1 failure conditions updated accordingly; D1 updated). Run 8: `publication_mode = confirmed` under the two-state contract. |
| **R8 — V-2 composition evidence** | V-2 asserted the composition recovery path without an explicit per-link evidence block or status, leaving the validation claim implicit. | Existing V-2 criterion extended (no new gate): `composition_evidence` evaluates **at least two non-identity links** (`l_ankle`, `r_ankle`) via `T_world_model × T_model_link`, compares against independent available pose sources (`/body_pose` — identity link only; `dynamic_pose/info` — measured parent-relative, not a world-pose source; standing geometry — envelope sanity only: **none usable**), and reports **`composition_status: UNVERIFIED`** — validation is not claimed. Composed foot/CoM fields remain **`derived_pending_validation`** until evidence exists; the collector may still store raw `Pose_V` and the model pose. RFC § 3.3 note/§ 3.5, App. B V-2.2; contract § 4.3. |
| **R9 — measured/derived labels** | `right_foot_pose` was still classified `measured` although it is a composed pose (REV1.2 had corrected only the left row) — schema classes must match the composition semantics. | `right_foot_pose` (and `left_foot_pose`, `com_position`) now `derived` (with `derived_pending_validation` marking); RFC § 3.0 adds the classification summary: **measured** = raw gz pose messages, model pose, raw contacts, joint states; **derived** = foot pose, CoM, CoP, ZMP, velocities, transforms. |
| **R10 — timestamp wording** | "timestamp semantics confirmed" over-claimed: nearest-neighbor statistics over a bounded window prove the stamps are *associable*, not that the clocks are universally identical. | Criterion renamed `timestamp_association_compatibility` (probe + RFC App. B V-1.4 + contract § 3.7): the evidence proves **source stamps are associable** within the declared windows; it does **NOT prove universal clock identity**. Implementation note: run 7 exposed the boundary case (nondeterministic rclpy/DDS discovery 1.8–4.5 s: contact stamps before the joint subscription went live had no joint stamps to associate with, p95 3.495 s); the statistics are now evaluated on the **common subscription window** with boundary-excluded samples counted and reported (86 left / 159 right in run 8) — the documented collector-contract § 5.3 method. Run 8: median 0.003 s / p95 0.005 s, `association_compatible = true`. |

## 3. Frozen-component and scope confirmation

- **No Stage 0 changes**: `git diff` over `dataset/stage0/**`, the Stage 0 tools, and Stage 0 documents is empty; `corpus_index.json` = `260fd9e7…` (pin re-verified).
- **No controller / gait / model / world / physics changes**: `git diff` over `tools/run_phase_a_episode.py`, `tools/run_p0a_gait_candidate.py`, `src/**` is empty; `nao_simple.sdf` = `44ed8285…` (pin re-verified).
- **No dataset acquisition**: no pilot episode, none of the 96 pre-registered episodes started; no collector writer executed; storage written only under `dataset/stage1/probe/`.
- **No redesign**: no new V-gate (R8 extends the existing V-2 criterion as evidence, not a gate), no dataset schema change, no optimization, no training.

## 4. Validation results (run 8 — REV1.3 criteria, walk + FROZEN_GAIT-equivalent, in-gait window)

Command:

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe8 --report-suffix REV13
```

Pinning: `git_commit = 559c24374064…`; `rfc.sha256 = d6a5e02286bb…` (REV1.3); `config_sha256 = b1847e15…`; robot walked (114 alternations, 0.442 m, body z ≥ 0.330 — no fall).

**V-1 = PASS** (`PROBE_EXIT = 0` for the first time under REV1.3 criteria):

| Criterion | Status | Key evidence |
|---|---|---|
| contact_publication_events (R6) | confirmed | 28/24 bursts; median/p95/max event dt 0.001/0.001/0.040 s (L) and 0.001/0.001/0.038 s (R); active_contact_event_rate 898.4/937.4 Hz; burst duration median 0.0595/0.0585 s; 27/23 silence intervals counted, not failed |
| body_identity | confirmed | fraction_expected_pair 1.0; verbatim pairs; 0 cross-contamination |
| publication_mode (R7) | confirmed | event-only publication, 0 empty messages — **two-state contract-consistent** (`two_state_contract` evidence block emitted); field population fractions 1.0 |
| timestamp_association_compatibility (R10) | confirmed | common-window nearest-neighbor: median 0.003 s / p95 0.005 s / max 0.005 s; boundary-excluded 86/159 samples reported; monotone stamps; bridge exact-stamp preservation 1.0 |
| frame_reference | unknown (non-blocking) | unchanged: N3L/standing are plausibility-only; CoP/derived-moment derivation stays blocked |

**V-2 = PASS**:

| Criterion | Status | Key evidence |
|---|---|---|
| unique_entities | confirmed | 25/25 links, single-message completeness 1.0 |
| pose_frame_semantics (R4) + composition_evidence (R8) | confirmed | `model_relative_for_links` (base_link identity 1.0 / model entry identity 0.0); `/body_pose` anchor exact-stamp 1.0, diff 0.0; identity-link composition consistency 0.0 m; **`composition_status = UNVERIFIED`** for l_ankle/r_ankle (no independent non-identity-link world-pose source available) — validation not claimed; foot/CoM remain `derived_pending_validation` |
| source_stamps_cadence | confirmed | measured cadence, monotone stamps |
| alignment_hit_rate | confirmed | 0.02 s causal hit rate 1.0 |

Expected outcomes from the task were met: publication_mode no longer blocking (two-state policy), cadence evaluated by event semantics, V-2 composition status explicit.

## 5. Remaining blockers

1. **Final independent audit of REV1.3** (process): the RFC is ACQUISITION-CONTRACT-READY but not self-accepted; Level 3 additionally requires audit acceptance, probe-result pinning finalization (local commits done this session), and remote publication of the Stage 0 closure commit and this contract (owner action, GPT-5.6 correction 8).
2. **`frame_reference` (wrench axes/reference point) — unknown (non-blocking)**: CoP and derived-moment derivation remain blocked pending the common-frame/common-origin comparison; the collector records raw wrenches verbatim.
3. **Composition validation — `composition_status: UNVERIFIED` (non-blocking)**: no independent non-identity-link world-pose source exists in the current pipeline; composed foot/CoM fields remain `derived_pending_validation` (an FK-based or otherwise independent pose source would be a future RFC extension).
4. **Data-quality observability (contract-level, pre-registered, not a probe blocker)**: event-only publication means no-contact frames carry `UNKNOWN_NO_MESSAGE` (missing-evidence) semantics by design — downstream analyses must treat them as missing, and flight/no-contact inferences from silence are forbidden (R7).

No acquisition is authorized by this report.

## 6. Commit hashes

- `fe606f2` — "stage1: apply REV1.3 minimal hardening to RFC and probe (R6-R10)" (RFC + probe; 2 files, +460/−188)
- `559c243` — "stage1: evaluate V-1 association statistics on common subscription window" (probe + RFC wording + run-7 skew archives; 4 files)
- Final evidence commit (this report, collector contract v1.2, run-8 REV13 reports): created after this file — `git log -1 --format=%H`; all commits on `main`, **not pushed** (push is an owner action).
